// XTronical I2S Audio Library, currently supporting ESP32
// May work with other processors and/or DAC's with or without modifications
// (c) XTronical 2018-2021, Licensed under GNU GPL 3.0 and later, under this license absolutely no warranty given
// See www.xtronical.com for documentation
// This software is currently under active development (Jan 2018) and may change and break your code with new versions
// No responsibility is taken for this. Stick with the version that works for you, if you need newer commands from later versions
// you will have to alter your original code to work with the new if required.
// NOTES: Wav samples can be mono or stereo and 8 or 16 bit. Maximum sample rate of 44100 samples per second. They must be
// signed waves NOT unsigned. Check you settings on your wavs.

#include "driver/i2s.h" // Library of I2S routines, comes with ESP32 standard install
#include "SD.h" // SD Card library, usually part of the standard install

#define SAMPLES_PER_SEC 44100			 // The rate at which samples are sent out. A single sample is fixed at 2 bytes stereo
										 // So that's 4 bytes (2 x 16bits) per sample, 44100 x 4bytes per second
										 // We can play samples at different rates to this as the code automatically
										 // manipulates/re-samples the outputed waveform/ data to still appear to play at the
										 // intended bit rate even though it will come out at this speed.
#define SINE_RESOLUTION 1024			 // Used when outputing sine waves, higher the number the better the sound, although 1024 seems opitimal
										 // to my ear anyway. But this takes up memory as 2 bytes per entry
#define BUFFER_SIZE 1024				 // Size of buffers
#define NUM_BYTES_TO_READ_FROM_FILE 1024 // How many bytes to read from wav file at a time

#define MAX_WAVE_HEIGHT 32767  // Max height wave can be
#define MIN_WAVE_HEIGHT -32768 // Min height wave can be

#define DEFAULT_MASTER_VOLUME 50 // Master output volume for the final mixed sound

class XT_Envelope_Class; // forward ref for this class

//------------------------------------------------------------------------------------------------------------------------
// structs

// Wave header struct, makes it easier to access data in wav header part of file

struct WavHeader_Struct
{
	//   RIFF Section
	char RIFFSectionID[4]; // Letters "RIFF"
	uint32_t Size;		   // Size of entire file less 8
	char RiffFormat[4];	   // Letters "WAVE"

	//   Format Section
	char FormatSectionID[4]; // letters "fmt"
	uint32_t FormatSize;	 // Size of format section less 8
	uint16_t FormatID;		 // 1=uncompressed PCM
	uint16_t NumChannels;	 // 1=mono,2=stereo
	uint32_t SampleRate;	 // 44100, 16000, 8000 etc.
	uint32_t ByteRate;		 // =SampleRate * Channels * (BitsPerSample/8)
	uint16_t BlockAlign;	 // =Channels * (BitsPerSample/8)
	uint16_t BitsPerSample;	 // 8,16,24 or 32

	// Data Section
	char DataSectionID[4]; // The letters "data"
	uint32_t DataSize;	   // Size of the data that follows
};						   // We'll copy data into this struct as we need to

//------------------------------------------------------------------------------------------------------------------------
// classes

class XT_Filter_Class
{
	// base class for a filter, a filter is a device that manipulates the wave form
	// after it has been produced. Various are available inheriting from this class
public:
	virtual void FilterWave(int16_t *Left, int16_t *Right);
};

class XT_PlayListItem_Class
{
	// The main play list class from which other classes derive
private:
protected:
public:
	// These are the linked list items for the sounds to play through the I2S. Wehn you add a new item to play (mxing)
	// it will be added as an item to the mian playlist

	XT_PlayListItem_Class *PreviousItem = nullptr; // Previous item in list, 0 if none (i.e. start of list)
	XT_PlayListItem_Class *NextItem = nullptr;	   // Next item in list, 0 if none (i.e. end of list)
	XT_Filter_Class *Filter = nullptr;
	bool DumpData = false; // For debug only

	bool Playing = false; // true if currently being played else false
	bool NewSound;		  // When first set to play this will be true, once this
						  // sound has started sending bytes to the buffer it will be false
						  // Helps when setting NextFillPos (below) to initial value
	uint32_t PlayingTime; // Total playing time of this item in 1000's of a second
	uint32_t TimeElapsed; // Total playing time elapsed so far in 1000's of a second
	uint32_t TimeLeft;	  // Total playing time left so far in 1000's of a second

	uint32_t NextFillPos; // Next fill position in buffer to fill, because we have
						  // a large buffer, all sounds need their individual fill positions
						  // because you could be playing one sound which has filled perhaps
						  // 2 seconds worth of buffer (but not yet played those 2 s)
						  // and then mix in another. In this situation the bytes from the new
						  // sound must be mixed at the current playing point. If you did
						  // not do this then your new sound would not start until about
						  // 2 seconds later and not when you intended.

	char Name[32];		 // Name of this sound - optional, used mostly in debugging
	uint8_t LastValue;	 // Last value returned from NextByte function below
	int16_t Volume = 25; // VOl as %, 0=silence, 50 half, 100 full, 200 twice as loud as original

	bool RepeatForever; // if true repeats forever, if true value for repeat below ignored
	uint16_t Repeat;	// Number of times to repeat, 1 to 65535
	uint16_t RepeatIdx; // internal use only, do not use, should put this in protected or
						// private really and have an access function, do in future

	virtual void NextSample(int16_t *Left, int16_t *Right); // to be overridden by any descendants
	virtual void Init();									// initialize any default values

	XT_PlayListItem_Class();
};

// Envelope class for instruments to use, see www.xtronical.com for details
class XT_EnvelopePart_Class
{
	// Definition of a single part of a sound envelope

private:
	uint16_t Duration; // How long this part plays for in 1000's of a second

public:
	int16_t StartVolume = -1; // As %, if negative then indicates the volume should be left at current value for this sound
	int16_t TargetVolume;	  // volume to reach in this part 0-100
	uint32_t RawDuration;	  // duration in SAMPLE_RATE's, so if SAMPLE_RATE os 44100, then a val of 44100 would be a duration of 1 sec.
	bool Completed;

	XT_EnvelopePart_Class *PreviousPart = nullptr;
	XT_EnvelopePart_Class *NextPart = nullptr;

	// functions
	void SetDuration(uint16_t Duration);
	uint16_t GetDuration();
};

class XT_Wave_Class
{
	// Base Wave class for all others to inherit from (sine, square etc.)
	// Every Instrument must have a wave form class, by default if none specified
	// then uses square

protected:
	int16_t CurrentLeftSample = MAX_WAVE_HEIGHT;
	int16_t CurrentRightSample = MAX_WAVE_HEIGHT;
	float ChangeAmplitudeBy;
	float NextAmplitude;
	float CounterStartValue;
	float Counter;

public:
	uint16_t Frequency; // Note frequency
	virtual void NextSample(int16_t *Left, int16_t *Right);
	virtual void Init(int8_t Note);
};

class XT_SquareWave_Class : public XT_Wave_Class
{
	// Square wave class
public:
	void NextSample(int16_t *Left, int16_t *Right);
	void Init(int8_t Note);
};

// The Main Wave class for sound samples
class XT_Wav_Class : public XT_PlayListItem_Class
{
private:
protected:
public:
	File WavFile; // Object for accessing the opened wavfile
	String FileName;
	uint8_t Samples[NUM_BYTES_TO_READ_FROM_FILE];
	bool SamplesBufferIsEmpty;
	uint32_t TotalBytesRead = 0; // Number of bytes read from file so far
	uint16_t LastNumBytesRead;	 // Num bytes actually read from the wav file which will either be
								 // NUM_BYTES_TO_READ_FROM_FILE or less than this if we are very
								 // near the end of the file. i.e. we can't read beyond the file.

	// bool RepeatForever;						// if true repeats forever, if true value for repeat below ignored

	uint16_t SampleRate;
	uint8_t BytesPerSample; // i.e. 1,2 4 etc. if stereo and 2 bytes per sample this would be 4
	uint16_t NumChannels;	 // 1=mono,2=stereo
	uint32_t DataSize = 0;	// The last integer part of count
	uint32_t DataStart;		// offset of the actual data.
	int64_t SamplesDataIdx; // Index into the wavs sample data in buffer data loaded from file
	const unsigned char *Data;
	float IncreaseBy = 0;	   // The amount to increase the counter by per call to "onTimer"
	float Count = 0;		   // The counter counting up, we check this to see if we need to send
							   // a new byte to the buffer
	int32_t LastIntCount = -1; // The last integer part of count
	float SpeedUpCount = 0;	   // The decimal part of the amount to increment the dataptr by for the
							   // next byte to send to the DAC. As we cannot move through data in
							   // partial amounts then we keep track of the decimal and when it
							   // "clicks over" to a new integer we increment the dataptr an extra
							   // "1" place.
	float Speed = 1.0;		   // 1 is normal, less than 1 slower, more than 1 faster, i.e. 2
							   // is twice as fast, 0.5 half as fast., use the getter and setter
							   // functions to access. default is 1, normal speed.

	// constructors
	// XT_Wav_Class(const unsigned char *WavData);
	XT_Wav_Class(const String &FileName);

	// functions
	void NextSample(int16_t *Left, int16_t *Right);
	void Init() override; // initialize any default values

	void ReadFile();
	void LoadWavFile();
  void UnLoadWavFile();
	bool OpenWavFile();
	bool ValidWavData(WavHeader_Struct *Wav);
	void DumpWAVHeader(WavHeader_Struct *Wav);
	void PrintData(const char *Data, uint8_t NumBytes);
};

class XT_I2S_Class
{
private:
	i2s_config_t i2s_config;
	i2s_pin_config_t pin_config;
	i2s_port_t PortNum;
	XT_PlayListItem_Class *FirstPlayListItem = nullptr; // first play list item to play in linked list.
	XT_PlayListItem_Class *LastPlayListItem = nullptr;	// last play list item to play in linked list.

public:
	XT_I2S_Class(uint8_t LRCLKPin, uint8_t BCLKPin, uint8_t I2SOutPin, i2s_port_t PortNum);
	~XT_I2S_Class();
	void FillBuffer();						// Fills the buffer
	uint32_t MixSamples();					// Goes through all sounds and mixes together, returns the mixed byte
	int16_t Volume = DEFAULT_MASTER_VOLUME; // Default master volume
	bool DumpData = false;					// For debug only

	void Play(XT_PlayListItem_Class *Sound);
	void Play(XT_PlayListItem_Class *Sound, bool Mix);
	void Play(XT_PlayListItem_Class *Sound, bool Mix, int16_t TheVolume);
	void StopAllSounds();
	bool AlreadyPlaying(XT_PlayListItem_Class *Item);
	void RemoveFromPlayList(XT_PlayListItem_Class *ItemToRemove);
};
