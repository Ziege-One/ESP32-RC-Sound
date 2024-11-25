// XTronical I2S Audio Library, currently supporting ESP32
// May work with other processors and/or I2S DAC's with or without modifications
// (c) XTronical 2018-2021, Licensed under GNU GPL 3.0 and later, under this license absolutely no warranty given
// See www.xtronical.com for documentation
// This software is currently under active development (Feb 2021) and may change and break your code with new versions
// No responsibility is taken for this. Stick with the version that works for you, if you need newer commands from later versions
// you may have to alter your original code

#include "XT_I2S_Audio.h"
#include <math.h>
#include <cstring>
#include <hardwareSerial.h>

WavHeader_Struct WavHeader;				// Used as a place to store the header data from the wav data

/**********************************************************************************************************************/
/* global functions                                                                                                   */
/**********************************************************************************************************************/

void SetVolume(int16_t *Left, int16_t *Right, int16_t Volume)
{
	// returns the sound samples adjusted for the volume passed, volume is passed as a %
	// 0 would cause silence, 50 would be half volume, 100 would be the original volume (unchanged).  Anything above
	// 100 would cause the volume to increase (i.e. gain); A negative value will leave volume alone and return
	// original values passed in

	float Vol = float(Volume) / 100;
	if (Volume < 0)
		return;
	*Left = (*Left) * Vol;
	*Right = (*Right) * Vol;
}

/**********************************************************************************************************************/
/* XT_PlayListItem_Class                                                                                              */
/**********************************************************************************************************************/

XT_PlayListItem_Class::XT_PlayListItem_Class()
{
	RepeatForever = false;
	Repeat = 0;
	RepeatIdx = 0;
}

// Despite being marked virtual and being overridden in desendents without these I get vtable error
// issues which I really don't understand, so I put these in even though never called and all is well
void XT_PlayListItem_Class::NextSample(int16_t *Left, int16_t *Right)
{
}
void XT_PlayListItem_Class::Init()
{
}

/**********************************************************************************************************************/
/* Wave class 			                                                                                              */
/**********************************************************************************************************************/
// Despite being marked virtual and being overridden in desendents without these I get vtable error
// issues which I really don't understand, so I put these in even though never called and all is well
void XT_Wave_Class::NextSample(int16_t *Left, int16_t *Right)
{
}
void XT_Wave_Class::Init(int8_t Note)
{
}

/**********************************************************************************************************************/
/* XT_WAV_Class                                                                                              		*/
/**********************************************************************************************************************/

#define DATA_CHUNK_ID 0x61746164
#define FMT_CHUNK_ID 0x20746d66
// Convert 4 byte little-endian to a long.
#define longword(bfr, ofs) (bfr[ofs + 3] << 24 | bfr[ofs + 2] << 16 | bfr[ofs + 1] << 8 | bfr[ofs + 0])

// XT_Wav_Class::XT_Wav_Class(const unsigned char *WavData)
// {
//     // create a new wav class object
// 	memcpy(&WavHeader,WavData,44);                     // Copy the header part of the wav data into our structure
// 	SampleRate=WavHeader.SampleRate;
// 	BytesPerSample=(WavHeader.BitsPerSample/8)*WavHeader.NumChannels;
// 	DataSize=WavHeader.DataSize;
//     IncreaseBy=float(SampleRate)/SAMPLES_PER_SEC;
//     PlayingTime = (1000 * DataSize) / (uint32_t)(SampleRate);
//     Data=WavData+44;
//     Speed=1.0;
// 	Volume=100;
// }

XT_Wav_Class::XT_Wav_Class(const String &name) // Load data from file on SD Card
	: FileName{name}
{
	SamplesBufferIsEmpty = true;
	// create a new wav class object

	// 	unsigned char c;
	// size_t numRead = file.read(&c, sizeof(c));
	// if (numRead != sizeof(c)) {
	//   // error, like end of file
	//   return;
	// }
	// // process c
}

//****** LATEST ADDITIONS **************************************************

bool XT_Wav_Class::ValidWavData(WavHeader_Struct *Wav)
{

	if (memcmp(Wav->RIFFSectionID, "RIFF", 4) != 0)
	{
		Serial.print("Invalid data - Not RIFF format");
		return false;
	}
	if (memcmp(Wav->RiffFormat, "WAVE", 4) != 0)
	{
		Serial.print("Invalid data - Not Wave file");
		return false;
	}
	if (memcmp(Wav->FormatSectionID, "fmt", 3) != 0)
	{
		Serial.print("Invalid data - No format section found");
		return false;
	}
	if (memcmp(Wav->DataSectionID, "data", 4) != 0)
	{
		Serial.print("Invalid data - data section not found");
		return false;
	}
	if (Wav->FormatID != 1)
	{
		Serial.print("Invalid data - format Id must be 1");
		return false;
	}
	if (Wav->FormatSize != 16)
	{
		Serial.print("Invalid data - format section size must be 16.");
		return false;
	}
	if ((Wav->NumChannels != 1) & (Wav->NumChannels != 2))
	{
		Serial.print("Invalid data - only mono or stereo permitted.");
		return false;
	}
	if (Wav->SampleRate > 48000)
	{
		Serial.print("Invalid data - Sample rate cannot be greater than 48000");
		return false;
	}
	if ((Wav->BitsPerSample != 8) & (Wav->BitsPerSample != 16))
	{
		Serial.print("Invalid data - Only 8 or 16 bits per sample permitted.");
		return false;
	}
	return true;
}

void XT_Wav_Class::PrintData(const char *Data, uint8_t NumBytes)
{
	for (uint8_t i = 0; i < NumBytes; i++)
		Serial.print(Data[i]);
	Serial.println();
}

void XT_Wav_Class::DumpWAVHeader(WavHeader_Struct *Wav)
{
	if (memcmp(Wav->RIFFSectionID, "RIFF", 4) != 0)
	{
		Serial.print("Not a RIFF format file - ");
		PrintData(Wav->RIFFSectionID, 4);
		return;
	}
	if (memcmp(Wav->RiffFormat, "WAVE", 4) != 0)
	{
		Serial.print("Not a WAVE file - ");
		PrintData(Wav->RiffFormat, 4);
		return;
	}
	if (memcmp(Wav->FormatSectionID, "fmt", 3) != 0)
	{
		Serial.print("fmt ID not present - ");
		PrintData(Wav->FormatSectionID, 3);
		return;
	}
	if (memcmp(Wav->DataSectionID, "data", 4) != 0)
	{
		Serial.print("data ID not present - ");
		PrintData(Wav->DataSectionID, 4);
		return;
	}
	// All looks good, dump the data
	Serial.println();
	Serial.print("Total size :");
	Serial.println(Wav->Size);
	Serial.print("Format section size :");
	Serial.println(Wav->FormatSize);
	Serial.print("Wave format :");
	Serial.println(Wav->FormatID);
	Serial.print("Channels :");
	Serial.println(Wav->NumChannels);
	Serial.print("Sample Rate :");
	Serial.println(Wav->SampleRate);
	Serial.print("Byte Rate :");
	Serial.println(Wav->ByteRate);
	Serial.print("Block Align :");
	Serial.println(Wav->BlockAlign);
	Serial.print("Bits Per Sample :");
	Serial.println(Wav->BitsPerSample);
	Serial.print("Data Size :");
	Serial.println(Wav->DataSize);
}

bool XT_Wav_Class::OpenWavFile()
{
	// Load wav file, if all goes ok returns true else false
	// WavHeader_Struct WavHeader;

	WavFile = SD.open(FileName); // Open the wav file
	if (WavFile == false)
	{
		Serial.print("Could not open :");
		Serial.println(FileName);
		return false;
	}
	else
	{
		WavFile.read((byte *)&WavHeader, 44); // Read in the WAV header, which is first 44 bytes of the file.
											  // We have to typecast to bytes for the "read" function
		return true;
	}
}

void XT_Wav_Class::UnLoadWavFile()
{
  WavFile.close();
}

void XT_Wav_Class::LoadWavFile()
{
	if (OpenWavFile()) // Load file,
	{
		if (ValidWavData(&WavHeader))
		{
			DumpWAVHeader(&WavHeader); // Dump the header data to serial, optional!
			Serial.println();
			// DataSize = WavHeader.DataSize; // Copy the data size into our wav structure
		}
		else
		{
			Serial.print("Ivalid Wave file header! Filename: ");
			Serial.println(FileName);
		}

		SampleRate = WavHeader.SampleRate;
		BytesPerSample = (WavHeader.BitsPerSample / 8) * WavHeader.NumChannels;
		NumChannels = WavHeader.NumChannels;
		DataSize = WavHeader.DataSize;
		IncreaseBy = float(SampleRate) / SAMPLES_PER_SEC;
		PlayingTime = (1000 * DataSize) / (uint32_t)(SampleRate);
		WavFile.seek(44);	// Start of wav data
		TotalBytesRead = 0; // Clear to no bytes read in so far

		Speed = 1.0;
		Volume = 100;
	}
}

///**** End Latest additions **************************************

void XT_Wav_Class::ReadFile()
{
	//Serial.println("Read");
	if (TotalBytesRead + NUM_BYTES_TO_READ_FROM_FILE > DataSize) // If next read will go past the end then adjust the
		LastNumBytesRead = DataSize - TotalBytesRead;			 // amount to read to whatever is remaining to read
	else
		LastNumBytesRead = NUM_BYTES_TO_READ_FROM_FILE; // Default to max to read

	WavFile.read(Samples, LastNumBytesRead); // Read in the bytes from the file
	SamplesDataIdx = 0;
	TotalBytesRead += LastNumBytesRead; // Update the total bytes red in so far

	// if (TotalBytesRead >= DataSize) // Have we read in all the data?
	// {
	// 	if (RepeatForever)
	// 	{
	// 		WavFile.seek(44);	// Reset to start of wav data
	// 		TotalBytesRead = 0; // Clear to no bytes read in so far
	// 	}
	// 	else
	// 		Playing = false; // Flag that wav has completed
	// }
}

void XT_Wav_Class::Init()
{
	//Serial.println("Init-------");
	LastIntCount = 0;
	if (Speed >= 0)
	{
		TotalBytesRead = 0;
		WavFile.seek(44); // Reset to start of wav data
		ReadFile();
	}

	// DataIdx = DataStart; // reset data pointer back to start of WAV data
	// else
	// DataIdx = DataStart + DataSize; // reset data pointer back to end of WAV data
	Count = 0;
	SpeedUpCount = 0;
	TimeElapsed = 0;
	TimeLeft = PlayingTime;
}

void XT_Wav_Class::NextSample(int16_t *Left, int16_t *Right)
{
	// Returns the next samples to be played, note that this routine will return values suitable to
	// be played back at SAMPLES_PER_SEC.
	// If mono then will return same data on left and right channels
	// Serial.println(Count);

	uint32_t IntPartOfCount;
	float ActualIncreaseBy;
	float CopyOfSpeed = Speed; // Copy into this var as it could potentially change during this routine giving odd results
	// increase the counter, if it goes to a new integer digit then write to DAC

	ActualIncreaseBy = IncreaseBy; // default if not playing slower than normal
	if (CopyOfSpeed <= 1.0)		   // manipulate IncreaseBy
		ActualIncreaseBy = IncreaseBy * (abs(CopyOfSpeed));
	Count += ActualIncreaseBy;
	IntPartOfCount = floor(Count);
	// return previous value by default

	if (NumChannels == 2)
	{
		*Left = (Samples[SamplesDataIdx + 1] << 8) + Samples[SamplesDataIdx];	   // Get last left channel Data
		*Right = (Samples[SamplesDataIdx + 3] << 8) + Samples[SamplesDataIdx + 2]; // Get last right channel Data
	}
	else if (NumChannels == 1)
	{																		   // if Mono Channel
		*Left = (Samples[SamplesDataIdx + 1] << 8) + Samples[SamplesDataIdx];  // Get last left channel Data
		*Right = (Samples[SamplesDataIdx + 1] << 8) + Samples[SamplesDataIdx]; // Right is same as left channel
	}
	// Serial.println(BytesPerSample);

	SetVolume(Left, Right, Volume);
	if (IntPartOfCount > LastIntCount)
	{
		if (CopyOfSpeed >= 0) // Playing in forward direction
		{
			if (CopyOfSpeed > 1.0)
			{
				// for speeding up we need to basically go through the data quicker as upping the frequency
				// that this routine is called is not an option as many sounds can potentially play through it
				// and we have speed control over each.

				// we now get the integer and decimal parts of this number and move the DataIdx on by "int" amount first
				double IntPartAsFloat, DecimalPart, TempSpeed;
				TempSpeed = CopyOfSpeed - 1.0;
				DecimalPart = modf(TempSpeed, &IntPartAsFloat);
				SamplesDataIdx += BytesPerSample * int(IntPartAsFloat); // always increase by the integer part
				SpeedUpCount += DecimalPart;							// This keeps track of the decimal part
				// If SpeedUpCount >1 then add this extra sample to the DataIdx too and subtract 1 from SpeedUpCount
				// This allows us "apparently" increment the DataIdx by a decimal amount
				if (SpeedUpCount >= 1)
				{
					SamplesDataIdx += BytesPerSample; // move another pos into data
					SpeedUpCount--;					  // Take it off SpeedUpCount
				}
			}
			// gone to a new integer of count, we need to send a new value to the I2S DAC next time
			// update the DataIDx counter

			LastIntCount = IntPartOfCount;
			SamplesDataIdx += BytesPerSample; // 4, because 2 x 16bit samples
			// if (NumChannels == 2)
			// 	SamplesDataIdx += 4; // 4, because 2 x 16bit samples
			// else
			// 	SamplesDataIdx += 2; // Mono Channel

			TimeElapsed = 1000 * SamplesDataIdx / BytesPerSample;
			TimeLeft = PlayingTime - TimeElapsed;

			if (TotalBytesRead >= DataSize) // end of data, flag end
			{
				Count = 0; // reset frequency counter
				SamplesDataIdx = 0;
				Playing = false; // mark as completed
				TimeLeft = 0;
			}

			else if (SamplesDataIdx >= LastNumBytesRead) // end of data, flag end
			{
				ReadFile(); // read file to reset data back to beginning of WAV data
			}
		}
		// else
		// {
		// 	// Playing sound in reverse
		// 	if(CopyOfSpeed<-1.0)
		// 	{
		// 		// for speeding up we need to basically go backwards through the data quicker as upping the frequency
		// 		// that this routine is called is not an option as many sounds can potentially play through it
		// 		// and we have speed control over each.

		// 		// we now get the integer and decimal parts of this number and move the DataIdx on by "int" amount first
		// 		double IntPartAsFloat,DecimalPart,TempSpeed;
		// 		TempSpeed=CopyOfSpeed+1.0;
		// 		DecimalPart=abs(modf(TempSpeed,&IntPartAsFloat));
		// 		DataIdx-=BytesPerSample*int(IntPartAsFloat);			// always decrease by the integer part
		// 		SpeedUpCount+=DecimalPart;								//This keeps track of the decimal part
		// 		// If SpeedUpCount >1 then add this extra sample to the DataIdx too and subtract 1 from SpeedUpCount
		// 		// This allows us "apparently" increment the DataIdx by a decimal amount
		// 		if(SpeedUpCount>=1)
		// 		{
		// 			DataIdx-=BytesPerSample;				// move another pos into data
		// 			SpeedUpCount--;			// Take it off SpeedUpCount
		// 		}
		// 	}
		// 	// gone to a new integer of count, we need to send a new value to the I2S DAC next time
		// 	// update the DataIDx counterz

		// 	LastIntCount=IntPartOfCount;
		// 	DataIdx-=4;							// 4, because 2 x 16bit samples
		// 	TimeElapsed = PlayingTime-(1000 * DataIdx / BytesPerSample);
		// 	TimeLeft = PlayingTime - TimeElapsed;
		// 	if(DataIdx<0)  				// end of data, flag end
		// 	{

		// 		Count=0;						// reset frequency counter
		// 		DataIdx=DataStart+DataSize;		// reset data pointer back to end of WAV data
		// 		Playing=false;  				// mark as completed
		// 		TimeLeft = 0;
		// 	}
		// }
	}

	// Serial.println("OUT"); // debugging
}



/**********************************************************************************************************************/
/* XT_I2S_Class                                                                                                       */
/**********************************************************************************************************************/

XT_I2S_Class::XT_I2S_Class(uint8_t LRCLKPin, uint8_t BCLKPin, uint8_t I2SOutPin, i2s_port_t PassedPortNum)
{
	// Set up I2S config structure8
	// Port number is either 0 or 1, I2S_NUM_0, I2S_NUM_1, at least for ESP32 it is

	i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
	i2s_config.sample_rate = SAMPLES_PER_SEC; // Note, max sample rate
	i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
	i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
	i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
	i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1; // high interrupt priority
	i2s_config.dma_buf_count = 8;						// 8 buffers
	i2s_config.dma_buf_len = 256;						// 256 bytes per buffer, so 2K of buffer space
	i2s_config.use_apll = 0;
	i2s_config.tx_desc_auto_clear = true;
	i2s_config.fixed_mclk = -1;

	// Set up I2S pin config structure
	pin_config.bck_io_num = BCLKPin;			// The bit clock connection
	pin_config.ws_io_num = LRCLKPin;			// Word select, also known as left right clock
	pin_config.data_out_num = I2SOutPin;		// Data out from the ESP32, connect to DIN on 38357A
	pin_config.data_in_num = I2S_PIN_NO_CHANGE; // we are not interested in I2S data into the ESP32

	PortNum = PassedPortNum; // Store port number used (for freeing up resources in desconstructor)

	i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL); // ESP32 will allocated resources to run I2S
	i2s_set_pin(PortNum, &pin_config);					 // Tell it the pins you will be using


}

XT_I2S_Class::~XT_I2S_Class()
{
	i2s_driver_uninstall(PortNum); // Free resources
}

void XT_I2S_Class::Play(XT_PlayListItem_Class *Sound)
{
	Play(Sound, true, -1); // Sounds mix by default
}

void XT_I2S_Class::Play(XT_PlayListItem_Class *Sound, bool Mix)
{
	Play(Sound, Mix, -1); // Sounds mix by default
}

void XT_I2S_Class::Play(XT_PlayListItem_Class *Sound, bool Mix, int16_t TheVolume)
{

	// check if this sound is already playing, if so it is removed first and will be re-played
	// This limitation maybe removed in later version so that multiple versions of the same
	// sound can be played at once. Trying to do that now will corrupt the list of items to play
	// Volume of -1 will leave at default. As a work around just create a new sound object of you
	// want to play a sound more than once. For example if you want a laser sound to happen perhaps
	// many times as user presses fire, then just create a new instance of that sound first and pass to
	// this routine.

	if (AlreadyPlaying(Sound))
		RemoveFromPlayList(Sound);
	if (Mix == false) // stop all currently playing sounds and just have this one
		StopAllSounds();

	Sound->NewSound = true;			  // Flags to fill buffer routine that this is brand new sound
									  // with nothing yet put into buffer for playing
	Sound->RepeatIdx = Sound->Repeat; // Initialise any repeats
	if (TheVolume >= 0)
		Sound->Volume = TheVolume;

	// set up this sound to play, different types of sound may initialise differently
	Sound->Init();

	// add to list of currently playing items
	// create a new play list entry
	if (FirstPlayListItem == nullptr) // no items to play in list yet. TEB, Oct-10-2019
	{
		FirstPlayListItem = Sound;
		LastPlayListItem = Sound;
	}
	else
	{
		// add to end of list
		LastPlayListItem->NextItem = Sound;
		Sound->PreviousItem = LastPlayListItem;
		LastPlayListItem = Sound;
	}
	Sound->Playing = true; // Will start it playing
}

void XT_I2S_Class::RemoveFromPlayList(XT_PlayListItem_Class *ItemToRemove)
{
	// removes a play item from the play list
	if (ItemToRemove->PreviousItem != nullptr)
		ItemToRemove->PreviousItem->NextItem = ItemToRemove->NextItem;
	else
		FirstPlayListItem = ItemToRemove->NextItem;
	if (ItemToRemove->NextItem != nullptr)
		ItemToRemove->NextItem->PreviousItem = ItemToRemove->PreviousItem;
	else
		LastPlayListItem = ItemToRemove->PreviousItem;

	ItemToRemove->PreviousItem = nullptr;
	ItemToRemove->NextItem = nullptr;
}

bool XT_I2S_Class::AlreadyPlaying(XT_PlayListItem_Class *Item)
{
	// returns true if sound already in list of items to play else false
	XT_PlayListItem_Class *PlayItem;
	PlayItem = FirstPlayListItem;
	while (PlayItem != nullptr)
	{
		if (PlayItem == Item)
			return true;
		PlayItem = PlayItem->NextItem;
	}
	return false;
}

void XT_I2S_Class::StopAllSounds()
{
	// stop all sounds and clear the play list

	XT_PlayListItem_Class *PlayItem;
	PlayItem = FirstPlayListItem;
	while (PlayItem != nullptr)
	{
		PlayItem->Playing = false;
		RemoveFromPlayList(PlayItem);
		PlayItem = FirstPlayListItem;
	}
	FirstPlayListItem = nullptr;
}

int16_t CheckTopBottomedOut(int32_t Sample)
{
	// checks if sample is greate or less than would be allowed of it was a 16 but number, if so returns the max value
	// that wpuld be allowed for a signed 16 bt number
	if (Sample > 32767)
		return 32767; // Sound has topped out, return max
	if (Sample < -32768)
		return -32768; // Sound has bottomed out, return max
	return Sample;	   // Sound is within range, return unchanged
}

uint32_t XT_I2S_Class::MixSamples()
{

	// Goes through all sounds and mixes the next sample together, returns the mixed stereo 16 bit sample (so 4 bytes)
	XT_PlayListItem_Class *PlayItem, *NextPlayItem;
	uint32_t MixedSample;
	int16_t LeftSample, RightSample; // returned samples
	int32_t LeftMix, RightMix;		 // Bigger than 16bits as we are adding several samples together

	PlayItem = FirstPlayListItem;
	LeftMix = 0;  // Start with silence
	RightMix = 0; // Start with silence
	while (PlayItem != nullptr)
	{
		if (PlayItem->Playing) // If still playing
		{

			PlayItem->NextSample(&LeftSample, &RightSample); // Get next byte from this sound to play
			if (PlayItem->Filter != 0)
				PlayItem->Filter->FilterWave(&LeftSample, &RightSample); // Adjust by any set filter, a Work in Progress
			// set any volume for this sample
			SetVolume(&LeftSample, &RightSample, PlayItem->Volume);
			LeftMix += LeftSample;
			RightMix += RightSample;
		}

		NextPlayItem = PlayItem->NextItem; // move to next play item
		if (PlayItem->Playing == false)	   // If this play item completed
		{
			if (PlayItem->RepeatForever)
			{
				PlayItem->Init(); // initialise for playing again
				PlayItem->Playing = true;
			}
			else
			{
				if (PlayItem->RepeatIdx > 0) // Repeat sound X amount of times
				{
					PlayItem->RepeatIdx--;
					PlayItem->Init(); // initialise for playing again
					PlayItem->Playing = true;
				}
				else
					RemoveFromPlayList(PlayItem); // no repeats, remove from play list
			}
		}
		PlayItem = NextPlayItem; // Set to next item
	}
	// master volume (volume of all output), have to convert to 16bit temporarily for the volume function
	int16_t Left = int16_t(LeftMix);
	int16_t Right = int16_t(RightMix);
	SetVolume(&Left, &Right, Volume);
	LeftMix = Left; // Back into 32bit here as
	RightMix = Right;
	// Correct sample if topped or bottemed out (wave gone beyond max values for 16 bit), and ensure it's set to a 16 bit number
	// We had to work in 32 bits whilst mixing so we didn't potentially lose any resolution when mixing. But now need 16 bits
	// again before we combine and return the final 2 samples as 1 32bit value
	LeftMix = uint16_t(CheckTopBottomedOut(LeftMix));
	RightMix = uint16_t(CheckTopBottomedOut(RightMix));
	// move the 2 16bit samples into one 32bit variable
	MixedSample = (LeftMix << 16) | (RightMix);
	return MixedSample;
}

void XT_I2S_Class::FillBuffer()
{
	// Basically fill the I2S buffer with data. However as we want to mix sounds etc and a sound may have been playing for
	// a while when we add another sound (to mix) then we need somewhere where we can mix these sounds, as once sent to the
	// I2S buffer it would be hard if not impossible to try to mix in those buffers. So we have two more buffers that we
	// have control over that we can use to mix in sounds as required. It's one of these buffers that we will be sending to
	// the I2S system, we will fill the I2S with one buffer as we mix sounds together into the other and then swap
	// periodically.

	uint32_t MixedSample;
	size_t NumBytesWritten = 4; // Set to 4 so loop enters below

	while (NumBytesWritten > 0) // keep filling until full, note of we run out of samples then MixSamples will return 0 (silence)
	{							// i2s_write (below) should fill in 4 byte chunks, so will return 0 when full
		MixedSample = MixSamples();
		// Store sample in buffer, increment buffer pointer, check for end of buffer or for end of sample data
		// The 4 is number of bytes to write, it's 4 as 2 bytes per sample and stereo (2 samples)
		// 1 is max number of rtos ticks to wait
		i2s_write(I2S_NUM_0, &MixedSample, 4, &NumBytesWritten, 1);
		
	}
	//Serial.println(NumBytesWritten);
}
