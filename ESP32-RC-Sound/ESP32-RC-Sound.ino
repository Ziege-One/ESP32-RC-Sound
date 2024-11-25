/*
   ESP32-RC-Sound
   Ziege-One (Der RC-Modellbauer)
 
 ESP32 + SD + S2I Audio

 /////Pin Belegung////
 GPIO 13: Wifi Pin
 GPIO 14: Hardware Pin
 GPIO 27: 
 GPIO 33: 
 GPIO 32: 
 GPIO 15: 
 GPIO 16: Input 1
 GPIO 17: Input 2
 GPIO 22: Input 3
 GPIO 00: Input 4
 GPIO 02: Input 5  
 GPIO 04: Input 6

 GPIO 05: SD_CS
 GPIO 18: SD_CLK  
 GPIO 19: SD_MISO
 GPIO 23: SD_MOSI

 GPIO 21: I2S_DOUT
 GPIO 25: I2S_LRC
 GPIO 26: I2S_BCLK

 */

// ======== ESP32-RC-Sound =======================================

/* Boardversion
ESP32                                         2.0.9
 */

/* Installierte Bibliotheken
Bolder Flight Systems SBUS                    8.1.4
 */

#include <Arduino.h>
#include "XT_I2S_Audio.h"
#include <WiFi.h>
#include <EEPROM.h>
#include "sbus.h"

const float Version = 0.36; // Software Version

// EEprom

#define EEPROM_SIZE 132

#define adr_eprom_Source_Speed_Sound_0        4   // Source_Speed_Sound 1
#define adr_eprom_Source_Start_Sound_0        8   // Source_Start_Sound 1
#define adr_eprom_Source_Start_Sound_1        12  // Source_Start_Sound 2
#define adr_eprom_Source_Start_Sound_2        16  // Source_Start_Sound 3
#define adr_eprom_Source_Start_Sound_3        20  // Source_Start_Sound 4
#define adr_eprom_Source_Start_Sound_4        24  // Source_Start_Sound 5
#define adr_eprom_Source_Start_Sound_5        28  // Source_Start_Sound 6
#define adr_eprom_Source_Start_Sound_6        32  // Source_Start_Sound 7
#define adr_eprom_Source_Start_Sound_7        36  // Source_Start_Sound 8
#define adr_eprom_Source_Start_Sound_8        40  // Source_Start_Sound 9
#define adr_eprom_Source_throttle_mode        44  // Source_Gas_Modus 1 = Vor/Zurück
#define adr_eprom_Min_Speed_Sound_0           48  // untere Geschwindigkeit Motor in % (100% ist 1fache Wiedergabegeschwindigkeit
#define adr_eprom_Max_Speed_Sound_0           52  // obere Geschwindigkeit Motor in % (150% ist 1,5fache Wiedergabegeschwindigkeit
#define adr_eprom_Volumen_Sound_0             56  // Volumen Sound 1 Normal = 100
#define adr_eprom_Volumen_Sound_1             60  // Volumen Sound 2 Normal = 100
#define adr_eprom_Volumen_Sound_2             64  // Volumen Sound 3 Normal = 100
#define adr_eprom_Volumen_Sound_3             68  // Volumen Sound 4 Normal = 100
#define adr_eprom_Volumen_Sound_4             72  // Volumen Sound 5 Normal = 100
#define adr_eprom_Volumen_Sound_5             76  // Volumen Sound 6 Normal = 100
#define adr_eprom_Volumen_Sound_6             80  // Volumen Sound 7 Normal = 100
#define adr_eprom_Volumen_Sound_7             84  // Volumen Sound 8 Normal = 100
#define adr_eprom_Volumen_Sound_8             88  // Volumen Sound 9 Normal = 100
#define adr_eprom_RepeatForever               92  // Sound Repeat (Loop)
#define adr_eprom_sbus_channel_einkanal       96  // SBUS Channel Einkanal
#define adr_eprom_sbus_channel_einkanal_mode  100  // SBUS Channel Mode Einkanal
#define adr_eprom_shutdowndelay               104  // Sekunden Gas 0 dann Motor aus
#define adr_eprom_engine_on_toogle            108  // Motor ein aus durch Toogle
#define adr_eprom_Source_Ebenen_Um_Kanal      112  // Kanal für Ebenen umschaltung
#define adr_eprom_Source_Ebenen_Kanal         116  // Kanal für Ebene
#define adr_eprom_throttle_ramp               120  // max change of throttle in one Second
#define adr_eprom_throttle_dead_band          124  // Totband Gas
#define adr_eprom_Hardware_Config             128  // Hardware Config

int Hardware_Config;                                                                            // Hardware Config
int Source_Speed_Sound_0;                                                                       // Source_Speed_Sound 1
int Min_Speed_Sound_0;                                                                          // untere Geschwindigkeit Motor in % (100% ist 1fache Wiedergabegeschwindigkeit
int Max_Speed_Sound_0;                                                                          // obere Geschwindigkeit Motor in % (150% ist 1,5fache Wiedergabegeschwindigkeit
int Source_Start_Sound[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};                                        // Source_Start_Sound 1-9
bool Source_Start_Sound_OK[9] = {false, false, false, false, false, false, false, false, false};// Source_Start_Sound 1-9 Quellen OK (SBUS oder PWM vorhanden)
int Volumen_Sound[9] = {100, 100, 100, 100, 100, 100, 100, 100, 100};                           // Lautstärke Sound 1-9

bool Sound_on[9] = {false, false, false, false, false, false, false, false, false};        // Sound starten 1-9
bool Sound_play[9] = {false, false, false, false, false, false, false, false, false};      // Sound spielt 1-9
bool Sound_on_web[9] = {false, false, false, false, false, false, false, false, false};    // Sound starten über Webinterface 1-9

bool Sound_on_Motor;       // Sound Motor starten
bool Sound_on_Motor_state; // Sound Motor Toogle Speicher

int RepeatForever = 0;     // Sound Repeat (Loop)

// EinKanalModus
int sbus_channel_einkanal = 4; // Kanal 5 default
bool sbus_channel_einkanal_mode = false;
uint16_t einkanal_Data;         // Daten für die Ausgänge

//EbenenUmschaltung
int Source_Ebenen_Um_Kanal; // 
int Source_Ebenen_Um_Kanal_wert = 0;
int Source_Ebenen_Kanal; //
int Source_Ebenen_Kanal_wert = 0;
uint16_t ebenenkanal_Data = 99;         // Daten für die Ausgänge 

// SD Card 
#define SD_CS 5 // SD Card chip select

// I2S Audio
#define I2S_DOUT 21 // i2S Data out oin
#define I2S_BCLK 26 // Bit clock
#define I2S_LRC  25 // Left/Right clock, also known as Frame clock or word select

// Pins zuweisen 
volatile unsigned char WifiPin = 13;
volatile unsigned char LedPin = 2;
volatile unsigned char Input_Pin[6]  ={16, 17, 22, 0, 2, 4}; // Pin-Eingang


// Motor
bool throttle_mode; 
bool engine_break;
bool engine_on_toogle;
int shutdowndelay = 15;               					          // Number of seconds to wait before shutdown sound is played.
unsigned long shutdown_timer;
volatile uint8_t engine_State = 0;                        // Motor Status 
enum Engine_State                                         // Motor Status enum
{
  OFF,
  STARTING,
  RUNNING,
  STOPPING,
};

// Ramp
unsigned long previousTime_ramp = 0;         // Previous time
float last_throttle = 0;
int throttle_ramp = 20;       // max change of throttle in one Second
float throttle;      
int throttle_dead_band = 10;  //Totband Gas

// Wlan Einstellungen AP Modus
const char* ssid     = "ESP32-RC-Sound";
const char* password = "123456789";

// Webserver auf Port 80
WiFiServer server(80);

// Speicher HTTP request
String header;

// Für HTTP GET value
String valueString = String(5);
int pos1 = 0;
int pos2 = 0;

int Menu = 0;

//Timer Zeiten
unsigned long currentTime = millis();   // Aktuelle Zeit
unsigned long previousTime = 0;         // Previous time
const long timeoutTime = 2000;          // Define timeout time in milliseconds (example: 2000ms = 2s)

// SBUS
bfs::SbusRx sbus_rx(&Serial2,16, 27,true); // Sbus auf Serial2

bfs::SbusData sbus_data;

bool SBUS_OK = false;

// I2S 
XT_I2S_Class I2SAudio(I2S_LRC, I2S_BCLK, I2S_DOUT, I2S_NUM_0);

// Sounds Laden 
XT_Wav_Class Sound_loop("/loop.wav");
XT_Wav_Class Sound_shut("/shut.wav");
XT_Wav_Class Sound_start("/start.wav");
XT_Wav_Class Sound1("/sound1.wav");
XT_Wav_Class Sound2("/sound2.wav");
XT_Wav_Class Sound3("/sound3.wav");
XT_Wav_Class Sound4("/sound4.wav");
XT_Wav_Class Sound5("/sound5.wav");
XT_Wav_Class Sound6("/sound6.wav");
XT_Wav_Class Sound7("/sound7.wav");
XT_Wav_Class Sound8("/sound8.wav");

// PWM lesen mit Interrupt
volatile unsigned int PWM_pulse_width[6] = {3000, 3000, 3000, 3000, 3000, 3000};  
volatile unsigned int PWM_prev_time[6] = {0, 0, 0, 0, 0, 0}; 

void PWM_UP_0() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[0]), &PWM_DOWN_0, FALLING);  // wird der PIN false, rufe PWM_DOWN_X auf
  PWM_prev_time[0] = micros();
}
 
void PWM_DOWN_0() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[0]), &PWM_UP_0, RISING);  // wird der PIN high, rufe PWM_UP_X auf
  PWM_pulse_width[0] = micros() - PWM_prev_time[0];
}

void PWM_UP_1() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[1]), &PWM_DOWN_1, FALLING);  // wird der PIN false, rufe PWM_DOWN_X auf
  PWM_prev_time[1] = micros();
}
 
void PWM_DOWN_1() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[1]), &PWM_UP_1, RISING);  // wird der PIN high, rufe PWM_UP_X auf
  PWM_pulse_width[1] = micros() - PWM_prev_time[1];
}

void PWM_UP_2() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[2]), &PWM_DOWN_2, FALLING);  // wird der PIN false, rufe PWM_DOWN_X auf
  PWM_prev_time[2] = micros();
}
 
void PWM_DOWN_2() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[2]), &PWM_UP_2, RISING);  // wird der PIN high, rufe PWM_UP_X auf
  PWM_pulse_width[2] = micros() - PWM_prev_time[2];
}

void PWM_UP_3() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[3]), &PWM_DOWN_3, FALLING);  // wird der PIN false, rufe PWM_DOWN_X auf
  PWM_prev_time[3] = micros();
}
 
void PWM_DOWN_3() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[3]), &PWM_UP_3, RISING);  // wird der PIN high, rufe PWM_UP_X auf
  PWM_pulse_width[3] = micros() - PWM_prev_time[3];
}

void PWM_UP_4() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[4]), &PWM_DOWN_4, FALLING);  // wird der PIN false, rufe PWM_DOWN_X auf
  PWM_prev_time[4] = micros();
}
 
void PWM_DOWN_4() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[4]), &PWM_UP_4, RISING);  // wird der PIN high, rufe PWM_UP_X auf
  PWM_pulse_width[4] = micros() - PWM_prev_time[4];
}

void PWM_UP_5() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[5]), &PWM_DOWN_5, FALLING);  // wird der PIN false, rufe PWM_DOWN_X auf
  PWM_prev_time[5] = micros();
}
 
void PWM_DOWN_5() {
  attachInterrupt(digitalPinToInterrupt(Input_Pin[5]), &PWM_UP_5, RISING);  // wird der PIN high, rufe PWM_UP_X auf
  PWM_pulse_width[5] = micros() - PWM_prev_time[5];
}

// ======== Setup =======================================
void setup()
{
  Serial.begin(115200); // Used for info/debug

  EEPROM.begin(EEPROM_SIZE); //Eprom init

  EEprom_Load(); // Einstellung laden
  
  sbus_rx.Begin(); // SBUS init

  // Config Modus
  if (Hardware_Config == 0)
  {
    Serial.print("Config V1");
    Input_Pin[0] = 16; // Pin-Eingang
    Input_Pin[1] = 17; // Pin-Eingang
    Input_Pin[2] = 22; // Pin-Eingang
    Input_Pin[3] =  0; // Pin-Eingang
    Input_Pin[4] =  2; // Pin-Eingang
    Input_Pin[5] =  4; // Pin-Eingang
  }
  else if (Hardware_Config == 1)
  {
    Serial.print("Config V2");
    Input_Pin[0] = 16; // Pin-Eingang
    Input_Pin[1] = 17; // Pin-Eingang
    Input_Pin[2] = 14; // Pin-Eingang
    Input_Pin[3] = 27; // Pin-Eingang
    Input_Pin[4] = 32; // Pin-Eingang
    Input_Pin[5] = 33; // Pin-Eingang 
  }

  pinMode(WifiPin, INPUT_PULLUP);
  //pinMode(LedPin, OUTPUT);

  //pinMode(Input_Pin[0], INPUT_PULLUP); 
  pinMode(Input_Pin[1], INPUT_PULLUP); 
  pinMode(Input_Pin[2], INPUT_PULLUP);    
  pinMode(Input_Pin[3], INPUT_PULLUP);    
  pinMode(Input_Pin[4], INPUT_PULLUP); 
  pinMode(Input_Pin[5], INPUT_PULLUP); 
  
  SDCardInit();
  LoadFiles();  // lade Wave Dateien

  if (!digitalRead(WifiPin)) // Setup Modus
  {
    //digitalWrite(LedPin, HIGH);  
    Serial.print("AP (Zugangspunkt) einstellen…");
    WiFi.softAP(ssid, password);

    Serial.println("IP Adresse einstellen");
    IPAddress Ip(192, 168, 1, 1);
    IPAddress NMask(255, 255, 255, 0);
    WiFi.softAPConfig(Ip, Ip, NMask);
  
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP-IP-Adresse: ");
    Serial.println(IP);
  
    server.begin();  // Start Webserver
  }
  else
  {
    Serial.println("Kein AP-Normalmodus");  
  }

}


// ======== Loop =======================================
void loop()
{
  currentTime = millis();           // Aktuelle Zeit erneuern
  
  if (!digitalRead(WifiPin))        // Setup Modus
  {     
    Webpage();                      // Webseite laden
  }
  
  if (sbus_rx.Read())               // SBUS Daten vorhanden ?
  {
    SBUS_OK = true;                 // SBUS ist vorhanden  
    sbus_data = sbus_rx.data();     // SBUS Daten lesen 

    einkanalFunction();             // Einkanal auswerten

  }

  Config();                 // Einstellungen anwenden
  
  I2SAudio.FillBuffer();

  if (!Source_Start_Sound_OK[0] or !Source_Start_Sound_OK[1] or !Source_Start_Sound_OK[2] or !Source_Start_Sound_OK[3] or !Source_Start_Sound_OK[4]) // eine Einstellung nicht OK Augabe Fehler
  {
    Serial.print("Signal OK für Sound 1: ");
    Serial.print(Source_Start_Sound_OK[0]);
    Serial.print(" 2: "); 
    Serial.print(Source_Start_Sound_OK[1]);
    Serial.print(" 3: "); 
    Serial.print(Source_Start_Sound_OK[2]);
    Serial.print(" 4: "); 
    Serial.print(Source_Start_Sound_OK[3]);
    Serial.print(" 5: "); 
    Serial.print(Source_Start_Sound_OK[4]);
  }

  if (currentTime > 5000)  //Startup warte eben 5 Sekunden
  {

    ebenenFunction();

  if (Sound_on[1] or Sound_on_web[1])    // Spiele Sound 1
  {
    if (not Sound_play[1])
    {  
      Sound_play[1] = true;
      Sound1.LoadWavFile();
      Sound1.Volume = Volumen_Sound[1];
      Sound1.RepeatForever = bitRead(RepeatForever , 1);
      I2SAudio.Play(&Sound1);
      Sound_on_web[1] = false;   
    }
  }
  else
  {
    Sound1.RepeatForever = false;
    if (not Sound1.Playing and Sound_play[1])
    {
      Sound1.UnLoadWavFile();
      Sound_play[1] = false;      
    }    
  } 
  
  if (Sound_on[2] or Sound_on_web[2])    // Spiele Sound 2
  { 
    if (not Sound_play[2])
    {      
      Sound_play[2] = true;
      Sound2.LoadWavFile();
      Sound2.Volume = Volumen_Sound[2];
      Sound2.RepeatForever = bitRead(RepeatForever , 2);
      I2SAudio.Play(&Sound2);
      Sound_on_web[2] = false;
    }    
  }
  else
  {
    Sound2.RepeatForever = false;
    if (not Sound2.Playing and Sound_play[2])
    {
      Sound2.UnLoadWavFile();
      Sound_play[2] = false;      
    }    
  } 

  if (Sound_on[3] or Sound_on_web[3])    // Spiele Sound 3
  {
    if (not Sound_play[3])
    {  
      Sound_play[3] = true;   
      Sound3.LoadWavFile();
      Sound3.Volume = Volumen_Sound[3];
      Sound3.RepeatForever = bitRead(RepeatForever , 3);
      I2SAudio.Play(&Sound3);
      Sound_on_web[3] = false;      
    }
  }
  else
  {
    Sound3.RepeatForever = false;
    if (not Sound3.Playing and Sound_play[3])
    {
      Sound3.UnLoadWavFile();
      Sound_play[3] = false;
    }    
  } 

  if (Sound_on[4] or Sound_on_web[4])    // Spiele Sound 4
  {
    if (not Sound_play[4])
    {      
      Sound_play[4] = true;  
      Sound4.LoadWavFile();
      Sound4.Volume = Volumen_Sound[4];
      Sound4.RepeatForever = bitRead(RepeatForever , 4);
      I2SAudio.Play(&Sound4);
      Sound_on_web[4] = false;      
    }
  }
  else
  {
    Sound4.RepeatForever = false;
    if (not Sound4.Playing and Sound_play[4])
    {
      Sound4.UnLoadWavFile();
      Sound_play[4] = false;
    }    
  }   

  if (Sound_on[5] or Sound_on_web[5])    // Spiele Sound 5
  {
    if (not Sound_play[5])
    {      
      Sound_play[5] = true;  
      Sound5.LoadWavFile();
      Sound5.Volume = Volumen_Sound[5];
      Sound5.RepeatForever = bitRead(RepeatForever , 5);
      I2SAudio.Play(&Sound5);
      Sound_on_web[5] = false;      
    }
  }
  else
  {
    Sound5.RepeatForever = false;
    if (not Sound5.Playing and Sound_play[5])
    {
      Sound5.UnLoadWavFile();
      Sound_play[5] = false;
    }    
  }  

  if (Sound_on[6] or Sound_on_web[6])    // Spiele Sound 6
  {
    if (not Sound_play[6])
    {      
      Sound_play[6] = true;  
      Sound6.LoadWavFile();
      Sound6.Volume = Volumen_Sound[6];
      Sound6.RepeatForever = bitRead(RepeatForever , 6);
      I2SAudio.Play(&Sound6);
      Sound_on_web[6] = false;      
    }
  }
  else
  {
    Sound6.RepeatForever = false;
    if (not Sound6.Playing and Sound_play[6])
    {
      Sound6.UnLoadWavFile();
      Sound_play[6] = false;
    }    
  }  

  if (Sound_on[7] or Sound_on_web[7])    // Spiele Sound 7
  {
    if (not Sound_play[7])
    {      
      Sound_play[7] = true;  
      Sound7.LoadWavFile();
      Sound7.Volume = Volumen_Sound[7];
      Sound7.RepeatForever = bitRead(RepeatForever , 7);
      I2SAudio.Play(&Sound7);
      Sound_on_web[7] = false;      
    }
  }
  else
  {
    Sound7.RepeatForever = false;
    if (not Sound7.Playing and Sound_play[7])
    {
      Sound7.UnLoadWavFile();
      Sound_play[7] = false;
    }    
  }  

  if (Sound_on[8] or Sound_on_web[8])    // Spiele Sound 8
  {
    if (not Sound_play[8])
    {      
      Sound_play[8] = true;  
      Sound8.LoadWavFile();
      Sound8.Volume = Volumen_Sound[8];
      Sound8.RepeatForever = bitRead(RepeatForever , 8);
      I2SAudio.Play(&Sound8);
      Sound_on_web[8] = false;      
    }
  }
  else
  {
    Sound8.RepeatForever = false;
    if (not Sound8.Playing and Sound_play[8])
    {
      Sound8.UnLoadWavFile();
      Sound_play[8] = false;
    }    
  }    


  if (Sound_on[0] == true && !Sound_on_Motor_state && engine_on_toogle) {
    Sound_on_Motor = !Sound_on_Motor; // Toogle Motor Sound On
    Sound_on_Motor_state = true; 
  }
  else if (Sound_on[0] == false && Sound_on_Motor_state && engine_on_toogle) {
    Sound_on_Motor_state = false; 
  }
  else if (!engine_on_toogle) {
    Sound_on_Motor = Sound_on[0];
  }

	  // Motor ON OFF
  if (((Sound_on_Motor or Sound_on_web[0]) and not engine_break and not Sound_play[0]) or (throttle > 0 and engine_break and not Sound_play[0]) or (throttle > 0 and Sound_play[0])) //on
  {
	  Sound_play[0] = true;
	  shutdown_timer = millis();
	}  
  if (not Sound_on_Motor and not Sound_on_web[0]) //off
	{
	   Sound_play[0] = false;
	   engine_break = false;
	}
  if ((millis() - shutdown_timer) > (shutdowndelay * 1000) && Sound_play[0] && (shutdowndelay > 0)) //off über shutdown_timer
  {
	   Sound_play[0] = false;
	   engine_break = true;
  }	  

   switch (engine_State) {       // Motor Schrittkette
    case OFF:  
      if (Sound_play[0])
      {
        Sound_start.Volume = Volumen_Sound[0];
        I2SAudio.Play(&Sound_start);
        engine_State = STARTING;
      }
      break;
    case STARTING:
      if (not Sound_start.Playing)
      {  
        throttle = 0;
        last_throttle = 0;
        Sound_loop.RepeatForever = true;
        Sound_loop.Volume = Volumen_Sound[0];
        I2SAudio.Play(&Sound_loop);
        engine_State = RUNNING;
      }  
      break;
    case RUNNING:  
      if (not Sound_play[0])
      {
        throttle = 0;
        if (last_throttle == 0)
        {
          Sound_loop.RepeatForever = false;
          I2SAudio.StopAllSounds();
          engine_State = STOPPING;
        }
      }
      break;
    case STOPPING:
      if (not Sound_loop.Playing)
      {
        Sound_loop.RepeatForever = false;
        Sound_shut.Volume = Volumen_Sound[0];
        I2SAudio.Play(&Sound_shut);
        engine_State = OFF;
      }  
      break;  
  }

  // Geschwindigkeit Rampe und Sound Anpassung
  throttle = ramp_throttle(throttle);
  Sound_loop.Speed = floatMap(throttle, 0, 100, Min_Speed_Sound_0, Max_Speed_Sound_0) / 100;
  }
}

//------------------------------------------------------------------------------------------------------------------------

// ======== Intput  =======================================
void Config() {
  // Quelle für Sound Start 1-9 (Throttle; Sound1-8)
  int duration; //Impulslänge für PWM
  for(int x = 0; x <=8; x++)     // Throttle; Sound1-8
  {
    Source_Start_Sound_OK[x] = false; //Reset Ok
    if (Source_Start_Sound[x] < 20)  //SBUS L 
    {
      if (SBUS_OK) // SBUS OK?
      {
        Source_Start_Sound_OK[x] = true;
        if (sbus_data.ch[Source_Start_Sound[x]] < 624)
        {
          Sound_on[x] = true;
        }
        else
        {
          Sound_on[x] = false;
        }
      }  
    }
    else if (Source_Start_Sound[x] < 40)  //SBUS H 
    {
      if (SBUS_OK) // SBUS OK?
      {
        Source_Start_Sound_OK[x] = true;
        if (sbus_data.ch[Source_Start_Sound[x]-20] > 1424)
        {
          Sound_on[x] = true;
        }
        else
        {
          Sound_on[x] = false;
        }
      }    
    }
    else if (Source_Start_Sound[x] < 50)  //PWM Pin L
    {
      switch (Source_Start_Sound[x]-40) {
        case 0: attachInterrupt(digitalPinToInterrupt(Input_Pin[0]), &PWM_UP_0, RISING); break;
        case 1: attachInterrupt(digitalPinToInterrupt(Input_Pin[1]), &PWM_UP_1, RISING); break;
        case 2: attachInterrupt(digitalPinToInterrupt(Input_Pin[2]), &PWM_UP_2, RISING); break;
        case 3: attachInterrupt(digitalPinToInterrupt(Input_Pin[3]), &PWM_UP_3, RISING); break;
        case 4: attachInterrupt(digitalPinToInterrupt(Input_Pin[4]), &PWM_UP_4, RISING); break;
        case 5: attachInterrupt(digitalPinToInterrupt(Input_Pin[5]), &PWM_UP_5, RISING); break;
      }        
      sei();  // Interrupts freigeben
      duration = PWM_pulse_width[Source_Start_Sound[x]-40];
      if (duration < 2600) // PWM OK?  
      {
        Source_Start_Sound_OK[x] = true;      
        if (duration < 1250)
        {
          Sound_on[x] = true;
        }
        else
        {
          Sound_on[x] = false;
        }
      }
    }  
    else if (Source_Start_Sound[x] < 60)  //PWM Pin H
    {
      switch (Source_Start_Sound[x]-50) {
        case 0: attachInterrupt(digitalPinToInterrupt(Input_Pin[0]), &PWM_UP_0, RISING); break;
        case 1: attachInterrupt(digitalPinToInterrupt(Input_Pin[1]), &PWM_UP_1, RISING); break;
        case 2: attachInterrupt(digitalPinToInterrupt(Input_Pin[2]), &PWM_UP_2, RISING); break;
        case 3: attachInterrupt(digitalPinToInterrupt(Input_Pin[3]), &PWM_UP_3, RISING); break;
        case 4: attachInterrupt(digitalPinToInterrupt(Input_Pin[4]), &PWM_UP_4, RISING); break;
        case 5: attachInterrupt(digitalPinToInterrupt(Input_Pin[5]), &PWM_UP_5, RISING); break;
      }        
      sei();  // Interrupts freigeben
      duration = PWM_pulse_width[Source_Start_Sound[x]-50];
      if (duration < 2600) // PWM OK?  
      {
        Source_Start_Sound_OK[x] = true;  
        if (duration > 1750)
        {
          Sound_on[x] = true;
        }
        else
        {
          Sound_on[x] = false;
        }
      }
    }      
    else if (Source_Start_Sound[x] < 70)   //Pin Input
    {
      Source_Start_Sound_OK[x] = true;
      Sound_on[x] = not digitalRead(Input_Pin[Source_Start_Sound[x]-60]);
    }
    else if (Source_Start_Sound[x] < 80)      //Einkanal
    {
      if (SBUS_OK) // SBUS OK?
      {
        Source_Start_Sound_OK[x] = true;
        if (bitRead (einkanal_Data ,(Source_Start_Sound[x]-70))) // Abfrage für Ausgang
        {
          Sound_on[x] = true;
        }
        else
        {
          Sound_on[x] = false;
        }
      }  
    } 
    else if (Source_Start_Sound[x] < 110)      //Ebenen Umschaltung
    {
      if ((Source_Start_Sound[x]-80) == ebenenkanal_Data)
      {
        Sound_on[x] = true;
      }
      else
      {
        Sound_on[x] = false;
      }
    }
        else if (Source_Start_Sound[x] < 201)      //Dauer an
    {
      Sound_on[x] = true;
    }
    else // Nichts
    {
      Sound_on[x] = false;
    }    
  }
  // Quelle Speed Sound 1 (throttle)
  
  if (Source_Speed_Sound_0 < 20)  //SBUS Input 
  {
    throttle = map(sbus_data.ch[Source_Speed_Sound_0], 82, 1900, 0, 100);
  }
  else //PWM
  {
    switch (Source_Speed_Sound_0-20) {
      case 0: attachInterrupt(digitalPinToInterrupt(Input_Pin[0]), &PWM_UP_0, RISING); break;
      case 1: attachInterrupt(digitalPinToInterrupt(Input_Pin[1]), &PWM_UP_1, RISING); break;
      case 2: attachInterrupt(digitalPinToInterrupt(Input_Pin[2]), &PWM_UP_2, RISING); break;
      case 3: attachInterrupt(digitalPinToInterrupt(Input_Pin[3]), &PWM_UP_3, RISING); break;
      case 4: attachInterrupt(digitalPinToInterrupt(Input_Pin[4]), &PWM_UP_4, RISING); break;
      case 5: attachInterrupt(digitalPinToInterrupt(Input_Pin[5]), &PWM_UP_5, RISING); break;
    }        
    sei();  // enable interrupts
    duration = PWM_pulse_width[Source_Speed_Sound_0-20];
    throttle = map(duration, 1000, 2000, 0, 100);
  }

  if (throttle_mode) //vor und Zurück
  {
    if (throttle > 50 + throttle_dead_band) // Totband
    {
      throttle = map(throttle, 50 + throttle_dead_band, 100, 0, 100);
    }
    else if (throttle < 50 - throttle_dead_band) // Totband
    {
      throttle = map(throttle, 50 - throttle_dead_band, 0, 0, 100);
    }
    else
    {
      throttle = 0;
    }
  }  
  else
  {
    if (throttle > throttle_dead_band) // Totband
    {
      throttle = map(throttle, throttle_dead_band, 100, 0, 100);
    }
    else
    {
      throttle = 0;
    }	 
  }  
  //Serial.println(" ");
}

// ======== Float Map  =======================================
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ======== SD Card Init  =======================================
void SDCardInit()
{
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH); // SD-Karte auswählen, muss GPIO 5 verwenden (ESP32 )
  if (!SD.begin(SD_CS))
  {
    Serial.println("Fehler beim Lesen der SD-Karte!");
    //while (true)
      ; // Programm Ende
  }
}

// ======== LoadFiles  =======================================
void LoadFiles()
{
  Sound_loop.LoadWavFile();
  Sound_shut.LoadWavFile();
  Sound_start.LoadWavFile();
}

// ======== Rampe Throttle  =======================================
// Rampe need (currentTime as millis(); unsigned long previousTime_ramp as timesaver; float last_throttle as throttlesaver; float throttle_ramp as ramp per Second)
float ramp_throttle(float throttle_new)
{
     unsigned long RampTime = currentTime - previousTime_ramp;
     previousTime_ramp = currentTime;
   //  Serial.print("RampTime ms : ");
   //  Serial.println(RampTime);
     float throttle_ramp_f = throttle_ramp;
     float throttle_ramp_now = (throttle_ramp_f / 1000) * RampTime;
     if (throttle_new > last_throttle)          //throttel go up
    {
        if (last_throttle + throttle_ramp_now < throttle)
        {
          throttle_new = last_throttle + throttle_ramp_now;
        }
        else
        {
          throttle_new = throttle_new;
        }
    }
    else                                   //throttel go down
    {
        if (last_throttle - throttle_ramp_now > throttle)
        {
          throttle_new = last_throttle - throttle_ramp_now;
        }
        else
        {
          throttle_new = throttle_new;
        }
    }
    last_throttle = throttle_new;
    return throttle_new;
}

// ======== Einkanal  =======================================
void einkanalFunction() {
    einkanal_Data = sbus_data.ch[sbus_channel_einkanal];
  if (sbus_channel_einkanal_mode == true) {    
    if (einkanal_Data < 206) { einkanal_Data = 206;}
    if (einkanal_Data > 1837) { einkanal_Data= 1837;}
    einkanal_Data = einkanal_Data - 206 ;
    einkanal_Data = (einkanal_Data * 10);
    einkanal_Data = einkanal_Data + 20;  
    einkanal_Data = einkanal_Data  / 8;
    einkanal_Data = einkanal_Data  / 8;
  } else {
    einkanal_Data = einkanal_Data  / 8;
    einkanal_Data = einkanal_Data; 
  }
}

// ======== EbenenUmschaltung  =======================================
void ebenenFunction() {
  // Kanalwert lesen
  Source_Ebenen_Um_Kanal_wert = 1;
  Source_Ebenen_Kanal_wert = 0;
  if (Source_Ebenen_Um_Kanal < 20)  //SBUS Input 
  {
    Source_Ebenen_Um_Kanal_wert = map(sbus_data.ch[Source_Ebenen_Um_Kanal], 207, 1833, 0, 100);
  }
  else if (Source_Ebenen_Um_Kanal < 30) //PWM
  {
    switch (Source_Ebenen_Um_Kanal-20) {
      case 0: attachInterrupt(digitalPinToInterrupt(Input_Pin[0]), &PWM_UP_0, RISING); break;
      case 1: attachInterrupt(digitalPinToInterrupt(Input_Pin[1]), &PWM_UP_1, RISING); break;
      case 2: attachInterrupt(digitalPinToInterrupt(Input_Pin[2]), &PWM_UP_2, RISING); break;
      case 3: attachInterrupt(digitalPinToInterrupt(Input_Pin[3]), &PWM_UP_3, RISING); break;
      case 4: attachInterrupt(digitalPinToInterrupt(Input_Pin[4]), &PWM_UP_4, RISING); break;
      case 5: attachInterrupt(digitalPinToInterrupt(Input_Pin[5]), &PWM_UP_5, RISING); break;
    }        
    sei();  // enable interrupts
    Source_Ebenen_Um_Kanal_wert = map(PWM_pulse_width[Source_Ebenen_Um_Kanal], 938, 2063, 0, 100);
  }

  if (Source_Ebenen_Kanal < 20)  //SBUS Input 
  {
    Source_Ebenen_Kanal_wert = map(sbus_data.ch[Source_Ebenen_Kanal], 207, 1833, 0, 100);
  }
  else if (Source_Ebenen_Kanal < 30) //PWM
  {
    switch (Source_Ebenen_Kanal-20) {
      case 0: attachInterrupt(digitalPinToInterrupt(Input_Pin[0]), &PWM_UP_0, RISING); break;
      case 1: attachInterrupt(digitalPinToInterrupt(Input_Pin[1]), &PWM_UP_1, RISING); break;
      case 2: attachInterrupt(digitalPinToInterrupt(Input_Pin[2]), &PWM_UP_2, RISING); break;
      case 3: attachInterrupt(digitalPinToInterrupt(Input_Pin[3]), &PWM_UP_3, RISING); break;
      case 4: attachInterrupt(digitalPinToInterrupt(Input_Pin[4]), &PWM_UP_4, RISING); break;
      case 5: attachInterrupt(digitalPinToInterrupt(Input_Pin[5]), &PWM_UP_5, RISING); break;
    }        
    sei();  // enable interrupts
    Source_Ebenen_Kanal_wert = map(PWM_pulse_width[Source_Ebenen_Kanal], 938, 2063, 0, 100);
  }
    
  Source_Ebenen_Um_Kanal_wert = (Source_Ebenen_Um_Kanal_wert) / 33;     // Wert 0-2
  //Source_Ebenen_Kanal_wert = ((Source_Ebenen_Kanal_wert * 10) / 125);   // Wert 0-8

  if (Source_Ebenen_Kanal_wert < 6)  // Wert 0
  {
    ebenenkanal_Data = (Source_Ebenen_Um_Kanal_wert * 8) + 0;
  }
  else if (Source_Ebenen_Kanal_wert < 19)  // Wert 1
  {
    ebenenkanal_Data = (Source_Ebenen_Um_Kanal_wert * 8) + 1;
  }
  else if (Source_Ebenen_Kanal_wert < 31)  // Wert 2
  {
    ebenenkanal_Data = (Source_Ebenen_Um_Kanal_wert * 8) + 2;
  }
  else if (Source_Ebenen_Kanal_wert < 44)  // Wert 3
  {
    ebenenkanal_Data = (Source_Ebenen_Um_Kanal_wert * 8) + 3;
  }
  else if (Source_Ebenen_Kanal_wert < 56)  // Wert 4
  {
    ebenenkanal_Data = 99;
  }
  else if (Source_Ebenen_Kanal_wert < 69)  // Wert 5
  {
    ebenenkanal_Data = (Source_Ebenen_Um_Kanal_wert * 8) + 4;
  }
  else if (Source_Ebenen_Kanal_wert < 81)  // Wert 6
  {
    ebenenkanal_Data = (Source_Ebenen_Um_Kanal_wert * 8) + 5;
  }
  else if (Source_Ebenen_Kanal_wert < 94)  // Wert 7
  {
    ebenenkanal_Data = (Source_Ebenen_Um_Kanal_wert * 8) + 6;
  }
  else if (Source_Ebenen_Kanal_wert < 106)  // Wert 8
  {
    ebenenkanal_Data = (Source_Ebenen_Um_Kanal_wert * 8) + 7;
  }
      // Serial.print(" Ebenen UM Wert: ");
      // Serial.print(Source_Ebenen_Um_Kanal_wert);
      // Serial.print(" Ebenen Wert: ");
      // Serial.print(Source_Ebenen_Kanal_wert);
      // Serial.print(" Ebenen Data: ");
      // Serial.print(ebenenkanal_Data);
}

// ======== Werkseinstellungen  =======================================
void Reset_all() {
  Source_Start_Sound[0] = 999;
  Source_Start_Sound[1] = 999;
  Source_Start_Sound[2] = 999;
  Source_Start_Sound[3] = 999;
  Source_Start_Sound[4] = 999;
  Source_Start_Sound[5] = 999;
  Source_Start_Sound[6] = 999;
  Source_Start_Sound[7] = 999;
  Source_Start_Sound[8] = 999;

  Source_Speed_Sound_0 = 999;
  Min_Speed_Sound_0 = 100;
  Max_Speed_Sound_0 = 300;

  Volumen_Sound[0] = 100;
  Volumen_Sound[1] = 100;
  Volumen_Sound[2] = 100;
  Volumen_Sound[3] = 100;
  Volumen_Sound[4] = 100;
  Volumen_Sound[5] = 100;
  Volumen_Sound[6] = 100;
  Volumen_Sound[7] = 100;
  Volumen_Sound[8] = 100;

  RepeatForever = 0;

  shutdowndelay = 0;
  engine_on_toogle = 0;

  sbus_channel_einkanal = 999;
  Source_Ebenen_Um_Kanal = 999;
  Source_Ebenen_Kanal = 999;

  throttle_ramp = 20;
  throttle_dead_band = 10; 
  
}
// ======== Voreinstellungen SBUS  =======================================
void set_sbus() {
  Source_Start_Sound[0] = 10;
  Source_Start_Sound[1] = 11;
  Source_Start_Sound[2] = 12;
  Source_Start_Sound[3] = 13;
  Source_Start_Sound[4] = 14;
  Source_Start_Sound[5] = 999;
  Source_Start_Sound[6] = 999;
  Source_Start_Sound[7] = 999;
  Source_Start_Sound[8] = 999;

  Source_Speed_Sound_0 = 1;
  Min_Speed_Sound_0 = 100;
  Max_Speed_Sound_0 = 300;

  Volumen_Sound[0] = 100;
  Volumen_Sound[1] = 100;
  Volumen_Sound[2] = 100;
  Volumen_Sound[3] = 100;
  Volumen_Sound[4] = 100;
  Volumen_Sound[5] = 100;
  Volumen_Sound[6] = 100;
  Volumen_Sound[7] = 100;
  Volumen_Sound[8] = 100;

  RepeatForever = 0;

  shutdowndelay = 0;
  engine_on_toogle = 0;

  sbus_channel_einkanal = 999;
  Source_Ebenen_Um_Kanal = 999;
  Source_Ebenen_Kanal = 999;

  throttle_ramp = 20;
  throttle_dead_band = 10; 
}

// ======== Voreinstellungen PWM  =======================================
void set_pwm() {
  Source_Start_Sound[0] = 41;
  Source_Start_Sound[1] = 42;
  Source_Start_Sound[2] = 43;
  Source_Start_Sound[3] = 44;
  Source_Start_Sound[4] = 45;
  Source_Start_Sound[5] = 999;
  Source_Start_Sound[6] = 999;
  Source_Start_Sound[7] = 999;
  Source_Start_Sound[8] = 999;

  Source_Speed_Sound_0 = 20;
  Min_Speed_Sound_0 = 100;
  Max_Speed_Sound_0 = 300;

  Volumen_Sound[0] = 100;
  Volumen_Sound[1] = 100;
  Volumen_Sound[2] = 100;
  Volumen_Sound[3] = 100;
  Volumen_Sound[4] = 100;
  Volumen_Sound[5] = 100;
  Volumen_Sound[6] = 100;
  Volumen_Sound[7] = 100;
  Volumen_Sound[8] = 100;

  RepeatForever = 0;

  shutdowndelay = 0;
  engine_on_toogle = 0;

  sbus_channel_einkanal = 999;
  Source_Ebenen_Um_Kanal = 999;
  Source_Ebenen_Kanal = 999;

  throttle_ramp = 20;
  throttle_dead_band = 10;  
}

// ======== Voreinstellungen Pin  =======================================
void set_pin() {
  Source_Start_Sound[0] = 61;
  Source_Start_Sound[1] = 62;
  Source_Start_Sound[2] = 63;
  Source_Start_Sound[3] = 64;
  Source_Start_Sound[5] = 999;
  Source_Start_Sound[6] = 999;
  Source_Start_Sound[7] = 999;
  Source_Start_Sound[8] = 999;

  Source_Speed_Sound_0 = 20;
  Min_Speed_Sound_0 = 100;
  Max_Speed_Sound_0 = 300;
                     
  Volumen_Sound[0] = 100;
  Volumen_Sound[1] = 100;
  Volumen_Sound[2] = 100;
  Volumen_Sound[3] = 100;
  Volumen_Sound[4] = 100;
  Volumen_Sound[5] = 100;
  Volumen_Sound[6] = 100;
  Volumen_Sound[7] = 100;
  Volumen_Sound[8] = 100;

  RepeatForever = 0;

  shutdowndelay = 0;
  engine_on_toogle = 0;

  sbus_channel_einkanal = 999;
  Source_Ebenen_Um_Kanal = 999;
  Source_Ebenen_Kanal = 999;

  throttle_ramp = 20;
  throttle_dead_band = 10;        
}

// ======== EEprom  =======================================
void EEprom_Load() {
  Source_Speed_Sound_0  = EEPROM.readInt(adr_eprom_Source_Speed_Sound_0);
  Source_Start_Sound[0] = EEPROM.readInt(adr_eprom_Source_Start_Sound_0);
  Source_Start_Sound[1] = EEPROM.readInt(adr_eprom_Source_Start_Sound_1);
  Source_Start_Sound[2] = EEPROM.readInt(adr_eprom_Source_Start_Sound_2);
  Source_Start_Sound[3] = EEPROM.readInt(adr_eprom_Source_Start_Sound_3);
  Source_Start_Sound[4] = EEPROM.readInt(adr_eprom_Source_Start_Sound_4);
  Source_Start_Sound[5] = EEPROM.readInt(adr_eprom_Source_Start_Sound_5);
  Source_Start_Sound[6] = EEPROM.readInt(adr_eprom_Source_Start_Sound_6);
  Source_Start_Sound[7] = EEPROM.readInt(adr_eprom_Source_Start_Sound_7);
  Source_Start_Sound[8] = EEPROM.readInt(adr_eprom_Source_Start_Sound_8);
  throttle_mode         = EEPROM.readInt(adr_eprom_Source_throttle_mode);
  Min_Speed_Sound_0     = EEPROM.readInt(adr_eprom_Min_Speed_Sound_0);
  Max_Speed_Sound_0     = EEPROM.readInt(adr_eprom_Max_Speed_Sound_0);
  Volumen_Sound[0]      = EEPROM.readInt(adr_eprom_Volumen_Sound_0);
  Volumen_Sound[1]      = EEPROM.readInt(adr_eprom_Volumen_Sound_1);
  Volumen_Sound[2]      = EEPROM.readInt(adr_eprom_Volumen_Sound_2);
  Volumen_Sound[3]      = EEPROM.readInt(adr_eprom_Volumen_Sound_3);
  Volumen_Sound[4]      = EEPROM.readInt(adr_eprom_Volumen_Sound_4);
  Volumen_Sound[5]      = EEPROM.readInt(adr_eprom_Volumen_Sound_5);
  Volumen_Sound[6]      = EEPROM.readInt(adr_eprom_Volumen_Sound_6);
  Volumen_Sound[7]      = EEPROM.readInt(adr_eprom_Volumen_Sound_7);
  Volumen_Sound[8]      = EEPROM.readInt(adr_eprom_Volumen_Sound_8);
  RepeatForever               = EEPROM.readInt(adr_eprom_RepeatForever); 
  sbus_channel_einkanal       = EEPROM.readInt(adr_eprom_sbus_channel_einkanal);
  sbus_channel_einkanal_mode  = EEPROM.readInt(adr_eprom_sbus_channel_einkanal_mode);
  shutdowndelay               = EEPROM.readInt(adr_eprom_shutdowndelay);
  engine_on_toogle            = EEPROM.readInt(adr_eprom_engine_on_toogle);
  Source_Ebenen_Um_Kanal      = EEPROM.readInt(adr_eprom_Source_Ebenen_Um_Kanal); 
  Source_Ebenen_Kanal         = EEPROM.readInt(adr_eprom_Source_Ebenen_Kanal); 
  throttle_ramp               = EEPROM.readInt(adr_eprom_throttle_ramp);             
  throttle_dead_band          = EEPROM.readInt(adr_eprom_throttle_dead_band); 
  Hardware_Config             = EEPROM.readInt(adr_eprom_Hardware_Config);
  Serial.println("EEPROM gelesen.");
}
void EEprom_Save() {
  EEPROM.writeInt(adr_eprom_Source_Speed_Sound_0, Source_Speed_Sound_0 );
  EEPROM.writeInt(adr_eprom_Source_Start_Sound_0, Source_Start_Sound[0]);
  EEPROM.writeInt(adr_eprom_Source_Start_Sound_1, Source_Start_Sound[1]);
  EEPROM.writeInt(adr_eprom_Source_Start_Sound_2, Source_Start_Sound[2]);
  EEPROM.writeInt(adr_eprom_Source_Start_Sound_3, Source_Start_Sound[3]);
  EEPROM.writeInt(adr_eprom_Source_Start_Sound_4, Source_Start_Sound[4]);
  EEPROM.writeInt(adr_eprom_Source_Start_Sound_5, Source_Start_Sound[5]);
  EEPROM.writeInt(adr_eprom_Source_Start_Sound_6, Source_Start_Sound[6]);
  EEPROM.writeInt(adr_eprom_Source_Start_Sound_7, Source_Start_Sound[7]);
  EEPROM.writeInt(adr_eprom_Source_Start_Sound_8, Source_Start_Sound[8]);
  EEPROM.writeInt(adr_eprom_Source_throttle_mode, throttle_mode);
  EEPROM.writeInt(adr_eprom_Min_Speed_Sound_0, Min_Speed_Sound_0);
  EEPROM.writeInt(adr_eprom_Max_Speed_Sound_0, Max_Speed_Sound_0);
  EEPROM.writeInt(adr_eprom_Volumen_Sound_0, Volumen_Sound[0]);
  EEPROM.writeInt(adr_eprom_Volumen_Sound_1, Volumen_Sound[1]);
  EEPROM.writeInt(adr_eprom_Volumen_Sound_2, Volumen_Sound[2]);
  EEPROM.writeInt(adr_eprom_Volumen_Sound_3, Volumen_Sound[3]);
  EEPROM.writeInt(adr_eprom_Volumen_Sound_4, Volumen_Sound[4]);
  EEPROM.writeInt(adr_eprom_Volumen_Sound_5, Volumen_Sound[5]);
  EEPROM.writeInt(adr_eprom_Volumen_Sound_6, Volumen_Sound[6]);
  EEPROM.writeInt(adr_eprom_Volumen_Sound_7, Volumen_Sound[7]);
  EEPROM.writeInt(adr_eprom_Volumen_Sound_8, Volumen_Sound[8]);
  EEPROM.writeInt(adr_eprom_RepeatForever, RepeatForever);
  EEPROM.writeInt(adr_eprom_sbus_channel_einkanal, sbus_channel_einkanal);
  EEPROM.writeInt(adr_eprom_sbus_channel_einkanal_mode, sbus_channel_einkanal_mode);
  EEPROM.writeInt(adr_eprom_shutdowndelay, shutdowndelay);
  EEPROM.writeInt(adr_eprom_engine_on_toogle, engine_on_toogle);
  EEPROM.writeInt(adr_eprom_Source_Ebenen_Um_Kanal, Source_Ebenen_Um_Kanal);
  EEPROM.writeInt(adr_eprom_Source_Ebenen_Kanal, Source_Ebenen_Kanal);
  EEPROM.writeInt(adr_eprom_throttle_ramp, throttle_ramp);            
  EEPROM.writeInt(adr_eprom_throttle_dead_band, throttle_dead_band);
  EEPROM.writeInt(adr_eprom_Hardware_Config, Hardware_Config);
  EEPROM.commit();
  Serial.println("EEPROM gespeichert.");
}

// ======== Webseite  =======================================  
void Webpage()
{  
  WiFiClient client = server.available();   // Listen for incoming clients
  
  if (client) {                             // If a new client connects,
    currentTime = millis();                 // kann weg ------------------------------------------------------------------------
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP-Header fangen immer mit einem Response-Code an (z.B. HTTP/1.1 200 OK)
            // gefolgt vom Content-Type damit der Client weiss was folgt, gefolgt von einer Leerzeile:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Webseiten Eingaben abfragen

            if(header.indexOf("GET /?mode=true")>=0) {  // Abfrage der Checkbox
              throttle_mode = true;
              Serial.println("Throttle_Mode 1");
            }  
            else if (header.indexOf("GET /?mode=false")>=0) { 
              throttle_mode = false;  
              Serial.println("Throttle_Mode 0");
            }

            if (header.indexOf("GET /Sound1/on") >= 0) 
            {
               Sound_on_web[1] = true;
            }

            if (header.indexOf("GET /Sound2/on") >= 0) 
            {
              Sound_on_web[2] = true;
            }
            
            if (header.indexOf("GET /Sound3/on") >= 0) 
            {
              Sound_on_web[3] = true;
            }

            if (header.indexOf("GET /Sound4/on") >= 0) 
            {
              Sound_on_web[4] = true;
            }

            if (header.indexOf("GET /save") >= 0) {  // Abfrage Button Save
              EEprom_Save();
            } 

            if (header.indexOf("GET /reset") >= 0) {  // Abfrage Button reset
              Reset_all();
            } 

            if (header.indexOf("GET /setsbus") >= 0) {  // Abfrage Button Voreinstellungen SBUS
              set_sbus();
            } 

            if (header.indexOf("GET /setpwm") >= 0) {  // Abfrage Button Voreinstellungen PWM
              set_pwm();
            } 

            if (header.indexOf("GET /setpin") >= 0) {  // Abfrage Button Voreinstellungen PIN
              set_pin();
            }                                     

            if (header.indexOf("GET /next") >= 0) {  // Abfrage Button Next
              Menu ++;
            } 

            if (header.indexOf("GET /back") >= 0) {  // Abfrage Button Back
              Menu --;
            }            

            if(header.indexOf("GET /?VolumenSound=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              Volumen_Sound[Menu] = (valueString.toInt());
            }

            if(header.indexOf("GET /?Drehzahlmin=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              Min_Speed_Sound_0 = (valueString.toInt());
            }

            if(header.indexOf("GET /?Drehzahlmax=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              Max_Speed_Sound_0 = (valueString.toInt());
            }

            if(header.indexOf("GET /?shutdowndelay=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              shutdowndelay = (valueString.toInt());
            }            

            if(header.indexOf("GET /?MotorMODE=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              throttle_mode = (valueString.toInt());
            }

            if(header.indexOf("GET /?MotorOnToggle=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              engine_on_toogle = (valueString.toInt());
            }

            if(header.indexOf("GET /?SoundON=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              Source_Start_Sound[Menu] = (valueString.toInt());
            }

            if(header.indexOf("GET /?SoundSPEED=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              Source_Speed_Sound_0 = (valueString.toInt());
            }

            if(header.indexOf("GET /?SbuschannelEinkanal=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              sbus_channel_einkanal = (valueString.toInt());
            }
            if(header.indexOf("GET /?KompatibilitaetsMode=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              sbus_channel_einkanal_mode = (valueString.toInt());
            }  

            if(header.indexOf("GET /?EbenenUmschaltungKanal=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              Source_Ebenen_Um_Kanal = (valueString.toInt());
            }    
            if(header.indexOf("GET /?EbenenKanal=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              Source_Ebenen_Kanal = (valueString.toInt());
            }       

            if(header.indexOf("GET /?LoopMODE=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              bitWrite(RepeatForever, Menu, (valueString.toInt()));
            } 
            
            if(header.indexOf("GET /?ThrottleRamp=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              throttle_ramp = (valueString.toInt());
            } 

            if(header.indexOf("GET /?ThrottleDeadBand=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              throttle_dead_band = (valueString.toInt());
            }            

            if(header.indexOf("GET /?HardwareConfig=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);
              Hardware_Config = (valueString.toInt());
            }    

            //Werte begrenzen

            Menu = constrain(Menu, 0, 10); // Begrenzt den Bereich Menu auf 1 bis 10.
            
            //HTML Seite angezeigen:
            client.println("<!DOCTYPE html><html>");
            //client.println("<meta http-equiv='refresh' content='5'>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS zum Stylen der Ein/Aus-Schaltflächen
            // Fühlen Sie sich frei, die Attribute für Hintergrundfarbe und Schriftgröße nach Ihren Wünschen zu ändern
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { border: 4px solid black; color: white; padding: 10px 40px; width: 100%; border-radius: 10px;");
            client.println("text-decoration: none; font-size: 20px; margin: 2px; cursor: pointer;}");
            client.println(".slider { -webkit-appearance: none; width: 80%; height: 30px; background: #d3d3d3; outline: none; opacity: 0.7; -webkit-transition: .2s; transition: opacity .2s; }");
            client.println(".slider::-moz-range-thumb { width: 30px; height: 30px; background: #04AA6D; cursor: pointer; }");
            client.println(".buttonA {outline: none; cursor: pointer; padding: 14px; margin: 10px; width: 90%; font-family: Verdana, Helvetica, sans-serif; font-size: 20px; background-color: #2222; color: #111; border: 4px solid black; border-radius: 10px; text-align: center;}");
            client.println(".text1 {font-family: Verdana, Helvetica, sans-serif; font-size: 25px; color: #111; text-align: center; margin-top: 0px; margin-bottom: 0px; }");
            client.println(".text2 {font-family: Verdana, Helvetica, sans-serif; font-size: 20px; color: #111; text-align: center; margin-top: 0px; margin-bottom: 0px; }");
            client.println(".text3 {font-family: Verdana, Helvetica, sans-serif; font-size: 15px; color: #111; text-align: center; margin-top: 0px; margin-bottom: 0px; }");
            client.println(".text4 {font-family: Verdana, Helvetica, sans-serif; font-size: 10px; color: #111; text-align: center; margin-top: 0px; margin-bottom: 0px; }");

            client.println(".center {text-align: center; }");
            client.println(".left {text-align: left; }");
            client.println(".right {text-align: right; }");
            
          
            client.println(".button1 {background-color: #4CAF50;}");
            client.println(".button2 {background-color: #ff0000;}");
            client.println(".button3 {background-color: #4CAF50; float: left; width: 45%;}");
            client.println(".button4 {background-color: #ff0000; float: right; width: 45%;}");
            client.println(".button5 {background-color: #ffe453; float: left; width: 45%;}");
            client.println(".button6 {background-color: #ffe453; float: center; width: 45%;}");
            client.println(".button7 {background-color: #ffe453; float: right; width: 45%;}");
            client.println(".disabled {opacity: 0.4; cursor: not-alloed;}");
            client.println(".textbox {font-size: 25px; text-align: center;}");
            client.println("</style></head>");    
                     
            // Webseiten-Überschrift
            client.println("<body>");
            client.println("<p class=\"text1\" ><b>ESP32 RC-Sound</b></p>"); 
            client.println("<p class=\"text3\" >Version : " + String(Version) + "</p>");

            switch (Menu) {
              case 0:

            client.println("<p class=\"text2\" ><b>Motor Einstellung</b></p>"); 
            
            client.println("<p><a href=\"/back\"><button class=\"button button3\">Back</button></a>");
            client.println("<a href=\"/next\"><button class=\"button button4\">Next</button></a></p>");

            client.println("<br />");
            client.println("<br />");
            client.println("<br />");
            client.println("<br />");            
        
            client.println("<p class=\"text2\" >Motor Mode</p>");  

            // Motor Mode  
            client.println("<p><select id=\"MotorMODE\" class=\"buttonA\" onchange=\"setmotormode()\">");
      	    client.println("<option value=\"0\">Eine Richtung</option>");
            client.println("<option value=\"1\">Zwei Richtungen</option>");
            client.println("</select><br></p>");

            client.println("<script> function setmotormode() { ");
            client.println("var sel = document.getElementById(\"MotorMODE\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?MotorMODE=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(throttle_mode, DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"MotorMODE\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");                 
        
            client.println("<p class=\"text2\" >Motor EIN Modus</p>");  

            // Motor Toggle On  
            client.println("<p><select id=\"MotorOnToggle\" class=\"buttonA\" onchange=\"setmotorontoggle()\">");
      	    client.println("<option value=\"0\">Normal</option>");
            client.println("<option value=\"1\">Tippbetrieb</option>");
            client.println("</select><br></p>");

            client.println("<script> function setmotorontoggle() { ");
            client.println("var sel = document.getElementById(\"MotorOnToggle\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?MotorOnToggle=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(engine_on_toogle, DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"MotorOnToggle\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");                      


            client.println("<p class=\"text2\" >Quelle Einschalten Motor</p>");

            // Quelle Einschalten Motor
	          client.println("<p><select id=\"SoundON\" class=\"buttonA\" onchange=\"setsoundon()\">");
            client.println("<optgroup label=\"SBUS Kanal Low\">");
      	    client.println("<option value=\"0\">SBUS Kanal Low 01</option>");
            client.println("<option value=\"1\">SBUS Kanal Low 02</option>");
            client.println("<option value=\"2\">SBUS Kanal Low 03</option>");
            client.println("<option value=\"3\">SBUS Kanal Low 04</option>");
            client.println("<option value=\"4\">SBUS Kanal Low 05</option>");
            client.println("<option value=\"5\">SBUS Kanal Low 06</option>");
            client.println("<option value=\"6\">SBUS Kanal Low 07</option>");
            client.println("<option value=\"7\">SBUS Kanal Low 08</option>");
            client.println("<option value=\"8\">SBUS Kanal Low 09</option>");
            client.println("<option value=\"9\">SBUS Kanal Low 10</option>");
            client.println("<option value=\"10\">SBUS Kanal Low 11</option>");
            client.println("<option value=\"11\">SBUS Kanal Low 12</option>");
            client.println("<option value=\"12\">SBUS Kanal Low 13</option>");
            client.println("<option value=\"13\">SBUS Kanal Low 14</option>");
            client.println("<option value=\"14\">SBUS Kanal Low 15</option>");
            client.println("<option value=\"15\">SBUS Kanal Low 16</option>");
            client.println("</optgroup>");
            client.println("<optgroup label=\"SBUS Kanal High\">");
      	    client.println("<option value=\"20\">SBUS Kanal High 01</option>"); 
            client.println("<option value=\"21\">SBUS Kanal High 02</option>"); 
            client.println("<option value=\"22\">SBUS Kanal High 03</option>"); 
            client.println("<option value=\"23\">SBUS Kanal High 04</option>"); 
            client.println("<option value=\"24\">SBUS Kanal High 05</option>"); 
            client.println("<option value=\"25\">SBUS Kanal High 06</option>"); 
            client.println("<option value=\"26\">SBUS Kanal High 07</option>"); 
            client.println("<option value=\"27\">SBUS Kanal High 08</option>"); 
            client.println("<option value=\"28\">SBUS Kanal High 09</option>"); 
            client.println("<option value=\"29\">SBUS Kanal High 10</option>"); 
            client.println("<option value=\"30\">SBUS Kanal High 11</option>"); 
            client.println("<option value=\"31\">SBUS Kanal High 12</option>"); 
            client.println("<option value=\"32\">SBUS Kanal High 13</option>"); 
            client.println("<option value=\"33\">SBUS Kanal High 14</option>"); 
            client.println("<option value=\"34\">SBUS Kanal High 15</option>"); 
            client.println("<option value=\"35\">SBUS Kanal High 16</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"PWM Pin Low\">");
            client.println("<option value=\"40\">PWM Pin Low 01</option>"); 
            client.println("<option value=\"41\">PWM Pin Low 02</option>"); 
            client.println("<option value=\"42\">PWM Pin Low 03</option>"); 
            client.println("<option value=\"43\">PWM Pin Low 04</option>"); 
            client.println("<option value=\"44\">PWM Pin Low 05</option>"); 
            client.println("<option value=\"45\">PWM Pin Low 06</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"PWM Pin High\">");
            client.println("<option value=\"50\">PWM Pin High 01</option>"); 
            client.println("<option value=\"51\">PWM Pin High 02</option>"); 
            client.println("<option value=\"52\">PWM Pin High 03</option>"); 
            client.println("<option value=\"53\">PWM Pin High 04</option>"); 
            client.println("<option value=\"54\">PWM Pin High 05</option>"); 
            client.println("<option value=\"55\">PWM Pin High 06</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"Eingang Pin\">");
            client.println("<option value=\"60\">Eingang Pin 01</option>"); 
            client.println("<option value=\"61\">Eingang Pin 02</option>"); 
            client.println("<option value=\"62\">Eingang Pin 03</option>"); 
            client.println("<option value=\"63\">Eingang Pin 04</option>"); 
            client.println("<option value=\"64\">Eingang Pin 05</option>"); 
            client.println("<option value=\"65\">Eingang Pin 06</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"Einkanal\">");
            client.println("<option value=\"70\">Einkanal 01</option>"); 
            client.println("<option value=\"71\">Einkanal 02</option>"); 
            client.println("<option value=\"72\">Einkanal 03</option>"); 
            client.println("<option value=\"73\">Einkanal 04</option>"); 
            client.println("<option value=\"74\">Einkanal 05</option>"); 
            client.println("<option value=\"75\">Einkanal 06</option>"); 
            client.println("<option value=\"76\">Einkanal 07</option>"); 
            client.println("<option value=\"77\">Einkanal 08</option>");                         
            client.println("</optgroup>");   
            client.println("<optgroup label=\"Optionen\">");
            client.println("<option value=\"200\">Dauerbetrieb an</option>"); 
            client.println("<option value=\"999\">Deaktiviert</option>");                            
            client.println("</optgroup>");                     
            client.println("</select><br></p>");


            client.println("<p class=\"text2\" >Quelle Motorspeed Motor</p>");

            // Quelle Motorspeed Motor
	          client.println("<p><select id=\"SoundSPEED\" class=\"buttonA\" onchange=\"setsoundspeed()\">");
            client.println("<optgroup label=\"SBUS Kanal\">");
      	    client.println("<option value=\"0\">SBUS Kanal 01</option>");
            client.println("<option value=\"1\">SBUS Kanal 02</option>");
            client.println("<option value=\"2\">SBUS Kanal 03</option>");
            client.println("<option value=\"3\">SBUS Kanal 04</option>");
            client.println("<option value=\"4\">SBUS Kanal 05</option>");
            client.println("<option value=\"5\">SBUS Kanal 06</option>");
            client.println("<option value=\"6\">SBUS Kanal 07</option>");
            client.println("<option value=\"7\">SBUS Kanal 08</option>");
            client.println("<option value=\"8\">SBUS Kanal 09</option>");
            client.println("<option value=\"9\">SBUS Kanal 10</option>");
            client.println("<option value=\"10\">SBUS Kanal 11</option>");
            client.println("<option value=\"11\">SBUS Kanal 12</option>");
            client.println("<option value=\"12\">SBUS Kanal 13</option>");
            client.println("<option value=\"13\">SBUS Kanal 14</option>");
            client.println("<option value=\"14\">SBUS Kanal 15</option>");
            client.println("<option value=\"15\">SBUS Kanal 16</option>");
            client.println("<optgroup label=\"PWM Pin\">");
            client.println("<option value=\"20\">PWM Pin 01</option>"); 
            client.println("<option value=\"21\">PWM Pin 02</option>"); 
            client.println("<option value=\"22\">PWM Pin 03</option>"); 
            client.println("<option value=\"23\">PWM Pin 04</option>"); 
            client.println("<option value=\"24\">PWM Pin 05</option>"); 
            client.println("<option value=\"25\">PWM Pin 06</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"Optionen\">"); 
            client.println("<option value=\"999\">Deaktiviert</option>");                            
            client.println("</optgroup>"); 
            client.println("</select><br></p>");

            client.println("<script> function setsoundspeed() { ");
            client.println("var sel = document.getElementById(\"SoundSPEED\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?SoundSPEED=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(Source_Speed_Sound_0, DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"SoundSPEED\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");            
             
             

            valueString = String(Volumen_Sound[Menu], DEC);
            client.println("<p class=\"text2\">Volumen : <span id=\"textVolumenSound\">" + valueString + "</span></p>");
            client.println("<p><input type=\"range\" min=\"0\" max=\"200\" step=\"5\" class=\"slider\" id=\"VolumenSound\" onchange=\"setVolumenSound(this.value)\" value=\"" + valueString + "\" /></p>");            

            valueString = String(Min_Speed_Sound_0, DEC);
            client.println("<p class=\"text2\">Drehzahl min : <span id=\"textDrehzahlmin\">" + valueString + " %</span></p>");
            client.println("<p><input type=\"range\" min=\"0\" max=\"200\" step=\"5\" class=\"slider\" id=\"Drehzahlmin\" onchange=\"setDrehzahlmin(this.value)\" value=\"" + valueString + "\" /></p>");            

            valueString = String(Max_Speed_Sound_0, DEC);
            client.println("<p class=\"text2\">Drehzahl max : <span id=\"textDrehzahlmax\">" + valueString + " %</span></p>");
            client.println("<p><input type=\"range\" min=\"100\" max=\"600\" step=\"5\" class=\"slider\" id=\"Drehzahlmax\" onchange=\"setDrehzahlmax(this.value)\" value=\"" + valueString + "\" /></p>");
            
            valueString = String(shutdowndelay, DEC);
            client.println("<p class=\"text2\">Motor aus Standgas in : <span id=\"textshutdowndelay\">" + valueString + " s</span></p>");
            client.println("<p><input type=\"range\" min=\"0\" max=\"60\" step=\"2\" class=\"slider\" id=\"Motor aus Verzögerung\" onchange=\"setshutdowndelay(this.value)\" value=\"" + valueString + "\" /></p>");

            valueString = String(throttle_ramp, DEC);
            client.println("<p class=\"text2\">Motor Rampe in : <span id=\"textthrottle_ramp\">" + valueString + " %/s</span></p>");
            client.println("<p><input type=\"range\" min=\"0\" max=\"50\" step=\"2\" class=\"slider\" id=\"Motor Rampe in\" onchange=\"setthrottle_ramp(this.value)\" value=\"" + valueString + "\" /></p>");

            valueString = String(throttle_dead_band, DEC);
            client.println("<p class=\"text2\">Standgas Totband in : <span id=\"textthrottle_dead_band\">" + valueString + " %</span></p>");
            client.println("<p><input type=\"range\" min=\"0\" max=\"50\" step=\"2\" class=\"slider\" id=\"Standgas Totband in\" onchange=\"setthrottle_dead_band(this.value)\" value=\"" + valueString + "\" /></p>");

            // Save Button
            client.println("<p><a href=\"/save\"><button class=\"button button2\">Save</button></a></p>");   

            client.println("<br />");
            client.println("<br />");
            client.println("<p class=\"text2\" >Voreinstellungen</p>");
            client.println("<br />");
            client.println("<p><a href=\"/setsbus\"><button class=\"button button2\">SBUS</button></a></p>");
            client.println("<br />");
            client.println("<p><a href=\"/setpwm\"><button class=\"button button2\">PWM</button></a></p>");
            client.println("<br />");
            client.println("<p><a href=\"/setpin\"><button class=\"button button2\">PIN</button></a></p>");
            client.println("<br />");

            client.println("<p><a href=\"/reset\"><button class=\"button button2\">Werkseinstellung</button></a></p>");

            break;

              case 1:
              case 2:
              case 3:
              case 4:
              case 5:
              case 6:
              case 7:
              case 8:
              
            client.println("<p class=\"text2\" ><b>Sound " + String(Menu) + " Einstellung</b></p>");  
            
            client.println("<p><a href=\"/back\"><button class=\"button button3\">Back</button></a>");
            client.println("<a href=\"/next\"><button class=\"button button4\">Next</button></a></p>");

            client.println("<br />");
            client.println("<br />");
            client.println("<br />");
            client.println("<br />");

            client.println("<p class=\"text2\" >Quelle Einschalten Sound</p>");


            // Quelle Einschalten Sound
	          client.println("<p><select id=\"SoundON\" class=\"buttonA\" onchange=\"setsoundon()\">");
            client.println("<optgroup label=\"SBUS Kanal Low\">");
      	    client.println("<option value=\"0\">SBUS Kanal Low 01</option>");
            client.println("<option value=\"1\">SBUS Kanal Low 02</option>");
            client.println("<option value=\"2\">SBUS Kanal Low 03</option>");
            client.println("<option value=\"3\">SBUS Kanal Low 04</option>");
            client.println("<option value=\"4\">SBUS Kanal Low 05</option>");
            client.println("<option value=\"5\">SBUS Kanal Low 06</option>");
            client.println("<option value=\"6\">SBUS Kanal Low 07</option>");
            client.println("<option value=\"7\">SBUS Kanal Low 08</option>");
            client.println("<option value=\"8\">SBUS Kanal Low 09</option>");
            client.println("<option value=\"9\">SBUS Kanal Low 10</option>");
            client.println("<option value=\"10\">SBUS Kanal Low 11</option>");
            client.println("<option value=\"11\">SBUS Kanal Low 12</option>");
            client.println("<option value=\"12\">SBUS Kanal Low 13</option>");
            client.println("<option value=\"13\">SBUS Kanal Low 14</option>");
            client.println("<option value=\"14\">SBUS Kanal Low 15</option>");
            client.println("<option value=\"15\">SBUS Kanal Low 16</option>");
            client.println("</optgroup>");
            client.println("<optgroup label=\"SBUS Kanal High\">");
      	    client.println("<option value=\"20\">SBUS Kanal High 01</option>"); 
            client.println("<option value=\"21\">SBUS Kanal High 02</option>"); 
            client.println("<option value=\"22\">SBUS Kanal High 03</option>"); 
            client.println("<option value=\"23\">SBUS Kanal High 04</option>"); 
            client.println("<option value=\"24\">SBUS Kanal High 05</option>"); 
            client.println("<option value=\"25\">SBUS Kanal High 06</option>"); 
            client.println("<option value=\"26\">SBUS Kanal High 07</option>"); 
            client.println("<option value=\"27\">SBUS Kanal High 08</option>"); 
            client.println("<option value=\"28\">SBUS Kanal High 09</option>"); 
            client.println("<option value=\"29\">SBUS Kanal High 10</option>"); 
            client.println("<option value=\"30\">SBUS Kanal High 11</option>"); 
            client.println("<option value=\"31\">SBUS Kanal High 12</option>"); 
            client.println("<option value=\"32\">SBUS Kanal High 13</option>"); 
            client.println("<option value=\"33\">SBUS Kanal High 14</option>"); 
            client.println("<option value=\"34\">SBUS Kanal High 15</option>"); 
            client.println("<option value=\"35\">SBUS Kanal High 16</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"PWM Pin Low\">");
            client.println("<option value=\"40\">PWM Pin Low 01</option>"); 
            client.println("<option value=\"41\">PWM Pin Low 02</option>"); 
            client.println("<option value=\"42\">PWM Pin Low 03</option>"); 
            client.println("<option value=\"43\">PWM Pin Low 04</option>"); 
            client.println("<option value=\"44\">PWM Pin Low 05</option>"); 
            client.println("<option value=\"45\">PWM Pin Low 06</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"PWM Pin High\">");
            client.println("<option value=\"50\">PWM Pin High 01</option>"); 
            client.println("<option value=\"51\">PWM Pin High 02</option>"); 
            client.println("<option value=\"52\">PWM Pin High 03</option>"); 
            client.println("<option value=\"53\">PWM Pin High 04</option>"); 
            client.println("<option value=\"54\">PWM Pin High 05</option>"); 
            client.println("<option value=\"55\">PWM Pin High 06</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"Eingang Pin\">");
            client.println("<option value=\"60\">Eingang Pin 01</option>"); 
            client.println("<option value=\"61\">Eingang Pin 02</option>"); 
            client.println("<option value=\"62\">Eingang Pin 03</option>"); 
            client.println("<option value=\"63\">Eingang Pin 04</option>"); 
            client.println("<option value=\"64\">Eingang Pin 05</option>"); 
            client.println("<option value=\"65\">Eingang Pin 06</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"Einkanal\">");
            client.println("<option value=\"70\">Einkanal 01</option>"); 
            client.println("<option value=\"71\">Einkanal 02</option>"); 
            client.println("<option value=\"72\">Einkanal 03</option>"); 
            client.println("<option value=\"73\">Einkanal 04</option>"); 
            client.println("<option value=\"74\">Einkanal 05</option>"); 
            client.println("<option value=\"75\">Einkanal 06</option>"); 
            client.println("<option value=\"76\">Einkanal 07</option>"); 
            client.println("<option value=\"77\">Einkanal 08</option>");                         
            client.println("</optgroup>");  
            client.println("<optgroup label=\"Ebenen Umschaltung\">");
            client.println("<option value=\"80\">Ebene 01 Kanal 01</option>"); 
            client.println("<option value=\"81\">Ebene 01 Kanal 02</option>"); 
            client.println("<option value=\"82\">Ebene 01 Kanal 03</option>"); 
            client.println("<option value=\"83\">Ebene 01 Kanal 04</option>"); 
            client.println("<option value=\"84\">Ebene 01 Kanal 05</option>"); 
            client.println("<option value=\"85\">Ebene 01 Kanal 06</option>"); 
            client.println("<option value=\"86\">Ebene 01 Kanal 07</option>"); 
            client.println("<option value=\"87\">Ebene 01 Kanal 08</option>");  
            client.println("<option value=\"88\">Ebene 02 Kanal 01</option>"); 
            client.println("<option value=\"89\">Ebene 02 Kanal 02</option>"); 
            client.println("<option value=\"90\">Ebene 02 Kanal 03</option>"); 
            client.println("<option value=\"91\">Ebene 02 Kanal 04</option>"); 
            client.println("<option value=\"92\">Ebene 02 Kanal 05</option>"); 
            client.println("<option value=\"93\">Ebene 02 Kanal 06</option>"); 
            client.println("<option value=\"94\">Ebene 02 Kanal 07</option>"); 
            client.println("<option value=\"95\">Ebene 02 Kanal 08</option>");
            client.println("<option value=\"96\">Ebene 03 Kanal 01</option>"); 
            client.println("<option value=\"97\">Ebene 03 Kanal 02</option>"); 
            client.println("<option value=\"98\">Ebene 03 Kanal 03</option>"); 
            client.println("<option value=\"99\">Ebene 03 Kanal 04</option>"); 
            client.println("<option value=\"100\">Ebene 03 Kanal 05</option>"); 
            client.println("<option value=\"101\">Ebene 03 Kanal 06</option>"); 
            client.println("<option value=\"102\">Ebene 03 Kanal 07</option>"); 
            client.println("<option value=\"103\">Ebene 03 Kanal 08</option>");                      
            client.println("</optgroup>");       
            client.println("<optgroup label=\"Optionen\">");
            client.println("<option value=\"200\">Dauerbetrieb an</option>"); 
            client.println("<option value=\"999\">Deaktiviert</option>");                            
            client.println("</optgroup>");        
            client.println("</select><br></p>");

            

            valueString = String(Volumen_Sound[Menu], DEC);
            client.println("<p class=\"text2\">Volumen : <span id=\"textVolumenSound\">" + valueString + "</span></p>");
            client.println("<p><input type=\"range\" min=\"0\" max=\"200\" step=\"5\" class=\"slider\" id=\"VolumenSound\" onchange=\"setVolumenSound(this.value)\" value=\"" + valueString + "\" /></p>");
            
            client.println("<p class=\"text2\" >Wiedergabe Sound</p>");
            // Loop Mode  
            client.println("<p><select id=\"LoopMODE\" class=\"buttonA\" onchange=\"setloopmode()\">");
      	    client.println("<option value=\"0\">Normal</option>");
            client.println("<option value=\"1\">Loop</option>");
            client.println("</select><br></p>");

            client.println("<script> function setloopmode() { ");
            client.println("var sel = document.getElementById(\"LoopMODE\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?LoopMODE=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(bitRead(RepeatForever , Menu), DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"LoopMODE\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");   

            // Save Button
            client.println("<p><a href=\"/save\"><button class=\"button button2\">Save</button></a></p>");
                          
            break;                      
              case 9:
              
            client.println("<p class=\"text2\" ><b>Einstellung</b></p>");  
            
            client.println("<p><a href=\"/back\"><button class=\"button button3\">Back</button></a>");
            client.println("<a href=\"/next\"><button class=\"button button4\">Next</button></a></p>");

            client.println("<br />");
            client.println("<br />");
            client.println("<br />");
            client.println("<br />");

            client.println("<p class=\"text2\" >Einkanal Einstellungen</p>");
            client.println("<p class=\"text2\" >Einkanal Kanal</p>");

            client.println("<p><select id=\"SbuschannelEinkanal\" class=\"buttonA\" onchange=\"setsbuschanneleinkanal()\">");
            client.println("<optgroup label=\"SBUS Kanal\">");
      	    client.println("<option value=\"0\">SBUS Kanal 01</option>");
            client.println("<option value=\"1\">SBUS Kanal 02</option>");
            client.println("<option value=\"2\">SBUS Kanal 03</option>");
            client.println("<option value=\"3\">SBUS Kanal 04</option>");
            client.println("<option value=\"4\">SBUS Kanal 05</option>");
            client.println("<option value=\"5\">SBUS Kanal 06</option>");
            client.println("<option value=\"6\">SBUS Kanal 07</option>");
            client.println("<option value=\"7\">SBUS Kanal 08</option>");
            client.println("<option value=\"8\">SBUS Kanal 09</option>");
            client.println("<option value=\"9\">SBUS Kanal 10</option>");
            client.println("<option value=\"10\">SBUS Kanal 11</option>");
            client.println("<option value=\"11\">SBUS Kanal 12</option>");
            client.println("<option value=\"12\">SBUS Kanal 13</option>");
            client.println("<option value=\"13\">SBUS Kanal 14</option>");
            client.println("<option value=\"14\">SBUS Kanal 15</option>");
            client.println("<option value=\"15\">SBUS Kanal 16</option>");
            client.println("</optgroup>");
            client.println("<optgroup label=\"Optionen\">");
            client.println("<option value=\"999\">Deaktiviert</option>");                            
            client.println("</optgroup>"); 
            client.println("</select><br></p>");

            client.println("<script> function setsbuschanneleinkanal() { ");
            client.println("var sel = document.getElementById(\"SbuschannelEinkanal\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?SbuschannelEinkanal=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(sbus_channel_einkanal, DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"SbuschannelEinkanal\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");

            client.println("<p class=\"text2\" >Kompatibilitaets-Mode</p>");
            client.println("<p><select id=\"KompatibilitaetsMode\" class=\"buttonA\" onchange=\"setkompatibilitaetsmode()\">");
      	    client.println("<option value=\"0\">Normal</option>");
            client.println("<option value=\"1\">Ein</option>");
            client.println("</select><br></p>");

            client.println("<script> function setkompatibilitaetsmode() { ");
            client.println("var sel = document.getElementById(\"KompatibilitaetsMode\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?KompatibilitaetsMode=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(sbus_channel_einkanal_mode, DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"KompatibilitaetsMode\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");

            client.println("<hr>");

            // Ebenen Umschaltung

            client.println("<p class=\"text2\" >Ebenen Umschaltung Einstellungen</p>");
            client.println("<p class=\"text2\" >Ebenen Umschaltung Kanal</p>");

            client.println("<p><select id=\"EbenenUmschaltungKanal\" class=\"buttonA\" onchange=\"setebenenumschaltungkanal()\">");
            client.println("<optgroup label=\"SBUS Kanal\">");
      	    client.println("<option value=\"0\">SBUS Kanal 01</option>");
            client.println("<option value=\"1\">SBUS Kanal 02</option>");
            client.println("<option value=\"2\">SBUS Kanal 03</option>");
            client.println("<option value=\"3\">SBUS Kanal 04</option>");
            client.println("<option value=\"4\">SBUS Kanal 05</option>");
            client.println("<option value=\"5\">SBUS Kanal 06</option>");
            client.println("<option value=\"6\">SBUS Kanal 07</option>");
            client.println("<option value=\"7\">SBUS Kanal 08</option>");
            client.println("<option value=\"8\">SBUS Kanal 09</option>");
            client.println("<option value=\"9\">SBUS Kanal 10</option>");
            client.println("<option value=\"10\">SBUS Kanal 11</option>");
            client.println("<option value=\"11\">SBUS Kanal 12</option>");
            client.println("<option value=\"12\">SBUS Kanal 13</option>");
            client.println("<option value=\"13\">SBUS Kanal 14</option>");
            client.println("<option value=\"14\">SBUS Kanal 15</option>");
            client.println("<option value=\"15\">SBUS Kanal 16</option>");
            client.println("<optgroup label=\"PWM Pin\">");
            client.println("<option value=\"20\">PWM Pin 01</option>"); 
            client.println("<option value=\"21\">PWM Pin 02</option>"); 
            client.println("<option value=\"22\">PWM Pin 03</option>"); 
            client.println("<option value=\"23\">PWM Pin 04</option>"); 
            client.println("<option value=\"24\">PWM Pin 05</option>"); 
            client.println("<option value=\"25\">PWM Pin 06</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"Optionen\">");
            client.println("<option value=\"999\">Deaktiviert</option>");                            
            client.println("</optgroup>"); 
            client.println("</select><br></p>");

            client.println("<script> function setebenenumschaltungkanal() { ");
            client.println("var sel = document.getElementById(\"EbenenUmschaltungKanal\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?EbenenUmschaltungKanal=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(Source_Ebenen_Um_Kanal, DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"EbenenUmschaltungKanal\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");

            client.println("<p class=\"text2\" >Ebenen Kanal</p>");

            client.println("<p><select id=\"EbenenKanal\" class=\"buttonA\" onchange=\"setebenenkanal()\">");
            client.println("<optgroup label=\"SBUS Kanal\">");
      	    client.println("<option value=\"0\">SBUS Kanal 01</option>");
            client.println("<option value=\"1\">SBUS Kanal 02</option>");
            client.println("<option value=\"2\">SBUS Kanal 03</option>");
            client.println("<option value=\"3\">SBUS Kanal 04</option>");
            client.println("<option value=\"4\">SBUS Kanal 05</option>");
            client.println("<option value=\"5\">SBUS Kanal 06</option>");
            client.println("<option value=\"6\">SBUS Kanal 07</option>");
            client.println("<option value=\"7\">SBUS Kanal 08</option>");
            client.println("<option value=\"8\">SBUS Kanal 09</option>");
            client.println("<option value=\"9\">SBUS Kanal 10</option>");
            client.println("<option value=\"10\">SBUS Kanal 11</option>");
            client.println("<option value=\"11\">SBUS Kanal 12</option>");
            client.println("<option value=\"12\">SBUS Kanal 13</option>");
            client.println("<option value=\"13\">SBUS Kanal 14</option>");
            client.println("<option value=\"14\">SBUS Kanal 15</option>");
            client.println("<option value=\"15\">SBUS Kanal 16</option>");
            client.println("<optgroup label=\"PWM Pin\">");
            client.println("<option value=\"20\">PWM Pin 01</option>"); 
            client.println("<option value=\"21\">PWM Pin 02</option>"); 
            client.println("<option value=\"22\">PWM Pin 03</option>"); 
            client.println("<option value=\"23\">PWM Pin 04</option>"); 
            client.println("<option value=\"24\">PWM Pin 05</option>"); 
            client.println("<option value=\"25\">PWM Pin 06</option>"); 
            client.println("</optgroup>");
            client.println("<optgroup label=\"Optionen\">");
            client.println("<option value=\"999\">Deaktiviert</option>");                            
            client.println("</optgroup>"); 
            client.println("</select><br></p>");

            client.println("<script> function setebenenkanal() { ");
            client.println("var sel = document.getElementById(\"EbenenKanal\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?EbenenKanal=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(Source_Ebenen_Kanal, DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"EbenenKanal\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");

            client.println("<hr>");

            // Ebenen Umschaltung

            client.println("<p class=\"text2\" >Hardware Einstellungen</p>");
            client.println("<p class=\"text2\" >Hardwareconfig</p>");

            client.println("<p><select id=\"HardwareConfig\" class=\"buttonA\" onchange=\"setHardwareConfig()\">");
      	    client.println("<option value=\"0\">V1</option>");
            client.println("<option value=\"1\">V2</option>");
            client.println("</select><br></p>");

            client.println("<script> function setHardwareConfig() { ");
            client.println("var sel = document.getElementById(\"HardwareConfig\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?HardwareConfig=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(Hardware_Config, DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"HardwareConfig\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");

            // Save Button
            client.println("<p><a href=\"/save\"><button class=\"button button2\">Save</button></a></p>");

            client.println("<br />");
            client.println("<br />");
              
            client.println("<p><a href=\"/Sound1/on\"><button class=\"button button2\">Sound1</button></a></p>");
            client.println("<p><a href=\"/Sound2/on\"><button class=\"button button2\">Sound2</button></a></p>");
            client.println("<p><a href=\"/Sound3/on\"><button class=\"button button2\">Sound3</button></a></p>");
            client.println("<p><a href=\"/Sound4/on\"><button class=\"button button2\">Sound4</button></a></p>");

            break;
              case 10:
              
            client.println("<p class=\"text2\" ><b>Debug Info</b></p>");  
            
            client.println("<p><a href=\"/back\"><button class=\"button button3\">Back</button></a>");
            client.println("<a href=\"/next\"><button class=\"button button4\">Next</button></a></p>");
            
            client.println("<br />");
            client.println("<br />");
            
            // SBUS Kanal 1 bis 8 Ausgeben
            client.println("<div class=\"left\" >");
            valueString = String(sbus_data.ch[0], DEC);
            client.println("<p> SUBS K1: " + valueString);        
            valueString = String(sbus_data.ch[1], DEC);
            client.println("<br> SUBS K2: " + valueString);
            valueString = String(sbus_data.ch[2], DEC);
            client.println("<br> SUBS K3: " + valueString);
            valueString = String(sbus_data.ch[3], DEC);
            client.println("<br> SUBS K4: " + valueString);
            valueString = String(sbus_data.ch[4], DEC);
            client.println("<br> SUBS K5: " + valueString);
            valueString = String(sbus_data.ch[5], DEC);
            client.println("<br> SUBS K6: " + valueString);
            valueString = String(sbus_data.ch[6], DEC);
            client.println("<br> SUBS K7: " + valueString);
            valueString = String(sbus_data.ch[7], DEC);
            client.println("<br> SUBS K8: " + valueString + "</p> <hr>");

            // PWM Pin 1 bis 6 Ausgeben
            valueString = String(PWM_pulse_width[0], DEC);
            client.println("<p> PWM PIN 1: " + valueString);
            valueString = String(PWM_pulse_width[1], DEC);
            client.println("<br> PWM PIN 2: " + valueString);
            valueString = String(PWM_pulse_width[2], DEC);
            client.println("<br> PWM PIN 3: " + valueString);
            valueString = String(PWM_pulse_width[3], DEC);
            client.println("<br> PWM PIN 4: " + valueString);
            valueString = String(PWM_pulse_width[4], DEC);
            client.println("<br> PWM PIN 5: " + valueString); 
            valueString = String(PWM_pulse_width[5], DEC);
            client.println("<br> PWM PIN 6: " + valueString + "</p> <hr>");

            // Konfig Sound 1 bis 9 Ausgeben
            valueString = String(Hardware_Config, DEC);
            client.println("<p> Konfig Hardware: " + valueString);  
            valueString = String(Source_Start_Sound[0], DEC);
            client.println("<br> Konfig Motor Start: " + valueString);        
            valueString = String(Source_Start_Sound[1], DEC);
            client.println("<br> Konfig Sound 1 Start: " + valueString);
            valueString = String(Source_Start_Sound[2], DEC);
            client.println("<br> Konfig Sound 2 Start: " + valueString);
            valueString = String(Source_Start_Sound[3], DEC);
            client.println("<br> Konfig Sound 3 Start: " + valueString);
            valueString = String(Source_Start_Sound[4], DEC);
            client.println("<br> Konfig Sound 4 Start: " + valueString);
            valueString = String(Source_Start_Sound[5], DEC);
            client.println("<br> Konfig Sound 5 Start: " + valueString);
            valueString = String(Source_Start_Sound[6], DEC);
            client.println("<br> Konfig Sound 6 Start: " + valueString);
            valueString = String(Source_Start_Sound[7], DEC);
            client.println("<br> Konfig Sound 7 Start: " + valueString);
            valueString = String(Source_Start_Sound[8], DEC);
            client.println("<br> Konfig Sound 8 Start: " + valueString);            
            valueString = String(Source_Speed_Sound_0, DEC);
            client.println("<br> Konfig Speed Sound 1: " + valueString); 
            valueString = String(throttle_ramp, DEC);
            client.println("<br> Konfig Gas Ranpe: " + valueString);
            valueString = String(throttle_dead_band, DEC);
            client.println("<br> Konfig Totband: " + valueString);                        
            valueString = String(Source_Ebenen_Um_Kanal, DEC);
            client.println("<br> Konfig Eben Kanal Umschalter: " + valueString);
            valueString = String(Source_Ebenen_Kanal, DEC);
            client.println("<br> Konfig Eben Kanal Wert: " + valueString + "</p> </div>");


            }

            client.println("<script> function setsoundon() { ");
            client.println("var sel = document.getElementById(\"SoundON\");");
            client.println("var opt = sel.options[sel.selectedIndex];");
            client.println("var val = opt.value;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?SoundON=\" + val + \"&\", true);");
            client.println("xhr.send(); } </script>");

            valueString = String(Source_Start_Sound[Menu], DEC);

            client.println("<script> ");
            client.println("var selectedOption =" + valueString + ";");
            client.println("var selectElement = document.getElementById(\"SoundON\");");
            client.println("for (var i = 0; i < selectElement.options.length; i++) {");
            client.println("var option = selectElement.options[i];");
            client.println("if (option.value == selectedOption) {");
            client.println("option.selected = true;");
            client.println("break; }  } </script>");

            client.println("<script> function setVolumenSound(pos) { ");
            client.println("var VolumenValue = document.getElementById(\"VolumenSound\").value;");
            client.println("document.getElementById(\"textVolumenSound\").innerHTML = VolumenValue;");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?VolumenSound=\" + pos + \"&\", true);");
            client.println("xhr.send(); } </script>");

            client.println("<script> function setDrehzahlmin(pos) { ");
            client.println("var VolumenValue = document.getElementById(\"Drehzahlmin\").value;");
            client.println("document.getElementById(\"textDrehzahlmin\").innerHTML = VolumenValue + \" %\";");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?Drehzahlmin=\" + pos + \"&\", true);");
            client.println("xhr.send(); } </script>");

            client.println("<script> function setDrehzahlmax(pos) { ");
            client.println("var VolumenValue = document.getElementById(\"Drehzahlmax\").value;");
            client.println("document.getElementById(\"textDrehzahlmax\").innerHTML = VolumenValue + \" %\";");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?Drehzahlmax=\" + pos + \"&\", true);");
            client.println("xhr.send(); } </script>");

            client.println("<script> function setshutdowndelay(pos) { ");
            client.println("var VolumenValue = document.getElementById(\"Motor aus Verzögerung\").value;");
            client.println("document.getElementById(\"textshutdowndelay\").innerHTML = VolumenValue + \" s\";");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?shutdowndelay=\" + pos + \"&\", true);");
            client.println("xhr.send(); } </script>");

            client.println("<script> function setthrottle_ramp(pos) { "); 
            client.println("var VolumenValue = document.getElementById(\"Motor Rampe in\").value;");
            client.println("document.getElementById(\"textthrottle_ramp\").innerHTML = VolumenValue + \" %/s\";");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?ThrottleRamp=\" + pos + \"&\", true);");
            client.println("xhr.send(); } </script>");

            client.println("<script> function setthrottle_dead_band(pos) { ");
            client.println("var VolumenValue = document.getElementById(\"Standgas Totband in\").value;");
            client.println("document.getElementById(\"textthrottle_dead_band\").innerHTML = VolumenValue + \" %\";");
            client.println("var xhr = new XMLHttpRequest();");
            client.println("xhr.open('GET', \"/?ThrottleDeadBand=\" + pos + \"&\", true);");
            client.println("xhr.send(); } </script>");                        
            
            client.println("</body></html>");     
                    
            // Die HTTP-Antwort endet mit einer weiteren Leerzeile
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // header löschen
    header = "";
    // Schließen Sie die Verbindung
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
    }
}    
