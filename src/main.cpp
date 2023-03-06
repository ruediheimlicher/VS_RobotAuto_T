/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>

#include "main.h"
#include <Ticker.h> 
#include <driver/adc.h>
#include "elapsedMillis.h"
#include "expo.h"
#include "settings.h"
#include "lcd.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define COLUMS           20   //LCD columns
#define ROWS             4    //LCD rows
#define LCD_SPACE_SYMBOL 0x20 //space symbol from LCD ROM, see p.9 of GDM2004D datasheet

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4,5, 6, 16, 11, 12, 13, 14, POSITIVE);
//LiquidCrystal_I2C lcd(0x3F, 16, 2);

// OLED
//#include "SSD1306Wire.h"  

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//
//#include <Fonts/FreeSans9pt7b.h>
//#include <Fonts/FreeSans12pt7b.h>

# include "display.h"

// ASCII: https://www.i8086.de/zeichensatz/code-page-437.html


#define OLED_RESET -1

#define TASTEOK            1
#define AKTIONOK           2
#define UPDATEOK           3


elapsedMillis              zeitintervall;
uint8_t           sekundencounter = 0;

uint8_t           startcounter=0; // timeout-counter beim Start von Settings, schneller als manuellcounter. Ermoeglicht Dreifachklick auf Taste 5
uint8_t           settingstartcounter=0; // Counter fuer Klicks auf Taste 5elapsedMicros sinceusb;

elapsedMicros sincelastbeepA;

elapsedMillis sincelastseccond;
elapsedMillis sinceimpulsstart;


uint8_t char_height_mul = 0;
uint8_t char_width_mul = 0;

uint16_t stopsekunde=0;
uint16_t stopminute=0;
uint16_t motorsekunde=0;
uint16_t motorminute=0;
uint8_t   motorstunde=0;

uint16_t sendesekunde=0;
uint16_t sendeminute=0;
uint8_t sendestunde=0;

uint8_t    curr_model; // aktuelles modell
uint8_t    speichermodel=0;
uint8_t    curr_kanal=0; // aktueller kanal
uint8_t    curr_impuls=0; // aktueller impuls

uint8_t    curr_setting=0; // aktuelles Setting fuer Modell
uint8_t   speichersetting=0;

uint8_t    curr_trimmkanal=0; // aktueller  Kanal fuerTrimmung
uint8_t    curr_trimmung=0; // aktuelle  Trimmung fuer Trimmkanal


uint8_t    curr_screen = 0; // aktueller screen
uint8_t    last_screen=0; // letzter screen

uint8_t    curr_page=7; // aktuelle page
uint8_t    curr_col=0; // aktuelle colonne

uint8_t    curr_cursorzeile=0; // aktuelle zeile des cursors
uint8_t    curr_cursorspalte=0; // aktuelle colonne des cursors
uint8_t    last_cursorzeile=0; // letzte zeile des cursors
uint8_t    last_cursorspalte=0; // letzte colonne des cursors


 uint8_t       curr_levelarray[8];
 uint8_t       curr_expoarray[8];
 uint8_t       curr_mixarray[8];
 uint8_t       curr_funktionarray[8];
 uint8_t       curr_statusarray[8];
 uint8_t       curr_ausgangarray[8];

 uint8_t       curr_devicearray[8];

uint16_t    blink_cursorpos=0xFFFF;

uint8_t     displaystatus=0x00; // Tasks fuer Display
 uint8_t                  masterstatus = 0;
uint8_t                  eepromsavestatus = 0;

// Tastatur
uint8_t                 Tastenindex=0;
uint16_t                Tastenwert=0;
uint8_t                 adcswitch=0;
uint16_t                lastTastenwert=0;
int16_t                 Tastenwertdiff=0;
uint16_t                tastaturcounter=0;

uint8_t trimmtastaturstatus = 0;
uint16_t                Trimmtastenwert=0;
uint8_t                 Trimmtastenindex=0;
uint16_t                lastTrimmtastenwert=0;
int16_t                 Trimmtastenwertdiff=0;
uint16_t                trimmtastaturcounter=0;


uint8_t kanalsettingarray[ANZAHLMODELLE][NUM_SERVOS][KANALSETTINGBREITE] = {};

uint8_t mixingsettingarray[ANZAHLMODELLE][4][2] = {};


Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

#define DEMO_DURATION 3000
typedef void (*Demo)(void);
int counter = 1;
// end OLED
#define JOYSTICK_B

#define ESPOK 0

#define SENDINTERVALL 20

#define ESP8266_D1_MINI_PEER  1
#define ESP8266_ROBOTSTEPPER_PEER 2

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
//uint8_t broadcastAddress1[] = {0x48, 0x3F, 0xDA, 0xA4, 0x36, 0x57};
uint8_t broadcastAddress1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // stub
//uint8_t broadcastAddress2[] = {0x8C, 0xAA, 0xB5, 0x7B, 0xA3, 0x28}; 

//uint8_t broadcastAddress2[] = {0x48, 0x3F, 0xDA, 0xA4, 0x36, 0x57};
//uint8_t broadcastAddress2[] = {0x44, 0x17, 0x93, 0x14, 0xF6, 0x6F}; // MINI_PRO 1
uint8_t broadcastAddress2[] = {0x44, 0x17, 0x93, 0x14, 0xF7, 0x17}; // ESP8266 D1 MINI
//uint8_t broadcastAddress3[] = {0x48, 0x3F, 0xDA, 0xA4, 0x36, 0x57}; // RobotStepper
uint8_t broadcastAddress4[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t broadcastAddressArray[8][6] = 
{{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0x44, 0x17, 0x93, 0x14, 0xF7, 0x17}, // ESP8266 D1 MINI
{0x48, 0x3F, 0xDA, 0xA4, 0x36, 0x57}, // RobotStepper
{0x44, 0x17, 0x93, 0x14, 0xF7, 0x17},
{0x44, 0x17, 0x93, 0x14, 0xF6, 0x6F},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
uint8_t peerpos = 1; // geladener peer

#define NUM_SERVOS 4

#define MAX_TICKS 2010 // Maxwert ms fur Impulslaenge
#define MIN_TICKS 990  // Minwert ms fuer Impulslaenge

// Graupner:
//#define MAX_ADC 3500
//#define MIN_ADC 700

/*
// joystick runf
#define MAX_ADC 4050 // Max wert vom ADC
#define MIN_ADC 1250 // Min wert vom ADC
*/

// Joystick Multiplex
#define MAX_ADC 3310 // Max wert vom ADC
#define MIN_ADC 2150 // Min wert vom ADC

#define NULLBAND 10 // nichts tun bei kleineren Kanalwerten
uint16_t   servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte
uint16_t   maxADCarray[NUM_SERVOS] = {};
uint16_t   minADCarray[NUM_SERVOS] = {};

 

uint16_t maxwinkel = 180;

#define KANAL_X 0
#define KANAL_Y 1

uint16_t grenzex = servomittearray[KANAL_X]; // wert fuer Eichung
uint16_t grenzey = servomittearray[KANAL_Y];

uint8_t   servostatus=0;
uint8_t   programmstatus = 0;
uint8_t   tastaturstatus = 0;
uint8_t buttonstatus = 0;
uint8_t tonindex = 0;
void playTon(int ton);
#define START_TON 0
#define LICHT_ON 1

uint8_t expolevelarray[NUM_SERVOS] = {1,1,0,0}; // expo-levels pro kanal

uint16_t ubatt = 0;

int ledintervall = 800;
Ticker timer;
elapsedMillis ledmillis;

elapsedMillis displaymillis;

int displayintervall = 100;
uint16_t          updatecounter=0; // Zaehler fuer Einschalten
uint16_t          manuellcounter=0;

// debounce
Ticker debouncetimer;
elapsedMillis debouncemillis;
#define MAX_CHECKS 10
uint8_t DebouncedState;
uint8_t lastDebouncedState;
uint8_t debouncestatus;

#define AVERAGE 8
uint8_t averagecounter = 0;
uint16_t lxmittelwertarray[AVERAGE];
uint16_t lymittelwertarray[AVERAGE];

uint16_t lxmittel = 0;
uint16_t lymittel = 0;

uint8_t propfaktorx = 1.0;
uint8_t propfaktory = 1.0;
uint8_t State[MAX_CHECKS];
uint8_t Index = 0;
/*
taste0: GPIO1
taste1: GBIO3
taste2: GPIO4
taste3: GPIO5

*/

#define BOARD_TASTE 33
#define BOARD_MINI 1
#define BOARD_NODE 0
uint8_t boardstatus = 0;
uint16_t tastaturmittelwertarray[AVERAGE];
uint8_t analogtastaturstatus = 0;
uint8_t TastaturCount=0;
#define TASTE_OFF  0
#define TASTE_ON  1

uint16_t TastenStatus=0;
uint16_t Tastenprellen=0x1F;


uint8_t firstrun = 1;

#define TASTE0 14
#define TASTE1 27
#define TASTE2 26
#define TASTE3 25

#define TASTATUR_PIN  33
uint8_t tastencounter = 0;
uint16_t tastaturmittel = 0;
uint16_t tastaturwert = 0;
uint16_t ADCwert = 0;

//char leerstring[] = {219,219,219,0};
char leerstring[] = "          \0";
uint16_t Taste = 0;

uint16_t lcdtest = 0;

 uint8_t tastenstatus() // bitmuster aller Tasten
 {
   uint8_t returnstatus = 0;
 if(digitalRead(TASTE0) == 0)
   {
      tastencounter++;
      returnstatus |= (1<<0);
   }
  
 if(digitalRead(TASTE1) == 0)
   {

     returnstatus |= (1<<1);
   }
  if(digitalRead(TASTE2) == 0)
   {
     returnstatus |= (1<<2);
   }
  if(digitalRead(TASTE3) == 0)
   {
     returnstatus |= (1<<3);
   }
  
 //Serial.printf("Tastenstatus: %d\n",returnstatus);
 return returnstatus;

 }
 
void DebounceSwitch (void)
{
  uint8_t i,j;
  State[Index] = tastenstatus(); // aktuellen status laden
  ++Index;
  j = 0xFF;
  for(i=0;i<MAX_CHECKS;i++)
  {
    j = j & State[i];
  }
  DebouncedState = j;
  if(Index >= MAX_CHECKS)
  {
      Index = 0;
  }

}

 typedef struct EEPROMdata_struct
  {
    uint16_t xH = MAX_ADC;
    uint16_t xL = MIN_ADC;
    uint16_t yH = MAX_ADC;
    uint16_t yL = MIN_ADC;

  }EEPROMdata_struct;

EEPROMdata_struct EEPROMdata;


/*
typedef struct canal_struct 
{
  uint16_t lx;
  uint16_t ly;
  uint16_t rx;
  uint16_t ry;

  uint8_t digi;

  uint16_t x;
  uint16_t y;
} canal_struct;
*/
//Create a struct_message called canaldata
canal_struct canaldata;

elapsedMillis sendtimer;
uint8_t loopstatus = 0;



esp_now_peer_info_t peerInfo;


void deletePeer() {
	//const esp_now_peer_info_t *peer = &slave;
	//const uint8_t *peer_addr = slave.peer_addr;
 uint8_t *peer_addr = peerInfo.peer_addr;
	esp_err_t delStatus = esp_now_del_peer(peer_addr);
	Serial.print("Slave Delete Status: ");
	if (delStatus == ESP_OK) {
		// Delete success
		Serial.println("Success");
	}
	else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
		// How did we get so far!!
		Serial.println("ESPNOW Not Init");
	}
	else if (delStatus == ESP_ERR_ESPNOW_ARG) {
		Serial.println("Invalid Argument");
	}
	else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
		Serial.println("Peer not found.");
	}
	else {
		Serial.println("Not sure what happened");
	}
}




// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  char macStr[18];
  //Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  //snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
 // Serial.print(macStr);
 // Serial.print(" send status:\t");
 // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

uint16_t tickslimited(uint16_t inticks)
{
  if(inticks > MAX_TICKS)
  {
    return  MAX_TICKS;
  }
  else if (inticks < MIN_TICKS)
  {
    return MIN_TICKS;
  }
 return inticks;
}

uint16_t servoticks(uint16_t inticks)
{
  uint16_t expovalue = 0;
  inticks = map(inticks,MIN_TICKS,MAX_TICKS, 0,maxwinkel);
  return inticks;
/*
  if (inticks > maxwinkel/2)
  {
     expovalue = maxwinkel/2 +  expoarray[expolevelarray][inticks - maxwinkel/2];
  }
  else
  {
     expovalue = maxwinkel/2 -  expoarray[expolevelarray][maxwinkel/2 - inticks ]; 
  }
 return expovalue;
 */
}



uint16_t mapADC(uint16_t inADC)
{
  uint16_t raw_in = inADC;
  if(raw_in > MAX_ADC)
  {
    raw_in = MAX_ADC;
  }
  if(raw_in < MIN_ADC)
  {
    raw_in = MIN_ADC;
  }
  // adc-wert(Ausgabe des ADC) auf Tick-Bereich (ms, Impulslaenge) umsetzen
  return map(raw_in, MIN_ADC, MAX_ADC, MIN_TICKS, MAX_TICKS);
}

uint16_t expovalue(uint8_t pin, uint8_t expolevel, uint16_t invalue)
{
  uint16_t expowert= 0;
  
  uint16_t servomitte = mapADC(servomittearray[pin]);
  //Serial.printf("fixServoMitte pin: %d : %d servomitte: %d\n", pin, servomittearray[pin], servomitte);
  uint16_t expopos = 0;
  if (invalue > servomitte) 
  {
    expopos = invalue - servomitte;
    if(expopos > 0x200)
    {
      expopos = 0x200;
    }
    expowert =  servomitte + propfaktorx * (expoarray512[expolevel][expopos]);
  }
  else
  { 
    expopos = servomitte - invalue;
     if(expopos > 0x200)
    {
      expopos = 0x200;
    }
    expowert =  servomitte - propfaktorx * expoarray512[expolevel][expopos];
  }
  //Serial.printf(" expopos: %d\t",expopos);
  return expowert;
}

void fixServoMitte()
{
  uint16_t firstlxmittel = 0;
  uint16_t firstlymittel = 0;
  for (uint8_t i=0;i<AVERAGE;i++)
  {
    uint16_t rawlx = adc1_get_raw(ADC1_CHANNEL_0);
    firstlxmittel += rawlx;
    Serial.printf("i: %d wert: %d\n",i,rawlx, firstlxmittel);
    uint16_t rawly = adc1_get_raw(ADC1_CHANNEL_3);
    firstlymittel += rawly;
 
  
  }
  //Serial.printf("firstlxmittel: %d \n",firstlxmittel);
  firstlxmittel /= AVERAGE;
  firstlymittel /= AVERAGE;
 // Grenzwerte einhalten

uint16_t lx = mapADC(firstlxmittel);
//Serial.printf("fixServoMitte firstlxmittel: %d  lx: %d\n", firstlxmittel,lx);
//servomittearray[KANAL_X] =  mapADC(firstlxmittel);
servomittearray[KANAL_X] =  (firstlxmittel);

uint16_t ly = mapADC(firstlymittel);
//servomittearray[KANAL_Y] = mapADC(firstlymittel);
servomittearray[KANAL_Y] = (firstlymittel);

//Serial.printf("fixServoMitte X: %d \n", servomittearray[KANAL_X]);
//Serial.printf("fixServoMitte Y: %d \n", servomittearray[KANAL_Y]);
}


void boardchange()
{
  Serial.print("boardchange");
}


uint8_t Tastenwahl(uint16_t Tastaturwert)
{
   if (Tastaturwert < TASTE01)
      return 1;
   if (Tastaturwert < TASTE02)
      return 2;
   if (Tastaturwert < TASTE03)
      return 3;
   if (Tastaturwert < TASTE04)
      return 4;
   if (Tastaturwert < TASTE05)
      return 5;
   if (Tastaturwert < TASTE06)
      return 6;
   if (Tastaturwert < TASTE07)
      return 7;
   if (Tastaturwert < TASTE08)
      return 8;
   if (Tastaturwert < TASTE09)
      return 9;
      /*
   if (Tastaturwert < TASTEL)
      return 10;
   if (Tastaturwert < TASTE00)
      return 0;
   if (Tastaturwert < TASTER)
      return 12;
   */
   return 0;
}

void tastenfunktion(uint16_t Tastenwert)
{
 //Serial.printf("\t\t\tTastenwert IN: %d\n",Tastenwert);
   if (Tastenwert>20) // ca Minimalwert der Matrix
   {
    Serial.printf("\t\t\tTastenwert IN > 20: %d\n",Tastenwert);
      //         wdt_reset();
      /*
       0: Wochenplaninit
       1: IOW 8* 2 Bytes auf Bus laden
       2: Menu der aktuellen Ebene nach oben
       3: IOW 2 Bytes vom Bus in Reg laden
       4: Auf aktueller Ebene nach rechts (Heizung: Vortag lesen und anzeigen)                           
       5: Ebene tiefer
       6: Auf aktueller Ebene nach links (Heizung: Folgetag lesen und anzeigen)                           
       7: 
       8: Menu der aktuellen Ebene nach unten
       9: DCF77 lesen
       
       12: Ebene höher
       */
      Serial.printf("TastaturCount: %d Tastenwert: %d \n",TastaturCount,Tastenwert);
      TastaturCount++;
      if (TastaturCount > 1)  //   Prellen
      {
        
         TastaturCount=0x00;
         
         if (analogtastaturstatus & (1<<TASTE_ON)) // 
         {
            
         }
         else 
         
         {
            
            analogtastaturstatus |= (1<<TASTE_ON);
            
            Taste=Tastenwahl(Tastenwert);
            Serial.printf("Tastenwert: %d Taste: %d \n",Tastenwert,Taste);
            TastaturCount=0;
            //Tastenwert=0x00;
            
            uint8_t inBytes[4]={};
            
            switch (Taste)
            {
               case 0://
               { 
                  Serial.printf("\tTaste 0\n");
                  
                  break;
                  // Blinken auf C2
                  
               }
                  break;
                  
                  
               case 1: 
               {
                  Serial.printf("\tTaste 1\n");
                if (tastaturstatus & (1<<AKTIONOK))
                {
                  programmstatus ^= (1<<MOTOR_ON);
                  tastaturstatus &=  ~(1<<AKTIONOK);
                }
                  manuellcounter=0;
                } // case 1
                  break;
                  
               case 2:     // up                             //   Menu vorwaertsschalten   
               {
                Serial.printf("\tTaste 2\n");
                  
               }break;
                  
               case 3:   //
               {
                Serial.printf("\tTaste 3\n");
                if (manuellcounter)
                {
                  if (tastaturstatus & (1<<AKTIONOK))
                  {
                    //Serial.printf("H3 2 programmstatus vor: %d\n",programmstatus);
                    programmstatus ^= (1<<STOP_ON);
                    tastaturstatus &=  ~(1<<AKTIONOK);
                    tastaturstatus |= (1<<UPDATEOK);
                    //Serial.printf("H3 2 programmstatus nach: %d\n",programmstatus);

                  }

                  manuellcounter=0;

                } // if (manuellcounter)
 
                
               }break;
                  
               case 4:   // left
               {
                  Serial.printf("\tTaste 4\n");
               } break; // 
                  
                  
               case 5:                        // Ebene tiefer
               {
                  Serial.printf("\tTaste 5 startcounter: %d manuellcounter: %d\n",startcounter,manuellcounter);
                  if ((startcounter == 0) && (manuellcounter)) // Settings sind nicht aktiv
                  {
                    Serial.printf("Taste 5 Settings sind nicht aktiv > SETTINGWAIT aktiviert\n");
                    //lcd_gotoxy(0,2);
                    //lcd_putc('1');
                    //lcd_putc(' ');
                    
                    programmstatus |= (1<< SETTINGWAIT);
                    settingstartcounter=1;
                    manuellcounter = 1;
                    
                  }
                  else
                  {

                    if (startcounter > 3) // Irrtum, kein Umschalten
                      {
                        Serial.printf("Taste 5 startcounter > 3 ERR\n");
                        programmstatus &= ~(1<< SETTINGWAIT);
                        settingstartcounter=0;
                        startcounter=0;
                        manuellcounter = 1;
                      }
                    else
                    {
                      Serial.printf("\tTaste 5 programmstatus: %d\n",programmstatus);
                      if ((programmstatus & (1<< SETTINGWAIT))&& (manuellcounter)) // Umschaltvorgang noch aktiv
                      {

                        settingstartcounter++; // counter fuer klicks
                        Serial.printf("SETTINGWAIT settingstartcounter: %d\n",settingstartcounter);
                        if (settingstartcounter == 3)
                        {
                          //OSZI_A_LO();
                          //lcd_gotoxy(2,2);
                          //lcd_putc('3');
                          Serial.printf("*** settingstartcounter 3\n");
                          programmstatus &= ~(1<< SETTINGWAIT);
                          //           programmstatus |= (1<<UPDATESCREEN);
                          settingstartcounter=0;
                          startcounter=0;
                          // Umschalten
                          display.clearDisplay();
                          //lcd_putc('D');
                          // Serial.printf("*H5*D \t");
                          
                          setsettingscreen();
                          //lcd_putc('E');
                          curr_screen = SETTINGSCREEN;
                          curr_cursorspalte=0;
                          curr_cursorzeile=0;
                          last_cursorspalte=0;
                          last_cursorzeile=0;
                          blink_cursorpos=0xFFFF;
                          // Serial.printf("*H5*F \n");
                          manuellcounter = 1;
                          Serial.printf("setsettingscreen C \n");
                          //servostatus |=  (1<<RUN);
                          //OSZI_A_HI();
                          } // if settingcounter <
                                 //manuellcounter = 0;
                      } // programmstatus & (1<< SETTINGWAIT

                        

                    }
                        


                  }


                Serial.printf("\tTaste 5 end\n");
                  
               } break;
                  
               case 6: // right
               {
                Serial.printf("\tTaste 6\n");
                  
               } break; // 
                  
               case 7:
               {
                  Serial.printf("\tTaste 7 \n");
                  
               }break;
                  
                  
               case 8:    // down                                //Menu rueckwaertsschalten
               {
                Serial.printf("\tTaste 8\n");
              
                  
               }break;
                  
               case 9:
               {
                Serial.printf("\tTaste 9\n");
                  DebouncedState |= (1<<3);
                  
               }break;
                  
               case 10: // set Nullpunkt
               {
                  Serial.printf("Taste 10\n");
                  
               }break;
               case 12:// 
               {
                Serial.printf("Taste 12\n");
                  //STOP
                  
                  
               }break;
                  
                  
            }//switch Taste
            //Serial.printf("(1<<TASTE_ON) end\n");
         }
         
      }
      Serial.printf("tastenfunktion end\n");
   }
   else 
   {
      //Serial.printf("tastenfunktion else\n");
      if (analogtastaturstatus & (1<<TASTE_ON))
      {
        Taste = 0;
        Serial.printf("Tastenwert 0\n");
        analogtastaturstatus &= ~(1<<TASTE_ON);

      }
   }
   
}

uint16_t readTastatur(void)
{
   uint16_t adctastenwert = adc1_get_raw(ADC1_CHANNEL_5);
   adctastenwert /= 4;
   if (adctastenwert > 10)
   {
      //Serial.printf("readTastatur adctastenwert: %d\n",adctastenwert);
      //lcd.setCursor(10, 2); 
      //lcd.print(adctastenwert);
      return adctastenwert;
   }
   return 0;
}


 
void setup() 
{
  Serial.begin(115200);
  //EEPROM.begin(512);
  delay(500);




// OLED 
// Initialising the UI will init the display too.
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  delay(2000);
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  delay(500);
  //display.fillCircle(display.width()/2, display.height()/2, 10, WHITE);
  
  display.display(); 
 delay(500);
// end OLED

for(int i=0;i<NUM_SERVOS;i++)
{
  servomittearray[i]= (MAX_TICKS + MIN_TICKS)/2;
  maxADCarray[i] = MAX_ADC - 200;
  minADCarray[i] = MIN_ADC + 200;
}
  pinMode(TASTATUR_PIN, INPUT);
 
  pinMode(TASTE0,INPUT_PULLUP);
  pinMode(TASTE1,INPUT_PULLUP);
  pinMode(TASTE2,INPUT_PULLUP);
  pinMode(TASTE3,INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT); 

adc1_config_width(ADC_WIDTH_BIT_12);
//adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);

  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
   
  //esp_now_register_send_cb(OnDataSent);
  esp_err_t registererr = esp_now_register_send_cb(OnDataSent);
  Serial.printf("registererr: %d\n",registererr);

    
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer 1");
    //return;
  }
  else
  {
    Serial.println("add peer 1 OK");
  }
/*
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 2");
    //return;
  }
  else
  {
    Serial.println("add peer 2 OK");
  }
  */
  /*
  /// register third peer
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 3");
    return;
  }
  else
  {
    Serial.println("add peer 3 OK");
  }
  */
  /// register forth peer
  memcpy(peerInfo.peer_addr, broadcastAddress4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer 4");
    return;
  }
  else
  {
    Serial.println("add peer 4 OK");
  }

  sys_delay_ms(100);

   //adc1_config_width(ADC_WIDTH_BIT_12);
   adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
   adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_BIT_12);

//   adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_0);
//   adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_0);

 //int erfolg = lcd.begin(COLUMS, ROWS, LCD_5x8DOTS, 21, 22, 400000, 250);
 // Serial.printf("erfolg: %d\n",erfolg);

//EEPROM.get(0,EEPROMdata);
//EEPROM.end();
//uint16_t xwerthigh = EEPROMdata.xH;
//Serial.print("EEPROM read xH:\n ");
//Serial.print("EEPROM xh: %d", EEPROMdata.xH);
//  int erfolg = lcd.begin(COLUMS, ROWS, LCD_5x8DOTS, 21, 22, 400000, 250);
buttonstatus |= (1<<START_TON);
//buttonstatus = 13;



}

// OLED functions


// end OLED functions
 
 
void loop() 
{

if (firstrun)
{

/*
// lcd.print(F("PCF8574 is OK..."));
 int erfolg = lcd.begin(COLUMS, ROWS, LCD_5x8DOTS, 21, 22, 400000, 250);
 // Serial.printf("erfolg: %d\n",erfolg);
  //lcd.print(F("PCF8574 is OK..."));
  lcd.setCursor(0, 0);              //set 1-st colum & 2-nd row, 1-st colum & row started at zero
  lcd.print(F("VS_ROBOTAUTO_T"));
*/
  fixServoMitte();

  for (int i=0;i<AVERAGE;i++)
  {
    lxmittelwertarray[i] = servomittearray[KANAL_X];
    lymittelwertarray[i] = servomittearray[KANAL_Y];
  }
// register  peer  
// 1: D1 MINI, 2: RobotStepper
  memcpy(peerInfo.peer_addr, broadcastAddressArray[1], 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 2");
    //return;
  }
  else
  {
    Serial.println("add peer 2 OK");
  }
 buttonstatus |= (1<<START_TON);
 sethomescreen();
 
  firstrun = 0;

}

if (zeitintervall > 500)
{
  sekundencounter++;
  if (sekundencounter%2)
  {
  //Serial.printf("refresh_screen sendesekunde: %d\n",sendesekunde);
    sendesekunde++;
    if (manuellcounter && (blink_cursorpos < 0xFFFF))
     {
    display_setcursorblink(sendesekunde);
     }
      // Akku-stuff here

      // end Akkustuff
    /*
    if ((timeoutcounter > TIMEOUT) )
			 {
				// Test
				//masterstatus |= (1<<TIMEOUT_BIT);
			
				if (digitalRead(AKKU_OFF_PIN)) // Akku ist ON,timeout ist relevant
				{
				   masterstatus |= (1<<TIMEOUT_BIT);
				}
				else
				{
				   timeoutcounter = 0;
				}
        
			 }
       */
      		uint8_t levelwert0 = kanalsettingarray[curr_model][0][1]; // levelarray
					 uint8_t levelwert1 = kanalsettingarray[curr_model][1][1];
					 //       Serial.printf("curr_levelarray 0: %d, 1: %d\t",curr_levelarray[0], curr_levelarray[1]);
					 uint8_t expowert0 = kanalsettingarray[curr_model][0][2]; // expoarray
					 uint8_t expowert1 = kanalsettingarray[curr_model][1][2];
					 //      Serial.printf("curr_expoarray 0: %d, 1: %d\n",curr_expoarray[0], curr_expoarray[1]);
		 
					 // kanalsettingarray[model][kanal][1] = curr_levelarray[kanal];
					 uint8_t savelevelarray0 = kanalsettingarray[curr_model][0][1];
					 uint8_t savelevelarray1 = kanalsettingarray[curr_model][1][1];
					 //        Serial.printf("savelevelarray 0: %d, 1: %d\t",savelevelarray0, savelevelarray1);
		 
					 //kanalsettingarray[model][kanal][2] = curr_expoarray[kanal];
					 uint8_t saveexpowertarray0 = kanalsettingarray[curr_model][0][2];
					 uint8_t saveexpowertarray1 = kanalsettingarray[curr_model][1][2];
					 //       Serial.printf("saveexpowertarray0 0: %d, 1: %d\n",saveexpowertarray0, saveexpowertarray1);
					 uint8_t savedevicewertarray0 = kanalsettingarray[curr_model][0][3];
					 uint8_t savedevicewertarray1 = kanalsettingarray[curr_model][1][3];

      
      
      

  
    //Serial.printf("send usb: pot0 %d\n",pot0);
      
    //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
    // sendeminute
    if (sendesekunde == 60)
    {
      sendeminute++;
      sendesekunde = 0;
   
   
      if (curr_screen == 0)
      {
         //Serial.printf("refresh_screen sendeminute: %d\n",sendeminute);
        servostatus &=  ~(1<<RUN); 
      
        refresh_screen();
        servostatus |=  (1<<RUN); 
        //display.setCursor(120,12);
        //display_write_stopzeit(sendesekunde, sendeminute, 1);
        display.display();
      }
    } // sendesekunde == 60
    if (sendeminute == 60)
	  {
	    sendestunde++;
	  	sendeminute = 0;
    }
   
  
    if (programmstatus & (1<<STOP_ON))
    {
    //   lcd_gotoxy(15,0);
    //   lcd_putint2(stopsekunde);
            
      stopsekunde++;
      if (stopsekunde == 60)
      {
        stopminute++;
        stopsekunde=0;
      }
      if (stopminute >= 60)
      {
        stopminute = 0;
      }
      if (curr_screen == 0)
      {
        //update_time();
      }
    }  // if STOP_ON}
    displaystatus |= (1<<UHR_UPDATE);//XX
  } // if sekundencounter%2

if (manuellcounter && (blink_cursorpos < 0xFFFF))
{
   display_setcursorblink(updatecounter);
}
updatecounter++;
manuellcounter++;               
if (programmstatus & (1<<SETTINGWAIT))
{
  startcounter++;
 
}
else
{
         //startcounter = 0;
}

if ((manuellcounter > MANUELLTIMEOUT) )
{
         
         
//         programmstatus &= ~(1<< LEDON);
  //display_set_LED(0);
   manuellcounter=1;
   
  if (curr_screen) // nicht homescreen
  {
    display.clearDisplay();
    curr_screen = 0;
    curr_cursorspalte=0;
    curr_cursorzeile=0;
    last_cursorspalte=0;
    last_cursorzeile=0;
    settingstartcounter=0;
    startcounter=0;
    eepromsavestatus=0;
    //       read_Ext_EEPROM_Settings();// zuruecksetzen
    
    sethomescreen();
    
    programmstatus &= ~(1<<UPDATESCREEN);
  }
  else 
  {
    
    programmstatus &= ~(1<< SETTINGWAIT);
    curr_screen = 0;
    startcounter=0;
    settingstartcounter=0;
    
    /*
     lcd_gotoxy(0,2);
     lcd_putc(' ');
     lcd_putc(' ');
     lcd_putc(' ');
     */
    
  }
 
//
} // if ((manuellcounter > MANUELLTIMEOUT) )

      


  zeitintervall = 0;
} // if zeitintervall

if (displaymillis > displayintervall)
{
  //Serial.printf("refresh curr screen: %d\n",curr_screen);
  //refreshhomescreen();
  uint8_t refresh = refresh_screen();
  
  /*
  // https://en.wikipedia.org/wiki/Code_page_437
  displaymillis = 0;
  //display.clearDisplay();
  clearline(10);
  display.setCursor(0,0);
  display.println("RobotAuto_T");
  clearblock(0,20,60,8);
  display.setCursor(0,20);
  display.print(lxmittel);
  clearblock(0,32,60,8);
  display.setCursor(32,20);
  display.print(canaldata.x);
 
   display.setCursor(0, 32);
  display.print(lymittel);
  
   display.setCursor(32,32);
  display.print(canaldata.y);
  
 clearblock(100,0,20,8);
  display.setCursor(100,0);
  display.print(Taste);
   display.setCursor(110,0);
   //buttonstatus = 13;
  display.print(buttonstatus);
  display.display();
  */
//drawverticalrect();

}


if (ledmillis > ledintervall)
  {
    ledmillis = 0;
    ubatt++;
    //drawlevelmeter(120, 12,8,48,ubatt%100);

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //Serial.printf("DebouncedState: %d\n",DebouncedState);
    //Serial.println("led");
    //Serial.println(canaldata.lx);
    //Serial.printf("%d \t%d *%d*\n", canaldata.lx , canaldata.ly, canaldata.digi);
    //Serial.printf("%d \t%d \n", canaldata.x , canaldata.y);
    //Serial.printf("%d \t%d DebouncedState: %d\n", lxmittel , lymittel,DebouncedState);
    lcd.setCursor(0,1);
    lcd_puts("data");
    lcd.setCursor(6,1);
    lcd_puts("expo");

         // OLED
     //display.clearDisplay();
     //display.setTextSize(1);
    //display.setCursor(0,0);
     //display.print(lxmittel);
     //display.display();
    //display.setCursor(0, 46);
    //display.setTextSize(2);
    //display.setFont(&FreeSans12pt7b);
    //display.drawPixel(20,80,WHITE);
    //display.drawRect(20,20,30,10,WHITE);
     //display.print(canaldata.lx);
    //display.setCursor(40,30);
    //display.drawChar(40,35,0x10,WHITE,1,1);
    //display.drawChar(45,35,0x11,WHITE,1,1);

    
      //display.display();
 
     // end OLED
    /*
    lcd.setCursor(0,2);
    lcd_putint12(canaldata.lx);
    lcd.setCursor(6,2);
    lcd_putint1(expolevelarray[0]);
    lcd.setCursor(0,3);
    lcd_putint12(canaldata.ly);
    lcd.setCursor(6,3);
    lcd_putint1(expolevelarray[1]);

    lcd.setCursor(16,0);
    lcd_putint12(tastaturwert);
    lcd.setCursor(16,1);
    lcd_putint12(ADCwert);

    */
    /*
    lcd_putint12(tastaturwert);
    lcd.write(' ');
    lcd_putint1(Taste);
    lcd.setCursor(0,3);
    lcd_putint(lcdtest++);
    lcd_putc(' ');
    lcd_putint12(lcdtest++);
    */

    // OLED
    
    // end OLED


  // Joystick eichen
  if (DebouncedState & (1<<2)) // Taste 1
  {

    if(lxmittel > servomittearray[KANAL_X] + 300) // testausschlag x high
    {
      if (lxmittel > maxADCarray[KANAL_X])
      {
        maxADCarray[KANAL_X] = lxmittel;
      }
      //maxADCarray[KANAL_X] = grenzex;
    }

    if(lxmittel < (servomittearray[KANAL_X] - 300)) // testausschlag x low
    {
      if (lxmittel < minADCarray[KANAL_X])
      {
        minADCarray[KANAL_X] = lxmittel;
      }
    }

    // Seite Y
    if(lymittel > servomittearray[KANAL_Y] + 300) // testausschlag x high
    {
      if (lymittel > maxADCarray[KANAL_Y])
      {
        maxADCarray[KANAL_Y] = lymittel;
      }
     
    }

    if(lymittel < (servomittearray[KANAL_Y] - 300)) // testausschlag x low
    {
      if (lymittel < minADCarray[KANAL_Y])
      {
        minADCarray[KANAL_Y] = lymittel;
      }
    }


    Serial.printf("Eichung maxADC X: %d minADC X: %d maxADC Y: %d minADC Y: %d\n", maxADCarray[KANAL_X],minADCarray[KANAL_X],maxADCarray[KANAL_Y],minADCarray[KANAL_Y]);
  }

  }

 if (debouncemillis > 5)
  {

    debouncemillis = 0;
    DebounceSwitch();

  }
  
  if (sendtimer > SENDINTERVALL)
  {

if (DebouncedState & 0x04)
    {
      if(debouncestatus & 0x04)
      {}
      else
      {
      debouncestatus |= 0x04;
      tastencounter++;
      }
    }


uint16_t rawlx = adc1_get_raw(ADC1_CHANNEL_0);
lxmittelwertarray[(averagecounter & 0x07)] = rawlx;
lxmittel = 0;
for (int i=0;i < AVERAGE;i++)
{
  lxmittel += lxmittelwertarray[i];
}
 lxmittel /= AVERAGE;

 // Grenzwerte einhalten
//uint16_t lx = tickslimited(lxmittel);
uint16_t lx = mapADC(lxmittel);
uint16_t kanalwertx = lx;
//Serial.printf("lxmittel: %d kanalwertx: %d \t",lxmittel,kanalwertx); 


uint16_t rawly = adc1_get_raw(ADC1_CHANNEL_3);
lymittelwertarray[(averagecounter & 0x07)] = rawly;
lymittel = 0;
for (int i=0;i < AVERAGE;i++)
{
  lymittel += lymittelwertarray[i];
}
lymittel /= AVERAGE;



int16_t rawtastaturwert = adc1_get_raw(ADC1_CHANNEL_5);
tastaturmittelwertarray[(averagecounter & 0x07)] = rawtastaturwert;
tastaturmittel = 0;
for (int i=0;i < AVERAGE;i++)
{
  tastaturmittel += tastaturmittelwertarray[i];
}
ADCwert = rawtastaturwert;
tastaturmittel = rawtastaturwert;///= AVERAGE;

//tastaturmittel = 0xFFF - tastaturmittel;
tastaturmittel /= 4;
tastaturmittel = tastaturmittel ; //* 9 / 8 ;
tastaturwert = 0x3FF - tastaturmittel;
//tastaturwert =  tastaturmittel;

tastenfunktion(tastaturwert);
//Serial.printf("nach tastenfunktion\n");

averagecounter++;


    tastencounter++;
      if (tastencounter > 10)
      {
         tastencounter = 0;
         //tastenfunktion(tastaturmittel);
          

      }




//lcd.setCursor(0, 2); 
      //lcd.print(adctastenwert);

 // Grenzwerte einhalten
//uint16_t ly = tickslimited(lymittel); // Grenzen einhalten, MAX_TICKS, MIN_TICKS

uint16_t mittex = servomittearray[KANAL_X];
mittex = mapADC(mittex);

uint16_t mittey = servomittearray[KANAL_Y];
mittey = mapADC(mittey);

//Serial.printf("rawlx: %d \t rawly: %d \n",rawlx,rawly); 

// adc-wert(Ausgabe des ADC) auf Tick-Bereich (ms, Impulslaenge) umsetzen
uint16_t ly = mapADC(lymittel);

//Serial.printf("lxmittel: %d lx: %d mittex: %d  lymittel: %d ly: %d mittey: %d\n",lxmittel, lx, mittex,lymittel, ly, mittey); 

uint16_t kanalwerty = ly; // Werte von ADC

//Serial.printf("lymittel: %d kanalwerty: %d \n",lymittel,kanalwerty); 

uint16_t expokanalwertx = expovalue(KANAL_X,expolevelarray[KANAL_X],kanalwertx); // expolevelarray enthaelt expo-levels pro knal

//Serial.printf("kanalwertx: \t %d \texpokanalwertx: \t %d\t\t",kanalwertx,expokanalwertx);
kanalwertx = expokanalwertx;


uint16_t expokanalwerty = expovalue(KANAL_Y,expolevelarray[KANAL_Y],kanalwerty);


//Serial.printf(" kanalwerty: \t %d \t expokanalwerty: \t %d\n",kanalwerty,expokanalwerty);
kanalwerty = expokanalwerty;

canaldata.lx = kanalwertx;
canaldata.ly = kanalwerty;

canaldata.rx = lx;
canaldata.ry = ly;

 // MIX, von RC_22_32

 uint16_t mixkanalwertx = 0;
 uint16_t mixkanalwerty = 0;


  //Serial.printf("lxm: %d kan x: %d mx: %d\t",lxmittel, kanalwertx,  mittex); 
 // Serial.printf("\tlym: %d l kan y: %d my: %d\n",lymittel, kanalwerty,  mittey); 

uint16_t diffx = 0;
uint16_t diffy = 0;
float floatdiffx = 0.0; // Tempo
float floatdiffy = 0.0; // Turn
float korrfaktor = 1;
floatdiffx = kanalwertx - mittex;
floatdiffy = kanalwerty - mittey;
float diffsumme = abs(floatdiffx) + abs(floatdiffy);
if (diffsumme > 512)
{
  korrfaktor = 512/diffsumme;
}
floatdiffx *= korrfaktor;
floatdiffy *= korrfaktor;

float floatkanalwertx =  mittex + floatdiffx + floatdiffy;
if (fabs(floatkanalwertx - mittex) < NULLBAND)
{
  floatkanalwertx = mittex;
}
else
{
  //Serial.printf("kanalwertx: \t%d \texpokanalwertx: \t%d\n",kanalwertx,expokanalwertx);
}

float floatkanalwerty =  mittey + floatdiffx - floatdiffy;
if (fabs(floatkanalwerty - mittey) < NULLBAND)
{
  floatkanalwerty = mittey;
}
else
{
  //Serial.printf("kanalwerty: \t%d \texpokanalwerty: \t%d\n",kanalwerty,expokanalwerty);
}



/*
  if(kanalwertx > mittex)
  {

     diffx = kanalwertx - mittex;
    //Serial.printf(" > mx dx: +%d\t",diffx);
     mixkanalwertx = mittex + diffx;
     mixkanalwerty = mittey + diffx;
  }
  else 
  {
     diffx = mittex - kanalwertx;
     //Serial.printf(" < mx dx: -%d\t",diffx);
     mixkanalwertx = mittex - diffx;
     mixkanalwerty = mittey - diffx;   
  }

  

  if(kanalwerty > mittey) // Richtung
  {
     diffy = kanalwerty - mittey;
     //Serial.printf(" > my dy: +%d\t",diffy);
     mixkanalwertx += diffy;
     mixkanalwerty -= diffy;
  }
  else 
  {
     diffy = mittey - kanalwerty;
     //Serial.printf(" < my dy: -%d\t",diffy);
     mixkanalwertx -= diffy;
     mixkanalwerty += diffy;   
  }
*/
//mixkanalwertx = servoticks(mixkanalwertx);
//mixkanalwerty = servoticks(mixkanalwerty);
 //Serial.printf("mixkanalwertx: %d  mixkanalwerty: %d \t",mixkanalwertx, mixkanalwerty); 

// ticks umrechnen von MAX_TICKS, MIN_TICKS auf maxwinkel
uint16_t outvalue_lx = servoticks(kanalwertx);

//canaldata.lx = outvalue_lx;
//canaldata.lx = mixkanalwertx;

canaldata.lx = uint16_t(floatkanalwertx);

//canaldata.x = map(uint16_t(floatkanalwertx),MIN_TICKS,MAX_TICKS,0,180);
canaldata.x = map(uint16_t(floatkanalwertx),mittex - 0x200,mittex + 0x200,0,180);

//canaldata.x = map(uint16_t(kanalwertx),MIN_TICKS,MAX_TICKS,0,180);

// ticks umrechnen
uint16_t outvalue_ly = servoticks(kanalwerty);

//canaldata.y = map(uint16_t(floatkanalwerty),MIN_TICKS,MAX_TICKS,0,180);
canaldata.y = map(uint16_t(floatkanalwerty),mittey - 0x200,mittey + 0x200,0,180);

//canaldata.y = map(uint16_t(kanalwerty),MIN_TICKS,MAX_TICKS,0,180);


//canaldata.ly = outvalue_ly;
//Serial.printf("data.x: %d data.y: %d \n",canaldata.x, canaldata.y); 
//canaldata.ly = mixkanalwerty;
canaldata.ly = uint16_t(floatkanalwerty);

//Serial.printf("data.lx: %d data.ly: %d \n",canaldata.lx, canaldata.ly); 

  // Tasten uebergeben

  canaldata.digi = DebouncedState;

  if (buttonstatus & (1<<START_TON))
  {
    Serial.printf("buttonstatus: %d \n",buttonstatus);
    canaldata.digi |= (1<<3);
    buttonstatus &= ~(1<<START_TON);
  }

  
  if (lastDebouncedState != DebouncedState)
  {
    //Serial.print("DebouncedState: ");
    //Serial.println(DebouncedState);
    lastDebouncedState = DebouncedState;
  }
  //Serial.printf("%d \t%d \t%d \t**  \t%d\t%d  \t%d\n", rawlx , lx, canaldata.lx , rawly , ly, canaldata.ly);
    //Serial.printf("%d\t%d  \t%d m: %d\n",  rawly , ly, canaldata.ly, lymittel);
  //Serial.printf("lx: %d \tly: %d \tx: %d \ty: %d \n", canaldata.lx , canaldata.ly,canaldata.x , canaldata.y);

    sendtimer = 0;
    //Serial.print(canaldata.lx);
    //Serial.print(" ");
    //Serial.println(canaldata.ly);
    esp_err_t result = esp_now_send(0, (uint8_t *) &canaldata, sizeof(canal_struct));
    //Serial.print("result: ");
    //Serial.println(result);
   
  }

 
}