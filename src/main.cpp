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

#include <Ticker.h> 
#include <driver/adc.h>
#include "elapsedMillis.h"
#include "expo.h"


#define ESPOK 0

#define SENDINTERVALL 50

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
//uint8_t broadcastAddress1[] = {0x48, 0x3F, 0xDA, 0xA4, 0x36, 0x57};
uint8_t broadcastAddress1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t broadcastAddress2[] = {0x8C, 0xAA, 0xB5, 0x7B, 0xA3, 0x28}; // ESP8266 D1 MINI

//uint8_t broadcastAddress2[] = {0x48, 0x3F, 0xDA, 0xA4, 0x36, 0x57};
uint8_t broadcastAddress3[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define NUM_SERVOS 4

#define MAX_TICKS 2010
#define MIN_TICKS 990

#define MAX_ADC 3500
#define MIN_ADC 700

uint16_t   servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte

uint16_t maxwinkel = 180;

#define KANAL_X 0
#define KANAL_Y 1

uint8_t buttonstatus = 0;
uint8_t tonindex = 0;
void playTon(int ton);
#define START_TON 0
#define LICHT_ON 1

uint8_t expolevel[NUM_SERVOS] = {1,0,0,0};

uint16_t ubatt = 0;

int ledintervall = 1000;
Ticker timer;
elapsedMillis ledmillis;


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
volatile uint8_t boardstatus = 0;

uint8_t firstrun = 1;

#define TASTE0 14
#define TASTE1 27
#define TASTE2 26
#define TASTE3 25
uint8_t tastencounter = 0;


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
 
void DebounceSwitch3 (void)
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
 // Serial.print("Packet to: ");
  // Copies the sender mac address to a string
 // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
 //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
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
     expovalue = maxwinkel/2 +  expoarray[expolevel][inticks - maxwinkel/2];
  }
  else
  {
     expovalue = maxwinkel/2 -  expoarray[expolevel][maxwinkel/2 - inticks ]; 
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
  return map(raw_in, MIN_ADC, MAX_ADC, MIN_TICKS, MAX_TICKS);
}

uint16_t expovalue(uint8_t pin, uint8_t expolevel, uint16_t invalue)
{
  uint16_t expowert= 0;
  uint16_t servomitte = servomittearray[pin];
  uint16_t expopos = 0;
  if (invalue > servomitte) 
  {
    expopos = invalue - servomitte;
    if(expopos > 0x200)
    {
      expopos = 0x200;
    }
    expowert =  servomitte + expoarray512[expolevel][expopos];
  }
  else
  { 
    expopos = servomitte - invalue;
     if(expopos > 0x200)
    {
      expopos = 0x200;
    }
    expowert =  servomitte - expoarray512[expolevel][expopos];
  }
  Serial.printf("expopos: %d\t",expopos);
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
  //Serial.printf("i: %d wert: %d\n",i,rawlx, firstlxmittel);
    uint16_t rawly = adc1_get_raw(ADC1_CHANNEL_3);
    firstlymittel += rawly;
 
  
  }
  Serial.printf("firstlxmittel: %d \n",firstlxmittel);
  firstlxmittel /= AVERAGE;
  firstlymittel /= AVERAGE;
 // Grenzwerte einhalten

uint16_t lx = mapADC(firstlxmittel);
//Serial.printf("fixServoMitte firstlxmittel: %d  lx: %d\n", firstlxmittel,lx);
servomittearray[KANAL_X] =  mapADC(firstlxmittel);
uint16_t ly = mapADC(firstlymittel);
servomittearray[KANAL_Y] = mapADC(firstlymittel);
//Serial.printf("fixServoMitte X: %d \n", servomittearray[KANAL_X]);
//Serial.printf("fixServoMitte Y: %d \n", servomittearray[KANAL_Y]);
}

void boardchange()
{
  Serial.print("boardchange");
}


 
void setup() {
  Serial.begin(115200);

for(int i=0;i<NUM_SERVOS;i++)
{
  servomittearray[i]= (MAX_TICKS + MIN_TICKS)/2;
}
  pinMode(BOARD_TASTE, INPUT_PULLUP);
  //attachInterrupt(BOARD_TASTE, boardchange,FALLING);
  pinMode(TASTE0,INPUT_PULLUP);
  pinMode(TASTE1,INPUT_PULLUP);
  pinMode(TASTE2,INPUT_PULLUP);
  pinMode(TASTE3,INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT); 
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  pinMode(TASTE0,INPUT_PULLUP);
  pinMode(TASTE1,INPUT_PULLUP);
  
  //esp_now_register_send_cb(OnDataSent);
  esp_err_t registererr = esp_now_register_send_cb(OnDataSent);
  Serial.printf("registererr: %d\n",registererr);

 if (digitalRead(BOARD_TASTE) == 0)
    {
      Serial.println("boardchange setup");
    }
   
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer 1");
    return;
  }
  else
  {
    Serial.println("add peer 1 OK");
  }

  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("add peer 2 OK");
  }
  /// register third peer
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("add peer 3 OK");
  }
     
   adc1_config_width(ADC_WIDTH_BIT_12);
   adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
   adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_11);

//   adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_0);
//   adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_0);

  
}
 
 
void loop() 
{

if (firstrun)
{
  fixServoMitte();
  firstrun = 0;
}
if (ledmillis > ledintervall)
  {
    ledmillis = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //Serial.printf("DebouncedState: %d\n",DebouncedState);
    //Serial.println("led")
    //Serial.println(canaldata.lx);
    //Serial.printf("%d \t%d *%d*\n", canaldata.lx , canaldata.ly, canaldata.digi);
    if (digitalRead(BOARD_TASTE) == 0)
    {
      Serial.println("boardchange");
    }
  }

 if (debouncemillis > 5)
  {

    debouncemillis = 0;
    DebounceSwitch3();
  

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
uint16_t lxmittel = 0;
for (int i=0;i < AVERAGE;i++)
{
  lxmittel += lxmittelwertarray[i];
}
 lxmittel /= AVERAGE;

 // Grenzwerte einhalten
//uint16_t lx = tickslimited(lxmittel);
uint16_t lx = mapADC(lxmittel);
uint16_t kanalwertx = lx;
//Serial.printf("rawlx: %d kanalwertx: %d \n",rawlx,kanalwertx); 


uint16_t rawly = adc1_get_raw(ADC1_CHANNEL_3);
lymittelwertarray[(averagecounter & 0x07)] = rawly;
uint16_t lymittel = 0;
for (int i=0;i < AVERAGE;i++)
{
  lymittel += lymittelwertarray[i];
}
lymittel /= AVERAGE;

averagecounter++;

 // Grenzwerte einhalten
//uint16_t ly = tickslimited(lymittel); // Grenzen einhalten, MAX_TICKS, MIN_TICKS


uint16_t ly = mapADC(lymittel);
uint16_t kanalwerty = ly; // Werte von ADC

uint16_t expokanalwertx = expovalue(KANAL_X,expolevel[KANAL_X],kanalwertx);
kanalwertx = expokanalwertx;
Serial.printf("kanalwertx: %d expokanalwertx: %d\t\t",kanalwertx,expokanalwertx);

uint16_t expokanalwerty = expovalue(KANAL_Y,expolevel[KANAL_Y],kanalwerty);
kanalwerty = expokanalwerty;

Serial.printf("kanalwerty: %d expokanalwerty: %d\n",kanalwerty,expokanalwerty);


canaldata.lx = kanalwertx;
canaldata.ly = kanalwerty;

 // MIX, von RC_22_32

 uint16_t mixkanalwertx = 0;
 uint16_t mixkanalwerty = 0;
uint16_t mittex = servomittearray[KANAL_X];
uint16_t mittey = servomittearray[KANAL_Y];

  //Serial.printf("lxm: %d kan x: %d mx: %d\t",lxmittel, kanalwertx,  mittex); 
 // Serial.printf("\tlym: %d l kan y: %d my: %d\n",lymittel, kanalwerty,  mittey); 

  uint16_t diffx = 0; // Tempo
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

  uint16_t diffy = 0;
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
//mixkanalwertx = servoticks(mixkanalwertx);
//mixkanalwerty = servoticks(mixkanalwerty);
 Serial.printf("mixkanalwertx: %d  mixkanalwerty: %d \t",mixkanalwertx, mixkanalwerty); 

// ticks umrechnen von MAX_TICKS, MIN_TICKS auf maxwinkel
uint16_t outvalue_lx = servoticks(kanalwertx);

//canaldata.lx = outvalue_lx;
canaldata.lx = mixkanalwertx;
// ticks umrechnen
uint16_t outvalue_ly = servoticks(kanalwerty);
//canaldata.ly = outvalue_ly;

canaldata.ly = mixkanalwerty;

//Serial.printf("data.lx: %d data.ly: %d \n",canaldata.lx, canaldata.ly); 

  // Tasten uebergeben
  canaldata.digi = DebouncedState;
  if (lastDebouncedState != DebouncedState)
  {
    Serial.print("DebouncedState: ");
    Serial.println(DebouncedState);
    lastDebouncedState = DebouncedState;
  }
  //Serial.printf("%d \t%d \t%d \t**  \t%d\t%d  \t%d\n", rawlx , lx, canaldata.lx , rawly , ly, canaldata.ly);
    //Serial.printf("%d\t%d  \t%d m: %d\n",  rawly , ly, canaldata.ly, lymittel);
  //Serial.printf("%d \t%d *%d*\n", canaldata.lx , canaldata.ly, canaldata.digi);

    sendtimer = 0;
    //Serial.print(canaldata.lx);
    //Serial.print(" ");
    //Serial.println(canaldata.ly);
    esp_err_t result = esp_now_send(0, (uint8_t *) &canaldata, sizeof(canal_struct));
    //Serial.print("result: ");
    //Serial.println(result);
   
  }

 
}