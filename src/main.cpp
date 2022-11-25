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

#define SENDINTERVALL 20

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0x48, 0x3F, 0xDA, 0xA4, 0x36, 0x57};
uint8_t broadcastAddress2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t broadcastAddress3[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define NUM_SERVOS 4

#define MAX_TICKS 3400
#define MIN_TICKS 1700
uint16_t   servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte

uint16_t maxwinkel = 180;

uint8_t buttonstatus = 0;
uint8_t tonindex = 0;
void playTon(int ton);
#define START_TON 0
#define LICHT_ON 1

uint8_t expolevel = 0;

uint16_t ubatt = 0;

int ledintervall = 1000;
Ticker timer;
elapsedMillis ledmillis;


// debounce
Ticker debouncetimer;
elapsedMillis debouncemillis;
#define MAX_CHECKS 10
uint8_t DebouncedState;
uint8_t State[MAX_CHECKS];
uint8_t Index = 0;
/*
taste0: GPIO1
taste1: GBIO3
taste2: GPIO4
taste3: GPIO5

*/

#define TASTE0 14
#define TASTE1 15
#define TASTE2 16
#define TASTE3 17

uint8_t tastenstatus() // bitmusteraller Tasten
{
  uint8_t returnstatus = 0;
  if(digitalRead(TASTE0))
  {
    returnstatus |= (1<<0);
  }
  if(digitalRead(TASTE1))
  {
  returnstatus |= (1<<1);
  }
  if(digitalRead(TASTE2))
  {
    returnstatus |= (1<<2);
  }
  if(digitalRead(TASTE3))
  {
    returnstatus |= (1<<3);
  }
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
  uint8_t lx;
  uint8_t ly;
  uint8_t rx;
  uint8_t ry;

 uint8_t digi;

  uint8_t x;
  uint8_t y;
} canal_struct;

//Create a struct_message called canaldata
canal_struct canaldata;

elapsedMillis sendtimer;



esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  //Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print(macStr);
  //Serial.print(" send status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  if (inticks > maxwinkel/2)
  {
     expovalue = maxwinkel/2 +  expoarray[expolevel][inticks - maxwinkel/2];
  }
  else
  {
     expovalue = maxwinkel/2 -  expoarray[expolevel][maxwinkel/2 - inticks ];
  }
 return expovalue;
}

 
void setup() {
  Serial.begin(74880);

pinMode(LED_BUILTIN, OUTPUT); 
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  pinMode(TASTE0,INPUT_PULLUP);
  pinMode(TASTE1,INPUT_PULLUP);
  
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  /// register third peer
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
   //pinMode(5,INPUT);
   //pinMode(8,INPUT);
   adc1_config_width(ADC_WIDTH_BIT_12);
   adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);
   adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_0);
//   adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_0);
//   adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_0);

  
}
 uint8_t loopstatus = 0;
 
void loop() 
{
/*
   */
if (ledmillis > ledintervall)
  {
    ledmillis = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //Serial.printf("DebouncedState: %d\n",DebouncedState);
    Serial.println("led");
  }

 if (debouncemillis > 2)
  {

    debouncemillis = 0;
    DebounceSwitch3();
  

  }
  
  if (sendtimer > SENDINTERVALL)
  {

int rawlx = adc1_get_raw(ADC1_CHANNEL_0);
uint16_t lx = tickslimited(rawlx);
 // ticks umrechnen
uint16_t outvalue_lx = servoticks(lx);
canaldata.lx = outvalue_lx;

int rawly = adc1_get_raw(ADC1_CHANNEL_3);
 // Tickbereich einhalten
 uint16_t ly = tickslimited(rawly);
  // ticks umrechnen
  uint16_t outvalue_ly = servoticks(ly);
  canaldata.ly = outvalue_ly;

  // Tasten uebergeben
  canaldata.digi = DebouncedState;

    Serial.printf("%d \t%d *%d*\n", canaldata.lx , canaldata.ly, canaldata.digi);

    sendtimer = 0;
    //Serial.print(canaldata.lx);
    //Serial.print(" ");
    //Serial.println(canaldata.ly);
    esp_err_t result = esp_now_send(0, (uint8_t *) &canaldata, sizeof(canaldata));
    //Serial.print("result: ");
    //Serial.println(result);
   
  }

 // delay(50);
}