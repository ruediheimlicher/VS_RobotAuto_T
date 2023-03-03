//
//  display.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 01.03.2023.
//
//
#include <stdint.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//#include <Fonts/FreeSans9pt7b.h>
//#include <Fonts/FreeSans12pt7b.h>



extern Adafruit_SSD1306 display;

#define cursortab0 2
#define cursortab1 26
#define cursortab2 46
#define cursortab3 54
#define cursortab4 66
#define cursortab5 78
#define cursortab6 90
#define cursortab7 100

#define itemtab0  10
#define itemtab1  34
#define itemtab2  50
#define itemtab3  62
#define itemtab4  74
#define itemtab5  88
#define itemtab6  110
#define itemtab7  118


#define FONT_WIDTH 	6


#define DISPLAY_OFFSET		2
#define DISPLAY_COL_ADDRESS_LSB		0x00  
#define DISPLAY_PAGE_ADDRESS		0xB0
#define DISPLAY_COL_ADDRESS_MSB		0x10


extern  uint8_t       curr_levelarray[8];
extern  uint8_t       curr_expoarray[8];
extern  uint8_t       curr_mixarray[8];
extern  uint8_t       curr_funktionarray[8];
extern  uint8_t       curr_statusarray[8];
extern  uint8_t       curr_ausgangarray[8];

extern  uint8_t       curr_devicearray[8];

extern  int8_t        curr_trimmungarray[8];

extern  uint8_t       curr_mixstatusarray[8];
extern  uint8_t       curr_mixkanalarray[8];

extern  uint8_t       curr_screen;
extern  uint8_t       curr_page; // aktuelle page
extern  uint8_t       curr_col; // aktuelle colonne
extern  uint8_t       curr_model; // aktuelles modell

extern  uint8_t       curr_kanal; // aktueller kanal

extern  uint8_t       curr_richtung; // aktuelle richtung
extern  uint8_t       curr_impuls; // aktueller impuls

extern  uint8_t       eepromsavestatus;
extern  uint8_t       programmstatus;
extern  uint8_t       servostatus;

extern  uint8_t       curr_setting;
extern  uint8_t       curr_cursorzeile; // aktuelle zeile des cursors
extern  uint8_t       curr_cursorspalte; // aktuelle colonne des cursors

extern  uint8_t       last_cursorzeile; // letzte zeile des cursors
extern  uint8_t       last_cursorspalte; // letzte colonne des cursors
extern  uint16_t      blink_cursorpos;


void clearblock(uint8_t startx, uint8_t starty, uint8_t range);
void clearline(uint8_t starty);

extern  uint16_t  posregister[8][8]; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

extern  uint16_t  cursorpos[8][8]; // Aktueller screen: werte fuer page und daraufliegende col fuer cursor (hex). geladen aus progmem

// 
extern  uint16_t              updatecounter; // Zaehler fuer Einschalten

extern uint8_t char_x;
extern uint8_t char_y;
extern uint8_t char_height_mul;
extern uint8_t char_width_mul;


void sethomescreen(void);
void refreshhomescreen(void);

void setsettingscreen(void);
void setausgangscreen(void);
void setcanalscreen(void);
void settrimmscreen(void);
void setmixscreen(void);
void setsavescreen(void);

void display_write_str(const char *str, uint8_t prop);
void display_write_byte(unsigned char data);
void display_write_symbol(const char* symbol);

void drawverticalrect(void);
void drawlevelmeter(uint8_t x,uint8_t y,uint8_t w,uint8_t h, uint8_t level);
