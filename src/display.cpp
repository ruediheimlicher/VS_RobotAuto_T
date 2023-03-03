//
//  display.cpp
//  OLED
//
//  Created by Ruedi Heimlicher on 01.03.2023.
//
//
#include <stdint.h>
#include "display.h"
#include "text.h"
#include "font.h"
#include "main.h"

#define linetab 12
extern uint16_t lxmittel;
extern uint16_t lymittel;
extern uint8_t Taste;

uint8_t char_x;
uint8_t char_y;

extern canal_struct canaldata;
uint8_t cursortab[10] = {cursortab0,cursortab1,cursortab2,cursortab3,cursortab4,cursortab5,cursortab6,cursortab7,cursortab0,cursortab0};
uint8_t itemtab[10] = {itemtab0,itemtab1,itemtab2,itemtab3,itemtab4,itemtab5,itemtab6,itemtab7,itemtab0,itemtab0};


// https://forum.arduino.cc/t/clearing-a-line-on-oled-adafruit-ssd1306/369985/8
void clearblock(uint8_t startx, uint8_t starty, uint8_t range)
{
for (uint8_t y=starty; y<=starty+6; y++)
      {
       for (uint8_t x=startx; x<startx + range; x++)
       {
        display.drawPixel(x, y, BLACK); 
       }
      }

}

void clearline(uint8_t starty)
{
for (uint8_t y=starty; y<=starty+6; y++)
      {
       for (uint8_t x=0; x<127; x++)
       {
        display.drawPixel(x, y, BLACK); 
       }
      }

}

void resetRegister(void)
{
   uint8_t i=0,k=0;
   for(i=0;i<8;i++)
   {
      for (k=0;k<8;k++)
      {
         posregister[i][k]=0xFFFF;
      }
   }
}


void sethomescreen(void)
{
  char_x = cursortab0;
  char_y = linetab;
  //clearline(10);
  //display.setCursor(0,0);
  //display.print(TitelTable[0]);
  clearblock(0,20,60);
  display.setCursor(0,20);
  display.print(lxmittel);
  clearblock(0,32,60);
  display.setCursor(32,20);
  display.print(canaldata.x);
 
  display.setCursor(0, 32);
  display.print(lymittel);
  
  display.setCursor(32,32);
  display.print(canaldata.y);
  
 clearblock(100,0,20);
  display.setCursor(100,0);
  display.print(Taste);
  display.setCursor(110,0);
  display.display();
}// sethomescreen

void refreshhomescreen(void)
{
  //clearline(10);
  display.setCursor(0,0);
  display.print(TitelTable[0]);
  clearblock(0,20,68);
  display.setCursor(0,20);
  display.print("lx:");
  display.print(lxmittel);
  //clearblock(50,20,60);
  display.setCursor(48,20);
  display.print(canaldata.x);
 
  clearblock(0,32,68);
  display.setCursor(0, 32);
  display.print("ly:");
  display.print(lymittel);
  //display.setCursor(64, 32);
  //char pfeil[] = {0x1E,0x1F,0x10,0x04,0x03,0x00};
  //display.print('*');
  //display.print(pfeilvollrechts);
  //display.fillTriangle(10, 50, 14, 54, 10, 58, WHITE);
  pfeilvollrechts(30,50,1);
  //display.print(0x03,0x00);
  //display.print('*');
  //clearblock(0,48,60);
  display.setCursor(48,32);
  display.print(canaldata.y);
  
  clearblock(100,0,20);
  display.setCursor(100,0);
  display.print(Taste);
  display.setCursor(110,0);
  display.display();
}// refreshhomescreen

void setsettingscreen(void)
{
  // Serial.printf("setsettingscreen start\n");
   
   resetRegister();
   blink_cursorpos=0xFFFF;
   
  
   // 2. Zeile
   posregister[0][0] =  itemtab[0] |    (2 << 10); //Modellname
   posregister[0][1] =  itemtab[0] |    (2 << 10); //

   posregister[0][2] =  itemtab[5] |    (3 << 10); // settingtext
   posregister[0][3] =  itemtab[7] |    (3 << 10); // settingnummer
   
   posregister[1][0] =  itemtab[0] |    (4 << 10); // Kanaltext
   
   posregister[2][0] =  itemtab[0] |    (5 << 10); // mixtext

   posregister[3][0] =  itemtab[0] |    (6 << 10); // Zuteilungtext
 
   posregister[4][0] =  itemtab[0] |    (7 << 10); // Zuteilungtext
   
   cursorpos[0][0] = cursortab[0] |    (2 << 10); // modellcursor lo: tab hi: page
   // cursorpos fuer model zeile/colonne
   
   cursorpos[0][1] = cursortab[5] |    (3 << 10); //  cursorpos fuer setting
   cursorpos[1][0] = cursortab[0] |    (4 << 10);  // cursorpos fuer kanal
   cursorpos[2][0] = cursortab[0] |    (5 << 10);  // cursorpos fuer mix
   cursorpos[3][0] = cursortab[0] |    (6 << 10);  // cursorpos fuer zuteilung
   cursorpos[4][0] = cursortab[0] |    (7 << 10);  // cursorpos fuer zuteilung
  
   
   char_x=itemtab[0];
   char_y = 1;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(SettingTable[0],1);
   char_height_mul = 2;
   char_width_mul = 1;
   
   // Modell Name
   
   //display_write_prop_str(char_y,char_x,0,menubuffer,2);
   
   char_y= (posregister[0][0] & 0xFF00)>> 10;
   char_x = posregister[0][0] & 0x00FF;

   display_write_str(ModelTable[curr_model],1);
   char_height_mul = 2;
   char_width_mul = 1;

   char_y= (cursorpos[0][0] & 0xFF00)>> 10;
   char_x = cursorpos[0][0] & 0x00FF;
   //display_write_symbol(pfeilvollrechts);
   
   // 2. Zeile Set mit Nummer
   char_y= (posregister[0][2] & 0xFF00)>> 10;
   char_x = posregister[0][2] & 0x00FF;
   
   char_height_mul = 1;
   display_write_str(SettingTable[2],2);
   

   // Kanal-Zeile
   char_y= (posregister[1][0] & 0xFF00)>> 10;
   char_x = posregister[1][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[3],2);
   
   char_y= (posregister[1][1] & 0xFF00)>> 10;
   char_x = posregister[1][1] & 0x00FF;
//   display_write_int(curr_kanal,2);

   
   // Mix-Zeile
   char_y= (posregister[2][0] & 0xFF00)>> 10;
   char_x = posregister[2][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[4],2);
   
   // Zuteilung-Zeile
   char_y= (posregister[3][0] & 0xFF00)>> 10;
   char_x = posregister[3][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[5],2);
 
   // Output-Zeile
   char_y= (posregister[4][0] & 0xFF00)>> 10;
   char_x = posregister[4][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[6],2);
   
   //Serial.printf("setsettingscreen end\n");
} // setSettingscreen


void display_write_str(const char *str, uint8_t prop)
{
      display.write(str);

}

void display_write_byte(unsigned char data) 
{
      display.print(data);
 
 }

void display_go_to (unsigned char x, unsigned char y)
{
   
	display_write_byte(DISPLAY_PAGE_ADDRESS | ((y) & 0x0F));  
	display_write_byte(DISPLAY_COL_ADDRESS_MSB | ((x>>4) & 0x0F));
	display_write_byte(DISPLAY_COL_ADDRESS_LSB | ((x) & 0x0F));
	
}


void display_write_symbol(const char* symbol)
{
   unsigned char col,page,tmp1,tmp2,tmp3,tmp4,counter;
   const char* pointer = symbol;
   uint8_t symboldelay = 2;
   uint8_t count = 0;
   col=(char_x+DISPLAY_OFFSET);
   page=char_y;
   display.print(symbol);
      return;

	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((FONT_HEIGHT/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = (symbol[count++]);
			
			if (char_height_mul > 1)
			{
				tmp2 = (tmp1&0xf0)>>4;
				tmp1 = tmp1 & 0x0f;
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(tmp4);
					}
				}
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH*char_width_mul);
	}
	return;
}

void drawverticalrect(void) 
{
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  
}

void drawlevelmeter(uint8_t x,uint8_t y,uint8_t w,uint8_t h, uint8_t level)
{
      display.drawRect(x, y, w, h, WHITE);
      uint8_t anzeige = level*h/100;
      display.fillRect(x+1,y+1,w-2,h-anzeige-2,0); // oberen teil leeren
      display.fillRect(x,y+h-anzeige,w,anzeige,WHITE);




    display.display(); // Update screen with each newly-drawn rectangle

}

void pfeilvollrechts(uint8_t x, uint8_t y, uint8_t full)
{
  if(full)
  {
    display.fillTriangle(x, y, x+4, y+4, x, y+8, WHITE);
  }
  else
  {
    display.fillTriangle(x, y, x+4, y+4, x, y+8, BLACK);
  }
}