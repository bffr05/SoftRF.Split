/*
 * View_Status_EPD.cpp
 * Copyright (C) 2019-2022 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../system/SoC.h"

#if defined(USE_EPAPER)

#include "../driver/EPD.h"
#include "../TrafficHelper.h"
#include "../driver/Battery.h"
#include "../driver/EEPROM.h"
#include "../driver/RF.h"
#include <protocol.h>

#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include "U8g2_for_Adafruit_GFX.h"

extern uint32_t tx_packets_counter, rx_packets_counter,relay_packets_counter,hz_counter;


static U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;

uint16_t radar_center_x;
uint16_t radar_center_y;
uint16_t radar_radius;
bool blackmode = 0 ;
void EPD_main_setup()
{

  uint16_t radar_w = 156;
  radar_radius = radar_w / 2;

  radar_center_x = radar_radius+4;
  radar_center_y =  radar_radius+30;
  
  u8g2Fonts.begin(*display); // connect u8g2 procedures to Adafruit GFX
  u8g2Fonts.setFontDirection(3);
  u8g2Fonts.setForegroundColor(GxEPD_BLACK);
  u8g2Fonts.setBackgroundColor(GxEPD_WHITE);
  u8g2Fonts.setFont(u8g2_font_battery19_tn);

}


void EPD_main_loop(bool refresh)
{
  if (isTimeToEPD()) {
    EPDTimeMarker = millis();

    static bool forcerefresh = true;
    if (refresh)
      forcerefresh = true;
    //if (forcerefresh)
      display->fillScreen(GxEPD_WHITE);

    blackmode = !blackmode;
    //Serial.printf("blackmode=%u\n",blackmode);

    char buf[16];

    int16_t  tbx, tby;
    uint16_t tbw, tbh;
    uint xbase,ybase,width,height;


    display->fillCircle(radar_center_x, radar_center_y,
                            radar_radius, blackmode ? GxEPD_BLACK : GxEPD_WHITE);
    if (!blackmode)
    {
      display->drawCircle(  radar_center_x, radar_center_y,
                            radar_radius, GxEPD_BLACK);
      display->drawCircle(  radar_center_x, radar_center_y,
                            radar_radius-1, GxEPD_BLACK);
    }
    display->drawCircle(  radar_center_x, radar_center_y,
                          radar_radius / 2, blackmode ? GxEPD_WHITE : GxEPD_BLACK);

    display->drawFastHLine(radar_center_x-radar_radius+1,radar_center_y,7,blackmode ? GxEPD_WHITE : GxEPD_BLACK);
    display->drawFastHLine(radar_center_x+radar_radius-8,radar_center_y,7,blackmode ? GxEPD_WHITE : GxEPD_BLACK);
    //display->drawFastVLine(radar_center_x,radar_center_y-radar_radius+1,7,GxEPD_BLACK);
    display->drawFastVLine(radar_center_x,radar_center_y+radar_radius-8,7,blackmode ? GxEPD_WHITE : GxEPD_BLACK);
    display->drawFastHLine(radar_center_x-(radar_radius/2)+1,radar_center_y,3,blackmode ? GxEPD_WHITE : GxEPD_BLACK);
    display->drawFastHLine(radar_center_x+(radar_radius/2)-4,radar_center_y,3,blackmode ? GxEPD_WHITE : GxEPD_BLACK);
    display->drawFastVLine(radar_center_x,radar_center_y-(radar_radius/2)+1,3,blackmode ? GxEPD_WHITE : GxEPD_BLACK);
    display->drawFastVLine(radar_center_x,radar_center_y+(radar_radius/2)-4,3,blackmode ? GxEPD_WHITE : GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);
    display->getTextBounds("N", 0, 0, &tbx, &tby, &tbw, &tbh);
    display->fillRect( radar_center_x-tbw/2 , radar_center_y-radar_radius-tbh+5,tbw, tbh, GxEPD_WHITE);
    //Serial.printf("N x=%u y=%u w=%u h=%u\n",radar_center_x-tbw/2 , radar_center_y-radar_radius-tbh+5,tbw, tbh)
    display->setCursor(radar_center_x-tbw/2-1 , radar_center_y-radar_radius+5);
    display->print("N");

    uint speed = 33; //30kmh
    uint hdg = 303; //303deg
    uint hdgbase = 25; //303deg
    uint scale = 500; //500m
    uint scaletime = 30; // 30secs
    int vario = 2; //-1.2
    uint alt = 68; //2068m
    uint time = 34; //00:34
    uint distbase = 3000; //3km
    uint flytime = 83; // 01:23
   

    float hdgx = sin(radians(hdg));
    float hdgy = cos(radians(hdg));
    float hdgxm = sin(radians(hdg-4));
    float hdgym = cos(radians(hdg-4));
    float hdgxp = sin(radians(hdg+4));
    float hdgyp = cos(radians(hdg+4));

    float hdgbasex = sin(radians(hdgbase));
    float hdgbasey = cos(radians(hdgbase));
    float hdgbasexm = sin(radians(hdgbase-4));
    float hdgbaseym = cos(radians(hdgbase-4));
    float hdgbasexp = sin(radians(hdgbase+4));
    float hdgbaseyp = cos(radians(hdgbase+4));

    display->drawTriangle(
      radar_center_x+hdgbasex*(radar_radius),
      radar_center_y-hdgbasey*(radar_radius),
      radar_center_x+hdgbasexm*(radar_radius-12),
      radar_center_y-hdgbaseym*(radar_radius-12),
      radar_center_x+hdgbasexp*(radar_radius-12),
      radar_center_y-hdgbaseyp*(radar_radius-12),
      blackmode ? GxEPD_WHITE : GxEPD_BLACK);


    display->fillTriangle(
      radar_center_x+hdgx*(radar_radius),
      radar_center_y-hdgy*(radar_radius),
      radar_center_x+hdgxm*(radar_radius-12),
      radar_center_y-hdgym*(radar_radius-12),
      radar_center_x+hdgxp*(radar_radius-12),
      radar_center_y-hdgyp*(radar_radius-12),
      blackmode ? GxEPD_WHITE : GxEPD_BLACK);
    float scalems = scale / scaletime;
    float speedms = speed *1000/3600;
    int speedscale = (speedms/scalems)*radar_radius;
    if (speedscale > radar_radius)
      speedscale = radar_radius;
    display->drawLine(
      radar_center_x+hdgx*(radar_radius),
      radar_center_y-hdgy*(radar_radius),
      radar_center_x+hdgx*(radar_radius-speedscale),
      radar_center_y-hdgy*(radar_radius-speedscale),
      blackmode ? GxEPD_WHITE : GxEPD_BLACK);


    //drawLine



    u8g2Fonts.setCursor(display->width() - 5, 15);
    u8g2Fonts.print((Battery_charge() + 19) / 20);


    xbase= 4;ybase=2;width=58;height=22;
    display->setFont(&FreeSerifBold12pt7b);
    //display->drawRect( xbase, ybase,width, height, GxEPD_BLACK);
    display->setCursor(xbase , ybase+height-2);
    snprintf(buf, sizeof(buf), "%02u:%02u", time/60,time%60);
    display->print(buf);
    display->fillRect( xbase, ybase,width, 3,  GxEPD_BLACK);
    //display->fillRect( xbase, ybase,width, height, GxEPD_WHITE);

    xbase= 64;ybase=2;width=37;height=22;
    display->setFont(&FreeSerifBold12pt7b);
    //display->drawRect( xbase, ybase,width, height, GxEPD_BLACK);
    display->setCursor(xbase , ybase+height-6);
    snprintf(buf, sizeof(buf), "%03u", distbase/1000);
    display->print(buf);
    display->fillRect( xbase, ybase+height-3,width, 3,  GxEPD_BLACK);
    //display->fillRect( xbase, ybase,width, height, GxEPD_WHITE);

    xbase= 103;ybase=2;width=58;height=22;
    display->setFont(&FreeSerifBold12pt7b);
    //display->drawRect( xbase, ybase,width, height, GxEPD_BLACK);
    display->setCursor(xbase , ybase+height-2);
    snprintf(buf, sizeof(buf), "%02u:%02u", flytime/60,flytime%60);
    display->print(buf);
    display->fillRect( xbase, ybase,width, 3,  GxEPD_BLACK);
    //display->fillRect( xbase, ybase,width, height, GxEPD_WHITE);


    display->setFont(&FreeMono9pt7b);
    xbase= 4;ybase=25;width=31;height=12;
    //display->drawRect( xbase, ybase,width, height, GxEPD_BLACK);
    display->setCursor(xbase , ybase+height-1);
    if (scaletime<60)
    {
      snprintf(buf, sizeof(buf), "%02us", scaletime);
      display->print(buf);
    }
    else
    {
      snprintf(buf, sizeof(buf), "%02um", scaletime/60);
      display->print(buf);
    }
    //display->fillRect( xbase, ybase,width, height, GxEPD_WHITE);


    display->setFont(&FreeMono9pt7b);
    xbase= 4;ybase=39;width=21;height=12;
    //display->drawRect( xbase, ybase,width, height, GxEPD_BLACK);
    display->setCursor(xbase , ybase+height-1);
    if (scale<1000)
    {
      snprintf(buf, sizeof(buf), "%02u", scale/100);
      display->print(buf);
    }
    else
    {
      snprintf(buf, sizeof(buf), "%u", scale/1000);
      display->print(buf);
    }
    //display->fillRect( xbase, ybase,width, height, GxEPD_WHITE);

    
    xbase= 148;ybase=25;width=48;height=16;
    display->setFont(&FreeSerifBold12pt7b);
    //display->drawRect( xbase, ybase,width, height, GxEPD_BLACK);
    display->setCursor(xbase , ybase+height-1);
    snprintf(buf, sizeof(buf), "%04u", alt);
    display->print(buf);

    display->setFont(&FreeSerifBold12pt7b);
    xbase= 165;ybase=95;width=31;height=26;
    //display->drawRect( xbase, ybase,width, height, GxEPD_BLACK);
    display->fillRect( xbase, ybase,width, 3,  GxEPD_BLACK);
    display->fillRect( xbase, ybase+height-3,width, 3,  GxEPD_BLACK);
    display->setCursor(xbase , ybase+height-6);
    //display->setCursor(radar_center_x+radar_radius+3 , radar_center_y+6);
    //display->drawRect( xbase, ybase,width, height, GxEPD_BLACK);
    snprintf(buf, sizeof(buf), "%u.%u", vario/10,vario%10);
    display->print(buf);


    display->fillTriangle(
      165,93,
      165,43,
      148,43,GxEPD_BLACK);
    display->fillRect(
      165,43,31,50,GxEPD_BLACK);
    display->drawFastHLine(160,  83,     36, GxEPD_WHITE);
    display->drawFastHLine(157,  73,     39, GxEPD_WHITE);
    display->drawFastHLine(154,  63,     42, GxEPD_WHITE);
    display->drawFastHLine(151,  53,     45, GxEPD_WHITE);
    display->fillTriangle(
      148,157,
      165,123,
      165,157,
      GxEPD_BLACK);
    display->fillRect(
      165,123,31,35,GxEPD_BLACK);
    display->drawFastHLine(160,  133,     36, GxEPD_WHITE);
    display->drawFastHLine(154,  143,     42, GxEPD_WHITE);
    display->drawFastHLine(148,  153,     48, GxEPD_WHITE);

/*
    display->fillRect( radar_center_x+radar_radius+4, radar_center_y-13, 51, 24, GxEPD_WHITE);

    display->setFont(&FreeSerifBold12pt7b);
    snprintf(buf, sizeof(buf), "%c%u.%u", vario>0?'+':vario<0?'-':'=',vario/10,vario%10);
    display->setCursor(radar_center_x+radar_radius+1 , radar_center_y+4);
    display->print(buf);



    display->fillRect( radar_center_x+radar_radius+4, radar_center_y-13, 51, 24, GxEPD_WHITE);
    display->setCursor(radar_center_x+radar_radius-5 , radar_center_y-radar_radius+2);
    display->print(alt);
    display->fillRect( radar_center_x+radar_radius-5, radar_center_y-radar_radius-17, 50, 3,  GxEPD_BLACK);
    display->fillRect( radar_center_x+radar_radius-5, radar_center_y-radar_radius+5, 50, 3,  GxEPD_BLACK);
*/

#if defined(USE_EPD_TASK)
    /* a signal to background EPD update task */
      EPD_update_in_progress = EPD_UPDATE_FAST;
//    SoC->Display_unlock();
//    yield();
#else
      display->display(true);
#endif
      forcerefresh = false;

  }
}

void EPD_main_next()
{

}

void EPD_main_prev()
{

}

#endif /* USE_EPAPER */
