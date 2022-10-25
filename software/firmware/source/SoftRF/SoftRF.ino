
/*
 * SoftRF(.ino) firmware
 * Copyright (C) 2016-2022 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Credits:
 *   Arduino core for ESP8266 is developed/supported by ESP8266 Community (support-esp8266@esp8266.com)
 *   AVR/Arduino nRF905 Library/Driver is developed by Zak Kemble, contact@zakkemble.co.uk
 *   flarm_decode is developed by Stanislaw Pusep, http://github.com/creaktive
 *   Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 *   "Aircraft" and MAVLink Libraries are developed by Andy Little
 *   TinyGPS++ and PString Libraries are developed by Mikal Hart
 *   Adafruit NeoPixel Library is developed by Phil Burgess, Michael Miller and others
 *   TrueRandom Library is developed by Peter Knight
 *   IBM LMIC and Semtech Basic MAC frameworks for Arduino are maintained by Matthijs Kooijman
 *   ESP8266FtpServer is developed by David Paiva
 *   Lib_crc is developed by Lammert Bies
 *   OGN library is developed by Pawel Jalocha
 *   NMEA library is developed by Timur Sinitsyn, Tobias Simon, Ferry Huberts
 *   ADS-B encoder C++ library is developed by yangbinbin (yangbinbin_ytu@163.com)
 *   Arduino Core for ESP32 is developed by Hristo Gochkov
 *   ESP32 BT SPP library is developed by Evandro Copercini
 *   Adafruit BMP085 library is developed by Limor Fried and Ladyada
 *   Adafruit BMP280 library is developed by Kevin Townsend
 *   Adafruit MPL3115A2 library is developed by Limor Fried and Kevin Townsend
 *   U8g2 monochrome LCD, OLED and eInk library is developed by Oliver Kraus
 *   NeoPixelBus library is developed by Michael Miller
 *   jQuery library is developed by JS Foundation
 *   EGM96 data is developed by XCSoar team
 *   BCM2835 C library is developed by Mike McCauley
 *   SimpleNetwork library is developed by Dario Longobardi
 *   ArduinoJson library is developed by Benoit Blanchon
 *   Flashrom library is part of the flashrom.org project
 *   Arduino Core for TI CC13X0 and CC13X2 is developed by Robert Wessels
 *   EasyLink library is developed by Robert Wessels and Tony Cave
 *   Dump978 library is developed by Oliver Jowett
 *   FEC library is developed by Phil Karn
 *   AXP202X library is developed by Lewis He
 *   Arduino Core for STM32 is developed by Frederic Pillon
 *   TFT library is developed by Bodmer
 *   STM32duino Low Power and RTC libraries are developed by Wi6Labs
 *   Basic MAC library is developed by Michael Kuyper
 *   LowPowerLab SPIFlash library is maintained by Felix Rusu
 *   Arduino Core for ASR6x0x is developed by Aaron Lee (HelTec Automation)
 *   ADXL362 library is developed by Anne Mahaffey
 *   Arduino Core for nRF52 and TinyUSB library are developed by Ha Thach
 *   Arduino-NVM library is developed by Frank Holtz
 *   AceButton library is developed by Brian Park
 *   GxEPD2 library is developed by Jean-Marc Zingg
 *   Adafruit GFX library is developed by Adafruit Industries
 *   U8g2 fonts for Adafruit GFX are developed by Oliver Kraus
 *   Adafruit SPIFlash and SleepyDog libraries are developed by Adafruit Industries
 *   SdFat library is developed by Bill Greiman
 *   Arduino MIDI library is developed by Francois Best (Forty Seven Effects)
 *   Arduino uCDB library is developed by Ioulianos Kakoulidis
 *   Arduino Core for Atmel SAMD and FlashStorage library are developed by Arduino LLC
 *   USB host library 2.0 for Zero/M0/SAMD is developed by gdsports625@gmail.com
 *   EspTinyUSB library is developed by Dariusz Krempa <esp32@esp32.eu.org>
 *   Arduino Core for Raspberry Pi RP2040 is developed by Earle Philhower
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

#include "src/system/OTA.h"
#include "src/system/Time.h"
#include "src/driver/LED.h"
#include "src/driver/GNSS.h"
#include "src/driver/RF.h"
#include "src/driver/Sound.h"
#include "src/driver/EEPROM.h"
#include "src/driver/Battery.h"
#include "src/protocol/data/MAVLink.h"
#include "src/protocol/data/GDL90.h"
#include "src/protocol/data/NMEA.h"
#include "src/protocol/data/D1090.h"
#include "src/protocol/data/JSON.h"
#include "src/protocol/radio/RELAY.h"
#include "src/protocol/data/OGN.h"

#include "src/system/SoC.h"
#include "src/driver/WiFi.h"
#include "src/ui/Web.h"
#include "src/driver/Baro.h"
#include "src/TTNHelper.h"
#include "src/TrafficHelper.h"

#if defined(ENABLE_AHRS)
#include "src/driver/AHRS.h"
#endif /* ENABLE_AHRS */

#if LOGGER_IS_ENABLED
#include "src/system/Log.h"
#endif /* LOGGER_IS_ENABLED */

#if !defined(SERIAL_FLUSH)
#define SERIAL_FLUSH() Serial.flush()
#endif

#define DEBUG 0
#define DEBUG_TIMING 0

#define isTimeToDisplay() (millis() - LEDTimeMarker > 1000)
#define isTimeToExport() (millis() - ExportTimeMarker > 1000)

ufo_t ThisAircraft;

hardware_info_t hw_info = {
    .model = DEFAULT_SOFTRF_MODEL,
    .revision = 0,
    .soc = SOC_NONE,
    .rf = RF_IC_NONE,
    .gnss = GNSS_MODULE_NONE,
    .baro = BARO_MODULE_NONE,
    .display = DISPLAY_NONE,
    .storage = STORAGE_NONE,
    .rtc = RTC_NONE,
    .imu = IMU_NONE,
};

unsigned long LEDTimeMarker = 0;
unsigned long ExportTimeMarker = 0;

void setup()
{
  rst_info *resetInfo;

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  resetInfo = (rst_info *)SoC->getResetInfoPtr();

#if LOGGER_IS_ENABLED
  Logger_setup();
#endif /* LOGGER_IS_ENABLED */

  Serial.println();
  Serial.print(F(SOFTRF_IDENT "-"));
  Serial.print(Soc_Name[SoC->id]);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2015-2022 Linar Yusupov. All rights reserved."));

  SERIAL_FLUSH();

  if (resetInfo)
  {
    Serial.println("");
    Serial.print(F("Reset reason: "));
    Serial.println(resetInfo->reason);
  }
  Serial.println(SoC->getResetReason());
  Serial.print(F("Free heap size: "));
  Serial.println(SoC->getFreeHeap());
  Serial.println(SoC->getResetInfo());
  Serial.println("");

  SERIAL_FLUSH();

  SoC->WDT_fini();

  EEPROM_setup();

  {
    String json = SoC->JSONSettings();
    if (json != "")
      Serial.println(F("JSON Settings:"));
    Serial.println(SoC->JSONSettings());
    Serial.println();
  }

  SoC->Button_setup();
  if (settings->device_addr)
    ThisAircraft.addr = settings->device_addr & 0x00FFFFFF;
  else
    ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;
  Serial.printf("Hardware Address: %06X\n", SoC->getChipId() & 0x00FFFFFF);
  Serial.printf("Aircraft Address: %06X\n", ThisAircraft.addr & 0x00FFFFFF);
  strncpy(ThisAircraft.relay_name, settings->name, sizeof(ThisAircraft.relay_name) - 1);
  if (ThisAircraft.relay_name[0] == 0)
    sprintf(ThisAircraft.relay_name, "%06X", ThisAircraft.addr);
  Serial.printf("Aircraft Relay Name: %s\n", ThisAircraft.relay_name);
  ThisAircraft.aircraft_type = settings->aircraft_type;
  ThisAircraft.addr_type = ADDR_TYPE_FLARM;
  ThisAircraft.latitude = settings->latitude;
  ThisAircraft.longitude = settings->longitude;
  ThisAircraft.geoid_separation = (float)LookupSeparation(
      ThisAircraft.latitude,
      ThisAircraft.longitude);
  local = OGN(ThisAircraft.relay_name);

  SoC->WDT_fini();

  hw_info.rf = RF_setup();

  delay(100);

  hw_info.baro = Baro_setup();
#if defined(ENABLE_AHRS)
  hw_info.imu = AHRS_setup();
#endif /* ENABLE_AHRS */
  hw_info.display = SoC->Display_setup();

#if !defined(EXCLUDE_MAVLINK)
  if (settings->mode == SOFTRF_MODE_UAV)
  {
    Serial.begin(57600);
    MAVLink_setup();
    ThisAircraft.aircraft_type = AIRCRAFT_TYPE_UAV;
  }
  else
#endif /* EXCLUDE_MAVLINK */
  {
    hw_info.gnss = GNSS_setup();
    ThisAircraft.aircraft_type = settings->aircraft_type;
  }
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

  Battery_setup();
  Traffic_setup();

  SoC->swSer_enableRx(false);

  LED_setup();

  WiFi_setup();

  if (SoC->USB_ops)
  {
    SoC->USB_ops->setup();
  }

  if (SoC->Bluetooth_ops)
  {
    SoC->Bluetooth_ops->setup();
  }

  OTA_setup();
  Web_setup();
  NMEA_setup();

#if defined(ENABLE_TTN)
  TTN_setup();
#endif

  delay(1000);

  /* expedite restart on WDT reset */
  if (resetInfo->reason != REASON_WDT_RST)
  {
    LED_test();
  }

  Sound_setup();
  SoC->Sound_test(resetInfo->reason);

  SoC->swSer_enableRx(true);

  SoC->WDT_fini();

  SoC->post_init();

  SoC->WDT_setup();
}
float hz_counter = 0;
void loop()
{
      //Serial.printf("micros()=%u\n",micros());
  
    static uint cycles = 0;
    static uint cyclestartmicros = micros();
    static uint RF_loop_micros =0;
    static uint normal_micros =0;
    static uint Display_loop_micros =0;
    static uint LED_loop_micros =0;
    static uint WiFi_loop_micros =0;
    static uint Web_loop_micros =0;
    static uint OTA_loop_micros =0;
    static uint Bluetooth_ops_micros =0;
    static uint USB_ops_micros =0;
    static uint UART_ops_micros =0;
    static uint Battery_loop_micros =0;
    static uint Button_loop_micros =0;

    if (cycles && !(cycles%100))
    {
      uint cyclestopmicros = micros();
      hz_counter = 1.0f*cycles*1000000.0/(cyclestopmicros-cyclestartmicros);
      Serial.printf("Hz=%f\n",hz_counter);
      Serial.printf("RF_loop_micros=%u normal_micros=%u Display=%u LED%u WiFi=%u\n",RF_loop_micros/cycles,normal_micros/cycles,Display_loop_micros/cycles,LED_loop_micros/cycles,WiFi_loop_micros/cycles);
      Serial.printf("Web=%u OTA=%u Bluetooth=%u USB=%u UART=%u Bat=%u Butten=%u\n",Web_loop_micros/cycles,OTA_loop_micros/cycles,Bluetooth_ops_micros/cycles,USB_ops_micros/cycles,UART_ops_micros/cycles,Battery_loop_micros/cycles,Button_loop_micros/cycles);
      cycles = 0; cyclestartmicros = cyclestopmicros;
      RF_loop_micros =0;
      normal_micros =0;
      Display_loop_micros =0;
      LED_loop_micros =0;
      WiFi_loop_micros =0;
      Web_loop_micros =0;
      OTA_loop_micros =0;
      Bluetooth_ops_micros =0;
      USB_ops_micros =0;
      UART_ops_micros =0;
      Battery_loop_micros =0;
      Button_loop_micros =0;

    }
  
    cycles++;
  // Do common RF stuff first
  uint start = micros();
 

  RF_loop();
  RF_loop_micros += micros()-start;

  start = micros();
  normal();
  normal_micros += micros()-start;

  // Show status info on tiny OLED display
  start = micros();
   SoC->Display_loop();
  Display_loop_micros += micros()-start;

  // battery status LED
  start = micros();
  LED_loop();
  LED_loop_micros += micros()-start;

  // Handle DNS
  start = micros();
  WiFi_loop();
  WiFi_loop_micros += micros()-start;

  // Handle Web
  start = micros();
  Web_loop();
  Web_loop_micros += micros()-start;

  // Handle OTA update.
  start = micros();
  OTA_loop();
  OTA_loop_micros += micros()-start;

#if LOGGER_IS_ENABLED
  Logger_loop();
#endif /* LOGGER_IS_ENABLED */

  SoC->loop();

  if (SoC->Bluetooth_ops)
  {
    start = micros();
    SoC->Bluetooth_ops->loop();
    Bluetooth_ops_micros += micros()-start;
  }

  if (SoC->USB_ops)
  {
    start = micros();
     SoC->USB_ops->loop();
    USB_ops_micros += micros()-start;
   }

  if (SoC->UART_ops)
  {
    start = micros();
    SoC->UART_ops->loop();
    UART_ops_micros += micros()-start;
  }

    start = micros();
  Battery_loop();
    Battery_loop_micros += micros()-start;

    start = micros();
  SoC->Button_loop();
    Button_loop_micros += micros()-start;

#if defined(TAKE_CARE_OF_MILLIS_ROLLOVER)
  /* restart the device when uptime is more than 47 days */
  if (millis() > (47 * 24 * 3600 * 1000UL))
  {
    SoC->reset();
  }
#endif /* TAKE_CARE_OF_MILLIS_ROLLOVER */

  yield();
}

void shutdown(int reason)
{
  SoC->WDT_fini();

  SoC->swSer_enableRx(false);

  Sound_fini();

  NMEA_fini();

  Web_fini();

  if (SoC->Bluetooth_ops)
  {
    SoC->Bluetooth_ops->fini();
  }

  if (SoC->USB_ops)
  {
    SoC->USB_ops->fini();
  }

  WiFi_fini();

  GNSS_fini();

  SoC->Display_fini(reason);

  Baro_fini();

  RF_Shutdown();

  SoC->Button_fini();

  SoC_fini(reason);
}

void normal()
{
  bool success;

  Baro_loop();

#if defined(ENABLE_AHRS)
  AHRS_loop();
#endif /* ENABLE_AHRS */

  GNSS_loop();

  ThisAircraft.timestamp = now();
  if (isValidFix())
  {
    //Serial.printf("isValidFix()\n");

    ThisAircraft.latitude = gnss.location.lat();
    ThisAircraft.longitude = gnss.location.lng();
    ThisAircraft.altitude = gnss.altitude.meters();
    ThisAircraft.course = gnss.course.deg();
    ThisAircraft.speed = gnss.speed.knots();
    ThisAircraft.hdop = (uint16_t)gnss.hdop.value();
    ThisAircraft.geoid_separation = gnss.separation.meters();
    EEPROM_storeGPS(ThisAircraft.latitude, ThisAircraft.longitude);
    if (settings->reportme)
    {
#if !defined(EXCLUDE_EGM96)
      /*
       * When geoidal separation is zero or not available - use approx. EGM96 value
       */
      if (ThisAircraft.geoid_separation == 0.0)
      {
        ThisAircraft.geoid_separation = (float)LookupSeparation(
            ThisAircraft.latitude,
            ThisAircraft.longitude);
        /* we can assume the GPS unit is giving ellipsoid height */
        ThisAircraft.altitude -= ThisAircraft.geoid_separation;
      }
#endif /* EXCLUDE_EGM96 */

      RF_Transmit(RF_Encode(&ThisAircraft), true);
      OGN::station("",ThisAircraft.latitude,ThisAircraft.longitude,ThisAircraft.altitude,Battery_voltage(),(String("v") + SOFTRF_FIRMWARE_VERSION + ".SoftRF" + "-" + Soc_Name[SoC->id]).c_str(),ThisAircraft.timestamp);
    }
  }



  relay_loop();

  bool isrelayed = false;
  success = RF_Receive();
  if (success)
    success = !relay_handled(isrelayed);

  if (success)
  {
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
#ifdef FULLUFOST
    rx_size = rx_size > sizeof(fo.raw) ? sizeof(fo.raw) : rx_size;
    memset(fo.raw, 0, sizeof(fo.raw));
    memcpy(fo.raw, RxBuffer, rx_size);
#endif

#ifdef FULLUFOST

    if (settings->nmea_p)
    {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long)now());
      StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo.raw, rx_size));
      StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }
#endif
  }
#if defined(ENABLE_TTN)
  TTN_loop();
#endif

  if (isValidFix())
  {
    Traffic_loop();
  }

  if (isTimeToDisplay())
  {
    if (isValidFix())
    {
      LED_DisplayTraffic();
    }
    else
    {
      LED_Clear();
    }
    LEDTimeMarker = millis();
  }

  Sound_loop();

  if (isTimeToExport())
  {
    NMEA_Export();
    GDL90_Export();

    if (isValidFix())
    {
      D1090_Export();
    }
    ExportTimeMarker = millis();
  }

  // Handle Air Connect
  NMEA_loop();

  ClearExpired();
}

