/*
 * EEPROMHelper.cpp
 * Copyright (C) 2016-2022 Linar Yusupov
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

#if defined(EXCLUDE_EEPROM)
void EEPROM_setup() {}
void EEPROM_store() {}
#else

#include "EEPROM.h"
#include "RF.h"
#include "LED.h"
#include "Sound.h"
#include "Bluetooth.h"
#include "../TrafficHelper.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/JSON.h"
#include "Battery.h"

// start reading from the first byte (address 0) of the EEPROM

eeprom_t eeprom_block;
settings_t *settings;

void EEPROM_setup()
{
  //int cmd = EEPROM_EXT_LOAD;

  if (!SoC->EEPROM_begin(sizeof(eeprom_t)))
  {
    Serial.print(F("ERROR: Failed to initialize "));
    Serial.print(sizeof(eeprom_t));
    Serial.println(F(" bytes of EEPROM!"));
    Serial.flush();
    delay(1000000);
  }
  
  /*for (int i = 0; i < sizeof(eeprom_t); i++)
  {
    eeprom_block.raw[i] = EEPROM.read(i);
  }

  if (eeprom_block.field.magic != SOFTRF_EEPROM_MAGIC)
  {
    Serial.println(F("WARNING! User defined settings are not initialized yet. Loading defaults..."));

    EEPROM_defaults();
    cmd = EEPROM_EXT_DEFAULTS;
  }
  else
  {
    Serial.print(F("EEPROM version: "));
    Serial.println(eeprom_block.field.version);

    if (eeprom_block.field.version != SOFTRF_EEPROM_VERSION)
    {
      Serial.println(F("WARNING! Version mismatch of user defined settings. Loading defaults..."));

      EEPROM_defaults();
      cmd = EEPROM_EXT_DEFAULTS;
    }
  }*/
  settings = &eeprom_block.field.settings;
  EEPROM_defaults();
  
  SoC->EEPROM_extension(EEPROM_EXT_LOAD);
  //EEPROM_defaults();
  EEPROM_store();
}

void EEPROM_defaults()
{
  Serial.printf("EEPROM_defaults\n");
  eeprom_block.field.magic = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.settings.rf_protocol = RF_PROTOCOL_LEGACY; //hw_info.model == SOFTRF_MODEL_BRACELET ? RF_PROTOCOL_FANET : hw_info.model == SOFTRF_MODEL_ES ? RF_PROTOCOL_ADSB_1090 : RF_PROTOCOL_LEGACY;
  eeprom_block.field.settings.band = RF_BAND_EU;
  eeprom_block.field.settings.aircraft_type = hw_info.model == SOFTRF_MODEL_BRACELET ? AIRCRAFT_TYPE_STATIC : AIRCRAFT_TYPE_UNKNOWN;
  eeprom_block.field.settings.txpower = hw_info.model == SOFTRF_MODEL_ES ? RF_TX_POWER_OFF : RF_TX_POWER_FULL;
  eeprom_block.field.settings.bluetooth = BLUETOOTH_OFF;
  eeprom_block.field.settings.alarm = TRAFFIC_ALARM_DISTANCE;

  /* This will speed up 'factory' boot sequence on Editions other than Standalone */
  if (hw_info.model == SOFTRF_MODEL_STANDALONE)
  {
    eeprom_block.field.settings.volume = BUZZER_VOLUME_FULL;
    eeprom_block.field.settings.pointer = DIRECTION_NORTH_UP;
  }
  else
  {
    eeprom_block.field.settings.volume = BUZZER_OFF;
    eeprom_block.field.settings.pointer = LED_OFF;
  }

  eeprom_block.field.settings.nmea_g = true;
  eeprom_block.field.settings.nmea_p = false;
  eeprom_block.field.settings.nmea_l = true;
  eeprom_block.field.settings.nmea_s = true;

#if (ARDUINO_ESP32S2_USB && !defined(USE_USB_HOST)) || \
    (defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB))
  eeprom_block.field.settings.nmea_out = NMEA_USB;
#else
  eeprom_block.field.settings.nmea_out = hw_info.model == SOFTRF_MODEL_BADGE ? NMEA_BLUETOOTH : hw_info.model == SOFTRF_MODEL_ACADEMY ? NMEA_USB
                                                                                            : hw_info.model == SOFTRF_MODEL_ES        ? NMEA_OFF
                                                                                            : hw_info.model == SOFTRF_MODEL_LEGO      ? NMEA_USB
                                                                                                                                      : NMEA_UART;
#endif

  eeprom_block.field.settings.gdl90 = hw_info.model == SOFTRF_MODEL_ES ? GDL90_USB : GDL90_OFF;
  eeprom_block.field.settings.d1090 = D1090_OFF;
  eeprom_block.field.settings.json = JSON_OFF;
  eeprom_block.field.settings.stealth = false;
  eeprom_block.field.settings.no_track = false;
  eeprom_block.field.settings.power_save = hw_info.model == SOFTRF_MODEL_BRACELET ? POWER_SAVE_NORECEIVE : POWER_SAVE_NONE;
  eeprom_block.field.settings.freq_corr = 0;
  eeprom_block.field.settings.igc_key[0] = 0;
  eeprom_block.field.settings.igc_key[1] = 0;
  eeprom_block.field.settings.igc_key[2] = 0;
  eeprom_block.field.settings.igc_key[3] = 0;

  if (eeprom_block.field.settings.aircraft_type == AIRCRAFT_TYPE_UNKNOWN)
    eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_PARAGLIDER;

#ifndef EXCLUDE_WIFI
  eeprom_block.field.settings.wifiap = true;
  eeprom_block.field.settings.wifista = true;
  eeprom_block.field.settings.relay_ogn = false;
  eeprom_block.field.settings.relay_rf = false;
  strcpy(eeprom_block.field.settings.ssid, "");
  strcpy(eeprom_block.field.settings.psk, "");
#endif
  strcpy(eeprom_block.field.settings.name, "");
  eeprom_block.field.settings.reportme = true;
  eeprom_block.field.settings.latitude = 0.0;
  eeprom_block.field.settings.longitude = 0.0;
  eeprom_block.field.settings.device_addr = SoC->getChipId() & 0x00FFFFFF;

  SoC->EEPROM_extension(EEPROM_EXT_DEFAULTS);
  
  // BENJAMIN
/*#ifndef EXCLUDE_WIFI
  strcpy(eeprom_block.field.settings.ssid, "W33D");
  strcpy(eeprom_block.field.settings.psk, "champoleon44");
  eeprom_block.field.settings.relay_ogn = true;
  strcpy(eeprom_block.field.settings.name, "Chabottes");
#endif
  eeprom_block.field.settings.nmea_out = 0;
  eeprom_block.field.settings.latitude = 44.64573; //° N, 6-80.0; //1.0; // 90; //44.64392; //89.9; //74.72231;
  eeprom_block.field.settings.longitude = 6.17351; //0;  // 6.17429; //0.0; //-41.76135;
  eeprom_block.field.settings.relay_rf = false;
*/

}
void EEPROM_storeGPS(float latitude,float longitude)
{
  //Serial.println(F("EEPROM_storeGPS()"));
  static unsigned long EEPROM_storeGPS_Time_ms = 0;
  #define EEPROM_STOREGPS_TIMEOUT 180000

  if (!( (EEPROM_storeGPS_Time_ms == 0) || ((millis() - EEPROM_storeGPS_Time_ms) > EEPROM_STOREGPS_TIMEOUT)))
    return;
  if (abs(latitude-settings->latitude)<0.0001&&abs(longitude-settings->longitude)<0.0001)
    return;
  Serial.printf("EEPROM Storing GPS lat=%f lon=%f\n",latitude-settings->latitude,longitude-settings->longitude);

  EEPROM_storeGPS_Time_ms=millis();
  settings->latitude=latitude;
  settings->longitude=longitude;

  EEPROM_store();
}
void EEPROM_store()
{
  /*for (int i = 0; i < sizeof(eeprom_t); i++)
    EEPROM.write(i, eeprom_block.raw[i]);

  SoC->EEPROM_extension(EEPROM_EXT_STORE);
  
  EEPROM_commit();
  */
  SoC->EEPROM_extension(EEPROM_EXT_STORE);
}

#endif /* EXCLUDE_EEPROM */
