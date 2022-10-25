/*
 * Platform_RP2040.cpp
 * Copyright (C) 2022 Linar Yusupov
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

#if defined(ARDUINO_ARCH_RP2040)

#include <SPI.h>
#include <Wire.h>

#include "../system/SoC.h"
#include "../driver/RF.h"
#include "../driver/EEPROM.h"
#include "../driver/LED.h"
#include "../driver/OLED.h"
#include "../driver/Baro.h"
#include "../driver/Sound.h"
#include "../driver/Battery.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/JSON.h"

#include <hardware/watchdog.h>

#if !defined(ARDUINO_ARCH_MBED)
#include "pico/unique_id.h"

#define PICO_ON_DEVICE 1
extern "C" {
#include "pico/binary_info.h"
}
#else
extern "C"
{
#include "hardware/flash.h"
}
#endif /* ARDUINO_ARCH_MBED */

#include <Adafruit_SPIFlash.h>
#include <pico_sleep.h>

#if defined(USE_TINYUSB)
#include "Adafruit_TinyUSB.h"
#endif /* USE_TINYUSB */

#define SOFTRF_DESC "Multifunctional, compatible DIY general aviation proximity awareness system"
#define SOFTRF_URL  "https://github.com/lyusupov/SoftRF"

#if !defined(ARDUINO_ARCH_MBED)
bi_decl(bi_program_name(SOFTRF_IDENT));
bi_decl(bi_program_description(SOFTRF_DESC));
bi_decl(bi_program_version_string(SOFTRF_FIRMWARE_VERSION));
bi_decl(bi_program_build_date_string(__DATE__));
bi_decl(bi_program_url(SOFTRF_URL));
extern char __flash_binary_end;
bi_decl(bi_binary_end((intptr_t)&__flash_binary_end));

bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_GNSS_RX,  "GNSS RX"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_GNSS_TX,  "GNSS TX"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_GNSS_PPS, "GNSS PPS"));
#if SOC_GPIO_PIN_GNSS_RST != SOC_UNUSED_PIN
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_GNSS_RST, "GNSS RST"));
#endif
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_MOSI,     "SX1262 MOSI"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_MISO,     "SX1262 MISO"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_SCK,      "SX1262 SCK"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_SS,       "SX1262 SS"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_RST,      "SX1262 RST"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_BUSY,     "SX1262 BUSY"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_DIO1,     "SX1262 DIO1"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_ANT_RXTX, "RF ANT PWR"));

bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_CONS_RX,  "Console RX"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_CONS_TX,  "Console TX"));

bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_SDA,      "I2C SDA"));
bi_decl(bi_1pin_with_name(SOC_GPIO_PIN_SCL,      "I2C SCL"));
#endif /* ARDUINO_ARCH_MBED */

// SX126x pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = SOC_GPIO_PIN_BUSY,
    .tcxo = LMIC_UNUSED_PIN,
};

#if !defined(EXCLUDE_LED_RING)
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);
#endif /* EXCLUDE_LED_RING */

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));
static bool wdt_is_active = false;

static RP2040_board_id RP2040_board    = RP2040_RAK11300; /* default */

const char *RP2040_Device_Manufacturer = SOFTRF_IDENT;
const char *RP2040_Device_Model        = "Lego Edition";
const uint16_t RP2040_Device_Version   = SOFTRF_USB_FW_VERSION;

#define UniqueIDsize 2

static union {
#if !defined(ARDUINO_ARCH_MBED)
  pico_unique_board_id_t RP2040_unique_flash_id;
#endif /* ARDUINO_ARCH_MBED */
  uint32_t RP2040_chip_id[UniqueIDsize];
};

#if defined(EXCLUDE_EEPROM)
eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
#endif /* EXCLUDE_EEPROM */

#if !defined(ARDUINO_ARCH_MBED)
Adafruit_FlashTransport_RP2040 HWFlashTransport;
Adafruit_SPIFlash QSPIFlash(&HWFlashTransport);

static Adafruit_SPIFlash *SPIFlash = &QSPIFlash;

/// Flash device list count
enum {
  W25Q16JV_IQ_INDEX,
  EXTERNAL_FLASH_DEVICE_COUNT
};

/// List of all possible flash devices used by RP2040 boards
static SPIFlash_Device_t possible_devices[] = {
  [W25Q16JV_IQ_INDEX] = W25Q16JV_IQ,
};
#endif /* ARDUINO_ARCH_MBED */

static bool RP2040_has_spiflash = false;
static uint32_t spiflash_id     = 0;
static bool FATFS_is_mounted    = false;

#if defined(USE_TINYUSB)
// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;
#endif /* USE_TINYUSB */

// file system object from SdFat
FatFileSystem fatfs;

//#define RP2040_JSON_BUFFER_SIZE  1024

//StaticJsonBuffer<RP2040_JSON_BUFFER_SIZE> RP2040_jsonBuffer;

#if !defined(ARDUINO_ARCH_MBED)
// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t RP2040_msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t RP2040_msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
static void RP2040_msc_flush_cb (void)
{
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();
}
#endif /* ARDUINO_ARCH_MBED */

static void RP2040_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  pico_get_unique_board_id(&RP2040_unique_flash_id);
#else
  flash_get_unique_id((uint8_t *)&RP2040_chip_id);
#endif /* ARDUINO_ARCH_MBED */

#if SOC_GPIO_RADIO_LED_TX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_RADIO_LED_TX, OUTPUT);
  digitalWrite(SOC_GPIO_RADIO_LED_TX, ! LED_STATE_ON);
#endif /* SOC_GPIO_RADIO_LED_TX */
#if SOC_GPIO_RADIO_LED_RX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_RADIO_LED_RX, OUTPUT);
  digitalWrite(SOC_GPIO_RADIO_LED_RX, ! LED_STATE_ON);
#endif /* SOC_GPIO_RADIO_LED_RX */

#if defined(USE_TINYUSB)
  USBDevice.setManufacturerDescriptor(RP2040_Device_Manufacturer);
  USBDevice.setProductDescriptor(RP2040_Device_Model);
  USBDevice.setDeviceVersion(RP2040_Device_Version);
#endif /* USE_TINYUSB */

#if !defined(ARDUINO_ARCH_MBED)
  SerialOutput.setRX(SOC_GPIO_PIN_CONS_RX);
  SerialOutput.setTX(SOC_GPIO_PIN_CONS_TX);
  SerialOutput.setFIFOSize(255);
  SerialOutput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

  Serial_GNSS_In.setRX(SOC_GPIO_PIN_GNSS_RX);
  Serial_GNSS_In.setTX(SOC_GPIO_PIN_GNSS_TX);
  Serial_GNSS_In.setFIFOSize(255);

  SPI1.setRX(SOC_GPIO_PIN_MISO);
  SPI1.setTX(SOC_GPIO_PIN_MOSI);
  SPI1.setSCK(SOC_GPIO_PIN_SCK);
  SPI1.setCS(SOC_GPIO_PIN_SS);

  Wire.setSCL(SOC_GPIO_PIN_SCL);
  Wire.setSDA(SOC_GPIO_PIN_SDA);
#endif /* ARDUINO_ARCH_MBED */

#if SOC_GPIO_PIN_GNSS_RST != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_PIN_GNSS_RST, INPUT_PULLUP); // RAK5005-O 3V3_S PWR_EN
#endif

#if SOC_GPIO_PIN_ANT_RXTX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_PIN_ANT_RXTX, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_ANT_RXTX, HIGH);
#endif

#if defined(ARDUINO_RASPBERRY_PI_PICO)
  RP2040_board = (SoC->getChipId() == 0xcf516424) ?
                  RP2040_WEACT : RP2040_RPIPICO;
#endif /* ARDUINO_RASPBERRY_PI_PICO */

#if !defined(ARDUINO_ARCH_MBED)
  RP2040_has_spiflash = SPIFlash->begin(possible_devices,
                                        EXTERNAL_FLASH_DEVICE_COUNT);
  if (RP2040_has_spiflash) {
    spiflash_id = SPIFlash->getJEDECID();

    uint32_t capacity = spiflash_id & 0xFF;
    if (capacity >= 0x15) { /* equal or greater than 1UL << 21 (2 MiB) */
      hw_info.storage = STORAGE_FLASH;

#if defined(USE_TINYUSB)
      // Set disk vendor id, product id and revision
      // with string up to 8, 16, 4 characters respectively
      usb_msc.setID(RP2040_Device_Manufacturer, "External Flash", "1.0");

      // Set callback
      usb_msc.setReadWriteCallback(RP2040_msc_read_cb,
                                   RP2040_msc_write_cb,
                                   RP2040_msc_flush_cb);

      // Set disk size, block size should be 512 regardless of spi flash page size
      usb_msc.setCapacity(SPIFlash->size()/512, 512);

      // MSC is ready for read/write
      usb_msc.setUnitReady(true);

      usb_msc.begin();
#endif /* USE_TINYUSB */

      FATFS_is_mounted = fatfs.begin(SPIFlash);
    }
  }
#endif /* ARDUINO_ARCH_MBED */

  USBSerial.begin(SERIAL_OUT_BR);

  for (int i=0; i < 20; i++) {if (USBSerial) break; else delay(100);}
}

static void RP2040_post_init()
{
#if 0
    Serial.println();
    Serial.print(F("INFO: SPI FLASH JEDEC ID "));
    Serial.println(spiflash_id, HEX);
#endif
  {
    Serial.println();
    Serial.println(F("SoftRF Lego Edition Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      != RF_IC_NONE       ? F("PASS") : F("FAIL"));
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss    != GNSS_MODULE_NONE ? F("PASS") : F("FAIL"));

    Serial.println();
    Serial.println(F("External components:"));
    Serial.print(F("BARO    : "));
    Serial.println(hw_info.baro    != BARO_MODULE_NONE ? F("PASS") : F("N/A"));
    Serial.print(F("DISPLAY : "));
    Serial.println(hw_info.display != DISPLAY_NONE     ? F("PASS") : F("N/A"));

    Serial.println();
    Serial.println(F("Power-on Self Test is complete."));
    Serial.println();
    Serial.flush();
  }

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  switch (settings->nmea_out)
  {
    case NMEA_UART       :  Serial.println(F("UART"));    break;
    case NMEA_USB        :  Serial.println(F("USB CDC")); break;
    case NMEA_OFF        :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.print(F("GDL90  - "));
  switch (settings->gdl90)
  {
    case GDL90_UART      :  Serial.println(F("UART"));    break;
    case GDL90_USB       :  Serial.println(F("USB CDC")); break;
    case GDL90_OFF       :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.print(F("D1090  - "));
  switch (settings->d1090)
  {
    case D1090_UART      :  Serial.println(F("UART"));    break;
    case D1090_USB       :  Serial.println(F("USB CDC")); break;
    case D1090_OFF       :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.println();
  Serial.flush();

#if defined(USE_OLED)
  OLED_info1();
#endif /* USE_OLED */
}

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;
static unsigned long tx_led_time_marker = 0;
static unsigned long rx_led_time_marker = 0;

#define	LED_BLINK_TIME 100

static void RP2040_loop()
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    watchdog_update();
#endif /* ARDUINO_ARCH_MBED */
  }

#if SOC_GPIO_RADIO_LED_TX != SOC_UNUSED_PIN
  if (digitalRead(SOC_GPIO_RADIO_LED_TX) != LED_STATE_ON) {
    if (tx_packets_counter != prev_tx_packets_counter) {
      digitalWrite(SOC_GPIO_RADIO_LED_TX, LED_STATE_ON);
      prev_tx_packets_counter = tx_packets_counter;
      tx_led_time_marker = millis();
    }
  } else {
    if (millis() - tx_led_time_marker > LED_BLINK_TIME) {
      digitalWrite(SOC_GPIO_RADIO_LED_TX, ! LED_STATE_ON);
      prev_tx_packets_counter = tx_packets_counter;
    }
  }
#endif /* SOC_GPIO_RADIO_LED_TX */

#if SOC_GPIO_RADIO_LED_RX != SOC_UNUSED_PIN
  if (digitalRead(SOC_GPIO_RADIO_LED_RX) != LED_STATE_ON) {
    if (rx_packets_counter != prev_rx_packets_counter) {
      digitalWrite(SOC_GPIO_RADIO_LED_RX, LED_STATE_ON);
      prev_rx_packets_counter = rx_packets_counter;
      rx_led_time_marker = millis();
    }
  } else {
    if (millis() - rx_led_time_marker > LED_BLINK_TIME) {
      digitalWrite(SOC_GPIO_RADIO_LED_RX, ! LED_STATE_ON);
      prev_rx_packets_counter = rx_packets_counter;
    }
  }
#endif /* SOC_GPIO_RADIO_LED_RX */
}

static void RP2040_fini(int reason)
{
  if (RP2040_has_spiflash) {
#if defined(USE_TINYUSB)
    usb_msc.setUnitReady(false);
//  usb_msc.end(); /* N/A */
#endif /* USE_TINYUSB */
  }

#if !defined(ARDUINO_ARCH_MBED)
  if (SPIFlash != NULL) SPIFlash->end();
#endif /* ARDUINO_ARCH_MBED */

#if SOC_GPIO_PIN_ANT_RXTX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_PIN_ANT_RXTX, INPUT);
#endif

#if defined(USE_TINYUSB)
  // Disable USB
  USBDevice.detach();
#endif /* USE_TINYUSB */

  sleep_run_from_xosc();

#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  sleep_goto_dormant_until_edge_high(SOC_GPIO_PIN_BUTTON);
#else
  datetime_t alarm = {0};
  sleep_goto_sleep_until(&alarm, NULL);
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void RP2040_reset()
{
  NVIC_SystemReset();
}

static uint32_t RP2040_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = __builtin_bswap32(RP2040_chip_id[UniqueIDsize - 1]);

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* RP2040_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String RP2040_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String RP2040_getResetReason()
{
  switch (reset_info.reason)
  {
    case REASON_DEFAULT_RST       : return F("DEFAULT");
    case REASON_WDT_RST           : return F("WDT");
    case REASON_EXCEPTION_RST     : return F("EXCEPTION");
    case REASON_SOFT_WDT_RST      : return F("SOFT_WDT");
    case REASON_SOFT_RESTART      : return F("SOFT_RESTART");
    case REASON_DEEP_SLEEP_AWAKE  : return F("DEEP_SLEEP_AWAKE");
    case REASON_EXT_SYS_RST       : return F("EXT_SYS");
    default                       : return F("NO_MEAN");
  }
}

extern "C" void * _sbrk   (int);

static uint32_t RP2040_getFreeHeap()
{
  char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

static long RP2040_random(long howsmall, long howBig)
{
  if(howsmall >= howBig) {
      return howsmall;
  }
  long diff = howBig - howsmall;

  return random(diff) + howsmall;
}

static void RP2040_Sound_test(int var)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
    tone(SOC_GPIO_PIN_BUZZER, 440,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500); delay(600);
  }
}

static void RP2040_Sound_tone(int hz, uint8_t volume)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      tone(SOC_GPIO_PIN_BUZZER, hz, ALARM_TONE_MS);
    } else {
      noTone(SOC_GPIO_PIN_BUZZER);
    }
  }
}

static void RP2040_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void RP2040_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool RP2040_EEPROM_begin(size_t size)
{
#if !defined(EXCLUDE_EEPROM)
  EEPROM.begin(size);
#endif /* EXCLUDE_EEPROM */

  return true;
}

static void RP2040_EEPROM_extension(int cmd)
{
  if (cmd == EEPROM_EXT_LOAD) {
#if !defined(EXCLUDE_EEPROM)
    for (int i = 0; i < sizeof(eeprom_t); i++)
    {
      eeprom_block.raw[i] = EEPROM.read(i);
    }

    if (eeprom_block.field.magic != SOFTRF_EEPROM_MAGIC)
    {
      Serial.println(F("WARNING! User defined settings are not initialized yet. Loading defaults..."));

      EEPROM_defaults();
    }
    else
    {
      Serial.print(F("EEPROM version: "));
      Serial.println(eeprom_block.field.version);

      if (eeprom_block.field.version != SOFTRF_EEPROM_VERSION)
      {
        Serial.println(F("WARNING! Version mismatch of user defined settings. Loading defaults..."));

        EEPROM_defaults();
      }
    }
#else
    EEPROM_defaults();
#endif /* EXCLUDE_EEPROM */

    settings = &eeprom_block.field.settings;

    if ( RP2040_has_spiflash && FATFS_is_mounted ) {
      File file = fatfs.open("/settings.json", FILE_READ);

      if (file) {
        // StaticJsonBuffer<RP2040_JSON_BUFFER_SIZE>
        DynamicJsonBuffer jsonBuffer;
        JsonObject &root = jsonBuffer.parseObject(file);

        if (root.success()) {
          JsonVariant msg_class = root["class"];

          if (msg_class.success()) {
            const char *msg_class_s = msg_class.as<char*>();

            if (!strcmp(msg_class_s,"SOFTRF")) {
              parseSettings  (root);
            }
          }
        }
        file.close();
      }
    }

    if (settings->mode != SOFTRF_MODE_NORMAL
#if !defined(EXCLUDE_TEST_MODE)
        &&
        settings->mode != SOFTRF_MODE_TXRX_TEST
#endif /* EXCLUDE_TEST_MODE */
        ) {
      settings->mode = SOFTRF_MODE_NORMAL;
    }

    if (settings->nmea_out == NMEA_BLUETOOTH ||
        settings->nmea_out == NMEA_UDP       ||
        settings->nmea_out == NMEA_TCP ) {
      settings->nmea_out = NMEA_USB;
    }
    if (settings->gdl90 == GDL90_BLUETOOTH  ||
        settings->gdl90 == GDL90_UDP) {
      settings->gdl90 = GDL90_USB;
    }
    if (settings->d1090 == D1090_BLUETOOTH  ||
        settings->d1090 == D1090_UDP) {
      settings->d1090 = D1090_USB;
    }

#if defined(USE_USB_HOST)
    if (settings->nmea_out != NMEA_OFF) {
      settings->nmea_out = NMEA_UART;
    }
    if (settings->gdl90 != GDL90_OFF) {
      settings->gdl90 = GDL90_UART;
    }
    if (settings->d1090 != D1090_OFF) {
      settings->d1090 = D1090_UART;
    }
#endif /* USE_USB_HOST */
  }
  else if (cmd == EEPROM_EXT_STORE)
  {
#if !defined(EXCLUDE_EEPROM)

    for (int i = 0; i < sizeof(eeprom_t); i++)
      EEPROM.write(i, eeprom_block.raw[i]);

    EEPROM_commit();
#endif
    if (RP2040_has_spiflash && FATFS_is_mounted)
    {
      fatfs.remove("/settings.json");

      File file = fatfs.open("/settings.json", FILE_WRITE);

      if (file)
      {
        DynamicJsonBuffer jsonBuffer;
        JsonObject &root = jsonBuffer.createObject();
        root["class"] = "SOFTRF";
        exportSettings(root);
        root.prettyPrintTo(file);
        file.close();
      }
    }
    break;
  }
}

static void RP2040_SPI_begin()
{
  SPI1.begin();
}

static void RP2040_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);
}

static void RP2040_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte RP2040_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
  Wire.begin();
  /* I2C transaction @ SSD1306_OLED_I2C_ADDR is a part of OLED_setup() */
  Wire.beginTransmission(SSD1306_OLED_I2C_ADDR + 1);
  if (Wire.endTransmission() != 0)
    rval = OLED_setup();
#endif /* USE_OLED */

  return rval;
}

static void RP2040_Display_loop()
{
#if defined(USE_OLED)
  OLED_loop();
#endif /* USE_OLED */
}

static void RP2040_Display_fini(int reason)
{
#if defined(USE_OLED)
  OLED_fini(reason);
#endif /* USE_OLED */
}

static void RP2040_Battery_setup()
{

}

static float RP2040_Battery_param(uint8_t param)
{
  float rval, voltage;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = hw_info.model == SOFTRF_MODEL_LEGO ? BATTERY_THRESHOLD_LIPO   :
                                                BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = hw_info.model == SOFTRF_MODEL_LEGO ? BATTERY_CUTOFF_LIPO      :
                                                BATTERY_CUTOFF_NIMHX2;
    break;

  case BATTERY_PARAM_CHARGE:
    voltage = Battery_voltage();
    if (voltage < Battery_cutoff())
      return 0;

    if (voltage > 4.2)
      return 100;

    if (voltage < 3.6) {
      voltage -= 3.3;
      return (voltage * 100) / 3;
    }

    voltage -= 3.6;
    rval = 10 + (voltage * 150 );
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:

    {
      uint16_t mV = 0;
#if SOC_GPIO_PIN_BATTERY != SOC_UNUSED_PIN
      mV = analogRead(SOC_GPIO_PIN_BATTERY);
#endif
      rval = mV * SOC_ADC_VOLTAGE_DIV / 1000.0;
    }
    break;
  }

  return rval;
}

void RP2040_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long RP2040_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool RP2040_Baro_setup() {
  return true;
}

static void RP2040_UATSerial_begin(unsigned long baud)
{

}

static void RP2040_UATModule_restart()
{

}

static void RP2040_WDT_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  watchdog_enable(5000, 1);
#endif /* ARDUINO_ARCH_MBED */
  wdt_is_active = true;
}

static void RP2040_WDT_fini()
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
#endif /* ARDUINO_ARCH_MBED */
    wdt_is_active = false;
  }
}

#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
#include <AceButton.h>
using namespace ace_button;

AceButton button_1(SOC_GPIO_PIN_BUTTON, LOW);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

  switch (eventType) {
    case AceButton::kEventClicked:
    case AceButton::kEventReleased:
#if defined(USE_OLED)
      if (button == &button_1) {
        OLED_Next_Page();
      }
#endif
      break;
    case AceButton::kEventDoubleClicked:
      break;
    case AceButton::kEventLongPressed:
      if (button == &button_1) {
        shutdown(SOFTRF_SHUTDOWN_BUTTON);
//      Serial.println(F("This will never be printed."));
      }
      break;
  }
}

/* Callbacks for push button interrupt */
void onPageButtonEvent() {
  button_1.check();
}
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */

static void RP2040_Button_setup()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_LEGO) {
    int button_pin = SOC_GPIO_PIN_BUTTON;

    // Button(s) uses external pull down resistor.
    pinMode(button_pin, INPUT);

    button_1.init(button_pin, LOW);

    // Configure the ButtonConfig with the event handler, and enable all higher
    // level events.
    ButtonConfig* PageButtonConfig = button_1.getButtonConfig();
    PageButtonConfig->setEventHandler(handleEvent);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
//  PageButtonConfig->setDebounceDelay(15);
    PageButtonConfig->setClickDelay(600);
    PageButtonConfig->setLongPressDelay(2000);

//  attachInterrupt(digitalPinToInterrupt(button_pin), onPageButtonEvent, CHANGE );
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

#if defined(USE_BOOTSEL_BUTTON)
#define BOOTSEL_CLICK_DELAY     200
#define BOOTSEL_LONGPRESS_DELAY 2000

static unsigned long bootsel_time_marker = 0;
static bool prev_bootsel_state = false;
static bool is_bootsel_click = false;
#endif /* USE_BOOTSEL_BUTTON */

static void RP2040_Button_loop()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_LEGO) {
    button_1.check();
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */

#if defined(USE_BOOTSEL_BUTTON)
  if (BOOTSEL) {
    if (!prev_bootsel_state) {
      bootsel_time_marker = millis();
      prev_bootsel_state = true;
    } else {
      if (bootsel_time_marker && !is_bootsel_click &&
          millis() - bootsel_time_marker > BOOTSEL_CLICK_DELAY) {
        is_bootsel_click = true;
      }
      if (bootsel_time_marker &&
          millis() - bootsel_time_marker > BOOTSEL_LONGPRESS_DELAY) {
        shutdown(SOFTRF_SHUTDOWN_BUTTON);
//      Serial.println(F("This will never be printed."));
      }
    }
  } else {
    if (prev_bootsel_state) {
      if (is_bootsel_click) {
#if defined(USE_OLED)
        OLED_Next_Page();
#endif
        is_bootsel_click = false;
      }
      bootsel_time_marker = 0;
      prev_bootsel_state = false;
    }
  }
#endif /* USE_BOOTSEL_BUTTON */
}

static void RP2040_Button_fini()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_LEGO) {
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
    while (digitalRead(SOC_GPIO_PIN_BUTTON) == HIGH);
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */

#if defined(USE_BOOTSEL_BUTTON)
  while (BOOTSEL);
#endif /* USE_BOOTSEL_BUTTON */
}

static void RP2040_USB_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR);
  }
#endif /* ARDUINO_ARCH_MBED */
}

#include <api/RingBuffer.h>

#define USB_TX_FIFO_SIZE (MAX_TRACKING_OBJECTS * 65 + 75 + 75 + 42 + 20)
#define USB_RX_FIFO_SIZE (256)

#if !defined(USBD_CDC_IN_OUT_MAX_SIZE)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)
#endif /* USBD_CDC_IN_OUT_MAX_SIZE */

RingBufferN<USB_TX_FIFO_SIZE> USB_TX_FIFO = RingBufferN<USB_TX_FIFO_SIZE>();
RingBufferN<USB_RX_FIFO_SIZE> USB_RX_FIFO = RingBufferN<USB_RX_FIFO_SIZE>();

static void RP2040_USB_loop()
{
#if !defined(USE_TINYUSB)
  uint8_t buf[USBD_CDC_IN_OUT_MAX_SIZE];
  size_t size;

  while (USBSerial && (size = USBSerial.availableForWrite()) > 0) {
    size_t avail = USB_TX_FIFO.available();

    if (avail == 0) {
      break;
    }

    if (size > avail) {
      size = avail;
    }

    if (size > sizeof(buf)) {
      size = sizeof(buf);
    }

    for (size_t i=0; i < size; i++) {
      buf[i] = USB_TX_FIFO.read_char();
    }

    if (USBSerial) {
      USBSerial.write(buf, size);
    }
  }

  while (USBSerial && USBSerial.available() > 0) {
    if (!USB_RX_FIFO.isFull()) {
      USB_RX_FIFO.store_char(USBSerial.read());
    } else {
      break;
    }
  }
#endif /* USE_TINYUSB */
}

static void RP2040_USB_fini()
{
#if !defined(ARDUINO_ARCH_MBED)
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
#endif /* ARDUINO_ARCH_MBED */
}

static int RP2040_USB_available()
{
  int rval = 0;

#if defined(USE_TINYUSB)
  if (USBSerial) {
    rval = USBSerial.available();
  }
#else
  rval = USB_RX_FIFO.available();
#endif /* USE_TINYUSB */

  return rval;
}

static int RP2040_USB_read()
{
  int rval = -1;

#if defined(USE_TINYUSB)
  if (USBSerial) {
    rval = USBSerial.read();
  }
#else
  rval = USB_RX_FIFO.read_char();
#endif /* USE_TINYUSB */

  return rval;
}

static size_t RP2040_USB_write(const uint8_t *buffer, size_t size)
{
#if !defined(USE_TINYUSB)
  size_t written;

  for (written=0; written < size; written++) {
    if (!USB_TX_FIFO.isFull()) {
      USB_TX_FIFO.store_char(buffer[written]);
    } else {
      break;
    }
  }
  return written;
#else
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

  return rval;
#endif /* USE_TINYUSB */
}

IODev_ops_t RP2040_USBSerial_ops = {
  "RP2040 USB ACM",
  RP2040_USB_setup,
  RP2040_USB_loop,
  RP2040_USB_fini,
  RP2040_USB_available,
  RP2040_USB_read,
  RP2040_USB_write
};
static String JSONSettings()
{
    String out;
    return out;

}

const SoC_ops_t RP2040_ops = {
  SOC_RP2040,
  //"RP2040",
  RP2040_setup,
  RP2040_post_init,
  RP2040_loop,
  RP2040_fini,
  RP2040_reset,
  RP2040_getChipId,
  RP2040_getResetInfoPtr,
  RP2040_getResetInfo,
  RP2040_getResetReason,
  RP2040_getFreeHeap,
  RP2040_random,
  RP2040_Sound_test,
  RP2040_Sound_tone,
  NULL,
  RP2040_WiFi_set_param,
  RP2040_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  RP2040_EEPROM_begin,
  RP2040_EEPROM_extension,
  RP2040_SPI_begin,
  RP2040_swSer_begin,
  RP2040_swSer_enableRx,
  NULL, /* RP2040 has no built-in Bluetooth */
  &RP2040_USBSerial_ops,
  NULL,
  RP2040_Display_setup,
  RP2040_Display_loop,
  RP2040_Display_fini,
  RP2040_Battery_setup,
  RP2040_Battery_param,
  RP2040_GNSS_PPS_Interrupt_handler,
  RP2040_get_PPS_TimeMarker,
  RP2040_Baro_setup,
  RP2040_UATSerial_begin,
  RP2040_UATModule_restart,
  RP2040_WDT_setup,
  RP2040_WDT_fini,
  RP2040_Button_setup,
  RP2040_Button_loop,
  RP2040_Button_fini,
  NULL,
  JSONSettings
};

#endif /* ARDUINO_ARCH_RP2040 */
