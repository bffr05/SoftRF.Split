/*
 * BaroHelper.cpp
 * Copyright (C) 2018-2022 Linar Yusupov
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

#include "Baro.h"
#include "Baro/FILTER.h"

#if defined(EXCLUDE_BMP180) && defined(EXCLUDE_BMP280) && defined(EXCLUDE_MPL3115A2)
byte  Baro_setup()        {return BARO_MODULE_NONE;}
void  Baro_loop()         {}
void  Baro_fini()         {}
float Baro_altitude()     {return 0;}
float Baro_pressure()     {return 0;}
float Baro_temperature()  {return 0;}
#else

#if !defined(EXCLUDE_BMP180)
#include <Adafruit_BMP085.h>
#endif /* EXCLUDE_BMP180 */
#if !defined(EXCLUDE_BMP280)
#include <Adafruit_BMP280.h>
#endif /* EXCLUDE_BMP280 */
#if !defined(EXCLUDE_MPL3115A2)
#include <Adafruit_MPL3115A2.h>
#endif /* EXCLUDE_MPL3115A2 */

#include <TinyGPS++.h>

barochip_ops_t *baro_chip = NULL;

#if !defined(EXCLUDE_BMP180)
Adafruit_BMP085 bmp180;
#endif /* EXCLUDE_BMP180 */
#if !defined(EXCLUDE_BMP280)
Adafruit_BMP280 bmp280;
#endif /* EXCLUDE_BMP280 */
#if !defined(EXCLUDE_MPL3115A2)
Adafruit_MPL3115A2 mpl3115a2 = Adafruit_MPL3115A2();
#endif /* EXCLUDE_MPL3115A2 */


static float Baro_altitude_cache            = 0;
static float Baro_VSpeedFPS_cache              = 0;
static float Baro_pressure_cache            = 0;
static float Baro_temperature_cache         = 0;

static unsigned long BaroAltitudeTimeMarker = 0;
static unsigned long BaroPresTempTimeMarker = 0;
static float prev_pressure_altitude         = 0;

static float Baro_VS[VS_AVERAGING_FACTOR];
static int avg_ndx = 0;

#if !defined(EXCLUDE_BMP180)
static bool bmp180_probe()
{
  return bmp180.begin();
}

static void bmp180_setup()
{
  Serial.print(F("Temperature = "));
  Serial.print(bmp180.readTemperature());
  Serial.println(F(" *C"));
  
  Serial.print(F("Pressure = "));
  Serial.print(bmp180.readPressure());
  Serial.println(F(" Pa"));
  
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print(F("Altitude = "));
  Serial.print(bmp180.readAltitude());
  Serial.println(F(" meters"));
  
  Serial.println();
  delay(500);
}

static void bmp180_fini()
{
  /* TBD */
}

static float bmp180_altitude(float sealevelPressure)
{
  return bmp180.readAltitude(sealevelPressure * 100);
}

static float bmp180_pressure()
{
  return (float) bmp180.readPressure();
}

static float bmp180_temperature()
{
  return bmp180.readTemperature();
}

barochip_ops_t bmp180_ops = {
  BARO_MODULE_BMP180,
  "BMP180",
  bmp180_probe,
  bmp180_setup,
  bmp180_fini,
  bmp180_altitude,
  bmp180_pressure,
  bmp180_temperature
};
#endif /* EXCLUDE_BMP180 */

#if !defined(EXCLUDE_BMP280)
static bool bmp280_probe()
{
  return (
          bmp280.begin(BMP280_ADDRESS,     BMP280_CHIPID) ||
          bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID) ||
          bmp280.begin(BMP280_ADDRESS,     BME280_CHIPID) ||
          bmp280.begin(BMP280_ADDRESS_ALT, BME280_CHIPID)
         );
}

static void bmp280_setup()
{
    Serial.print(F("Temperature = "));
    Serial.print(bmp280.readTemperature());
    Serial.println(F(" *C"));
    
    Serial.print(F("Pressure = "));
    Serial.print(bmp280.readPressure());
    Serial.println(F(" Pa"));

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp280.readAltitude(1013.25)); // this should be adjusted to your local forcase
    Serial.println(F(" m"));
    
    Serial.println();
    delay(500);
}

static void bmp280_fini()
{
    bmp280.reset();
}

static float bmp280_altitude(float sealevelPressure)
{
    return bmp280.readAltitude(sealevelPressure);
}

static float bmp280_pressure()
{
    return bmp280.readPressure();
}

static float bmp280_temperature()
{
    return bmp280.readTemperature();
}

barochip_ops_t bmp280_ops = {
  BARO_MODULE_BMP280,
  "BMP280",
  bmp280_probe,
  bmp280_setup,
  bmp280_fini,
  bmp280_altitude,
  bmp280_pressure,
  bmp280_temperature
};
#endif /* EXCLUDE_BMP280 */

#if !defined(EXCLUDE_MPL3115A2)
static bool mpl3115a2_probe()
{
  return mpl3115a2.begin();
}

static void mpl3115a2_setup()
{
  float pascals = mpl3115a2.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(F(" Inches (Hg)"));

  float altm = mpl3115a2.getAltitude();
  Serial.print(altm); Serial.println(F(" meters"));

  float tempC = mpl3115a2.getTemperature();
  Serial.print(tempC); Serial.println(F("*C"));

  delay(250);
}

static void mpl3115a2_fini()
{
  /* TBD */
}

static float mpl3115a2_altitude(float sealevelPressure)
{
  mpl3115a2.setSeaPressure(sealevelPressure * 100);
  return mpl3115a2.getAltitude();
}

static float mpl3115a2_pressure()
{
  return mpl3115a2.getPressure();
}

static float mpl3115a2_temperature()
{
  return mpl3115a2.getTemperature();
}

barochip_ops_t mpl3115a2_ops = {
  BARO_MODULE_MPL3115A2,
  "MPL3115A2",
  mpl3115a2_probe,
  mpl3115a2_setup,
  mpl3115a2_fini,
  mpl3115a2_altitude,
  mpl3115a2_pressure,
  mpl3115a2_temperature
};
#endif /* EXCLUDE_MPL3115A2 */

bool Baro_probe()
{
  return (
#if !defined(EXCLUDE_BMP180)
           (baro_chip = &bmp180_ops,    baro_chip->probe()) ||
#else
           false                                            ||
#endif /* EXCLUDE_BMP180 */

#if !defined(EXCLUDE_BMP280)
           (baro_chip = &bmp280_ops,    baro_chip->probe()) ||
#else
           false                                            ||
#endif /* EXCLUDE_BMP280 */

#if !defined(EXCLUDE_MPL3115A2)
           (baro_chip = &mpl3115a2_ops, baro_chip->probe())
#else
           false
#endif /* EXCLUDE_MPL3115A2 */
         );
}

byte Baro_setup()
{
  if ( SoC->Baro_setup() && Baro_probe() ) {

    Serial.print(baro_chip->name);
    Serial.println(F(" barometric pressure sensor is detected."));

    baro_chip->setup();

    Baro_pressure_cache    = baro_chip->pressure();
    Baro_temperature_cache = baro_chip->temperature();
    BaroPresTempTimeMarker = millis();

    Baro_altitude_cache    = baro_chip->altitude(1013.25);
    ThisAircraft.pressure_altitude = prev_pressure_altitude = Baro_altitude_cache;
    BaroAltitudeTimeMarker = millis();

    for (int i=0; i<VS_AVERAGING_FACTOR; i++) {
      Baro_VS[i] = 0;
    }

    return baro_chip->type;

  } else {
    baro_chip = NULL;
    Serial.println(F("WARNING! Barometric pressure sensor is NOT detected."));

    return BARO_MODULE_NONE;
  }
}

/*float getVelocityFtPerSec(float altiFeet, unsigned long currentVeloMillis, int averageThisMany){
  static bool firstTimeVelo = true;
  static float prevAlti = 0;
  static unsigned long prevTimeVelo = 0;
  static const int maxVeloData = 31;
  static float VELO[maxVeloData-1] = {};

  if(firstTimeVelo){
    firstTimeVelo = false;
    prevAlti = altiFeet; //initializing
    prevTimeVelo = currentVeloMillis; //initializing
    return 0;
  }
  else{
    if(averageThisMany >= maxVeloData){averageThisMany = maxVeloData-1;}
    if(averageThisMany < 1){averageThisMany = 1;}
    
    for(int i = 1; i < averageThisMany; i++){VELO[i-1] = VELO[i]; }  //shift data to make room for more
    VELO[averageThisMany-1] = (1000.0*((float)altiFeet - (float)prevAlti)) / ((float)currentVeloMillis - (float)prevTimeVelo);  //add new data
    //Serial.print(VELO[averageThisMany-1]); Serial.print(" = ");
    
    prevAlti = altiFeet;
    prevTimeVelo = currentVeloMillis;
    double vsum = 0;
    for(int i = 0; i < averageThisMany; i++){vsum += VELO[i];} //add all data
    float velo = vsum / (float)averageThisMany;  //resulting velo is an average of all averageThisMany# of values
    //Serial.print(vsum); Serial.print(" / "); Serial.print(averageThisMany);
    //Serial.print(" = ["); Serial.print(velo); Serial.println("]");
    
    return velo; 
  }
}*/
float getVelocityFtPerSec(float altiFeet, unsigned long currentVeloMillis){
  static bool firstTimeVelo = true;
  static float prevAlti = 0;
  static unsigned long prevTimeVelo = 0;
  static const int maxVeloData = 31;
  static float VELO[maxVeloData-1] = {};
  if(firstTimeVelo){
    firstTimeVelo = false;
    prevAlti = altiFeet; //initializing
    prevTimeVelo = currentVeloMillis; //initializing
    return 0;
  }
  else{
    float velo = (1000.0*((float)altiFeet - (float)prevAlti)) / ((float)currentVeloMillis - (float)prevTimeVelo);
    prevAlti = altiFeet;
    prevTimeVelo = currentVeloMillis;
    return velo; 
  }
}
FILTER2 FILTER2;
#define ALTITUDE_FILTER_DURATION    1000    // (AVERAGING DURATION: 1ms to 2000ms)

void Baro_loop()
{
  if (baro_chip == NULL) return;

  static int samplesThisSec = 0;    // Used for calculating averaging duration
  static int samplesPerSec = 0;     // Used for displaying samplesPerSec updated every once second
  static unsigned long previousMillis = 0;
  samplesThisSec++; //increment each time the loop cycles
  if(millis() - previousMillis >= 1000){ // Update value of samplesPerSec once every second:
    previousMillis=millis();
    samplesPerSec = samplesThisSec;
    samplesThisSec=0; 
  }

  if (isTimeToBaroAltitude()) {

    /* Draft of pressure altitude and vertical speed calculation */
    Baro_altitude_cache = baro_chip->altitude(1013.25);
    ThisAircraft.pressure_altitude = Baro_altitude_cache;

    /*{
 
      if(ALTITUDE_FILTER_DURATION){
        Baro_altitude_cache = FILTER2.RUNNING_AVERAGE(
          Baro_altitude_cache, 
          samplesPerSec, 
          ALTITUDE_FILTER_DURATION
        );
      }
    }*/
    /*{
      static FILTER3 FILTER3;
      #define VSPEED_FILTER_DURATION       750    // (AVERAGING DURATION: 1ms to 2000ms)
      if(VSPEED_FILTER_DURATION){
          Baro_VSpeedFPS_cache = FILTER3.RUNNING_AVERAGE(
            getVelocityFtPerSec(Baro_altitude_cache, millis()), 
            samplesPerSec, 
            VSPEED_FILTER_DURATION
          );
        }
        else{
          Baro_VSpeedFPS_cache = getVelocityFtPerSec(Baro_altitude_cache, millis());
        }
    }
    ThisAircraft.vs = Baro_VSpeedFPS_cache;
    if (ThisAircraft.vs > -0.1 && ThisAircraft.vs < 0.1) {
      ThisAircraft.vs = 0;
    }
    ThisAircraft.vs *= (_GPS_FEET_PER_METER * 60.0) ; // feet per minute 
*/
    
    Baro_VS[avg_ndx] = (Baro_altitude_cache - prev_pressure_altitude) /
                       (millis() - BaroAltitudeTimeMarker) * 1000;  // in m/s 

    ThisAircraft.vs = 0;
    for (int i=0; i<VS_AVERAGING_FACTOR; i++) {
      ThisAircraft.vs += Baro_VS[i];
    }
    ThisAircraft.vs /= VS_AVERAGING_FACTOR;

    if (ThisAircraft.vs > -0.1 && ThisAircraft.vs < 0.1) {
      ThisAircraft.vs = 0;
    }

    ThisAircraft.vs *= (_GPS_FEET_PER_METER * 60.0) ; // feet per minute 

    prev_pressure_altitude = Baro_altitude_cache;
    avg_ndx = (avg_ndx + 1) % VS_AVERAGING_FACTOR;
    
    BaroAltitudeTimeMarker = millis();

#if 0
    Serial.print(F("P.Alt. = ")); Serial.print(ThisAircraft.pressure_altitude);
    Serial.print(F(" , VS avg. = ")); Serial.println(ThisAircraft.vs);
#endif
  }

  if (isTimeToBaroPresTemp()) {
    Baro_pressure_cache    = baro_chip->pressure();
    /*if(PRESSURE_FILTER_DURATION){
      Baro_pressure_cache = FILTER1.RUNNING_AVERAGE(
        baro_chip->pressure(),
        //MS5611_I2C.readPressure(), 
        20, 
        PRESSURE_FILTER_DURATION
      );
    }
    else{
      Baro_pressure_cache = MS5611.getPressurePa(D1_OSR);
      //pressurePa = MS5611_I2C.readPressure();
    }*/

    Baro_temperature_cache = baro_chip->temperature();
    /*if(TEMPERATURE_FILTER_DURATION){
      Baro_temperature_cache = FILTER4.RUNNING_AVERAGE(
      //(MS5611_I2C.readTemperature()*9/5.0)+32,
        baro_chip->temperature(), 
      20, 
      TEMPERATURE_FILTER_DURATION
      );
      //temperatureF = 77;
    }
    else
      Baro_temperature_cache = baro_chip->temperature();
    */
    BaroPresTempTimeMarker = millis();
  }
}

void Baro_fini()
{
  if (baro_chip != NULL) baro_chip->fini();
}

float Baro_altitude()
{
  return Baro_altitude_cache;
}

float Baro_pressure()
{
  return Baro_pressure_cache;
}

float Baro_temperature()
{
  return Baro_temperature_cache;
}

#endif /* EXCLUDE_BMP180 && EXCLUDE_BMP280 EXCLUDE_MPL3115A2 */
