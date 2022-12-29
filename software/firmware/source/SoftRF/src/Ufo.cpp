
/*
 * Ufo.cpp
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

#include "Ufo.h"
#include "driver/EEPROM.h"
#include "driver/RF.h"
#include "driver/GNSS.h"
#include "driver/Sound.h"
#include "ui/Web.h"
#include "protocol/radio/Legacy.h"

ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;


bool UFO_check(ufo_t *fo)
{

  int i;

  //Traffic_Update(&fo);

  for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr == fo->addr) {
      if (Container[i].timestamp>fo->timestamp)
        return false;
      // BENJAMIN Traiter les deplacement abusifs
      float distance = gnss.distanceBetween( Container[i].latitude,Container[i].longitude,fo->latitude,fo->longitude);
      int difftime = fo->timestamp==Container[i].timestamp?1:fo->timestamp-Container[i].timestamp;
      float kmh = (distance/1000*3600)/(difftime);

      if (kmh> 500.0&&Container[i].refused_timestamp!=0)
      {
        distance = gnss.distanceBetween( Container[i].refused_latitude,Container[i].refused_longitude,fo->latitude,fo->longitude);
        difftime=fo->timestamp==Container[i].refused_timestamp?1:fo->timestamp-Container[i].refused_timestamp;
        kmh = (distance/1000*3600)/(difftime);
      }
      if (kmh>500.0)
      {
        Container[i].refused_latitude = fo->latitude;
        Container[i].refused_longitude = fo->longitude;
        Container[i].refused_timestamp = fo->timestamp;
        Serial.printf("----------------------->Traffic Failed kmh=%f df=%i plat=%f plon=%f lat=%f lon=%f\n",kmh,difftime,Container[i].latitude,Container[i].longitude,fo->latitude,fo->longitude);
        return false;
      }

      uint8_t alert_bak = Container[i].alert;
      Container[i] = *fo;
      Container[i].alert = alert_bak;
      Container[i].refused_timestamp =0;

      Serial.printf("----------------------->Traffic Updated %06X\n",Container[i].addr);
      return true;
    }
  }
  int max_dist_ndx = 0;
  int min_level_ndx = 0;

  for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
    // BENJAMIN Traitement des vieux echos
    if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
      Container[i] = *fo;
      Container[i].refused_timestamp =0;
      Serial.printf("----------------------->Traffic Added %06X (timestamp) \n",Container[i].addr);
      return true;
    }
    // distance les plus loin
#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
    if  (Container[i].distance > Container[max_dist_ndx].distance)  {
      max_dist_ndx = i;
    }
    // alarm les plus faibles
    if  (Container[i].alarm_level < Container[min_level_ndx].alarm_level)  {
      min_level_ndx = i;
    }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */
  }

#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
  if (fo->alarm_level > Container[min_level_ndx].alarm_level) {
    Container[min_level_ndx] = *fo;
    Container[i].refused_timestamp =0;
    Serial.printf("----------------------->Traffic Added %06X (alarm) \n",Container[i].addr);
    return true;
  }

  if (fo->distance    <  Container[max_dist_ndx].distance &&
      fo->alarm_level >= Container[max_dist_ndx].alarm_level) {
    Container[max_dist_ndx] = *fo;
    Container[i].refused_timestamp =0;
    Serial.printf("----------------------->Traffic Added %06X (distance) \n",Container[i].addr);
    return true;
  }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */
  return false;
}
