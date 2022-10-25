
/*
 *
 * Relay.cpp
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
#include "RELAY_Protocol.h"
#include "RELAY.h"
#include <stdint.h>
#include "../../../SoftRF.h"
#include "../../driver/RF.h"
#include "../../driver/EEPROM.h"
#include "../../driver/Battery.h"
#include "../data/OGN.h"
#include "../../system/Time.h"
#include "../../Ufo.h"

uint32_t relay_packets_counter = 0;

unsigned int relay_magic(void *buffer)
{
    return ((relay_t *)buffer)->magic;
}

bool relay_isvalid(void *buffer)
{
    return relay_magic(buffer) == RELAY_MAGIC_RELAY || relay_magic(buffer) == RELAY_MAGIC_STATUS || relay_magic(buffer) == RELAY_MAGIC_TONET || relay_magic(buffer) == RELAY_MAGIC_TIME;
}

unsigned int relay_framesize(void *buffer, unsigned int oframesize)
{
    unsigned int framesize;
    switch (relay_magic(buffer))
    {
    case RELAY_MAGIC_RELAY:
        framesize = RELAY_SIZE;
        break;
    case RELAY_MAGIC_STATUS:
        framesize = RELAY_STATUS_SIZE;
        break;
    case RELAY_MAGIC_TONET:
        framesize = RELAY_TONET_SIZE;
        break;
    case RELAY_MAGIC_TIME:
        framesize = RELAY_TIME_SIZE;
        break;
    default:
        framesize = RF_Payload_Size(settings->rf_protocol);
        break;
    }
    if (oframesize < framesize)
        return oframesize;
    Serial.printf("MAGIC %X size=%u oframesize=%u\n", relay_magic(buffer), framesize, oframesize);
    return framesize;
}
bool legacy_decode(void *, uint32_t timestamp, ufo_t *, ufo_t *);
size_t legacy_encode(void *, ufo_t *);

// relay_t last_relay;
bool relay_handled(bool &isrelayed)
{
    isrelayed = false;
    if (!relay_isvalid(RxBuffer))
    {
        while (1)
        {
            if (!Time_isValid())
                break;
            if (!(protocol_decode && (*protocol_decode)((void *)RxBuffer, ThisAircraft.timestamp, &ThisAircraft, &fo)))
                break;
            fo.relayed = 0;
            fo.relay_name[0] = 0;

            if (OGN::local.active())
            {
                OGN::local.report(&fo);
                break;
            }
            if ((!settings->relay_rf) && fo.distance < 4000)
                break;
            relay_t *relay = (relay_t *)TxBuffer;
            relay->magic = RELAY_MAGIC_RELAY;
            relay->aircraft_type = fo.aircraft_type;
            relay->addr = fo.addr;
            relay->relay = ThisAircraft.addr;
            memcpy(relay->name, ThisAircraft.relay_name, sizeof(relay->name));
            relay->lat = ((double)fo.latitude + 90.0) / 180.0 * 0x3FFFFF;
            relay->lon = ((double)fo.longitude + 180.0) / 360.0 * 0x7FFFFF;
            relay->alt = fo.altitude < 0x3FFF ? fo.altitude : 0x3FFF;
            relay->timestamp = ThisAircraft.timestamp % 0x40;
            relay->stealth = fo.stealth;
            relay->no_track = fo.no_track;
            relay->course = fo.course;
            relay->speed = fo.speed < 0x1ff ? fo.speed : 0x1ff;
            if (fo.vs / 5 >= 0x80)
                relay->vs = 0xff;
            else if (fo.vs / 5 <= -0x80)
                relay->vs = 0x00;
            else
                relay->vs = ((int)fo.vs / 5) % 0x80 + 0x80;

            relay->relaycount = 0;
            // if (!settings->relay_rf) //relay once
            //     relay->norelay=1;
#ifdef DUMPRELAYTRAFFIC
            Serial.printf("------------------------------> RELAY name=%.6s relay=%06X count=%u ts=%u addr=%06X(%u|%u|%u) lat=%x lon=%x alt=%x c=%x s=%x vs=%x\n", //  c=%u s=%u vs=%u\n",
                          relay->name, relay->relay, relay->relaycount, relay->timestamp, relay->addr, relay->aircraft_type, relay->stealth, relay->no_track, relay->lat, relay->lon, relay->alt,
                          relay->course, relay->speed, relay->vs); //,relay->course,relay->speed,relay->vs*2);
#endif

            delay(30);
            RF_Transmit(sizeof(*relay), false);
            relay_packets_counter++;
            break;
        }
        return true;
    }
    isrelayed = true;

    switch (relay_magic(RxBuffer))
    {
    case RELAY_MAGIC_RELAY:
    {
        relay_t *relay = (relay_t *)RxBuffer;
        time_t ts;
        if (ThisAircraft.timestamp % 0x40 <= relay->timestamp)
            ts = ThisAircraft.timestamp - ThisAircraft.timestamp % 0x40 - 0x40 + relay->timestamp;
        else
            ts = ThisAircraft.timestamp - ThisAircraft.timestamp % 0x40 + relay->timestamp;

        fo.protocol = RF_PROTOCOL_LEGACY;
        fo.addr = relay->addr;
        fo.addr_type = ADDR_TYPE_FLARM;
        fo.timestamp = ts;
        fo.latitude = ((double)relay->lat) * 180.0 / 0x3FFFFF - 90.0;
        fo.longitude = ((double)relay->lon) * 360.0 / 0x7FFFFF - 180.0;
        fo.altitude = relay->alt;
        fo.speed = relay->speed;
        fo.course = relay->course;
        fo.vs = (((float)relay->vs) - 0x80) * 5;
        fo.aircraft_type = relay->aircraft_type;
        fo.stealth = relay->stealth;
        fo.no_track = relay->no_track;
        fo.relayed = 1;
        memcpy(fo.relay_name, relay->name, sizeof(relay->name));
        fo.relay_name[6] = 0;

        fo.distance = gnss.distanceBetween(ThisAircraft.latitude, ThisAircraft.longitude, fo.latitude, fo.longitude);
        fo.bearing = gnss.courseTo(ThisAircraft.latitude, ThisAircraft.longitude, fo.latitude, fo.longitude);
#ifdef DUMPRELAYTRAFFIC
        Serial.printf("<============================== RELAY name=%.6s relay=%06X count=%u ts=%u addr=%06X(%u|%u|%u) lat=%x lon=%x alt=%x c=%x s=%x vs=%x\n", //  c=%u s=%u vs=%u\n",
                      relay->name, relay->relay, relay->relaycount, relay->timestamp, relay->addr, relay->aircraft_type, relay->stealth, relay->no_track, relay->lat, relay->lon, relay->alt,
                      relay->course, relay->speed, relay->vs);
        /*Serial.printf("<============================== RELAY name=%.6s relay=%06X count=%u ts=%u addr=%06X(%u|%u|%u) lat=%f(%x) lon=%f(%x) alt=%u(%x) c=%u(%x) s=%u(%x) vs=%i(%x)\n", //  c=%u s=%u vs=%u\n",
                      relay->name, relay->relay, relay->relaycount, relay->timestamp, relay->addr, relay->aircraft_type, relay->stealth, relay->no_track, fo.latitude, relay->lat, fo.longitude, relay->lon, fo.altitude, relay->alt,
                      (int)fo.course, relay->course, (int)fo.speed, relay->speed, (int)fo.vs, relay->vs); //,relay->course,relay->speed,relay->vs*2);*/
#endif

        while (1)
        {
            if (fo.distance > MAXDISTANCE)
                break;
            if (!UFO_check(&fo))
                break;
            OGN *c = OGN::find(relay->name);
            if (c)
                if (c->report(&fo))
                    break;
            if ((settings->relay_rf) && (relay->relaycount < 3))
            {
                memcpy(TxBuffer, RxBuffer, sizeof(relay_t));
                relay = (relay_t *)TxBuffer;
                relay->relaycount++;
                delay(30);
                RF_Transmit(sizeof(*relay), false);
                relay_packets_counter++;
            }
            break;
        }
    }
        return true;
    case RELAY_MAGIC_TIME:
    {
        relay_time_t *time = (relay_time_t *)RxBuffer;
#ifdef DUMPRELAYTRAFFIC
        Serial.printf("<============================== RELAY_TIME addr=%06X time=%u src=%x\n",
                      time->addr, time->timestamp, time->source);
#endif
        if (time->source & TIME_RELAY)
            Time_set(TIME_RELAY, (time_t)time->timestamp);
        else
            Time_set(time->source | TIME_RELAY, (time_t)time->timestamp);
    }
        return true;
    case RELAY_MAGIC_TONET:
    {
        relay_tonet_t *tonet = (relay_tonet_t *)RxBuffer;
#ifdef DUMPRELAYTRAFFIC
        Serial.printf("<============================== RELAY_TONET count=%u %02X%02X%02X %02X%02X%02X %02X%02X%02X %02X%02X%02X\n",
                      tonet->relaycount, tonet->relay[2], tonet->relay[1], tonet->relay[0], tonet->relay[5], tonet->relay[4], tonet->relay[3], tonet->relay[8], tonet->relay[7], tonet->relay[6], tonet->relay[11], tonet->relay[10], tonet->relay[9]);
#endif
        while (1)
        {
            if (OGN::local.active())
                break;
            if (tonet->relaycount >= RELAY_MAX_RELAYCOUNT)
                break;
            if (memcmp(&tonet->relay[0], &ThisAircraft.addr, 3) == 0)
                break;
            if (tonet->relaycount >= 1 && memcmp(&tonet->relay[3], &ThisAircraft.addr, 3) == 0)
                break;
            if (tonet->relaycount >= 2 && memcmp(&tonet->relay[6], &ThisAircraft.addr, 3) == 0)
                break;
            memcpy(TxBuffer, RxBuffer, RELAY_TONET_SIZE);
            tonet = (relay_tonet_t *)TxBuffer;
            tonet->relaycount++;
            tonet->relay[tonet->relaycount * 3] = ((char *)&ThisAircraft.addr)[0];
            tonet->relay[tonet->relaycount * 3 + 1] = ((char *)&ThisAircraft.addr)[1];
            tonet->relay[tonet->relaycount * 3 + 2] = ((char *)&ThisAircraft.addr)[2];
#ifdef DUMPRELAYTRAFFIC
            Serial.printf("------------------------------> RELAY_TONET count=%u %02X%02X%02X %02X%02X%02X %02X%02X%02X %02X%02X%02X\n",
                          tonet->relaycount, tonet->relay[2], tonet->relay[1], tonet->relay[0], tonet->relay[5], tonet->relay[4], tonet->relay[3], tonet->relay[8], tonet->relay[7], tonet->relay[6], tonet->relay[11], tonet->relay[10], tonet->relay[9]);
#endif
            RF_Transmit(sizeof(*tonet), false);
            break;
        }
    }
        return true;
    case RELAY_MAGIC_STATUS:
    {
        relay_status_t *status = (relay_status_t *)RxBuffer;

#ifdef DUMPRELAYTRAFFIC
        Serial.printf("<============================== RELAY_STATUS name=%.6s addr=%06X(%u|%u) p=%.7s v=%u.%u volt=%f lat=%f lon=%f\n", //
                      status->name, status->addr, status->relay_rf, status->relay_net, Soc_Name[status->platform], status->version / 10, status->version % 10, status->voltage * 5.0 / 15.0, ((double)status->lat) * 180.0 / 0x1FFFFF - 90.0, ((double)status->lon) * 360.0 / 0x3FFFFF - 180.0);
#endif
        // send ogn
        if (OGN::local.active() && (status->relay_rf && (!status->relay_net)))
        {
            uint32_t timestamp = ThisAircraft.timestamp;

            // char name[7];
            // sprintf(name, "%06X", status->addr);
            char name[sizeof(status->name) + 1];
            name[sizeof(status->name)] = 0;
            strncpy(name, status->name, sizeof(status->name));

            String info = String("v") + status->version / 10 + "." + status->version % 10 + ".SoftRF" + "-" + Soc_Name[status->platform];
            OGN *c = OGN::find(name);
            if (c)
            {
                c->position(status->addr, ((double)status->lat) * 180.0 / 0x1FFFFF - 90.0, ((double)status->lon) * 360.0 / 0x3FFFFF - 180.0, /*status->altitude*5*/ 0.0, timestamp);
                c->status(status->addr, status->voltage * 5.0 / 256.0, info.c_str(), timestamp);
            }
        }
    }
        return true;
    default:
        return true;
    }
}
void relay_status()
{
    if (!OGN::local.active())
        // if (!settings->relay_rf && !OGN_Active(&local))
        return;

    relay_status_t *status = (relay_status_t *)TxBuffer;
    status->magic = RELAY_MAGIC_STATUS;
    status->addr = ThisAircraft.addr & 0x00FFFFFF;
    memcpy(status->name, ThisAircraft.relay_name, sizeof(status->name));
    status->platform = SoC->id;
    status->version = SOFTRF_FIRMWARE_VERSIONNUM * 10;
    status->relay_rf = settings->relay_rf;
    status->relay_net = 1;
    status->voltage = Battery_voltage() * 15.0 / 5.0;
    status->lat = ((double)ThisAircraft.latitude + 90.0) / 180.0 * 0x1FFFFF;
    status->lon = ((double)ThisAircraft.longitude + 180.0) / 360.0 * 0x3FFFFF;
    // status->altitude = ThisAircraft.altitude/50.0;
#ifdef DUMPRELAYTRAFFIC
    Serial.printf("------------------------------> RELAY_STATUS addr=%06X(%u|%u) p=%.7s v=%u.%u volt=%f lat=%f lon=%f\n", // name=%.6s
                  /*status->name,*/ status->addr, status->relay_rf, status->relay_net, Soc_Name[status->platform], status->version / 10, status->version % 10, status->voltage * 5.0 / 15.0, ((double)status->lat) * 180.0 / 0x1FFFFF - 90.0, ((double)status->lon) * 360.0 / 0x3FFFFF - 180.0);
#endif
    RF_Transmit(sizeof(*status), false);
}
void relay_tonet()
{
    if (!OGN::local.active())
        return;

    relay_tonet_t *tonet = (relay_tonet_t *)TxBuffer;
    tonet->magic = RELAY_MAGIC_TONET;
    tonet->relaycount = 0;
    memset(tonet->relay, 0, sizeof(tonet->relay));
    tonet->relay[tonet->relaycount * 3] = ((char *)&ThisAircraft.addr)[0];
    tonet->relay[tonet->relaycount * 3 + 1] = ((char *)&ThisAircraft.addr)[1];
    tonet->relay[tonet->relaycount * 3 + 2] = ((char *)&ThisAircraft.addr)[2];
#ifdef DUMPRELAYTRAFFIC
    Serial.printf("------------------------------> RELAY_TONET count=%u %02X%02X%02X %02X%02X%02X %02X%02X%02X %02X%02X%02X\n",
                  tonet->relaycount, tonet->relay[2], tonet->relay[1], tonet->relay[0], tonet->relay[5], tonet->relay[4], tonet->relay[3], tonet->relay[8], tonet->relay[7], tonet->relay[6], tonet->relay[11], tonet->relay[10], tonet->relay[9]);
#endif
    RF_Transmit(RELAY_TONET_SIZE, false);
}
void relay_time()
{
    if (!Time_isValid())
        return;

    relay_time_t *tim = (relay_time_t *)TxBuffer;
    tim->magic = RELAY_MAGIC_TIME;
    tim->timestamp = now();
    tim->source = Time_Source;
#ifdef DUMPRELAYTRAFFIC
    Serial.printf("------------------------------> RELAY_TIME addr=%06X time=%u src=%x\n",
                  tim->addr, tim->timestamp, tim->source);
#endif
    RF_Transmit(RELAY_TIME_SIZE, false);
}

void relay_loop(void)
{
    static unsigned long relay_Status_Time_ms = 0;
#define RELAY_STATUS_TIMEOUT 14000
    if ((relay_Status_Time_ms == 0) || ((millis() - relay_Status_Time_ms) > RELAY_STATUS_TIMEOUT))
    {
        relay_status();

        relay_Status_Time_ms = millis();
    }
    static unsigned long relay_Tonet_Time_ms = 0;
#define RELAY_TONET_TIMEOUT 33000

    if ((relay_Tonet_Time_ms == 0) || ((millis() - relay_Tonet_Time_ms) > RELAY_TONET_TIMEOUT))
    {
        relay_tonet();

        relay_Tonet_Time_ms = millis();
    }
    static unsigned long relay_Time_Time_ms = 0;
#define RELAY_TIME_TIMEOUT 41000
    if ((relay_Time_Time_ms == 0) || ((millis() - relay_Time_Time_ms) > RELAY_TIME_TIMEOUT))
    {
        relay_time();

        relay_Time_Time_ms = millis();
    }
}
