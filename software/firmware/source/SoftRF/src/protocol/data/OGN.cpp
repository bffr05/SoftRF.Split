/*
   OGN.cpp
   Copyright (C) 2022 Benjamin Fels and Manuel Rösel

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "OGN.h"

#if defined(EXCLUDE_WIFI)
void OGN_setup(void) {}
bool OGN_loop(void) {}
void OGN_fini() {}
void OGN_Report(ufo_t *ufo) {}
void OGN_ReportThis() {}
bool OGN_Active() { return false; }
void OGN_position(const char *indirect, uint32_t id, float latitude, float longitude, float altitude, uint32_t timestamp)
{
}
void OGN_status(const char *indirect, uint32_t id, float voltage, const char *info, uint32_t timestamp)
{
}
String OGN_message;
#else

#include "../../driver/RF.h"
#include "../../driver/EEPROM.h"
#include "../../driver/Battery.h"
#include "../../driver/Baro.h"
#include "../../TrafficHelper.h"
#include <WiFiClient.h>
#include <string.h>
#include <TimeLib.h>
#include <StringFormat.h>

#ifndef EXCLUDE_WIFI

#define OGNSERVER "relay_aprs.glidernet.org"
#define OGNSERVERBACKUP "glidern%u.glidernet.org"
const uint OGNServerCount = 5;

const uint16_t OGNPort = 14580;

//uint OGNServerIncrement = 1;


OGN OGN::local;
OGN OGN::relay[MAXRELAYOGN];

/*
WiFiClient OGN_tcp;
String OGNServername = "";
String OGNUser;
unsigned long OGN_ServerPing_Time_ms = 0;
String OGN_message;
*/


String OGN::zeroPadding(String data, int len)
{
    if (data.charAt(2) == '.')
        data.remove(len, data.length());

    if (data.charAt(1) == '.')
        data.remove(len - 1, data.length());
    String tmp = "";
    for (int i = data.length(); i < len; i++)
        tmp += "0";
    tmp += data;
    return tmp;
}

String OGN::getWW(String data)
{
    String tmp = data;
    int len = tmp.length();
    tmp.remove(0, len - 1);
    return tmp;
}

float OGN::SnrCalc(float rssi)
{
    float noise = -108.0;
    return rssi - (noise);
}

short OGN::AprsPasscode(const char *theCall)
{
    char rootCall[10];
    char *p1 = rootCall;

    while ((*theCall != '-') && (*theCall != 0) && (p1 < rootCall + 9))
    {
        *p1++ = toupper(*theCall++);
    }

    *p1 = 0;

    short hash = 0x73e2;
    short i = 0;
    short len = strlen(rootCall);
    char *ptr = rootCall;

    while (i < len)
    {
        hash ^= (*ptr++) << 8;
        hash ^= (*ptr++);
        i += 2;
    }

    return hash & 0x7fff;
}
void OGN::clean()
{
    OGNServerIncrement = 1;
    tcp.stop();
    Servername = "";
    User = "";
    Keepalive_Time_ms = ServerPing_Time_ms = 0;
    message = "";
}
OGN::OGN()
{
    clean();
}
OGN::OGN(String pUser)
{
    clean();
    User = pUser;
}
OGN::~OGN()
{
    tcp.stop();
}
bool OGN::active()
{
    return tcp && Servername != "";
}
void OGN::loopAll()
{
    local.loop();
    for (int i=0;i<MAXRELAYOGN;i++)
        relay[i].loop();

}
OGN *OGN::find(String User)
{
    return NULL;
}
void OGN::station(String User,float latitude, float longitude, float altitude, float voltage, const char *info, uint32_t timestamp)
{
    OGN* c = &local;
    if (User!="")
    {
        c = find(User);
        if (!c)
            for (int i=0;i<MAXRELAYOGN;i++)
                if (relay[i].User=="")
                {
                    c = &relay[i];
                    c.clean();
                    c.User = User;
                }
        if (!c)
            return;
        c->latitude = latitude;
        c->longitude = longitude;
        c->altitude = altitude;
        c->voltage = voltage;
        c->info = info;
        c->timestamp = timestamp;
        if (c->active())
        {
            c->status(c->voltage,c->info,c->timestamp);
            c->position(c->latitude,c->longitude,c->altitude,c->timestamp);
        }
    }
}


void OGN::keepalive()
{
    if (!active())
        return;
        // static unsigned long OGN_Keepalive_Time_ms = 0;
#define OGN_KEEPALIVE_TIMEOUT 240000
    if ((Keepalive_Time_ms == 0) || ((millis() - Keepalive_Time_ms) > OGN_KEEPALIVE_TIMEOUT))
    {
        String message;
        StringFormat(message, "#keepalive\n");
        tcp.print(message);
#ifdef DUMPOGNTRAFFIC
        Serial.printf("OGN(%s) Tx ->> %s\n", User, message);
#endif
        Keepalive_Time_ms = millis();
    }
}
void OGN::login()
{
    if (!tcp)
        return;
    /*if (settings->name[0] == 0)
    {
        OGNUser = zeroPadding(String(ThisAircraft.addr, HEX), 6);
        OGNUser.toUpperCase();
        OGNUser = String("srf") + OGNUser;
    }
    else
        OGNUser = settings->name;*/
    String pass = String(AprsPasscode(User.c_str()));

    String message;
    StringFormat(message, "user %s pass %s vers SoftRF-%s %s m/%u\n",
                 User.c_str(), pass.c_str(), Soc_Name[SoC->id], SOFTRF_FIRMWARE_VERSION, MAXDISTANCE);
    tcp.print(message);
#ifdef DUMPOGNTRAFFIC
    Serial.printf("OGN(%s) Tx ->> ", User, message);
#endif

    Servername = "";
    ServerPing_Time_ms = millis();
}


void OGN::position(float latitude, float longitude, float altitude, uint32_t timestamp)
{
    if (!active())
        return;

    float LAT = fabs(latitude);
    float LON = fabs(longitude);
    String alt = zeroPadding(String(int(altitude * 3.28084)), 6);
    String strtimestamp = zeroPadding(String(hour()), 2) + zeroPadding(String(minute()), 2) + zeroPadding(String(second()), 2) + "h";
    String lat_deg = zeroPadding(String(int(LAT)), 2);
    String lat_min = zeroPadding(String((LAT - int(LAT)) * 60, 3), 5);
    String lon_deg = zeroPadding(String(int(LON)), 3);
    String lon_min = zeroPadding(String((LON - int(LON)) * 60, 3), 5);

    String message;
    StringFormat(message, "%s>APRS,TCPIP*,qAC,%s:/%s%s%s%cI%s%s%c&/A=%s\r\n",
                 User.c_str(),
                 Servername.c_str(),
                 strtimestamp.c_str(),
                 lat_deg.c_str(),
                 lat_min.c_str(),
                 latitude < 0 ? 'S' : 'N',
                 lon_deg.c_str(),
                 lon_min.c_str(),
                 longitude < 0 ? 'W' : 'E',
                 alt.c_str());
    tcp.print(message);
#ifdef DUMPOGNTRAFFIC
    Serial.printf("OGN(%s) Tx ->> ", User, message);
#endif
}

void OGN::status(float voltage, const char *info, uint32_t timestamp)
{
    
    if (!active())
        return;
    String strtimestamp = zeroPadding(String(hour()), 2) + zeroPadding(String(minute()), 2) + zeroPadding(String(second()), 2) + "h";
    String voltagestr = String(voltage) + "V";

    String message;
    StringFormat(message, "%s>APRS,TCPIP*,qAC,%s:>%s %s %s\r\n",
                 User.c_str(),
                 Servername.c_str(),
                 strtimestamp.c_str(),
                 info.c_str(),
                 voltagestr.c_str());
    tcp.print(message);
#ifdef DUMPOGNTRAFFIC
    Serial.printf("OGN(%s) Tx ->> ", User, message);
#endif
}

/*void OGN::ReportThis()
{
    if (!OGN_Active(&local))
        return;
    if (!settings->reportme)
        return;
    static unsigned long OGN_ReportThis_Time_ms = 0;
#define OGN_REPORTTHIS_TIMEOUT 3000
    if ((OGN_ReportThis_Time_ms == 0) || ((millis() - OGN_ReportThis_Time_ms) > OGN_REPORTTHIS_TIMEOUT))
    {
        OGN_Report(&local, &ThisAircraft);
        OGN_ReportThis_Time_ms = millis();
    }
}*/

bool OGN::report(ufo_t *ufo)
{
    if (!active())
        return false;

    if (!ufo->stealth)
    {

        String addr = zeroPadding(String(ufo->addr, HEX), 6);

        String Uaddr = addr;
        Uaddr.toUpperCase();
        float LAT = fabs(ufo->latitude);
        float LON = fabs(ufo->longitude);
        String alt = zeroPadding(String(int(ufo->altitude * 3.28084)), 6);
        String timestamp = zeroPadding(String(hour()), 2) + zeroPadding(String(minute()), 2) + zeroPadding(String(second()), 2) + "h";
        String lat_deg = zeroPadding(String(int(LAT)), 2);
        String lat_min = zeroPadding(String((LAT - int(LAT)) * 60, 3), 5);
        String lon_deg = zeroPadding(String(int(LON)), 3);
        String lon_min = zeroPadding(String((LON - int(LON)) * 60, 3), 5);

        const char symbol_table[16] = {'/', '/', '/', '/', '/', '\\', '/', '/', '\\', '/', '/', '/', '/', '/', '/', '\\'}; // 0x79 -> aircraft type 1110 dec 14 & 0x51 -> aircraft type 4
        const char symbol[16] = {'z', '\'', '\'', 'X', 'g', '^', 'g', 'g', '^', '^', 'z', 'O', 'O', '\'', 'z', 'n'};
        String heading = zeroPadding(String(int(ufo->course)), 3);
        String ground_speed = zeroPadding(String(int(ufo->speed)), 3);
        String W_lat = String((LAT - int(LAT)) * 60, 3);
        String W_lon = String((LON - int(LON)) * 60, 3);
        String pos_precision = getWW(W_lat) + getWW(W_lon);
        String sender_details = zeroPadding(String((ufo->stealth << 7) | (ufo->no_track << 6) | (ufo->aircraft_type << 2) | ufo->addr_type, HEX), 2);
        sender_details.toUpperCase();
        String climbrate;
        if (ufo->vs >= 0)
            climbrate = "+" + zeroPadding(String(int(ufo->vs)), 3);
        else
            climbrate = zeroPadding(String(int(ufo->vs)), 3);
        String snr = String(SnrCalc(ufo->rssi), 1);
        String addrtype;

        /*if (ufo==&ThisAircraft)
            addrtype = "";
        else*/
        if (ufo->addr_type == ADDR_TYPE_ICAO)
            addrtype = "ICA";
        else if (ufo->addr_type == ADDR_TYPE_FLARM)
            addrtype = "FLR";
        else if (ufo->addr_type == ADDR_TYPE_ANONYMOUS)
            addrtype = "OGN";
        else if (ufo->addr_type == ADDR_TYPE_P3I)
            addrtype = "P3I";
        else if (ufo->addr_type == ADDR_TYPE_FANET)
            addrtype = "FNT";
        else if (ufo->addr_type == ADDR_TYPE_RANDOM)
            addrtype = "RANDOM";
        String message;

        StringFormat(message, "%s%s>APRS,qAS,%s:/%s%s%s%c%c%s%s%c%c%s/%s/A=%s !W%s! id%s%s %sfpm %sdB\r\n",
                     addrtype.c_str(),
                     Uaddr.c_str(),
                     User.c_str(),
                     timestamp.c_str(),
                     lat_deg.c_str(),
                     lat_min.c_str(),
                     ufo->latitude < 0 ? 'S' : 'N',
                     symbol_table[ufo->aircraft_type],
                     lon_deg.c_str(),
                     lon_min.c_str(),
                     ufo->longitude < 0 ? 'W' : 'E',
                     symbol[ufo->aircraft_type],
                     heading.c_str(),
                     ground_speed.c_str(),
                     alt.c_str(),
                     pos_precision.c_str(),
                     sender_details.c_str(),
                     Uaddr.c_str(),
                     climbrate.c_str(),
                     snr.c_str());
        tcp.print(message);
#ifdef DUMPOGNTRAFFIC
        Serial.printf("OGN(%s) Tx ->> %S\n", User, message);
#endif
    }
    return true;
}
#endif
void OGN::setup(void)
{
    if (!settings->relay_ogn)
        return;
    Serial.println(F("OGN Setup..."));
    if (WiFi.status() != WL_CONNECTED)
        return;
    if (WiFi.gatewayIP() == IPAddress(0, 0, 0, 0))
        return;
    if (WiFi.SSID() == MY_ACCESSPOINT_SSID)
        return;
    String relay_ognserver;
    if (OGNServerIncrement == 0)
        StringFormat(relay_ognserver, OGNSERVER);
    else
        StringFormat(relay_ognserver, OGNSERVERBACKUP, OGNServerIncrement);
    OGNServerIncrement++;
    if (OGNServerIncrement > OGNServerCount)
        OGNServerIncrement = 1;
    Serial.print(F("OGN Connecting... OGNServer="));
    Serial.print(relay_ognserver);
    Serial.println();
    if (!OGN_tcp)
        OGN_tcp.connect(relay_ognserver.c_str(), OGNPort);
    if (!OGN_tcp)
    {
        Serial.println(F("OGN Connect failed..."));
        return;
    }
    Serial.println(F("OGN Connected..."));

    login();
}

#ifndef EXCLUDE_WIFI
bool OGN::read()
{
    // Serial.println(F("OGN Read..."));

    static String messages;

    if (!tcp)
        return false;
    uint8_t buffer[512];
    int readed;
    while ((readed = tcp.read(buffer, sizeof(buffer) - 1)) > 0)
    {
        // Serial.print(F("OGN Readed "));Serial.println(readed);
        buffer[readed] = 0;
        messages += (char *)buffer;
        // Serial.print(F("OGN Readed "));Serial.println(messages);
    }
    if (readed == -1)
    {
        clean();
        return false;
    }

    int indexof;
    if ((indexof = messages.indexOf('\n')) != -1)
    {
        message = messages.substring(0, indexof);
        messages = messages.substring(indexof + 1, messages.length());
#ifdef DUMPOGNTRAFFIC
        Serial.printf("OGN(%s) Rx ->> %s\n", User, message);
#endif
        if (message.startsWith("# aprsc"))
        {
            ServerPing_Time_ms = millis();
        }
        else if (message.startsWith("# logresp"))
        {
            if (message.indexOf("verified") != -1)
            {
                Serial.println(F("OGN Logged in..."));
                String servertag = "server ";
                int pos = message.indexOf(servertag);
                if (pos == -1)
                {
                    Serial.print(F("OGN logresp server failed:"));
                    Serial.println(Servername);
                    clean();
                    return false;
                }
                Servername = message.substring(pos + servertag.length(), message.length());
                Servername.trim();
                Serial.print(F("Servername: "));
                Serial.println(Servername);
            }
            else
            {
                Serial.print(F("OGN Logged in failed:"));
                Serial.println(message);
                clean();
                return false;
            }
        }
        else
            return true;
    }
    return true;
}
#endif

void loop()
{
#ifndef EXCLUDE_WIFI
    if ((!settings->relay_ogn)||(WiFi.status() != WL_CONNECTED)||(WiFi.gatewayIP() == IPAddress(0, 0, 0, 0)))
        return;
        if (!tcp)
        {
            if ((Connect_Time_ms == 0) || ((millis() - Connect_Time_ms) > OGN_CONNECT_TIMEOUT))
            {
                SoC->WDT_fini();
                setup();
                SoC->WDT_setup();
                Connect_Time_ms = millis();
            }
            return false;
        }
        else if ((millis() - ServerPing_Time_ms) > OGN_SERVERPING_TIMEOUT)
        {
            Serial.println(F("OGN Timeoutted"));
            clean();
            return false;
        }
        if (!read())
            return ;
/*
        static unsigned long OGN_Position_Time_ms = 0;
    #define OGN_POSITON_TIMEOUT 120000
        if (OGN_Active())
        if ((OGN_Position_Time_ms == 0) || ((millis() - OGN_Position_Time_ms) > OGN_POSITON_TIMEOUT))
        {
            OGN_position("",ThisAircraft.addr,ThisAircraft.latitude,ThisAircraft.longitude,ThisAircraft.altitude,ThisAircraft.timestamp);
            OGN_status("",ThisAircraft.addr,Battery_voltage(),(String("v") + SOFTRF_FIRMWARE_VERSION + ".SoftRF" + "-" + Soc_Name[SoC->id]).c_str(),ThisAircraft.timestamp);
            OGN_Position_Time_ms = millis();

        }
        OGN_keepalive();
*/      
#endif

    return;
}
/*
void OGN_fini()
{
#ifndef EXCLUDE_WIFI
    Serial.println(F("OGN Fini..."));
    // OGN_tcp.stop();
    // OGNServername = "";
    // OGNUser = "";
    // OGNServerIncrement = 1;

#endif
}
*/
#endif