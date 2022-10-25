/*
 * TimeHelper.cpp
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

#include "SoC.h"
#include "Time.h"
#include <TimeLib.h>

char Time_Source = 0;
uint32_t Time_Setted =0;
#define TIMESET_TIMEOUT 90000

#if defined(EXCLUDE_WIFI)
void Time_setup()     {}
#else


unsigned int localPort = 2390;      // local port to listen for UDP packets


/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
//const char* ntpServerName = "time.nist.gov";
const String ntpServerName_suffix = ".pool.ntp.org";
const int ntpServerName_Count = 0;

int ntpServerName_Current=0;

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte NTPPacketBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
unsigned long SyncedTime_ms = 0;

// A UDP instance to let us send and receive packets over UDP
WiFiUDP NTP_udp;

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  //Serial.println(F("sending NTP packet..."));
  // set all bytes in the buffer to 0
  memset(NTPPacketBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  NTPPacketBuffer[0] = 0b11100011;   // LI, Version, Mode
  NTPPacketBuffer[1] = 0;     // Stratum, or type of clock
  NTPPacketBuffer[2] = 6;     // Polling Interval
  NTPPacketBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  NTPPacketBuffer[12]  = 49;
  NTPPacketBuffer[13]  = 0x4E;
  NTPPacketBuffer[14]  = 49;
  NTPPacketBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  NTP_udp.beginPacket(address, 123); //NTP requests are to port 123
  NTP_udp.write(NTPPacketBuffer, NTP_PACKET_SIZE);
  NTP_udp.endPacket();
}

void Time_setup()
{
  // Do not attempt to timesync in Soft AP mode
  if (WiFi.getMode() == WIFI_AP) {
    return;
  }

  static unsigned long Time_ms = 0;
#define TIME_TIMEOUT 900000
  if ((Time_ms==0)||((millis() - Time_ms) > TIME_TIMEOUT))
      Time_ms = millis();
  else  
    return;

  int cb = 0;


  Serial.println(F("Starting NTP UDP"));
  NTP_udp.begin(localPort);
  Serial.print(F("Local port: "));
  Serial.println(localPort);

/*
  for (int attempt = 0; attempt <= ntpServerName_Count; attempt++ ) {
    ntpServerName_Current++;
    if (ntpServerName_Current>ntpServerName_Count)
      ntpServerName_Current=1;
    //get a random server from the pool
    String ntpServerName = String(ntpServerName_Current-1) + ntpServerName_suffix;
    WiFi.hostByName(ntpServerName.c_str(), timeServerIP);

    Serial.print('#');
    Serial.print(ntpServerName);
    Serial.print(F(" NTP server's IP address: "));
    Serial.println(timeServerIP);

    sendNTPpacket(timeServerIP); // send an NTP packet to a time server

    // wait to see if a reply is available
    delay(2000);

    cb = NTP_udp.parsePacket();
    if (!cb) {
      Serial.print(F("No response on request #"));
      Serial.println(attempt);
      continue;
    }
    else {
      Serial.print(F("Reply packet received, length="));
      Serial.println(cb);
      // We've received a packet, read the data from it
      NTP_udp.read(NTPPacketBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
      break;
    }
  }

  //NTP_udp.stop();

  if (!cb) {
    Serial.println(F("WARNING! Unable to sync time by NTP."));
    return;
  }

  //the timestamp starts at byte 40 of the received packet and is four bytes,
  // or two words, long. First, esxtract the two words:

  unsigned long highWord = word(NTPPacketBuffer[40], NTPPacketBuffer[41]);
  unsigned long lowWord = word(NTPPacketBuffer[42], NTPPacketBuffer[43]);
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  Serial.print(F("Seconds since Jan 1 1900 = "));
  Serial.println(secsSince1900);

  // now convert NTP time into everyday time:
  Serial.print(F("Unix time = "));
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const unsigned long seventyYears = 2208988800UL;
  // subtract seventy years:
  unsigned long epoch = secsSince1900 - seventyYears;
  // print Unix time:
  Serial.println(epoch);

  setTime((time_t) epoch);

  // print the hour, minute and second:
  Serial.print(F("The UTC time is "));       // UTC is the time at Greenwich Meridian (GMT)
  Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
  Serial.print(':');
  if ( ((epoch % 3600) / 60) < 10 ) {
    // In the first 10 minutes of each hour, we'll want a leading '0'
    Serial.print('0');
  }
  Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
  Serial.print(':');
  if ( (epoch % 60) < 10 ) {
    // In the first 10 seconds of each minute, we'll want a leading '0'
    Serial.print('0');
  }
  Serial.println(epoch % 60); // print the second
  */
}
void Time_loop()
{
  if (WiFi.getMode() == WIFI_AP)
    return;

  static unsigned long StartSyncTime_ms = 0;
#define TIME_TIMEOUTATTEMPT 2000
  if ((StartSyncTime_ms==0)&&((Time_Setted==0)||((millis() - Time_Setted) > TIMESET_TIMEOUT)))
  {
      StartSyncTime_ms=millis()-(TIME_TIMEOUTATTEMPT+10);
  }
  if ((StartSyncTime_ms!=0)&&((millis() - StartSyncTime_ms) > TIME_TIMEOUTATTEMPT))
  {
    ntpServerName_Current++;
    if (ntpServerName_Current>ntpServerName_Count)
      ntpServerName_Current=1;
    //get a random server from the pool
    String ntpServerName = String(ntpServerName_Current-1) + ntpServerName_suffix;
    WiFi.hostByName(ntpServerName.c_str(), timeServerIP);

    Serial.printf("############################# syncing with %s\n",ntpServerName.c_str());

    sendNTPpacket(timeServerIP); // send an NTP packet to a time server

    StartSyncTime_ms=millis();
  }
  int cb =  NTP_udp.parsePacket();
  if (cb == NTP_PACKET_SIZE)
  if (NTP_udp.read(NTPPacketBuffer, NTP_PACKET_SIZE)==NTP_PACKET_SIZE) // read the packet into the buffer
  {
    unsigned long highWord = word(NTPPacketBuffer[40], NTPPacketBuffer[41]);
    unsigned long lowWord = word(NTPPacketBuffer[42], NTPPacketBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    //Serial.printf("Seconds since Jan 1 1900 = %u\n",secsSince1900);

    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears - (millis()-StartSyncTime_ms)/2000;
    // print Unix time:
    //Serial.printf("Unix time = %u\n",epoch);

    Time_set(TIME_NTP,(time_t) epoch);
    Serial.printf("############################# synced The UTC time is %02u:%02u:%02u\n",(epoch  % 86400L) / 3600,(epoch  % 3600) / 60,epoch % 60);       // UTC is the time at Greenwich Meridian (GMT)
    StartSyncTime_ms=0;
    ntpServerName_Current--;
  }

}



#endif /* EXCLUDE_WIFI */


bool  Time_isValid()
{
    if (Time_Source==0)
        return false;
    if (now()<1261440000) // 1261440000 = 40 years = 2010
        return false;
    return true;

}
void Time_set(char src, int hr,int min,int sec,int dy, int mnth, int yr)
{
  static tmElements_t tm;  
  if( yr > 99)
      yr = yr - 1970;
  else
      yr += 30;  
  tm.Year = yr;
  tm.Month = mnth;
  tm.Day = dy;
  tm.Hour = hr;
  tm.Minute = min;
  tm.Second = sec;
  Time_set(src,makeTime(tm));
}
void Time_set(char src, time_t t)
{
  Time_Setted = millis();
  if (t<1261440000) // 1261440000 = 40 years = 2010
    return;
  if (now()>1261440000 && now()<t)
    return;
  if ((src&TIME_RELAY)&&(Time_Source==TIME_GNSS||Time_Source==TIME_NTP))
    return;
  if ((src==TIME_NTP)&&(Time_Source==TIME_GNSS))
    return;
  Serial.printf("############################# time set t=%u ts=%u src=%u psrc=%u\n",millis(),src,t,Time_Source);
  setTime(t);
  Time_Source = src;
}