/*
 *
 * Relay_Protocol.h
 *
 * Decoder for Extended Squitter 1090 MHz ADS-B radio protocol
 * Copyright (C) 2021-2022 Linar Yusupov
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

#ifndef RELAY_PROTOCOL_H
#define RELAY_PROTOCOL_H

//#define OLD_RELAY_MAGIC         0x1234567
#define RELAY_MAGIC_RELAY 0x1000000
#define RELAY_MAGIC_STATUS 0x2000000
#define RELAY_MAGIC_TONET 0x3000000
#define RELAY_MAGIC_TIME 0x4000000

#define RELAY_MAX_RELAYCOUNT 3

typedef struct
{
  unsigned int magic : 28;
  unsigned int aircraft_type : 4;

  char name[6];
  unsigned int addr : 24;
  unsigned int relay : 24; // 10

  // float         latitude;
  // float         longitude;          //18
  unsigned int relaycount : 2; // 18
  unsigned int lat : 22;       // 0x3FFFFF
  unsigned int speed : 8;      /* ground speed in knots */

  unsigned int lon : 23; // 0x7FFFFF
  unsigned int stealth : 1;
  unsigned int vs : 8; /* feet per secs */

  unsigned int alt : 14;
  unsigned int unused : 2;

  unsigned int timestamp : 6;
  unsigned int no_track : 1;
  unsigned int course : 9; /* CoG */

  // unsigned int  unused         :2;  //25
} __attribute__((packed)) relay_t;

typedef struct
{
  unsigned int magic : 28;
  char relaycount : 2;
  unsigned int unused : 2;
  char relay[RELAY_MAX_RELAYCOUNT * 4];
} __attribute__((packed)) relay_tonet_t;

typedef struct
{
  unsigned int magic : 28;
  unsigned int relay_rf : 1;
  unsigned int relay_net : 1;
  unsigned int unused : 2;
  unsigned int addr : 24;
  char name[6];
  char version;
  unsigned int platform : 4;
  char voltage : 4;
  // float       latitude;
  // float       longitude;
  unsigned int lat : 21;
  unsigned int lon : 22;
  // char        altitude;
} __attribute__((packed)) relay_status_t;

typedef struct
{
  unsigned int magic : 28;
  unsigned char source : 4;
  unsigned int addr : 24;
  unsigned int timestamp;
} __attribute__((packed)) relay_time_t;

#define RELAY_SIZE sizeof(relay_t)
#define RELAY_STATUS_SIZE sizeof(relay_status_t)
#define RELAY_TONET_SIZE sizeof(relay_tonet_t)
#define RELAY_TIME_SIZE sizeof(relay_time_t)

#endif /* RELAY_PROTOCOL_H */
