/* Mode1090, a Mode S messages decoder.
 *
 * BSD 2-Clause License
 *
 * Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
 * Copyright (C) 2017, Thomas Watson
 * Copyright (C) 2021-2022 Linar Yusupov
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MODE_S_DECODER_H
#define __MODE_S_DECODER_H

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#ifndef ARDUINO_ARCH_AVR
#include <sys/time.h>
#else
typedef unsigned long time_t;
#endif

#define MODE_S_ICAO_CACHE_LEN  64 // Power of two required
#define MODE_S_LONG_MSG_BYTES  (112/8)
#define MODE_S_UNIT_FEET       0
#define MODE_S_UNIT_METERS     1

#define MODE_S_DEFAULT_RATE    2000000
#define MODE_S_DEFAULT_FREQ    1090000000

#if !defined(HACKRF_ONE) && !defined(ARDUINO)
#include <unistd.h>

#define MODE_S_INTERACTIVE_TTL 60 /* TTL before being removed */
typedef long long ms_time_t;

#else
#include <TimeLib.h>

#ifdef HACKRF_ONE
#undef time
extern time_t now_C();
#define time(x) now_C()
#endif

#define USE_BYTE_MAG
#define MODE_S_INTERACTIVE_TTL 10 /* TTL before being removed */

#ifdef DFU_MODE
#ifdef MAGLUT_IN_ROM
#undef MAGLUT_IN_ROM
#endif
#ifndef MAG_LUT_128X128
#define MAG_LUT_128X128
#endif
#endif

typedef unsigned long ms_time_t;
#endif

#if defined(USE_BYTE_MAG)
typedef uint8_t mag_t;
#else
typedef uint16_t mag_t;
#endif


/* Structure used to describe an aircraft in iteractive mode. */
struct mode_s_aircraft {
    uint32_t addr;      /* ICAO address */
    int aircraft_type;
    char hexaddr[7];    /* Printable ICAO address */
    char flight[9];     /* Flight number */
    int altitude;       /* Altitude */
    int unit;           /* meters or feet */
    int speed;          /* Velocity computed from EW and NS components. */
    int track;          /* Angle of flight. */
    time_t seen;        /* Time at which the last packet was received. */
    long messages;      /* Number of Mode S messages received. */
    /* Encoded latitude and longitude as extracted by odd and even
     * CPR encoded messages. */
    int odd_cprlat;
    int odd_cprlon;
    int even_cprlat;
    int even_cprlon;
    double lat, lon;    /* Coordinated obtained from CPR encoded data. */
    ms_time_t odd_cprtime, even_cprtime;
    struct mode_s_aircraft *next; /* Next aircraft in our linked list. */
};

// Program state
typedef struct {
  // Internal state
  uint32_t icao_cache[sizeof(uint32_t)*MODE_S_ICAO_CACHE_LEN*2]; // Recently seen ICAO addresses cache

  // Configuration
  int fix_errors; // Single bit error correction if true
  int aggressive; // Aggressive detection algorithm
  int check_crc;  // Only display messages with good CRC

  /* Interactive mode */
  struct mode_s_aircraft *aircrafts;
  int interactive_ttl; /* Interactive mode: TTL before deletion. */
} mode_s_t;

// The struct we use to store information about a decoded message
struct mode_s_msg {
  // Generic fields
  unsigned char msg[MODE_S_LONG_MSG_BYTES]; // Binary message
  int msgbits;                // Number of bits in message
  int msgtype;                // Downlink format #
  int crcok;                  // True if CRC was valid
  uint32_t crc;               // Message CRC
  int errorbit;               // Bit corrected. -1 if no bit corrected.
  int aa1, aa2, aa3;          // ICAO Address bytes 1 2 and 3
  int phase_corrected;        // True if phase correction was applied.

  // DF 11
  int ca;                     // Responder capabilities.

  // DF 17
  int metype;                 // Extended squitter message type.
  int mesub;                  // Extended squitter message subtype.
  int heading_is_valid;
  int heading;
  int aircraft_type;
  int fflag;                  // 1 = Odd, 0 = Even CPR message.
  int tflag;                  // UTC synchronized?
  int raw_latitude;           // Non decoded latitude
  int raw_longitude;          // Non decoded longitude
  char flight[9];             // 8 chars flight number.
  int ew_dir;                 // 0 = East, 1 = West.
  int ew_velocity;            // E/W velocity.
  int ns_dir;                 // 0 = North, 1 = South.
  int ns_velocity;            // N/S velocity.
  int vert_rate_source;       // Vertical rate source.
  int vert_rate_sign;         // Vertical rate sign.
  int vert_rate;              // Vertical rate.
  int velocity;               // Computed from EW and NS velocity.

  // DF4, DF5, DF20, DF21
  int fs;                     // Flight status for DF4,5,20,21
  int dr;                     // Request extraction of downlink request.
  int um;                     // Request extraction of downlink request.
  int identity;               // 13 bits identity (Squawk).

  // Fields used by multiple message types.
  int altitude, unit;
};

typedef void (*mode_s_callback_t)(mode_s_t *self, struct mode_s_msg *mm);

void mode_s_init(mode_s_t *self);
void mode_s_compute_magnitude_vector(unsigned char *data, mag_t *mag, uint32_t size);
void mode_s_detect(mode_s_t *self, mag_t *mag, uint32_t maglen, mode_s_callback_t);
void mode_s_decode(mode_s_t *self, struct mode_s_msg *mm, unsigned char *msg);

struct mode_s_aircraft* interactiveReceiveData(mode_s_t *self, struct mode_s_msg *mm);
void interactiveRemoveStaleAircrafts(mode_s_t *self);

#endif
