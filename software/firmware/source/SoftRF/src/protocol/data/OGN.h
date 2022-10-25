/*
 * OGN.h
 * Copyright (C) 2020 Manuel Rösel
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

#include "../../system/SoC.h"
#include "../../../SoftRF.h"

#ifndef OGN_H
#define OGN_H

#define MAXRELAYOGN   10
#define OGN_CONNECT_TIMEOUT 120000UL   /* 2 minutes */
#define OGN_SERVERPING_TIMEOUT 45000UL /* 45 secs */

class OGN;

class OGN
{
public:
    static OGN local;
    static OGN relay[MAXRELAYOGN];

private:
    static String zeroPadding(String data, int len);
    static String getWW(String data);
    static float SnrCalc(float rssi);
    static short AprsPasscode(const char *theCall);

private:
    uint OGNServerIncrement;
    WiFiClient tcp;
    String Servername;
    String User;
    unsigned long ServerPing_Time_ms;
    unsigned long Keepalive_Time_ms;
    unsigned long Connect_Time_ms;
    String message;

    float latitude;
    float longitude;
    float altitude;
    float voltage;
    String info;
    uint32_t timestamp;

public:
    static void loopAll();
    static OGN *find(String User);
    static void station(String User,float latitude, float longitude, float altitude, float voltage, const char *info, uint32_t timestamp);

public:
    OGN();
    OGN(String User);
    ~OGN();
    
    void loop(); 

    bool active();
    void keepalive();
    void login();
    bool report(ufo_t *ufo);
    void position(float latitude, float longitude, float altitude, uint32_t timestamp);
    void status(float voltage, const char *info, uint32_t timestamp);
private:
    bool read();
    void clean();
};
//void OGN_loop();

/*
extern OGN local;

OGN *OGN_Find(String User);
bool OGN_Active(OGN *c);
void OGN_setup();
void OGN_loop();
void OGN_fini();
bool OGN_Report(OGN *c, ufo_t *ufo);
void OGN_ReportThis();
void OGN_position(OGN *c, uint32_t id, float latitude, float longitude, float altitude, uint32_t timestamp);
void OGN_status(OGN *c, uint32_t id, float voltage, const char *info, uint32_t timestamp);
// extern String OGN_message;
*/
#endif /* OGN_H */