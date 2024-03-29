/*
 * TimeHelper.h
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

#ifndef TIMEHELPER_H
#define TIMEHELPER_H

enum
{
  RTC_NONE,
  RTC_PCF8563
};
enum 
{
  TIME_GNSS=1,
  TIME_NTP=2,
  TIME_RELAY=4
};

void Time_setup(void);
void Time_loop(void);
bool  Time_isValid();
void Time_set(char src, int hr,int min,int sec,int dy, int mnth, int yr);
void Time_set(char src, time_t t);
extern char Time_Source;
extern uint32_t Time_Setted;

#endif /* TIMEHELPER_H */
