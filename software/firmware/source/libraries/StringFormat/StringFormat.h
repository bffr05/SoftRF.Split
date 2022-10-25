
/*
 * StringFormat.h
 *
 * Copyright (C) 2017-2022 Benjamin Fels
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

#ifndef STRINGFORMAT_H
#define STRINGFORMAT_H

#include <WString.h>
#include <stdio.h>

#define StringDeclareFormat(str,pmstr, ...) String str; do { static const char fstr[] PROGMEM = pmstr; char rstr[sizeof(pmstr)]; memcpy_P(rstr, fstr, sizeof(rstr)); _StringFormat(str,rstr, ##__VA_ARGS__); } while (0)

#define StringAddFormat(str,pmstr, ...) do { static const char fstr[] PROGMEM = pmstr; char rstr[sizeof(pmstr)]; memcpy_P(rstr, fstr, sizeof(pmstr)); _StringAddFormat(str,rstr, ##__VA_ARGS__); } while (0)
#define StringFormat(str,pmstr, ...) do { static const char fstr[] PROGMEM = pmstr; char rstr[sizeof(pmstr)]; memcpy_P(rstr, fstr, sizeof(pmstr)); _StringFormat(str,rstr, ##__VA_ARGS__); } while (0)
#define StringAdd(str,pmstr) do { static const char fstr[] PROGMEM = pmstr; char rstr[sizeof(pmstr)]; memcpy_P(rstr, fstr, sizeof(pmstr)); str+=rstr; } while (0)

void _StringAddFormat(String& str,const char *format, ...);
void _StringAddFormat2(String& str,const char *format, ...);
void _StringFormat(String& str,const char *format, ...);

#endif