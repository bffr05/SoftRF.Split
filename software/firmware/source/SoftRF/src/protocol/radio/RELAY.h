/*
 *
 * Relay.h
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

#ifndef RELAY_H
#define RELAY_H
#include "../../driver/RF.h"

bool relay_isvalid(void* buffer);
unsigned int relay_magic(void* buffer);
//int relay_cmd(void* buffer);
unsigned int relay_framesize(void* buffer, unsigned int oframesize=RELAY_MAXPACKET_SIZE);

//extern relay_t last_relay;
bool relay_handled(bool& isrelayed);
void relay_loop(void);

#endif /* RELAY_H */
