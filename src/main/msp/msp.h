/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

struct mspPort_s;
typedef void (*mspPostProcessFuncPtr)(struct mspPort_s *); // msp post process function, used for gracefully handling reboots, etc.

void mspInit(void);
bool mspProcessReceivedData(struct mspPort_s *mspPort, uint8_t c);
mspPostProcessFuncPtr mspProcessReceivedCommand(struct mspPort_s *mspPort);

void mspServerPush(mspPort_t *, int, uint8_t *, int);
