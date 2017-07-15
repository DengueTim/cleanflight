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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#ifdef SNATCH

#include "common/maths.h"
#include "common/axis.h"

#include "build/build_config.h"

#include "config/parameter_group.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/time.h"

#include "rx/rx.h"
#include "io/serial.h"
#include "fc/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"

#include "flight/imu.h"

#include "snatch/snatch.h"

/*
 * Blackbox inspired streaming of state to a serial port + 'Override' RX that allows RC control on the same serial port.
 * Intended use is for allowing control from an on-board computer.
 * Unlike Blackbox state is streamed as 'events' which are designed from simple serialisation by dumping the data type to the port.
 * Hence the byte order simple types is important.
 *
 *
 * To enable.
 * - Set up serial RX like S.BUS.
 * - Enable feature SNATCH
 *  -Set up snatch on different serial port to RX:
 * 		serial 2 32768 115200 57600 0 921600
 *
 */
#define SNATCH_RX_CHANNELS 16

typedef enum {
	NO_CHANGE, CHANGE, FALL_BACK
} ChannelDataState;

static serialPort_t *serialPort = NULL;
static rcReadRawDataFnPtr fallbackRcReadRawFunc = nullReadRawRC;
static uint16_t snatchRxData[SNATCH_RX_CHANNELS];
static ChannelDataState cdState = FALL_BACK;
static bool streamState = false;

static void serialReceiveCallback(uint16_t byte);

static void sendFallbackChannelsEvent();
static void sendSnatchChannelsEvent();

static void setChannelDataState(ChannelDataState newCdState) {
	if (newCdState != cdState) {
		if (cdState == FALL_BACK) {
			LED1_ON;
		} else if (newCdState == FALL_BACK) {
			LED1_OFF;
		}
		cdState = newCdState;
	}
}

static uint16_t snatchReadRawRC(const struct rxRuntimeConfig_s *rxRuntimeConfig, uint8_t chan) {
	UNUSED(rxRuntimeConfig);
	if (cdState == FALL_BACK) {
		return fallbackRcReadRawFunc(rxRuntimeConfig, chan);
	}
	return snatchRxData[chan];
}

bool snatchInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig) {
	UNUSED(rxConfig);

	serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_SNATCH);
	if (!portConfig) {
		indicateFailure(10, 3);
		return false;
	}

	baudRate_e baudRateIndex = portConfig->blackbox_baudrateIndex;
	portOptions_t portOptions = SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1;

	serialPort = openSerialPort(portConfig->identifier, FUNCTION_SNATCH, &serialReceiveCallback, baudRates[baudRateIndex], MODE_RXTX, portOptions);

	if (serialPort == NULL) {
		indicateFailure(11, 3);
		return false;
	}

	for (int i = 0; i < SNATCH_RX_CHANNELS; i++) {
		snatchRxData[i] = 1500;
	}
	// Throttle before mapping.
	snatchRxData[2] = 1000;

	if (rxRuntimeConfig->rcReadRawFn) {
		fallbackRcReadRawFunc = *(rxRuntimeConfig->rcReadRawFn);
		rxRuntimeConfig->rcReadRawFn = snatchReadRawRC;
	}
	rxRuntimeConfig->channelCount = SNATCH_RX_CHANNELS;
	cdState = FALL_BACK;
	LED1_OFF; // LED on when snatch in control.

	return true;
}

uint8_t snatchFrameStatus(timeUs_t currentTimeUs, uint8_t frameStatus) {
	static uint32_t lastTime = 0;

	if (frameStatus & RX_FRAME_COMPLETE) {
		// Send the fall back RX values on every update.
		sendFallbackChannelsEvent(currentTimeUs);
	}

	if (cdState & FALL_BACK) {
		// In fall back.
		return frameStatus;
	}

	if (frameStatus & RX_FRAME_FAILSAFE) {
		// The fall back RX must be working.
		setChannelDataState(FALL_BACK);
		return RX_FRAME_FAILSAFE;
	}

	// TODO.  This method gets call repeatedly.  For fall-back, it should check it's received something recently.
	// uSecs. 100hz
	if (currentTimeUs - lastTime > 10000) {
		lastTime = currentTimeUs;
		// Spoof new data.
		cdState = CHANGE;
	}

	switch (cdState) {
	case NO_CHANGE:
		return RX_FRAME_PENDING;
	case CHANGE:
		cdState = NO_CHANGE;
		return RX_FRAME_COMPLETE;
	default: // FALL_BACK
		return frameStatus;
	}
}

static uint16_t hexCharToHexValue(uint16_t byte) {
	if (byte >= '0' && byte <= '9') {
		return byte - '0';
	} else if (byte >= 'A' && byte <= 'F') {
		return (byte - 'A') + 10;
	} else if (byte >= 'a' && byte <= 'f') {
		return (byte - 'a') + 10;
	}
	return 0xFFFF;
}
static uint8_t POWERS_OF_TEN[] = { 100, 10, 1 };

static void serialReceiveCallback(uint16_t byte) {
	static uint16_t value = 0;
	static int8_t channel = -1; // 0 to 15
	static int8_t valueByteCount = 0; // 0 to 2.

	if (byte == '+') {
		streamState = true;
		return;
	} else if (byte == '-') {
		streamState = false;
		return;
	} else if (byte == '?') {
		sendSnatchChannelsEvent(micros());
		return;
	} else if (byte == '<') {
		// Explicitly fall back
		setChannelDataState(FALL_BACK);
		// fall through and reset
	} else if (byte == '>') {
		// Take control.
		setChannelDataState(CHANGE);
	} else {
		uint16_t hex = hexCharToHexValue(byte);
		if (channel >= 0) {
			if (hex <= 9) {
				value += hex * POWERS_OF_TEN[valueByteCount++];
				if (valueByteCount <= 2) {
					return;
				}
				// save value
				snatchRxData[channel] = value + 1000;
				if (cdState != FALL_BACK) {
					setChannelDataState(CHANGE);
				}
				// fall through and reset
			}
		} else if (hex <= 0x0F) {
			channel = hex;
			return;
		}
	}

// Generally, if it's bad input we also want to reset local parsing state.
	value = 0;
	channel = -1;
	valueByteCount = 0;
}

/*
 * Everything below is about streaming state out of the FC.
 *
 *
 *
 */

#define SNATCH_EVENT_IMU_CHAR 'I'
#define SNATCH_EVENT_RX_CHAR 'R'
#define SNATCH_EVENT_STATUS_CHAR 'S'

static uint16_t checksum = 0;
static bool checkSumHighNotLow = false;

static void write8(uint8_t v) {
	if (serialPort != NULL) {
		serialWrite(serialPort, v);
		checksum += ((uint16_t)v) << (checkSumHighNotLow ? 8 : 0);
		checkSumHighNotLow = !checkSumHighNotLow;
	}
}

static void writeCheckSumNl() {
	serialWrite(serialPort, checksum & 0xFF);
	serialWrite(serialPort, (checksum >> 8) & 0xFF);
	checksum = 0;
	checkSumHighNotLow = false;
	serialWrite(serialPort, '\n');
}

static void write16(uint16_t v) {
	write8(v & 0xFF);
	write8((v >> 8) & 0xFF);
}

static void write32(uint32_t v) {
	write8(v & 0xFF);
	write8((v >> 8) & 0xFF);
	write8((v >> 16) & 0xFF);
	write8((v >> 24) & 0xFF);
}

void snatchSendPidLoopEvent(timeUs_t currentTimeUs) {
	// At 115200 this will overflow the tx buffer..
	if (!streamState) {
		return;
	}

	write8(SNATCH_EVENT_IMU_CHAR);
	write32(currentTimeUs);
	write16(attitude.values.roll);
	write16(attitude.values.pitch);
	write16(attitude.values.yaw);
	write16(acc.accSmooth[0]);
	write16(acc.accSmooth[1]);
	write16(acc.accSmooth[2]);
	write32(gyro.gyroADCf[0]);
	write32(gyro.gyroADCf[1]);
	write32(gyro.gyroADCf[2]);
	write32(mag.magADC[0]);
	write32(mag.magADC[1]);
	write32(mag.magADC[2]);
	writeCheckSumNl();
}

static void sendFallbackChannelsEvent(timeUs_t currentTimeUs) {
	if (!streamState) {
		return;
	}

	write8(SNATCH_EVENT_RX_CHAR);
	write32(currentTimeUs);
	for (int chan = 0; chan < SNATCH_RX_CHANNELS; chan++) {
		write16(fallbackRcReadRawFunc(NULL, chan));
	}
	writeCheckSumNl();
}

static void sendSnatchChannelsEvent(timeUs_t currentTimeUs) {
//	if (!streamState) {
//		return;
//	}

	write8(SNATCH_EVENT_STATUS_CHAR);
	write32(currentTimeUs);
	for (int chan = 0; chan < SNATCH_RX_CHANNELS; chan++) {
		write16(snatchRxData[chan]);
	}
	writeCheckSumNl();
}

#endif
