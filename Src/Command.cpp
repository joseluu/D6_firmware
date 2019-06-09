/* Copyright (C) 
 *		2019 Jose Luu
 * additional authors:
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstring>
#include "adc.h"
#include "main.h"
#include "tim.h"
#include "D6Board.h"
#include "Loop.h"
#include "Serial.h"
#include "Command.h"

#define STR(s) #s
char firmwareVersion[] = "D6 Simple Spectrum firmware by Jose F1FGV Version " STR(FIRMWARE_VERSION);

static unsigned long delayFactor = 100;
static long analyzerOffset = 10700000;
static bool trackingGeneratorEnabled = false;

void analyzerStandbyLedOn(bool newState)
{
		blinkD1(newState);
}

void enableTracking(bool newState)
{
	trackingGeneratorEnabled = newState;
	ledD2(newState);
	if (!newState) 
	{
		ADF4351Off(eTracking);
	}
}

bool isTrackingEnabled()
{
	return trackingGeneratorEnabled;
}

void setDelayFactor(unsigned long newDelayFactor)
{
	delayFactor = newDelayFactor;
}
unsigned long getDelayFactor(void)
{
	return delayFactor;
}
void setAnalyzerOffet(long newAnalyzerOffset)
{
	analyzerOffset = newAnalyzerOffset;
}
long getAnalyzerOffset(void)
{
	return analyzerOffset;
}
void sendStatus()
{
	sendChar(FIRMWARE_VERSION);
	sendChar(FIRMWARE_ENGINEERING_VARIANT);
	sendChar(0xFF);
	sendChar(0xFF);
}

void sendMeasurement()
{
	short value = 0;
	char cValue[2] = { 0, 0 };
	if (getADCMeasurement(&value)) {
//		value /= 4 ; // use 10 bits
		memcpy(cValue, &value, 2);
		sendChar(cValue[0]);
		sendChar(cValue[1]);
		sendChar(cValue[0]);
		sendChar(cValue[1]);
	}
}

void executeButtonAction(bool pushedButton)
{
	if (pushedButton) {
		enableTracking(!isTrackingEnabled());
	}
}

void executeCommand(Command command)
{
	unsigned int range;

	switch(command.code)
	{
	case 'f':
#ifdef DEBUG_SPI
	while (true) 
	{
		ledD1(true);
		probeSPI(eTracking);
		probeSPI(eAnalyzer);
		ledD1(false);
		delay_us_DWT(10);
	}
#else
		if (isTrackingEnabled()) {
			if (frequencySetup(eTracking, command.frequency, 4, range, false)) {
				analyzerStandbyLedOn(!frequencySetup(eAnalyzer, command.frequency + analyzerOffset, 4, range, true));
			} else {
				analyzerStandbyLedOn(true);
				ADF4351Off(eAnalyzer);
			}
		} else {
			analyzerStandbyLedOn(!frequencySetup(eAnalyzer, command.frequency + analyzerOffset, 4, range, false));
		}
#endif
		break;
	case 'a':
	case 'b':
	case 'c':
	case 'd':
	case 'w':
	case 'x':
		int i;
		for (i=0;i<command.count;i++)
		{
			unsigned long long  frequency;
			frequency = command.frequency + i*command.step;
			if (isTrackingEnabled()) {
				if (frequencySetup(eTracking, frequency, 4, range, false)) {
					analyzerStandbyLedOn(!frequencySetup(eAnalyzer, frequency + analyzerOffset, 4, range, true));
				} 
			} else {
				analyzerStandbyLedOn(!frequencySetup(eAnalyzer, frequency + analyzerOffset, 4, range, false));
			}
			if (command.code == 'a' || command.code == 'b' || command.code == 'c' || command.code == 'd') {
				delay_us_DWT(command.delay*100);
			} else {
				delay_us_DWT(600);
			}
			sendMeasurement();
		}
		break;
	case 'i': // 6 bytes command: Which ADF, register
		sendSingleRegister((WhichADF)command.which, command.r);
		break;
	case 'm': // send measurement (linear)
	case 'n': // send measurement (log)
		sendMeasurement();
		break;
	case 's': // send status
		sendStatus();
		break;
	case 'v': // version
		sendChar(FIRMWARE_VERSION);
		break;

	}
}

