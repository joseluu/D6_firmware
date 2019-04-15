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
#include "D6Board.h"
#include "Loop.h"
#include "Serial.h"
#include "Command.h"


static unsigned long delayFactor = 100;
static long analyzerOffset = 128000;

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
	sendChar(0xFF);
	sendChar(0xFF);
	sendChar(0xFF);
	sendChar(0xFF);
}

void sendMeasurement()
{
	short value = 0;
	char cValue[2] = { 0, 0 };
	if (getADCMeasurement(&value)) {
		memcpy(cValue, &value, 2);
		sendChar(cValue[0]);
		sendChar(cValue[1]);
		sendChar(cValue[0]);
		sendChar(cValue[1]);
	}
}

void execute(Command command)
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
	if (frequencySetup(eTracking, command.frequency,4, range, false)) 
	{
		frequencySetup(eAnalyzer, command.frequency + analyzerOffset, 4, range, true);
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
			if (frequencySetup(eTracking, frequency, 4, range, false)) {
				frequencySetup(eAnalyzer, frequency + analyzerOffset, 4, range, true);
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
		sendChar(119);
		break;

	}
}

