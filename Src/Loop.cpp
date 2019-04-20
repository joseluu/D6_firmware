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
#include "D6Board.h"
#include "main.h"
#include "Command.h"
#include "Loop.h"
#include "debouncedButton.h"

#include "Serial.h"

#define SERIAL_BUFFER_SIZE 50
static char bufferOut[SERIAL_BUFFER_SIZE];
static char bufferIn[SERIAL_BUFFER_SIZE];

static SerialOutput SerialOut(&huart1, bufferOut, SERIAL_BUFFER_SIZE);	 // via USB
static SerialInput SerialIn(&huart1, bufferIn, SERIAL_BUFFER_SIZE);

static SerialOutput* pSerialOut;
static SerialInput* pSerialIn;



typedef enum {
	eInteractive,	// command interpreter user friendly
	eDWT,			// DWT mode
	eAD				// analog device firmware
} eInputProtocol_t;

eInputProtocol_t inputProtocolState;
DebouncedButton *button;

void initializeListener()
{
	pSerialOut = &SerialOut;
	pSerialIn = &SerialIn;
	pSerialIn->initialize(NULL); // start receiving DMA and IT NULL means no echo channel
	//pSerialIn->initialize(pSerialOut);  // with echo
	pSerialOut->puts("\r\nD6 ready\r\n");
	inputProtocolState = eDWT;
	button = new DebouncedButton(SW_GPIO_Port,SW_Pin);
	analyzerStandbyLedOn(true);
	DebouncedButton::addButton(button);
}

void doInteractive(char c)
{
}
void doAD(char c)
{
}

void doNWT(char c);

void sendChar(char c)
{
	pSerialOut->putch(c);
}

bool getInputBinary(unsigned long &longValue, unsigned int timeoutMs)
{
	char cNext[2];
	char c;
	unsigned int inputBytes = 0;
	unsigned long inputValue = 0;
	int shiftCount = 0;
	while (inputBytes < sizeof(longValue)) {
		if (pSerialIn->fgetsNonBlocking(cNext, 2) != NULL) {
			cNext[0];
			longValue |= cNext[0]; // LSB first
			longValue <<= 8;
			inputBytes++;
		}
		// check for timeout
	}
}
bool getInputBinary(unsigned char &byteValue, unsigned int timeoutMs)
{
	char cNext[2];
	char c;
	unsigned int inputBytes = 0;
	int inputValue = 0;
	while (inputBytes < sizeof(byteValue)) {
		if (pSerialIn->fgetsNonBlocking(cNext, 2) != NULL) {
			cNext[0];
			byteValue = cNext[0];
			inputBytes++;
		}
		// check for timeout
	}
}
bool getInputInt(long long &value, unsigned int nbDigits, unsigned int timeoutMs)
{
	char cNext[2];
	char c;
	unsigned int inputDigits = 0;
	int inputValue = 0;
	while (inputDigits < nbDigits)
	{
		if (pSerialIn->fgetsNonBlocking(cNext, 2) != NULL) {
			c = cNext[0];
			if (c >= '0' && c <= '9') {
				inputValue *= 10;
				inputValue += (c - '0');
				inputDigits++;
			} 
			else 
			{
				return false;
			}
		}
		// check for timeout
	}
	value = inputValue;
	return true;
}

bool getInputC(char & c)
{
	char cNext[2];
	if (pSerialIn->fgetsNonBlocking(cNext, 2) != NULL) {
		c = cNext[0];
		return true;
	}
	return false;
}

void doListen()
{
	char c;

	do
	{
		bool buttonChanged;
		bool buttonStateDown = button->buttonState(&buttonChanged);
		bool releasedButton = buttonChanged && (!buttonStateDown);
		executeButtonRelease(releasedButton);
	}
	while (!getInputC(c))
		;

	switch (inputProtocolState)
	{
	case eInteractive:
		doInteractive(c);
		break;
	case eDWT:
		doNWT(c);
		break;
	case eAD:
		doAD(c);
		break;
	}
}


typedef enum {
	eFrame=0,
	eCode,
	eArgs
} eState;


#define RESTART_IF_FAILED(parse_expression)  if (!parse_expression) \
												{ \
													state = eFrame; \
												}
void doNWT(char c)
{
	static eState state = eFrame;
	Command command;
	bool bCommandReady = false;

	switch (state) 
	{
	case eFrame:
		if (c == 0x8F) 
		{
			state = eCode;
		}
		break;
	case eCode:
		command.code = c;
		switch (c) 
		{
		case 'a'://sweep with us delay between measurement and audiosens, 27 bytes command, returns log measurement n x 2 x 2 bytes
		case 'b'://sweep with us delay between measurement and audiosens, 27 bytes command, returns linear measurement n x 2 x 2 bytes
			int ff;
			RESTART_IF_FAILED(getInputInt(command.frequency, 9, 0));
			RESTART_IF_FAILED(getInputInt(command.step, 8, 0));
			RESTART_IF_FAILED(getInputInt(command.count, 4, 0));
			RESTART_IF_FAILED(getInputInt(command.delay, 3, 0));
			RESTART_IF_FAILED(getInputInt(command.audio, 2, 0));
			command.step *= 10;
			command.frequency *= 10;
			bCommandReady = true;
			break;
		case 'c'://sweep with us delay between measurement and audiosens, 25 bytes command, returns log measurement n x 2 x 2 bytes
		case 'd'://sweep with us delay between measurement and audiosens, 25 bytes command, returns linear measurement n x 2 x 2 bytes
			RESTART_IF_FAILED(getInputInt(command.frequency, 9, 0));
			RESTART_IF_FAILED(getInputInt(command.step, 8, 0));
			RESTART_IF_FAILED(getInputInt(command.count, 4, 0));
			RESTART_IF_FAILED(getInputInt(command.delay, 3, 0));
			command.step *= 10; 
			command.frequency *= 10;
			bCommandReady = true;
			break;
		case 'e'://pll adjust TODO
			break;
		case 'f'://VFO
			RESTART_IF_FAILED(getInputInt(command.frequency, 9, 0));
			command.step *= 1000; 
			command.frequency *= 10;
			bCommandReady = true;
			break;
		case 'h':// misc parameters
			break;
		case 'i'://read status of misc parameters
			bCommandReady = true;
			break;
		case 'k': // 6 bytes command: Which ADF, register
			RESTART_IF_FAILED(getInputBinary(command.which, 0));
			RESTART_IF_FAILED(getInputBinary(command.r, 0));
			break;
		case 'l': // 22 bytes: Which ADF 6 registers
			RESTART_IF_FAILED(getInputBinary(command.which, 0));
			RESTART_IF_FAILED(getInputBinary(command.r0, 0));
			RESTART_IF_FAILED(getInputBinary(command.r1, 0));
			RESTART_IF_FAILED(getInputBinary(command.r2, 0));
			RESTART_IF_FAILED(getInputBinary(command.r3, 0));
			RESTART_IF_FAILED(getInputBinary(command.r4, 0));
			RESTART_IF_FAILED(getInputBinary(command.r5, 0));
			break;
		case  'm':		//   read measure value now, 1 byte command, returns 4 bytes
			break;
		case  'n':		//   read measure value now (log?), 1 byte command, returns 4 bytes
			break;
		case 'o' ://not implemented: activate SWV relay, 2 byte command o0 or o1, no return
		case 'r' ://not implemented: attenuators setting, 3 bytes commannd, no return
			break;
		case  's':		//  status, 1 byte command 4 bytes return (version, atten, lo-hi of pin4 ADC measure)
			bCommandReady = true;
			break;
		case  'v':		//   get firmware version, 1 byte command, returns 1 byte
			bCommandReady = true;
			break;
		case  'w':		//  sweep AD8361 (linear RMS) 22 bytes command, returns  n x 2 x 2 bytes
		case 'x':		//  sweep AD8307 (log) 22 bytes command, returns  n x 2 x 2 bytes
			RESTART_IF_FAILED(getInputInt(command.frequency, 9, 0));
			RESTART_IF_FAILED(getInputInt(command.step, 8, 0));
			RESTART_IF_FAILED(getInputInt(command.count, 4, 0));
			command.step *= 10; 
			command.frequency *= 10;
			bCommandReady = true;
			break;
		case  'z':		//  counter, 9 bytes command, returns binary on 4 bytes
			RESTART_IF_FAILED(getInputInt(command.count, 4, 0));
			bCommandReady = true;
			break;
		}
	}
	if (bCommandReady)
	{
		executeCommand(command);
		state = eFrame;
	}
}
