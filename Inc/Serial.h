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

#pragma once
#ifndef SERIAL_H
#define SERIAL_H

#include "usart.h"


#ifdef Error_Handler
#undef Error_Handler
#endif // Error_Handler

class SerialOutput {
public:
	SerialOutput(UART_HandleTypeDef * pHandle, char * driverBuffer, unsigned int size);
	bool puts(const char * str);
	bool putch(char c);
	bool putsNonBlocking(const char * str);
	void doOutputIT();

	static SerialOutput * channel_1;
	static SerialOutput * channel_2;
	static SerialOutput * channel_3;

	char * driverBuffer;
	unsigned int driverBufferSize;
	bool busy;
	UART_HandleTypeDef *pHandle;
};


class SerialInput {
public:
	SerialInput(UART_HandleTypeDef * pHandle, char * buffer, unsigned int size);
	void initialize(SerialOutput * echoChannel = NULL);
	void doInputIT();
	char * fgets(char * str, int size);
	bool isCharAvailable();
	char * fgetsNonBlocking(char * str, int size);

	static SerialInput * channel_1;
	static SerialInput * channel_2;
	static SerialInput * channel_3;

	SerialOutput * echo;
	uint8_t rxBuffer; // used by DMA
	char inputBuffer[2];
	bool eol;
	bool textMode;
	unsigned int overrun;
	unsigned int nChars;
	char * driverBuffer;
	char * driverBufferNextChar;
	unsigned int driverBufferSize;
	SerialInput * serialObject;
	UART_HandleTypeDef *pHandle;
};


#endif

