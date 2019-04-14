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
	unsigned int overrun;
	unsigned int nChars;
	char * driverBuffer;
	char * driverBufferNextChar;
	unsigned int driverBufferSize;
	SerialInput * serialObject;
	UART_HandleTypeDef *pHandle;
};


#endif

