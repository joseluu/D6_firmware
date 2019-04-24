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

#include <string.h>
#include "main.h"
#include "Serial.h"

SerialInput * SerialInput::ACTIVE_CHANNEL = NULL;
SerialOutput * SerialOutput::ACTIVE_CHANNEL = NULL;

void SerialInput::initialize(SerialOutput * echoChannel)
{
	if (HAL_UART_Receive_DMA(&huart1, &rxBuffer, 1) != HAL_OK) {
		Error_Handler();
	}
	echo = echoChannel;
}

SerialInput::SerialInput(UART_HandleTypeDef * pHandle, char * buffer, unsigned int size)
	: pHandle(pHandle)
	, driverBuffer (buffer)
	, driverBufferSize (size)
{
	driverBufferNextChar = buffer;
	nChars = 0;
	eol = false;
	textMode = false;
	overrun = 0;
	inputBuffer[0] = 0;
	inputBuffer[1] = 0;

	if (pHandle == &ACTIVE_UART) {
		SerialInput::ACTIVE_CHANNEL = this;
	} else {
		Error_Handler();
	}
}

// append terminating 0, returns the newline and clip to size characters
char * SerialInput::fgets(char * str, int size)
{
	while (!eol && ((driverBufferNextChar - driverBuffer) < size)) {
		HAL_Delay(1);
	}
	return fgetsNonBlocking(str, size);
}

bool SerialInput::isCharAvailable()
{
	return driverBufferNextChar != driverBuffer;
}

bool SerialInput::fgetc(unsigned char & c) // non blocking
{
	if ((driverBufferNextChar - driverBuffer) < 1) 
	{
		return false;
	}
	const int sizeUseful = 1; // read one character
	c = driverBuffer[0];
	strncpy(driverBuffer, &driverBuffer[1], nChars - sizeUseful);
	nChars -= sizeUseful;
	driverBufferNextChar = &driverBuffer[nChars];
	return true;
}
char *  SerialInput::fgetsNonBlocking(char * str, int size)
{
	while (!eol && ((driverBufferNextChar - driverBuffer) < size-1)) {
		return NULL;
	}
	int sizeUseful;
	if ((driverBufferNextChar - driverBuffer) >= size-1) {
		sizeUseful = size - 1;
		strncpy(str, driverBuffer, sizeUseful); //toto: verify chars are not lost
		str[sizeUseful] = 0;
		strncpy(driverBuffer, &driverBuffer[sizeUseful], nChars - sizeUseful);
		nChars -= sizeUseful;
		driverBufferNextChar = &driverBuffer[nChars];
		goto cleanup;
	} else {
		sizeUseful = driverBufferNextChar - driverBuffer;
		memcpy(str, driverBuffer, sizeUseful);
		str[sizeUseful] = 0;
		driverBufferNextChar = driverBuffer;
		nChars = 0;
	}
cleanup:
	overrun = 0;
	eol = false;
	return str;
}

void SerialInput::doInputIT(void)
{
	unsigned char c = rxBuffer;
	if (echo) {
		echo->putch(c);
		if (c == '\r') {
			echo->putch('\n');
		}
	}
	if ((driverBufferNextChar - driverBuffer) >= driverBufferSize) {
		overrun++;
	} else {
		if (textMode) {
			if (c == '\n') {
				eol = true;
			}
			if (c == '\r') {
				c = '\n';
				eol = true;
			}
			if (c == 127 || c == 8) {
				 // handle backspace and delete (only when echoing ?)
				if(nChars > 0)
				{
					driverBufferNextChar--;
					nChars--;
				}
			} else {
				*driverBufferNextChar++ = c;
				nChars++;
			}
		} 
		else
		{
			*driverBufferNextChar++ = c;
			nChars++;
		}
	}
	#if 0
	HAL_StatusTypeDef status;
	status = HAL_UART_Receive_IT(pHandle, (uint8_t *)inputBuffer, 1);
	if (status == HAL_BUSY)
	{
		Error_Handler();
	}
	#endif
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	SerialInput * serialObject;
	if (UartHandle == &ACTIVE_UART) {
		serialObject  = SerialInput::ACTIVE_CHANNEL;
	} else {
		Error_Handler();
	}
	__HAL_UART_FLUSH_DRREGISTER(UartHandle); // Clear the buffer to prevent overrun
	serialObject->doInputIT();
}

SerialOutput::SerialOutput(UART_HandleTypeDef * pHandle, char * driverBuffer, unsigned int size)
	: pHandle(pHandle)
	, driverBuffer(driverBuffer)
	, driverBufferSize(size)
{
	busy = false;
	if (pHandle == &ACTIVE_UART) {
		SerialOutput::ACTIVE_CHANNEL = this;
	} else {
		Error_Handler();
	}
}

bool SerialOutput::puts(const char * str)
{
	int statusTransmit;
	busy = true;
	if (strlen(str) >= driverBufferSize) {
	}
	statusTransmit = HAL_UART_Transmit(pHandle, (uint8_t*)str, strlen(str), 1000);
	busy = false;
	return statusTransmit;
}

bool SerialOutput::putch(char c)
{
	int statusTransmit;
	busy = true;
	do 
	{
		statusTransmit = HAL_UART_Transmit(pHandle, (uint8_t*)&c, 1, 10);
	} 
		while (statusTransmit == HAL_BUSY);
	busy = false;
	return statusTransmit;
}


bool SerialOutput::putsNonBlocking(const char * str)
{
	int statusTransmit;
	if (busy) {
		return false;
	}
	busy = true;
	int len = strlen(str);
//	if (strlen(str) >= driverBufferSize) {
//	}

	statusTransmit = HAL_UART_Transmit_IT(pHandle, (uint8_t*) str, len);
	return statusTransmit;
}

void SerialOutput::doOutputIT()
{
	busy = false;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	SerialOutput * serialObject;
	if (UartHandle == &ACTIVE_UART) {
		serialObject  = SerialOutput::ACTIVE_CHANNEL;
	} else {
		Error_Handler();
	}
	serialObject->doOutputIT();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
//  HAL_UART_ERROR_NONE      = 0x00,    /*!< No error            */
//	HAL_UART_ERROR_PE        = 0x01,    /*!< Parity error        */
//	HAL_UART_ERROR_NE        = 0x02,    /*!< Noise error         */
//	HAL_UART_ERROR_FE        = 0x04,    /*!< frame error         */
//	HAL_UART_ERROR_ORE       = 0x08,    /*!< Overrun error       */
//	HAL_UART_ERROR_DMA       = 0x10,    /*!< DMA transfer error  */
//	HAL_UART_ERROR_BUSY      = 0x20     /*!< Busy Error          */
	static int nLastError = huart->ErrorCode;
	Error_Handler();
}

