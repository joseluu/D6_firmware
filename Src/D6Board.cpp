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

#include "gpio.h"
#include "main.h"
#include "D6Board.h"

void ledD1(bool stateOn)
{
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, stateOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ledD2(bool stateOn)
{
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, stateOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool buttonPressed(void)
{
	return (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET);
}


void sendSingleRegister(WhichADF indexADF, unsigned long reg)
{
	sendADF4351(indexADF, reg);
}