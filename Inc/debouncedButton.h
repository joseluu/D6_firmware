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
#ifndef _DEBOUNCEDBUTTON_H
#define _DEBOUNCEDBUTTON_H
#include "gpio.h"

#define MAX_DEBOUNCED_BUTTONS 1

#ifndef __cplusplus
// C interface
#include <stdbool.h>

void DebouncedButton_SysTick_Handler(); //insert call to this function in the SysTick_Handler function
bool DebouncedButton_buttonState(int index, bool * changed);
int DebouncedButton_addNewButton(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pin);
#else
class DebouncedButton 
{
	GPIO_TypeDef  *GPIOx;
	uint16_t GPIO_Pin;
	int buttonPressed_0;
	int buttonPressed_1;
	int debounceThreshold = 10;
	bool buttonPressConfirmed;
	bool buttonPressChanged;

	void buttonSysTick_Handler();
	bool readButtonPress();

public:
	static int addButton(DebouncedButton * button);
	static DebouncedButton * allButtons[MAX_DEBOUNCED_BUTTONS];
	static int debouncedButtonCount;
	static void SysTick_Handler();

	bool buttonState(bool * changed);

	DebouncedButton(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pin, int debounceThreshold = 50)
		: GPIOx(GPIOx)
		, GPIO_Pin(GPIO_Pin)
		, debounceThreshold(debounceThreshold)
		, buttonPressed_0(0)
		, buttonPressed_1(0)
		, buttonPressConfirmed(false)
		, buttonPressChanged(false)
	{	
	}
};
#endif


#endif