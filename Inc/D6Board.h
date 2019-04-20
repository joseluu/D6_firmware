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
#ifndef _D6CONFIGURATION_H
#define _D6CONFIGURATION_H


#define REFERENCE 25000000 // 25 MHz local reference
#define MODULUS_VALUE 3125 // must be less than 4096 (12 bits)
#define CHANNEL_SPACING (REFERENCE/MODULUS_VALUE)

typedef enum {
	eTracking = 0,
	eAnalyzer
} WhichADF;

#include "ADF4351.h"

//R1
#define R1 ((MODULUS_VALUE & MOD_VALUE_MASK) << MOD_VALUE_SHIFT) | PHASE_VALUE_DEFAULT | PRESCALER8_9 | R1_ADDRESS

//R2
#define R_COUNTER 1 // PFD=REFERENCE (25Mhz)
#define R2 PD_POLARITY_POS | LOCK_DETECT_PRECISION_10NS | CHARGE_CURRENT_2_5MA | (R_COUNTER << R_COUNTER_SHIFT ) | R2_ADDRESS

//R3
#define CLOCK_DIVIDER  150 << CLOCK_DIVIDER_VALUE_SHIFT
#define R3  CLOCK_DIVIDER | R3_ADDRESS

//R4
#define BS_CLOCK  200 << BAND_SELECT_CLOCK_DIVIDER_SHIFT
#define R4 RF_OUTPUT_MINUS4dBm | RF_OUTPUT_ENABLE | BS_CLOCK | FEEDBACK_FUNDAMENTAL | R4_ADDRESS // no AUX output

//R5
#define R5 R5_RESERVED | DIGITAL_LOCK_DETECT_PIN | R5_ADDRESS


#ifdef __cplusplus
bool frequencySetup(WhichADF indexADF, unsigned long long frequency, unsigned int level, unsigned int & range, bool forceRange);
void sendSingleRegister(WhichADF indexADF, unsigned long reg);
void sendADF4351(WhichADF indexADF, unsigned long reg);
void ADF4351Off(WhichADF indexADF);
#endif

#ifdef __cplusplus
extern "C" {
#endif

void ledD1(bool stateOn);
void ledD2(bool stateOn);
void probeSPI(WhichADF indexADF);

#ifdef __cplusplus
}
#endif





#endif