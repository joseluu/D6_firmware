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

#ifdef __cplusplus
bool frequencySetup(WhichADF indexADF, unsigned long long frequency, unsigned int level, unsigned int & range, bool forceRange);
void sendSingleRegister(WhichADF indexADF, unsigned long reg);
void sendADF4351(WhichADF indexADF, unsigned long reg);
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