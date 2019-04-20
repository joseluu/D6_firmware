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
#ifndef _ADF4351_H
#define _ADF4351_H


// do not include this file: include boardspecific file D6Board.h
// ADF4351 constants

#define ADF_MAX_FREQUENCY 4500000000LL
#define ADF_HIGH_FREQUENCY 4400000000LL
#define ADF_LOW_FREQUENCY 2200000000LL
#define ADF_MIN_FREQUENCY 2048000000LL
#define ADF_RANGE_COUNT 7

//R0
#define INT_VALUE_SHIFT 15
#define INT_VALUE_MASK 0xFFFF

#define FRAC_VALUE_SHIFT 3
#define FRAC_VALUE_MASK 0xFFF

#define R0_ADDRESS 0

//R1
#define PHASE_ADJUST_ON  0x10000000	//disables VCO band selection, use only for fixed frequency application
#define PHASE_ADJUST_OFF 0x00000000	//

#define PRESCALER8_9 0x08000000	//N must be >=75
#define PRESCALER4_5 0x00000000 //when used: max frequency: 3.6GHz N must be >=23

#define PHASE_VALUE_SHIFT 15       //can be used to optimize fractional spurs
#define PHASE_VALUE_MASK 0xFFF     // 12 bits
#define PHASE_VALUE_DEFAULT 0x8000 // (1 not shifted) use this value when not using it

#define MOD_VALUE_SHIFT 3
#define MOD_VALUE_MASK  0xFFF // 12 bits

#define R1_ADDRESS 1

//R2
#define LOW_SPUR    0x60000000 //dither fractional mode when using wide (> RFout/10) bandwith for fast lock
#define LOW_NOISE   0x00000000 //improve phase noise, must also use narrow bandwidth filter

#define MUXOUT_LOCK 0x18000000
#define MUXOUT_DL   0x18000000 //OUTPUT DIGITAL LOCK DETECT
#define MUXOUT_AL   0x14000000 //OUTPUT ANALOG LOCK DETECT
#define MUXOUT_N    0x10000000 //OUTPUT N-DIVIDER if used, VCO band selection does not operate properly
#define MUXOUT_R    0x0C000000 //OUTPUT R_COUNTER
#define MUXOUT_L    0x08000000 //OUTPUT LOW
#define MUXOUT_H    0x04000000 //OUTPUT HIGH
#define MUXOUT_TS   0x00000000 //OUTPUT THREE STATE

#define REF_DOUB_ENABLE	 0x02000000 //if used with wide bandwidth and low spur must have 50% duty cycle
#define REF_DOUB_DISABLE 0x00000000

#define REF_DIV2_ENABLE	 0x01000000 //useful for cycle slip reduction
#define REF_DIV2_DISABLE 0x00000000

#define R_COUNTER_SHIFT 14
#define R_COUNTER_MASK 0x3FF // 10 bits

#define DOUB_BUF_ENABLE	 0x00002000 //for the RF DIVIDER setting in R4
#define DOUB_BUF_DISABLE 0x00000000

#define CHARGE_CURRENT_5MA 0x00001E00 //5mA	(5.1k) value depends on loop filter design
#define CHARGE_CURRENT_2_5MA 0x00000E00 //2.5mA	(5.1k) value depends on loop filter design
#define CHARGE_CURRENT_1MA 0x00000600 //1.25mA (5.1k)
#define CHARGE_PUMP_CURRENT_SHIFT 9
#define CHARGE_PUMP_CURRENT_MASK 0xF // CP_CURRENT=(value+1)*312.5uA -> larger current increases loop bandwidth

#define LOCK_DEtECT_FUNCTION_5CYCLES  0x00000100 //use with integer mode
#define LOCK_DEtECT_FUNCTION_40CYCLES 0x00000000 //use with frac mode

#define	LOCK_DETECT_PRECISION_6NS  0x00000080 //comparison window size for digital lock detector
#define	LOCK_DETECT_PRECISION_10NS 0x00000000 //must use for fractional

#define	PD_POLARITY_POS 0x00000040 //use this value when a passive loop filter is used
#define	PD_POLARITY_NEG 0x00000000 //use when inverting amplifier in loop

#define	POWER_DOWN_ENABLE  0x00000020 //software power down
#define	POWER_DOWN_DISABLE 0x00000000 //normal value when active

#define	CP_THREE_STATE_ENABLE  0x00000010 //charge pump in three state mode
#define	CP_THREE_STATE_DISABLE 0x00000000 //normal value when active

#define	COUNTER_RESET_ENABLE  0x00000008 //keep R and N in reset value
#define	COUNTER_RESET_DISABLE 0x00000000 //normal value when active

#define R2_ADDRESS 2

//R3
#define	BAND_SELECT_CLOCK_HIGH 0x00800000 //Max PFD/BSclock_div < 500KHz
#define	BAND_SELECT_CLOCK_LOW  0x00000000 //Max PFD/BSClock div < 125KHz

#define	ANTIBACKLASH_PLUSE_WIDTH_3 0x00400000 //3ns INT-N
#define	ANTIBACKLASH_PLUSE_WIDTH_6 0x00000000 //6ns FRAC-N

#define CHARGE_CANCELATION_ENABLE  0x00200000 //reduces PFD spurs in INT mode
#define CHARGE_CANCELATION_DISABLE 0x00000000 // FRAC MODE

#define	CYCLE_SLIP_ENABLE  0x00040000 //Improves lock time when using narrow bandwidth filter PFD must have a 50% duty cycle
#define	CYCLE_SLIP_DISABLE 0x00000000

#define R3_RESERVED           0x00018000

#define CLOCK_DIVIDER_OFF  0x00000000
#define RESYNC_ENABLE      0x00010000
#define FAST_LOCK_ENABLE   0x00008000

#define CLOCK_DIVIDER_VALUE_SHIFT 3 // for phase resync and fast lock, determines time in wide bandwidth mode
#define CLOCK_DIVIDER_VALUE_MASK 0xFFF

#define R3_ADDRESS 3

//R4
#define	FEEDBACK_FUNDAMENTAL 0x00800000
#define	FEEDBACK_DIVIDED     0x00000000

#define RF_DIVIDER_SELECT_SHIFT 20
#define RF_DIVIDER_SELECT_MASK 0x3F

#define BAND_SELECT_CLOCK_DIVIDER_SHIFT 12    //divide output of R counter to feed band select logic
#define BAND_SELECT_CLOCK_DIVIDER_MASK  0xFF //band select logic freq must be < 125KHz OR < 500	KHz if band select clock high

#define	VCO_POWERED_DOWN 0x00000800
#define	VCO_POWERED_UP   0x00000000

#define MUTE_TILL_LOCK_DETECT_ENABLE  0x00000400 //RFout is muted until lock detect
#define MUTE_TILL_LOCK_DETECT_DISABLE 0x00000000

#define	AUX_FUNDAMENTAL_OUTPUT  0x00000200
#define	AUX_DIVIDED_OUTPUT      0x00000000

#define	AUX_OUTPUT_ENABLE   0x00000100
#define	AUX_OUTPUT_DISABLE  0x00000000

#define AUX_OUTPUT_5dBm      0x000000C0
#define AUX_OUTPUT_2dBm      0x00000080
#define AUX_OUTPUT_MINUS1dBm 0x00000040
#define AUX_OUTPUT_MINUS4dBm 0x00000000

#define	RF_OUTPUT_ENABLE    0x00000020
#define	RF_OUTPUT_DISABLE   0x00000000

#define RF_OUTPUT_5dBm      0x00000018
#define RF_OUTPUT_2dBm      0x00000010
#define RF_OUTPUT_MINUS1dBm 0x00000008
#define RF_OUTPUT_MINUS4dBm 0x00000000

#define R4_ADDRESS 4

//R5
#define HIGH_LOCK_DETECT_PIN    0x00C00000
#define DIGITAL_LOCK_DETECT_PIN 0x00400000
#define LOW_LOCK_DETECT_PIN     0x00000000
#define R5_RESERVED               0x180000
#define R5_ADDRESS 5

#endif
