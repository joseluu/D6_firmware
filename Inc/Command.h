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
#ifndef _COMMAND_H
#define _COMMAND_H

#define FIRMWARE_VERSION 5

#define FIRMWARE_ENGINEERING_VARIANT 1

void doInteractive(char c);
void doNWT(unsigned char c);
void doAD(char c);
void enableTracking(bool newState);

typedef struct {
	char code;
	long long frequency;
	long long step;
	long long count;
	long long delay;
	long long audio;
	unsigned char which;
	unsigned long r;
} Command;

void analyzerStandbyLedOn(bool standbyState);
void executeButtonAction(bool pushedButton);
void executeCommand(Command command);


#endif
