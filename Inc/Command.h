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

void doInteractive(char c);
void doNWT(char c);
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
	unsigned long r0;
	unsigned long r1;
	unsigned long r2;
	unsigned long r3;
	unsigned long r4;
	unsigned long r5;
} Command;

void analyzerStandbyLedOn(bool standbyState);
void executeButtonAction(bool pushedButton);
void executeCommand(Command command);


#endif
