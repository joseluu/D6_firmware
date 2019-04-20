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
void executeButtonRelease(bool releasedButton);
void executeCommand(Command command);


#endif
