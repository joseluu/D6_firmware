#pragma once
#ifndef _LOOP_H
#define _LOOP_H

void sendChar(char c);

#ifdef __cplusplus
extern "C" {
#endif

	void initializeListener();
void doListen(void);


#ifdef __cplusplus
}
#endif


#endif
