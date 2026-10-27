#ifndef __LED_H
#define __LED_H		
#include "SYS.h"

#define LED_RED 0
#define LED_BLUE 1
#define LED_GREEN 2

void Led_On(unsigned char LED);
void Led_Off(unsigned char LED);

#endif

