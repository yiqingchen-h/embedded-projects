#ifndef __LED_H
#define __LED_H

void LED_Init(void);

void LED1_ON(void);

void LED1_OFF(void);

void LED2_ON(void);

void LED2_OFF(void);
//翻转（A0口）LED当前状态
void LED1_Turn(void);
//翻转（A1口）LED当前状态
void LED2_Turn(void);
#endif
