#ifndef __FAN_H
#define __FAN_CONTROL_H
#include "main.h"

// 定义定时器句柄
extern TIM_HandleTypeDef htim5_fan;

// 函数声明
void Fan_PWM_Init(void);
void Fan_Set_Speed(uint8_t duty_cycle);

#endif
