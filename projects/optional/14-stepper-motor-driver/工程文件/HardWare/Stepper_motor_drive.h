#ifndef __STEPPER_MOTOR_DRIVE_H
#define __STEPPER_MOTOR_DRIVE_H
#include "stm32f1xx_hal.h"
/*
步进电机控制细分为tmc2209的ms1和ms2
MS1：0 MS2：0  为8细分
MS1：1 MS2：1  为16细分
MS1：1 MS2：0  为32细分
MS1：0 MS2：1  为64细分
细分意义 ： 如64细分时 当步进电机 步距角度 为1.8度时 控制精度=1.8°/64=0.0140625° 此时电机旋转一周需要12800个脉冲
*/
#define subdivision_8 0 //8细分
#define subdivision_16 1 //8细分
#define subdivision_32 2 //8细分
#define subdivision_64 3 //8细分
extern TIM_HandleTypeDef TIM_HandleType2;
// 控制电机旋转固定角度
void Stepper_Move(uint32_t step, GPIO_PinState direction, unsigned char Flog);
/*步进电机速度控制和初始化*/
void Stepper_Init(unsigned char level);

// 步进运动
void stepping(uint32_t step, GPIO_PinState direction, unsigned char Flog);

#endif
