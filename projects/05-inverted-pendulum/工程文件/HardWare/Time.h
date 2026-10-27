#ifndef __TIME_H
#define __TIME_H

extern unsigned short encoder_count; // 记录当前脉冲数
extern TIM_HandleTypeDef TIM_HandleType1;
extern TIM_HandleTypeDef TIM_HandleType2;
extern TIM_HandleTypeDef TIM_HandleType3;

void Tim_Pwm_Init(unsigned short psc, unsigned short arr);
void HAL_TIM_encoder_Init(unsigned int arr);
short Read_Encoder_Value(void);
void Time3_Init(unsigned int arr, unsigned int psc);

extern short speed, pwm, my_speed, speed_Places, my_Places;
extern short angle_now, angle_zero;
extern unsigned char balance_start_flag, balance_error_flag;

#endif
