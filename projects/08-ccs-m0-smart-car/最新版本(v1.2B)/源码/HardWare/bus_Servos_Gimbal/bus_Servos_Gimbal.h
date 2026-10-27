#ifndef __BSG_H
#define __BSG_H		
#include "SYS.h"

void Send_SingleServo(uint8_t index, uint16_t pwm, uint16_t time_ms);
void Send_SingleServo_double(uint8_t index_1, uint16_t pwm_1, uint16_t time_ms_1 , uint8_t index_2, uint16_t pwm_2, uint16_t time_ms_2);
void Bus_Servos_Gimbal_Init(void);
extern unsigned char L_Flag;
extern unsigned char R_Flag;
#endif