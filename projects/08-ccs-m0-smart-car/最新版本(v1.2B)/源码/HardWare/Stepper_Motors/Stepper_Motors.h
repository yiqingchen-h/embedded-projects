#ifndef __STEPPER_MOTORS_H
#define __STEPPER_MOTORS_H		
#include "SYS.h"

/*
步进电机控制细分为tmc2209的ms1和ms2
MS1：0 MS2：0  为8细分
MS1：1 MS2：1  为16细分
MS1：1 MS2：0  为32细分
MS1：0 MS2：1  为64细分
细分意义 ： 如64细分时 当步进电机 步距角度 为1.8度时 控制精度=1.8°/64=0.0140625° 此时电机旋转一周需要12800个脉冲
*/
#define subdivision_8 0 //8细分
#define subdivision_16 1 //16细分
#define subdivision_32 2 //32细分
#define subdivision_64 3 //64细分
//细分设置
void Stepper_Subdivision_Init(unsigned char subdivision);

//旋转固定角度
void Stepper_Move(int step, unsigned char Flog,unsigned char level);
void Stepper_Move_R(int step, unsigned char Flog,unsigned char level);
void Stepper_Move_L(int step, unsigned char Flog,unsigned char level);
// 步进运动
void stepping( unsigned char direction ,unsigned char Flog,unsigned char level);
// 单电机步进运动
void stepping_R( unsigned char direction ,unsigned char Flog,unsigned char level);
void stepping_L( unsigned char direction ,unsigned char Flog,unsigned char level);

//停止运动
void Stepper_Stop(void);

// 控制上电机运转固定角度
void Stepper_Move_Up(int step, unsigned char Flog,unsigned char level);
// 控制下电机运转固定角度
void Stepper_Move_Down(int step, unsigned char Flog,unsigned char level);




/*开启内部细分并设置旋转方向
direction 为 假时 逆时针旋转 真时 顺时针旋转
*/
void Stepper_Uart_Init_R(unsigned char direction);
// Uart——发送速度配置
void Stepper_Uart_control_R(long long speed);
/*
    细分设置
    无返回值
    输出需要的细分：256/128/64/32/16/8/4/2/0
*/
void Stepper_Uart_Subdivision_R(unsigned char Subdivision);

#endif
