#ifndef STEERING_COMMAND_H_
#define STEERING_COMMAND_H_
#include "hal_data.h"
// ================= 用户配置区域 =================

// 舵机机械中点修正值 (单位：度)
// 如果舵机装配不正，正数左修正，负数右修正
#define SERVO_OFFSET_ANGLE 6

// 软件限位 (单位：度)
// 即使输入-90，如果这里限制为-60，舵机只会转到-60
#define SERVO_LIMIT_LEFT_MAX  -50
#define SERVO_LIMIT_RIGHT_MAX 50

// 舵机脉宽参数 (单位：微秒 us)
// 标准舵机：-90度=500us, 0度=1500us, +90度=2500us
#define SERVO_PULSE_MIN_US    500
#define SERVO_PULSE_CENTER_US 1500
#define SERVO_PULSE_MAX_US    2500
#define SERVO_PERIOD_US       20000 // PWM周期 20ms

// PWM输出引脚
#define SERVO_OUTPUT_PIN GPT_IO_PIN_GTIOCB

// ==============================================

void Servo_Init(void);
void Servo_SetAngle(int16_t angle);

#endif
