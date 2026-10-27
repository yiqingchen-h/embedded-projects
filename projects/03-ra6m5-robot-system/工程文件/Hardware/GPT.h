#ifndef GPT_H_
#define GPT_H_
#include "hal_data.h"
#include <stdio.h>
#include "SysTick.h"
#include "Encoder.h"
#include "Wheel_control.h"
#include "steering_command.h"
    
// 小车状态
typedef enum {
    CAR_STATE_STOP = 0, // 停止状态（仅计算数据，不输出动力）
    CAR_STATE_RUN       // 运行状态（执行导航和电机PID）
} CarState_t;
 
extern CarState_t g_car_state; // 声明全局状态变量
extern int g_nav_target_speed; // 声明导航目标速度

void Init_All_Timers(void);
extern volatile bool get_voltage_mark; // 是否读取电压标记
void timer_1ms_start(void);
void timer_1ms_stop(void);
#endif