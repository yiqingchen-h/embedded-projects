#ifndef __TRACE_H
#define __TRACE_H
#include "SYS.h"
#include "bus_Servos_Gimbal.h"
//pid结构体
typedef struct {
    //——————————————————
    //转向环参数
    float steering_Error;       //误差
    float steering_Error_difference; // 误差差异
    float steering_Error_sum;   //误差累加
    float steering_last_Error;  //上次误差
    float steering_Kp;
    float steering_Kd;
    float steering_Ki;
    //————————————————————
}Pid_Data;
extern Pid_Data steering_Pid;
extern Pid_Data revolve_Pid;
extern unsigned char Black_fast_flag ;                  // 黑色快标志位
extern unsigned char Left_size_flag;
extern unsigned short turnings_Ok_decelerate_flag;
extern unsigned int time_size;
//左转标志位 供步进电机转弯后的低速重新启动使用
extern unsigned char Last_bit;
extern unsigned char Ok_falg;                          // 完成圈数
//读取灰度并处理
float Read_Grayscale(void);
//循迹PID
float Grayscale_Pid(float target_value , float Measurements); // 目标值  测量值

#endif