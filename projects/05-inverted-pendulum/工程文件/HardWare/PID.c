#include "UART.h"
#include "PID.h"
#include "math.h"

float Speed_Error_0, Speed_Error_1; // 本次误差，上次误差
float Speed_Error_Add;              // 误差累积值
float Speed_Kd_Out = 0;             // 微分滤波后输出

// Speed 本次速度 Target 目标速度
short Pid_Speed(short Speed, short Target)
{
    short Out     = 0;
    float a       = 0.7;

    Speed_Error_0 = Target - Speed;
    Speed_Error_Add += Speed_Error_0;
    Speed_Error_Add = Speed_Error_Add > 1000 ? 1000 : Speed_Error_Add;
    Speed_Error_Add = Speed_Error_Add < -1000 ? -1000 : Speed_Error_Add;

    Speed_Kd_Out = (1 - a) * Speed_Kd * (Speed_Error_0 - Speed_Error_1) + a * Speed_Kd_Out;
    Out          = Speed_Kp * Speed_Error_0 + Speed_Kd_Out + Speed_Ki * Speed_Error_Add;
    Speed_Error_1 = Speed_Error_0;
    return Out;
}

float Places_Error_0, Places_Error_1; // 本次误差，上次误差
float Places_Error_Add;               // 误差累积值
float Places_Kd_Out = 0;              // 微分滤波后输出

// Places 本次位置 Target 目标位置
short Pid_places(short Places, short Target)
{
    short Out      = 0;
    float a        = 0.7;

    Places_Error_0 = Target - Places;
    Places_Error_Add += Places_Error_0;

    if (fabs(Places_Error_0) < 10) {
        Places_Error_Add = 0;
    }

    Places_Kd_Out    = (1 - a) * Places_Kd * (Places_Error_0 - Places_Error_1) + a * Places_Kd_Out;
    Places_Error_Add = Places_Error_Add > 10000 ? 10000 : Places_Error_Add;
    Places_Error_Add = Places_Error_Add < -10000 ? -10000 : Places_Error_Add;
    Out              = Places_Kp * Places_Error_0 + Places_Kd_Out + Places_Ki * Places_Error_Add;
    Places_Error_1   = Places_Error_0;
    return Out;
}

// 双环PID
short Pid_Speed_places(short Speed, short Places, short Target_Places)
{
    short Out_Outer_ring = Pid_places(Places, Target_Places);
    short Out            = Pid_Speed(Speed, Out_Outer_ring);
    return Out;
}

// 倒立摆角度环
short PID_Place(short Place, short Target)
{
    short Out      = 0;
    float a        = 0.7;

    Places_Error_0 = Target - Place;
    if (fabs(Places_Error_0) < 5) {
        Places_Error_0 = 0;
    }

    Places_Kd_Out = (1 - a) * Places_Kd * (Places_Error_0 - Places_Error_1) + a * Places_Kd_Out;
    Out           = Places_Kp * Places_Error_0 + Places_Kd_Out;
    Places_Error_1 = Places_Error_0;
    return Out;
}
