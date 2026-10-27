#include "Servos.h"
// DL_TimerG_startCounter(Stepping_STEP_R_INST); // 开启PWM输出
//DL_TimerG_setCaptureCompareValue(Stepping_STEP_R_INST,period / 2, GPIO_Stepping_STEP_R_C0_IDX); // 50%占空比
/*
初始化默认回正90°
*/
void Servos_Init(void)
{
    DL_TimerG_setCaptureCompareValue(PWM_Servos_INST,((90 * 355 / 100) + 160), GPIO_PWM_Servos_C0_IDX);
    DL_TimerG_startCounter(PWM_Servos_INST); // 开启PWM
}
/*
  控制舵机移动固定角度
  angle ： 0 -180
*/
void Servos_Move(unsigned char angle)
{
    DL_TimerG_setCaptureCompareValue(PWM_Servos_INST,((angle * 355 / 100) + 160), GPIO_PWM_Servos_C0_IDX);
    DL_TimerG_startCounter(PWM_Servos_INST); // 开启PWM
}