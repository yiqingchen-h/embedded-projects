#include "pid_ctrl.h"

/* 参数统一放大 100 倍，避免在 8K 程序区里带浮点库。 */
void PIDCTRL_Init(pid_ctrl_t *pid, signed int kp_x100, signed int ki_x100, signed int kd_x100)
{
    pid->kp_x100 = kp_x100;
    pid->ki_x100 = ki_x100;
    pid->kd_x100 = kd_x100;
    pid->integral_accum = 0L;
    pid->prev_error_c10 = 0;
    pid->initialized = 0U;
}

void PIDCTRL_Reset(pid_ctrl_t *pid)
{
    pid->integral_accum = 0L;
    pid->prev_error_c10 = 0;
    pid->initialized = 0U;
}

unsigned char PIDCTRL_Compute(pid_ctrl_t *pid, signed int set_temp_c10, signed int cur_temp_c10)
{
    signed int error_c10;
    signed int delta_error_c10;
    signed long candidate_integral;
    signed long output_num;
    signed long output;

    error_c10 = (signed int)(set_temp_c10 - cur_temp_c10);
    if (pid->initialized == 0U)
    {
        pid->prev_error_c10 = error_c10;
        pid->initialized = 1U;
    }

    delta_error_c10 = (signed int)(error_c10 - pid->prev_error_c10);
    candidate_integral = pid->integral_accum + (signed long)error_c10;
    /* 积分做上限，防止长时间大偏差后放不开。 */
    if (candidate_integral > 10000L)
    {
        candidate_integral = 10000L;
    }
    else if (candidate_integral < -10000L)
    {
        candidate_integral = -10000L;
    }

    output_num = 0L;
    output_num += (signed long)pid->kp_x100 * (signed long)error_c10 * 10L;
    output_num += (signed long)pid->ki_x100 * candidate_integral;
    output_num += (signed long)pid->kd_x100 * (signed long)delta_error_c10 * 100L;
    if (output_num >= 0L)
    {
        output = (output_num + 5000L) / 10000L;
    }
    else
    {
        output = (output_num - 5000L) / 10000L;
    }

    if (output > 100L)
    {
        output = 100L;
        /* 顶到上限时只在误差开始回落后继续积积分。 */
        if (error_c10 <= 0)
        {
            pid->integral_accum = candidate_integral;
        }
    }
    else if (output < 0L)
    {
        output = 0L;
        /* 压到下限时只在需要重新加热时继续积积分。 */
        if (error_c10 >= 0)
        {
            pid->integral_accum = candidate_integral;
        }
    }
    else
    {
        pid->integral_accum = candidate_integral;
    }

    pid->prev_error_c10 = error_c10;
    return (unsigned char)output;
}
