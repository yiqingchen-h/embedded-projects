#include "hardware/Chassis/Chassis_pid.h"

#include <stddef.h>

/**
 * @brief  对 PID 输出进行正负对称限幅
 *
 * @param value  待限幅的 PID 输出
 * @param limit  输出绝对值上限
 * @return float  限幅后的 PID 输出
 */
static float chassis_pid_limit_float(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

/**
 * @brief  初始化底盘单路增量 PI 控制器
 *
 * @param p_ctrl        控制结构体
 * @param kp            比例系数
 * @param ki            积分系数
 * @param output_limit  输出绝对值上限，单位为 PWM 计数
 * @return void
 */
static void chassis_pid_init_impl(chassis_pid_controller_t *p_ctrl,
                                  float kp,
                                  float ki,
                                  float output_limit)
{
    if (NULL == p_ctrl)
    {
        return;
    }

    p_ctrl->kp = kp;
    p_ctrl->ki = ki;
    p_ctrl->output_limit = output_limit;
    p_ctrl->output = 0.0f;
    p_ctrl->last_error = 0.0f;
}

/**
 * @brief  复位底盘单路增量 PI 控制器状态
 *
 * @param p_ctrl  控制结构体
 * @return void
 */
static void chassis_pid_reset_impl(chassis_pid_controller_t *p_ctrl)
{
    if (NULL == p_ctrl)
    {
        return;
    }

    p_ctrl->output = 0.0f;
    p_ctrl->last_error = 0.0f;
}

/**
 * @brief  底盘输出 PID 计算
 *
 * @param p_ctrl    控制结构体
 * @param measured  测量值，单位为 m/s
 * @param target    目标值，单位为 m/s
 * @return float    PID 输出值，单位为 PWM 计数
 */
static float chassis_pid_calculate_impl(chassis_pid_controller_t *p_ctrl, float measured, float target)
{
    if (NULL == p_ctrl)
    {
        return 0.0f;
    }

    /* 增量 PI：
     * output += Kp * (当前误差 - 上次误差) + Ki * 当前误差。
     * 输出单位是 GPT PWM 计数，motor.c 会继续按 PWM 周期做最终限幅。
     */
    float error = target - measured;
    p_ctrl->output += p_ctrl->kp * (error - p_ctrl->last_error) + p_ctrl->ki * error;
    p_ctrl->output = chassis_pid_limit_float(p_ctrl->output, p_ctrl->output_limit);
    p_ctrl->last_error = error;

    return p_ctrl->output;
}

const chassis_pid_api_t chassis_pid =
{
    .init = chassis_pid_init_impl,
    .reset = chassis_pid_reset_impl,
    .calculate = chassis_pid_calculate_impl,
};
