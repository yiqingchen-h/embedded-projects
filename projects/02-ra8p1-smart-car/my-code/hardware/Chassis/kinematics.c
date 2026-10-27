#include "hardware/Chassis/kinematics.h"

#include <stddef.h>

/**
 * @brief  对浮点数进行上下限限幅
 *
 * @param value  待限幅的输入值
 * @param low    允许的最小值
 * @param high   允许的最大值
 * @return float  限幅后的输出值
 */
static float kinematics_limit_float(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}

/**
 * @brief  麦轮底盘逆运动学解算
 *
 * @param p_velocity         车体目标速度，vx/vy 单位 m/s，wz 单位 rad/s
 * @param wheel_targets_mps  输出四路轮目标线速度，单位 m/s
 * @return void
 */
static void kinematics_inverse_impl(const chassis_velocity_t *p_velocity,
                                    float wheel_targets_mps[CHASSIS_WHEEL_COUNT])
{
    if ((NULL == p_velocity) || (NULL == wheel_targets_mps))
    {
        return;
    }

    float rotation_radius = CHASSIS_HALF_WHEEL_SPACING_M + CHASSIS_HALF_AXLE_SPACING_M;
    float rotation_speed = p_velocity->wz_radps * rotation_radius;

    /* 麦轮逆运动学沿用 WHEELTEC 的符号约定：
     * vx 为前进方向速度，vy 为横移方向速度，wz 为车体旋转角速度。
     * 输出数组下标仍然是电机编号，轮位到电机的映射由 Chassis_config.h 控制。
     */
    wheel_targets_mps[CHASSIS_WHEEL_LEFT_FRONT] =
        p_velocity->vx_mps + p_velocity->vy_mps - rotation_speed;
    wheel_targets_mps[CHASSIS_WHEEL_RIGHT_FRONT] =
        p_velocity->vx_mps - p_velocity->vy_mps + rotation_speed;
    wheel_targets_mps[CHASSIS_WHEEL_LEFT_REAR] =
        p_velocity->vx_mps - p_velocity->vy_mps - rotation_speed;
    wheel_targets_mps[CHASSIS_WHEEL_RIGHT_REAR] =
        p_velocity->vx_mps + p_velocity->vy_mps + rotation_speed;

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        wheel_targets_mps[i] = kinematics_limit_float(wheel_targets_mps[i],
                                                      -CHASSIS_MAX_WHEEL_SPEED_MPS,
                                                      CHASSIS_MAX_WHEEL_SPEED_MPS);
    }
}

const kinematics_api_t kinematics =
    {
        .inverse = kinematics_inverse_impl,
};
