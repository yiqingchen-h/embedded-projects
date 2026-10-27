#include "hardware/Chassis/Chassis.h"

#include "hardware/Chassis/Chassis_pid.h"
#include "hardware/Chassis/encoder.h"
#include "hardware/Chassis/motor.h"

#include <stddef.h>

/* 当前车体速度目标，BODY 模式下参与麦轮逆解。 */
static chassis_velocity_t g_target_velocity;

/* 当前四路轮速目标，WHEEL 模式下直接参与闭环控制。 */
static float g_target_wheel_speed[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* 当前四路开环 PWM 诊断目标，PWM 模式下直接输出到电机。 */
static int32_t g_target_pwm_counts[CHASSIS_WHEEL_COUNT] = {0, 0, 0, 0};

/* 四路共用 PI 参数，支持运行时从 Flash 加载或由 UI 调整。 */
static chassis_pid_params_t g_chassis_pid_params =
{
    .kp = CHASSIS_PID_DEFAULT_KP,
    .ki = CHASSIS_PID_DEFAULT_KI,
};

/* 当前底盘控制模式。 */
static chassis_control_mode_t g_chassis_control_mode = CHASSIS_CONTROL_MODE_BODY;

/* 四路速度环 PI 控制器。 */
static chassis_pid_controller_t g_wheel_pid[CHASSIS_WHEEL_COUNT];

/**
 * @brief  对底盘速度命令进行上下限限幅
 *
 * @param value  待限幅的速度命令
 * @param low    允许的最小值
 * @param high   允许的最大值
 * @return float  限幅后的速度命令
 */
static float chassis_limit_float(float value, float low, float high)
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
 * @brief  对 PWM 诊断命令进行上下限限幅
 *
 * @param value 待限幅的 PWM 计数
 * @param low   允许的最小值
 * @param high  允许的最大值
 * @return int32_t 限幅后的 PWM 计数
 */
static int32_t chassis_limit_i32(int32_t value, int32_t low, int32_t high)
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
 * @brief  将浮点 PID 输出四舍五入为 PWM 计数
 *
 * @param value  浮点 PWM 计数
 * @return int32_t  四舍五入后的有符号 PWM 计数
 */
static int32_t chassis_float_to_i32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

/**
 * @brief  计算浮点数绝对值
 *
 * @param value  浮点数
 * @return float  绝对值
 */
static float chassis_abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief  将 PWM 计数值限幅到当前 GPT 周期范围内
 *
 * @param pwm_counts  待限幅的有符号 PWM 计数
 * @return int32_t  限幅后的 PWM 计数
 */
static int32_t chassis_limit_pwm_to_period(int32_t pwm_counts)
{
    int32_t pwm_period = (int32_t)motor.period_counts();

    if (pwm_period <= 0)
    {
        pwm_period = (int32_t)CHASSIS_PWM_PERIOD_FALLBACK;
    }

    if (pwm_counts > pwm_period)
    {
        return pwm_period;
    }

    if (pwm_counts < -pwm_period)
    {
        return -pwm_period;
    }

    return pwm_counts;
}

/**
 * @brief  将目标轮速和 PID 修正量转换为最终 PWM 输出
 *
 * 目标为 0 时直接返回 0。非零时使用死区补偿加速度前馈生成 PWM 主体，
 * 再叠加上 PI 修正量。
 *
 * @param target_speed_mps  目标轮速，单位 m/s
 * @param pid_correction    PI 输出修正量，单位 PWM 计数
 * @return int32_t  最终 PWM 计数
 */
static int32_t chassis_speed_to_pwm(float target_speed_mps, float pid_correction)
{
    int32_t sign = 0;
    float abs_target = 0.0f;
    float feedforward_abs = 0.0f;
    float pwm = 0.0f;

    if (chassis_abs_float(target_speed_mps) <= CHASSIS_ZERO_TARGET_EPS_MPS)
    {
        return 0;
    }

    sign = (target_speed_mps > 0.0f) ? 1 : -1;
    abs_target = chassis_abs_float(target_speed_mps);
    feedforward_abs = (float)CHASSIS_PWM_DEADBAND_COUNTS +
                      abs_target * CHASSIS_SPEED_FF_COUNTS_PER_MPS;
    pwm = (float)sign * feedforward_abs + pid_correction;

    return chassis_limit_pwm_to_period(chassis_float_to_i32(pwm));
}

/**
 * @brief  复位四路底盘 PI 控制器
 *
 * @return void
 */
static void chassis_reset_pid(void)
{
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        chassis_pid.reset(&g_wheel_pid[i]);
    }
}

/**
 * @brief  使用当前 PI 参数重新初始化四路控制器
 *
 * @return void
 */
static void chassis_apply_pid_params(void)
{
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        chassis_pid.init(&g_wheel_pid[i],
                         g_chassis_pid_params.kp,
                         g_chassis_pid_params.ki,
                         CHASSIS_PID_OUTPUT_LIMIT);
    }
}

/**
 * @brief  清空底盘速度目标
 *
 * @return void
 */
static void chassis_clear_targets(void)
{
    g_target_velocity.vx_mps = 0.0f;
    g_target_velocity.vy_mps = 0.0f;
    g_target_velocity.wz_radps = 0.0f;

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_target_wheel_speed[i] = 0.0f;
        g_target_pwm_counts[i] = 0;
    }
}

/**
 * @brief  初始化底盘闭环控制模块
 *
 * @return fsp_err_t  FSP_SUCCESS 表示电机、编码器和 PI 状态初始化成功
 */
static fsp_err_t chassis_init_impl(void)
{
    fsp_err_t err = motor.init();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = encoder.init();
    if (FSP_SUCCESS != err)
    {
        motor.stop_all();
        return err;
    }

    g_chassis_pid_params.kp = CHASSIS_PID_DEFAULT_KP;
    g_chassis_pid_params.ki = CHASSIS_PID_DEFAULT_KI;
    chassis_apply_pid_params();
    chassis_clear_targets();
    g_chassis_control_mode = CHASSIS_CONTROL_MODE_BODY;
    motor.stop_all();

    return FSP_SUCCESS;
}

/**
 * @brief  设置底盘目标车体速度
 *
 * @param vx_mps    前进方向速度，单位 m/s
 * @param vy_mps    横移方向速度，单位 m/s
 * @param wz_radps  旋转角速度，单位 rad/s
 * @return void
 */
static void chassis_set_velocity_impl(float vx_mps, float vy_mps, float wz_radps)
{
    g_target_velocity.vx_mps = chassis_limit_float(vx_mps, -CHASSIS_MAX_VX_MPS, CHASSIS_MAX_VX_MPS);
    g_target_velocity.vy_mps = chassis_limit_float(vy_mps, -CHASSIS_MAX_VY_MPS, CHASSIS_MAX_VY_MPS);
    g_target_velocity.wz_radps = chassis_limit_float(wz_radps, -CHASSIS_MAX_WZ_RADPS, CHASSIS_MAX_WZ_RADPS);
    g_chassis_control_mode = CHASSIS_CONTROL_MODE_BODY;
}

/**
 * @brief  设置四路轮目标速度
 *
 * @param wheel_speed_mps  四路轮速目标，单位 m/s
 * @return void
 */
static void chassis_set_wheel_speed_impl(const float wheel_speed_mps[CHASSIS_WHEEL_COUNT])
{
    if (NULL == wheel_speed_mps)
    {
        return;
    }

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_target_wheel_speed[i] = chassis_limit_float(wheel_speed_mps[i],
                                                      -CHASSIS_MAX_WHEEL_SPEED_MPS,
                                                      CHASSIS_MAX_WHEEL_SPEED_MPS);
    }

    g_chassis_control_mode = CHASSIS_CONTROL_MODE_WHEEL;
}

/**
 * @brief  设置四路开环 PWM 诊断输出
 *
 * @param pwm_counts 四路 PWM 计数，正负号表示方向
 * @return void
 */
static void chassis_set_pwm_impl(const int32_t pwm_counts[CHASSIS_WHEEL_COUNT])
{
    if (NULL == pwm_counts)
    {
        return;
    }

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_target_pwm_counts[i] = chassis_limit_i32(pwm_counts[i],
                                                   -CHASSIS_PWM_TEST_LIMIT_COUNTS,
                                                   CHASSIS_PWM_TEST_LIMIT_COUNTS);
    }

    g_chassis_control_mode = CHASSIS_CONTROL_MODE_PWM;
}

/**
 * @brief  设置四路共用速度环 PI 参数
 *
 * @param kp  比例系数
 * @param ki  积分系数
 * @return void
 */
static void chassis_set_pid_impl(float kp, float ki)
{
    g_chassis_pid_params.kp = chassis_limit_float(kp, CHASSIS_PID_PARAM_MIN, CHASSIS_PID_PARAM_MAX);
    g_chassis_pid_params.ki = chassis_limit_float(ki, CHASSIS_PID_PARAM_MIN, CHASSIS_PID_PARAM_MAX);
    chassis_apply_pid_params();
}

/**
 * @brief  获取四路共用速度环 PI 参数
 *
 * @param p_params  参数输出指针
 * @return void
 */
static void chassis_get_pid_impl(chassis_pid_params_t *p_params)
{
    if (NULL == p_params)
    {
        return;
    }

    *p_params = g_chassis_pid_params;
}

/**
 * @brief  执行一次底盘闭环控制
 *
 * @param interval_ms  控制周期，单位 ms， 10ms 调用一次
 * @return fsp_err_t   FSP_SUCCESS 表示本次编码器读取、PID 计算和 PWM 输出成功
 */
static fsp_err_t chassis_control_step_impl(uint32_t interval_ms)
{
    float measured_speed[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    float target_speed[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    int32_t pwm_counts[CHASSIS_WHEEL_COUNT] = {0, 0, 0, 0};
    fsp_err_t err = FSP_SUCCESS;

    if (CHASSIS_CONTROL_MODE_PWM == g_chassis_control_mode)
    {
        /* PWM 诊断模式仍刷新编码器速度，供 UI 显示实测轮速。 */
        (void)encoder.update(interval_ms, NULL);
        return motor.set_all_pwm(g_target_pwm_counts);
    }

    err = encoder.update(interval_ms, measured_speed);
    if (FSP_SUCCESS != err)
    {
        motor.stop_all();
        return err;
    }

    if (CHASSIS_CONTROL_MODE_WHEEL == g_chassis_control_mode)
    {
        for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            target_speed[i] = g_target_wheel_speed[i];
        }
    }
    else
    {
        kinematics.inverse(&g_target_velocity, target_speed);
    }

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (chassis_abs_float(target_speed[i]) <= CHASSIS_ZERO_TARGET_EPS_MPS)
        {
            chassis_pid.reset(&g_wheel_pid[i]);
            pwm_counts[i] = 0;
            continue;
        }

        float pid_output = chassis_pid.calculate(&g_wheel_pid[i],
                                                 measured_speed[i],
                                                 target_speed[i]);

        pwm_counts[i] = chassis_speed_to_pwm(target_speed[i], pid_output);
    }

    return motor.set_all_pwm(pwm_counts);
}

/**
 * @brief  停止底盘并清空速度目标和 PI 状态
 *
 * @return void
 */
static void chassis_stop_impl(void)
{
    chassis_clear_targets();
    g_chassis_control_mode = CHASSIS_CONTROL_MODE_BODY;
    chassis_reset_pid();
    motor.stop_all();
}

/**
 * @brief  获取当前底盘目标速度结构体
 *
 * @return const chassis_velocity_t*  当前目标速度，只读指针
 */
static const chassis_velocity_t *chassis_target_velocity_impl(void)
{
    return &g_target_velocity;
}

/**
 * @brief  获取当前四路轮目标速度
 *
 * @return const float*  四路轮目标速度，只读指针
 */
static const float *chassis_target_wheel_speed_impl(void)
{
    return g_target_wheel_speed;
}

/**
 * @brief  获取当前底盘控制模式
 *
 * @return chassis_control_mode_t  当前控制模式
 */
static chassis_control_mode_t chassis_control_mode_impl(void)
{
    return g_chassis_control_mode;
}

const chassis_api_t chassis =
{
    .init = chassis_init_impl,
    .set_velocity = chassis_set_velocity_impl,
    .set_wheel_speed = chassis_set_wheel_speed_impl,
    .set_pwm = chassis_set_pwm_impl,
    .set_pid = chassis_set_pid_impl,
    .get_pid = chassis_get_pid_impl,
    .control_step = chassis_control_step_impl,
    .stop = chassis_stop_impl,
    .target_velocity = chassis_target_velocity_impl,
    .target_wheel_speed = chassis_target_wheel_speed_impl,
    .control_mode = chassis_control_mode_impl,
};
