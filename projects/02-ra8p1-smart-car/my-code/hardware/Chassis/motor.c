#include "hardware/Chassis/motor.h"

#include <stddef.h>

#include "r_gpt.h"

typedef struct st_motor_channel
{
    const timer_instance_t *p_timer;
    uint32_t positive_pin;
    uint32_t negative_pin;
    int8_t polarity;
} motor_channel_t;

/* 四路 AT8236 与 RA8P1 GPT 的对应关系：
 * W1 左前：P915/GTIOC5A 为 IN1，P914/GTIOC5B 为 IN2。
 * W2 右前：P400/GTIOC6A 为 IN1，P401/GTIOC6B 为 IN2。
 * W3 左后：P907/GTIOC13A 为 IN1，P906/GTIOC13B 为 IN2。
 * W4 右后：P810/GTIOC10A 为 IN1，P811/GTIOC10B 为 IN2。
 */
static const motor_channel_t g_motor_channels[CHASSIS_WHEEL_COUNT] =
{
    {&g_timer5,  GPT_IO_PIN_GTIOCA, GPT_IO_PIN_GTIOCB, CHASSIS_MOTOR1_PWM_POLARITY},
    {&g_timer6,  GPT_IO_PIN_GTIOCA, GPT_IO_PIN_GTIOCB, CHASSIS_MOTOR2_PWM_POLARITY},
    {&g_timer13, GPT_IO_PIN_GTIOCA, GPT_IO_PIN_GTIOCB, CHASSIS_MOTOR3_PWM_POLARITY},
    {&g_timer10, GPT_IO_PIN_GTIOCA, GPT_IO_PIN_GTIOCB, CHASSIS_MOTOR4_PWM_POLARITY},
};

static uint32_t g_pwm_period_counts = CHASSIS_PWM_PERIOD_FALLBACK;

/**
 * @brief  打开并启动一个 GPT 电机 PWM 定时器
 *
 * @param p_timer  FSP 生成的定时器实例指针
 * @return fsp_err_t  FSP_SUCCESS 表示定时器已可输出 PWM
 */
static fsp_err_t motor_open_start(const timer_instance_t *p_timer)
{
    fsp_err_t err = p_timer->p_api->open(p_timer->p_ctrl, p_timer->p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    return p_timer->p_api->start(p_timer->p_ctrl);
}

/**
 * @brief  将 PWM 计数限制到当前 GPT 周期以内
 *
 * @param counts  待限制的 PWM 计数
 * @return uint32_t  限幅后的 PWM 计数
 */
static uint32_t motor_limit_counts(uint32_t counts)
{
    if (counts > g_pwm_period_counts)
    {
        return g_pwm_period_counts;
    }

    return counts;
}

/**
 * @brief  计算 int32_t 数据的绝对值并转为无符号计数
 *
 * @param value  有符号 PWM 计数
 * @return uint32_t  绝对值后的 PWM 计数
 */
static uint32_t motor_abs_i32(int32_t value)
{
    if (value < 0)
    {
        return (uint32_t)(-value);
    }

    return (uint32_t)value;
}

/**
 * @brief  设置单个 AT8236 双输入电机驱动的 PWM 输出
 *
 * @param p_channel   电机通道配置，包含 GPT 实例、A/B 输出脚和方向极性
 * @param pwm_counts  有符号 PWM 计数，正负号表示方向，绝对值表示占空比计数
 * @return fsp_err_t  FSP_SUCCESS 表示两路 PWM 占空比设置完成
 */
static fsp_err_t motor_set_dual_input(const motor_channel_t *p_channel, int32_t pwm_counts)
{
    uint32_t positive_counts = 0U;
    uint32_t negative_counts = 0U;
    int32_t corrected_pwm = pwm_counts * (int32_t)p_channel->polarity;
    uint32_t duty_counts = motor_limit_counts(motor_abs_i32(corrected_pwm));

    /* AT8236 使用双 PWM 输入控制方向：
     * 正方向时 IN1 输出 PWM、IN2 为 0；反方向时二者交换。
     * 如果实车方向相反，优先改 Chassis_config.h 中对应 polarity 宏。
     */
    if (corrected_pwm > 0)
    {
        positive_counts = duty_counts;
    }
    else if (corrected_pwm < 0)
    {
        negative_counts = duty_counts;
    }

    fsp_err_t err = p_channel->p_timer->p_api->dutyCycleSet(p_channel->p_timer->p_ctrl,
                                                            positive_counts,
                                                            p_channel->positive_pin);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return p_channel->p_timer->p_api->dutyCycleSet(p_channel->p_timer->p_ctrl,
                                                   negative_counts,
                                                   p_channel->negative_pin);
}

/**
 * @brief  初始化四路电机 PWM 定时器并默认停车
 *
 * @return fsp_err_t  FSP_SUCCESS 表示四路 GPT 已打开并启动
 */
static fsp_err_t motor_init_impl(void)
{
    fsp_err_t err = FSP_SUCCESS;

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        err = motor_open_start(g_motor_channels[i].p_timer);
        if (FSP_SUCCESS != err)
        {
            return err;
        }
    }

    timer_info_t timer_info;
    err = g_motor_channels[0].p_timer->p_api->infoGet(g_motor_channels[0].p_timer->p_ctrl, &timer_info);
    if (FSP_SUCCESS == err)
    {
        g_pwm_period_counts = timer_info.period_counts;
    }
    else
    {
        g_pwm_period_counts = CHASSIS_PWM_PERIOD_FALLBACK;
    }

    motor.stop_all();
    return FSP_SUCCESS;
}

/**
 * @brief  设置指定电机的 PWM 输出
 *
 * @param wheel       电机编号，取值为 CHASSIS_WHEEL_1 到 CHASSIS_WHEEL_4
 * @param pwm_counts  有符号 PWM 计数，正负号表示方向
 * @return fsp_err_t  FSP_SUCCESS 表示设置成功
 */
static fsp_err_t motor_set_pwm_impl(chassis_wheel_t wheel, int32_t pwm_counts)
{
    if ((uint32_t)wheel >= CHASSIS_WHEEL_COUNT)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    return motor_set_dual_input(&g_motor_channels[wheel], pwm_counts);
}

/**
 * @brief  一次性设置四路电机 PWM 输出
 *
 * @param pwm_counts  四路有符号 PWM 计数数组，数组下标对应电机编号
 * @return fsp_err_t  FSP_SUCCESS 表示四路设置成功
 */
static fsp_err_t motor_set_all_pwm_impl(const int32_t pwm_counts[CHASSIS_WHEEL_COUNT])
{
    fsp_err_t err = FSP_SUCCESS;

    if (NULL == pwm_counts)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        err = motor_set_pwm_impl((chassis_wheel_t)i, pwm_counts[i]);
        if (FSP_SUCCESS != err)
        {
            return err;
        }
    }

    return FSP_SUCCESS;
}

/**
 * @brief  停止所有电机输出
 *
 * @return void
 */
static void motor_stop_all_impl(void)
{
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        (void) motor_set_dual_input(&g_motor_channels[i], 0);
    }
}

/**
 * @brief  获取当前电机 PWM GPT 周期计数
 *
 * @return uint32_t  PWM 周期计数，用于调试或外部限幅参考
 */
static uint32_t motor_period_counts_impl(void)
{
    return g_pwm_period_counts;
}

const motor_api_t motor =
{
    .init = motor_init_impl,
    .set_pwm = motor_set_pwm_impl,
    .set_all_pwm = motor_set_all_pwm_impl,
    .stop_all = motor_stop_all_impl,
    .period_counts = motor_period_counts_impl,
};
