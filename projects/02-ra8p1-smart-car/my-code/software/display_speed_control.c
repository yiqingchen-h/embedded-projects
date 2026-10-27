#include "software/display_speed_control.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

/* 显示页面写入的整体速度控制值。 */
static ui_speed_control g_display_speed_control =
{
    .vx_mps = 0.0f,
    .vy_mps = 0.0f,
    .wz_radps = 0.0f,
};

/* 显示页面写入的四轮独立速度控制值。 */
static float g_display_wheel_speed_mps[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* 显示页面写入的四轮开环 PWM 诊断控制值。 */
static int32_t g_display_pwm_counts[CHASSIS_WHEEL_COUNT] = {0, 0, 0, 0};

/* 当前显示页面选择的底盘控制模式。 */
static chassis_control_mode_t g_display_speed_control_mode = CHASSIS_CONTROL_MODE_BODY;

/* 底盘调参控制使能，只有进入调参页面后才允许控制任务传入速度。 */
static bool g_display_speed_control_enabled = false;

/**
 * @brief  进入显示速度控制临界区
 *
 * @return void
 */
static void display_speed_control_enter_critical(void)
{
    taskENTER_CRITICAL();
}

/**
 * @brief  退出显示速度控制临界区
 *
 * @return void
 */
static void display_speed_control_exit_critical(void)
{
    taskEXIT_CRITICAL();
}

/**
 * @brief  清空内部速度控制值
 *
 * @return void
 */
static void display_speed_control_clear_unlocked(void)
{
    g_display_speed_control.vx_mps = 0.0f;
    g_display_speed_control.vy_mps = 0.0f;
    g_display_speed_control.wz_radps = 0.0f;
    g_display_speed_control_mode = CHASSIS_CONTROL_MODE_BODY;

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_display_wheel_speed_mps[i] = 0.0f;
        g_display_pwm_counts[i] = 0;
    }
}

/**
 * @brief  设置底盘调参控制使能
 *
 * @param enabled true 表示允许底盘控制任务传入速度
 * @return void
 */
static void display_speed_control_set_enable_impl(bool enabled)
{
    display_speed_control_enter_critical();
    g_display_speed_control_enabled = enabled;
    display_speed_control_exit_critical();
}

/**
 * @brief  设置整体速度控制值
 *
 * @param p_control 整体速度控制值
 * @return void
 */
static void display_speed_control_set_body_impl(const ui_speed_control *p_control)
{
    if (NULL == p_control)
    {
        return;
    }

    display_speed_control_enter_critical();
    g_display_speed_control = *p_control;
    g_display_speed_control_mode = CHASSIS_CONTROL_MODE_BODY;
    display_speed_control_exit_critical();
}

/**
 * @brief  设置四轮独立速度控制值
 *
 * @param wheel_speed_mps 四轮速度目标，单位 m/s
 * @return void
 */
static void display_speed_control_set_wheel_impl(const float wheel_speed_mps[CHASSIS_WHEEL_COUNT])
{
    if (NULL == wheel_speed_mps)
    {
        return;
    }

    display_speed_control_enter_critical();
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_display_wheel_speed_mps[i] = wheel_speed_mps[i];
    }
    g_display_speed_control_mode = CHASSIS_CONTROL_MODE_WHEEL;
    display_speed_control_exit_critical();
}

/**
 * @brief  设置四轮开环 PWM 诊断控制值
 *
 * @param pwm_counts 四轮 PWM 计数，正负号表示方向
 * @return void
 */
static void display_speed_control_set_pwm_impl(const int32_t pwm_counts[CHASSIS_WHEEL_COUNT])
{
    if (NULL == pwm_counts)
    {
        return;
    }

    display_speed_control_enter_critical();
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_display_pwm_counts[i] = pwm_counts[i];
    }
    g_display_speed_control_mode = CHASSIS_CONTROL_MODE_PWM;
    display_speed_control_exit_critical();
}

/**
 * @brief  清空底盘调参控制值并关闭使能
 *
 * @return void
 */
static void display_speed_control_clear_impl(void)
{
    display_speed_control_enter_critical();
    display_speed_control_clear_unlocked();
    g_display_speed_control_enabled = false;
    display_speed_control_exit_critical();
}

/**
 * @brief  读取底盘调参控制值
 *
 * @param p_control         整体速度控制值输出指针
 * @param wheel_speed_mps   四轮速度目标输出指针
 * @param pwm_counts        四轮开环 PWM 目标输出指针
 * @param p_mode            当前控制模式输出指针
 * @return bool true 表示当前允许底盘控制任务传入速度
 */
static bool display_speed_control_read_impl(ui_speed_control *p_control,
                                            float wheel_speed_mps[CHASSIS_WHEEL_COUNT],
                                            int32_t pwm_counts[CHASSIS_WHEEL_COUNT],
                                            chassis_control_mode_t *p_mode)
{
    bool enabled = false;

    display_speed_control_enter_critical();
    enabled = g_display_speed_control_enabled;

    if (NULL != p_control)
    {
        *p_control = g_display_speed_control;
    }

    if (NULL != wheel_speed_mps)
    {
        for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            wheel_speed_mps[i] = g_display_wheel_speed_mps[i];
        }
    }

    if (NULL != pwm_counts)
    {
        for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            pwm_counts[i] = g_display_pwm_counts[i];
        }
    }

    if (NULL != p_mode)
    {
        *p_mode = g_display_speed_control_mode;
    }

    display_speed_control_exit_critical();
    return enabled;
}

const display_speed_control_api_t display_speed_control =
{
    .set_enable = display_speed_control_set_enable_impl,
    .set_body = display_speed_control_set_body_impl,
    .set_wheel = display_speed_control_set_wheel_impl,
    .set_pwm = display_speed_control_set_pwm_impl,
    .clear = display_speed_control_clear_impl,
    .read = display_speed_control_read_impl,
};
