#include "hardware/Chassis/uart_motor.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "hardware/Chassis/kinematics.h"

#define UART_MOTOR_CMD_BUFFER_SIZE   (64U)
#define UART_MOTOR_TX_TIMEOUT_MS     (100U)
#define UART_MOTOR_SPEED_SCALE       (1000.0f)
#define UART_MOTOR_SPEED_CMD_MIN     (-1000)
#define UART_MOTOR_SPEED_CMD_MAX     (1000)

/* 如果实车某一路方向相反，只改对应极性，不改麦轮逆运动学公式。 */
#define UART_MOTOR_M1_POLARITY       (-1)
#define UART_MOTOR_M2_POLARITY       (-1)
#define UART_MOTOR_M3_POLARITY       (-1)
#define UART_MOTOR_M4_POLARITY       (-1)

static bool g_uart_motor_uart3_opened = false;
static volatile bool g_uart_motor_tx_complete = false;
static volatile bool g_uart_motor_uart_error = false;
static char g_uart_motor_last_cmd[UART_MOTOR_CMD_BUFFER_SIZE];
static fsp_err_t g_uart_motor_last_error = FSP_SUCCESS;

static fsp_err_t uart_motor_open(void);
static void uart_motor_delay_ms(uint32_t ms);
static fsp_err_t uart_motor_wait_tx_complete(uint32_t timeout_ms);
static void uart_motor_record_last_command(const char *cmd);
static float uart_motor_limit_float(float value, float low, float high);
static int32_t uart_motor_limit_i32(int32_t value, int32_t low, int32_t high);
static int32_t uart_motor_float_to_i32(float value);
static void uart_motor_inverse(const chassis_velocity_t *p_velocity, float wheel_targets_mps[CHASSIS_WHEEL_COUNT]);
static int16_t uart_motor_mps_to_cmd(float speed_mps, int32_t polarity);
static fsp_err_t uart_motor_write_command(const char *cmd);
static fsp_err_t uart_motor_send_spd(const int16_t speed_cmd[CHASSIS_WHEEL_COUNT]);

/**
 * @brief  打开 UART3，允许重复调用
 *
 * @return fsp_err_t  FSP_SUCCESS 表示 UART3 可用
 */
static fsp_err_t uart_motor_open(void)
{
    if (g_uart_motor_uart3_opened)
    {
        return FSP_SUCCESS;
    }

    fsp_err_t err = g_uart3.p_api->open(g_uart3.p_ctrl, g_uart3.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        g_uart_motor_last_error = err;
        return err;
    }

    g_uart_motor_uart3_opened = true;
    g_uart_motor_tx_complete = false;
    g_uart_motor_uart_error = false;
    g_uart_motor_last_error = FSP_SUCCESS;

    return FSP_SUCCESS;
}

/**
 * @brief  等待指定毫秒数，兼容调度器启动前后
 *
 * @param ms  等待时间，单位 ms
 * @return void
 */
static void uart_motor_delay_ms(uint32_t ms)
{
    TickType_t delay_ticks;

    if (0U == ms)
    {
        return;
    }

    delay_ticks = pdMS_TO_TICKS(ms);
    if ((taskSCHEDULER_RUNNING == xTaskGetSchedulerState()) && (delay_ticks > 0U))
    {
        vTaskDelay(delay_ticks);
    }
    else
    {
        R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

/**
 * @brief  等待 UART3 异步发送完成
 *
 * @param timeout_ms  超时时间，单位 ms
 * @return fsp_err_t  FSP_SUCCESS 表示发送完成
 */
static fsp_err_t uart_motor_wait_tx_complete(uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0U;

    while (!g_uart_motor_tx_complete)
    {
        if (g_uart_motor_uart_error)
        {
            return FSP_ERR_ABORTED;
        }

        if (elapsed_ms >= timeout_ms)
        {
            return FSP_ERR_TIMEOUT;
        }

        uart_motor_delay_ms(1U);
        elapsed_ms++;
    }

    if (g_uart_motor_uart_error)
    {
        return FSP_ERR_ABORTED;
    }

    g_uart_motor_tx_complete = false;
    return FSP_SUCCESS;
}

/**
 * @brief  记录最近一次发送的命令
 *
 * @param cmd  命令字符串
 * @return void
 */
static void uart_motor_record_last_command(const char *cmd)
{
    if (NULL == cmd)
    {
        g_uart_motor_last_cmd[0] = '\0';
        return;
    }

    (void)strncpy(g_uart_motor_last_cmd, cmd, sizeof(g_uart_motor_last_cmd) - 1U);
    g_uart_motor_last_cmd[sizeof(g_uart_motor_last_cmd) - 1U] = '\0';
}

/**
 * @brief  对 float 做上下限限幅
 *
 * @param value  待限幅数据
 * @param low    最小值
 * @param high   最大值
 * @return float 限幅后的数据
 */
static float uart_motor_limit_float(float value, float low, float high)
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
 * @brief  对 int32_t 做上下限限幅
 *
 * @param value  待限幅数据
 * @param low    最小值
 * @param high   最大值
 * @return int32_t  限幅后的数据
 */
static int32_t uart_motor_limit_i32(int32_t value, int32_t low, int32_t high)
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
 * @brief  将 float 四舍五入为 int32_t
 *
 * @param value  浮点输入
 * @return int32_t  四舍五入后的整数
 */
static int32_t uart_motor_float_to_i32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

/**
 * @brief  串口电机板麦轮逆运动学解算
 *
 * @param p_velocity         车体目标速度，vx/vy 单位 m/s，wz 单位 rad/s
 * @param wheel_targets_mps  输出四路轮目标线速度，单位 m/s，数组下标沿用 CHASSIS_WHEEL_x
 * @return void
 */
static void uart_motor_inverse(const chassis_velocity_t *p_velocity, float wheel_targets_mps[CHASSIS_WHEEL_COUNT])
{
    if ((NULL == p_velocity) || (NULL == wheel_targets_mps))
    {
        return;
    }

    float rotation_radius = CHASSIS_HALF_WHEEL_SPACING_M + CHASSIS_HALF_AXLE_SPACING_M;
    float rotation_speed = p_velocity->wz_radps * rotation_radius;

    /* 串口电机板实车横移需要对角轮反向：
     * vx 为前进，vy 为左平移，wz 为逆时针旋转。
     */
    wheel_targets_mps[CHASSIS_WHEEL_LEFT_FRONT] =
        p_velocity->vx_mps - p_velocity->vy_mps - rotation_speed;
    wheel_targets_mps[CHASSIS_WHEEL_RIGHT_FRONT] =
        p_velocity->vx_mps + p_velocity->vy_mps + rotation_speed;
    wheel_targets_mps[CHASSIS_WHEEL_LEFT_REAR] =
        p_velocity->vx_mps + p_velocity->vy_mps - rotation_speed;
    wheel_targets_mps[CHASSIS_WHEEL_RIGHT_REAR] =
        p_velocity->vx_mps - p_velocity->vy_mps + rotation_speed;

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        wheel_targets_mps[i] = uart_motor_limit_float(wheel_targets_mps[i],
                                                      -CHASSIS_MAX_WHEEL_SPEED_MPS,
                                                      CHASSIS_MAX_WHEEL_SPEED_MPS);
    }
}

/**
 * @brief  将轮速 m/s 换算为串口电机板速度命令
 *
 * @param speed_mps  轮速，单位 m/s
 * @param polarity   单路方向极性
 * @return int16_t   串口电机板速度值，范围 -1000~1000
 */
static int16_t uart_motor_mps_to_cmd(float speed_mps, int32_t polarity)
{
    int32_t speed_cmd = uart_motor_float_to_i32(speed_mps * UART_MOTOR_SPEED_SCALE);

    speed_cmd *= polarity;
    speed_cmd = uart_motor_limit_i32(speed_cmd, UART_MOTOR_SPEED_CMD_MIN, UART_MOTOR_SPEED_CMD_MAX);

    return (int16_t)speed_cmd;
}

/**
 * @brief  通过 UART3 发送一条完整 ASCII 命令
 *
 * @param cmd  以 '\0' 结尾的命令字符串
 * @return fsp_err_t  FSP_SUCCESS 表示发送完成
 */
static fsp_err_t uart_motor_write_command(const char *cmd)
{
    fsp_err_t err;

    if ((NULL == cmd) || ('\0' == cmd[0]))
    {
        g_uart_motor_last_error = FSP_ERR_INVALID_ARGUMENT;
        return g_uart_motor_last_error;
    }

    err = uart_motor_open();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    uart_motor_record_last_command(cmd);
    g_uart_motor_tx_complete = false;
    g_uart_motor_uart_error = false;

    err = g_uart3.p_api->write(g_uart3.p_ctrl, (uint8_t const *)cmd, (uint32_t)strlen(cmd));
    if (FSP_SUCCESS != err)
    {
        g_uart_motor_last_error = err;
        return err;
    }

    err = uart_motor_wait_tx_complete(UART_MOTOR_TX_TIMEOUT_MS);
    g_uart_motor_last_error = err;
    return err;
}

/**
 * @brief  发送 $spd:m1,m2,m3,m4# 速度命令
 *
 * @param speed_cmd  电机板 M1/M2/M3/M4 顺序的速度值
 * @return fsp_err_t  FSP_SUCCESS 表示命令发送成功
 */
static fsp_err_t uart_motor_send_spd(const int16_t speed_cmd[CHASSIS_WHEEL_COUNT])
{
    char cmd[UART_MOTOR_CMD_BUFFER_SIZE];
    int len;

    if (NULL == speed_cmd)
    {
        g_uart_motor_last_error = FSP_ERR_INVALID_POINTER;
        return g_uart_motor_last_error;
    }

    len = snprintf(cmd,
                   sizeof(cmd),
                   "$spd:%d,%d,%d,%d#",
                   (int)speed_cmd[0],
                   (int)speed_cmd[1],
                   (int)speed_cmd[2],
                   (int)speed_cmd[3]);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_uart_motor_last_error = FSP_ERR_INVALID_SIZE;
        return g_uart_motor_last_error;
    }

    return uart_motor_write_command(cmd);
}

/**
 * @brief  初始化 UART3 串口电机板，并默认停车
 *
 * @return fsp_err_t  FSP_SUCCESS 表示初始化成功
 */
static fsp_err_t uart_motor_init_impl(void)
{
    fsp_err_t err = uart_motor_open();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return uart_motor.stop_all();
}

/**
 * @brief  设置底盘目标车体速度
 *
 * @param vx_mps    前进方向速度，单位 m/s
 * @param vy_mps    横移方向速度，单位 m/s
 * @param wz_radps  旋转角速度，单位 rad/s
 * @return fsp_err_t  FSP_SUCCESS 表示速度命令发送成功
 */
static fsp_err_t uart_motor_set_velocity_impl(float vx_mps, float vy_mps, float wz_radps)
{
    chassis_velocity_t velocity;
    float wheel_speed_mps[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

    velocity.vx_mps = vx_mps;
    velocity.vy_mps = vy_mps;
    velocity.wz_radps = wz_radps;

    uart_motor_inverse(&velocity, wheel_speed_mps);

    return uart_motor.set_wheel_speed(wheel_speed_mps);
}

/**
 * @brief  按现有底盘轮序设置四路轮速
 *
 * @param wheel_speed_mps  CHASSIS_WHEEL_x 顺序的四路轮速，单位 m/s
 * @return fsp_err_t       FSP_SUCCESS 表示速度命令发送成功
 */
static fsp_err_t uart_motor_set_wheel_speed_impl(const float wheel_speed_mps[CHASSIS_WHEEL_COUNT])
{
    int16_t speed_cmd[CHASSIS_WHEEL_COUNT];

    if (NULL == wheel_speed_mps)
    {
        g_uart_motor_last_error = FSP_ERR_INVALID_POINTER;
        return g_uart_motor_last_error;
    }

    /* 串口板命令顺序与当前 PWM 实车轮序不同，这里按轮位宏重排。 */
    speed_cmd[0] = uart_motor_mps_to_cmd(wheel_speed_mps[CHASSIS_WHEEL_LEFT_FRONT], UART_MOTOR_M1_POLARITY);
    speed_cmd[1] = uart_motor_mps_to_cmd(wheel_speed_mps[CHASSIS_WHEEL_LEFT_REAR], UART_MOTOR_M2_POLARITY);
    speed_cmd[2] = uart_motor_mps_to_cmd(wheel_speed_mps[CHASSIS_WHEEL_RIGHT_FRONT], UART_MOTOR_M3_POLARITY);
    speed_cmd[3] = uart_motor_mps_to_cmd(wheel_speed_mps[CHASSIS_WHEEL_RIGHT_REAR], UART_MOTOR_M4_POLARITY);

    return uart_motor_send_spd(speed_cmd);
}

/**
 * @brief  停止四路串口电机输出
 *
 * @return fsp_err_t  FSP_SUCCESS 表示停车命令发送成功
 */
static fsp_err_t uart_motor_stop_all_impl(void)
{
    const int16_t stop_cmd[CHASSIS_WHEEL_COUNT] = {0, 0, 0, 0};

    return uart_motor_send_spd(stop_cmd);
}

/**
 * @brief  获取最近一次发送的命令
 *
 * @return const char*  最近一次命令字符串
 */
static const char *uart_motor_get_last_command_impl(void)
{
    return g_uart_motor_last_cmd;
}

/**
 * @brief  获取最近一次错误码
 *
 * @return fsp_err_t  最近一次错误码
 */
static fsp_err_t uart_motor_get_last_error_impl(void)
{
    return g_uart_motor_last_error;
}

const uart_motor_api_t uart_motor =
{
    .init = uart_motor_init_impl,
    .set_velocity = uart_motor_set_velocity_impl,
    .set_wheel_speed = uart_motor_set_wheel_speed_impl,
    .stop_all = uart_motor_stop_all_impl,
    .get_last_command = uart_motor_get_last_command_impl,
    .get_last_error = uart_motor_get_last_error_impl,
};

/**
 * @brief  UART3 回调，只处理发送完成和错误事件
 *
 * @param p_args  UART 回调参数
 * @return void
 */
void uart_3_callback(uart_callback_args_t *p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
    case UART_EVENT_TX_COMPLETE:
        g_uart_motor_tx_complete = true;
        break;

    case UART_EVENT_ERR_PARITY:
    case UART_EVENT_ERR_FRAMING:
    case UART_EVENT_ERR_OVERFLOW:
    case UART_EVENT_BREAK_DETECT:
        g_uart_motor_uart_error = true;
        g_uart_motor_tx_complete = true;
        break;

    case UART_EVENT_TX_DATA_EMPTY:
    case UART_EVENT_RX_CHAR:
    case UART_EVENT_RX_COMPLETE:
    default:
        break;
    }
}
