#include "hardware/Chassis/encoder.h"

#include <stddef.h>

#include "r_gpt.h"

typedef struct st_encoder_channel
{
    const timer_instance_t *p_timer;
    int8_t polarity;
    uint32_t last_counter;
    int32_t last_delta;
    volatile int32_t overflow_count;
    float speed_mps;
    float rpm;
} encoder_channel_t;

/* 四路编码器与 RA8P1 GPT 的对应关系：
 * W1 左前：P512/GTIOC0A，P511/GTIOC0B。
 * W2 右前：P710/GTIOC11B，P711/GTIOC11A。
 * W3 左后：P712/GTIOC2B，P713/GTIOC2A。
 * W4 右后：P714/GTIOC12B，P715/GTIOC12A。
 *
 * H2/H3/H4 在 IO 分配文档中标注 A/B 相相对计划反向，
 * 因此默认 polarity 为 -1。实车方向不对时只改 Chassis_config.h。
 */
static encoder_channel_t g_encoder_channels[CHASSIS_WHEEL_COUNT] =
{
    {&g_timer0,  CHASSIS_ENCODER1_POLARITY, 0U, 0, 0, 0.0f, 0.0f},
    {&g_timer11, CHASSIS_ENCODER2_POLARITY, 0U, 0, 0, 0.0f, 0.0f},
    {&g_timer2,  CHASSIS_ENCODER3_POLARITY, 0U, 0, 0, 0.0f, 0.0f},
    {&g_timer12, CHASSIS_ENCODER4_POLARITY, 0U, 0, 0, 0.0f, 0.0f},
};

/**
 * @brief  处理单路编码器 GPT 溢出中断
 *
 * @param wheel  电机编号，取值为 CHASSIS_WHEEL_1 到 CHASSIS_WHEEL_4
 * @return void
 */
static void encoder_handle_overflow(chassis_wheel_t wheel)
{
    if ((uint32_t)wheel >= CHASSIS_WHEEL_COUNT)
    {
        return;
    }

    encoder_channel_t *p_channel = &g_encoder_channels[wheel];
    gpt_instance_ctrl_t *p_ctrl = (gpt_instance_ctrl_t *)p_channel->p_timer->p_ctrl;

    /* 当前 GPT 编码器配置主要是外部事件向上计数。这里仍然读取硬件方向位：
     * 如果后续改成上下计数模式，溢出累计方向也能跟着硬件方向变化。
     */
    if (1U == p_ctrl->p_reg->GTUDDTYC_b.UD)
    {
        p_channel->overflow_count++;
    }
    else
    {
        p_channel->overflow_count--;
    }
}

/**
 * @brief  打开、复位并启动一个 GPT 编码器计数定时器
 *
 * @param p_timer  FSP 生成的定时器实例指针
 * @return fsp_err_t  FSP_SUCCESS 表示该路编码器定时器已开始计数
 */
static fsp_err_t encoder_open_start(const timer_instance_t *p_timer)
{
    fsp_err_t err = p_timer->p_api->open(p_timer->p_ctrl, p_timer->p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    err = p_timer->p_api->reset(p_timer->p_ctrl);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return p_timer->p_api->start(p_timer->p_ctrl);
}

/**
 * @brief  清空单路编码器的软件测速状态
 *
 * @param p_channel  编码器通道状态结构体
 * @return void
 */
static void encoder_clear_channel(encoder_channel_t *p_channel)
{
    timer_status_t status;

    p_channel->last_delta = 0;
    p_channel->overflow_count = 0;
    p_channel->speed_mps = 0.0f;
    p_channel->rpm = 0.0f;

    if (FSP_SUCCESS == p_channel->p_timer->p_api->statusGet(p_channel->p_timer->p_ctrl, &status))
    {
        p_channel->last_counter = status.counter;
    }
    else
    {
        p_channel->last_counter = 0U;
    }
}

/**
 * @brief  初始化四路编码器 GPT 定时器
 *
 * @return fsp_err_t  FSP_SUCCESS 表示四路编码器定时器均已启动
 */
static fsp_err_t encoder_init_impl(void)
{
    fsp_err_t err = FSP_SUCCESS;

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        err = encoder_open_start(g_encoder_channels[i].p_timer);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        encoder_clear_channel(&g_encoder_channels[i]);
    }

    return FSP_SUCCESS;
}

/**
 * @brief  复位四路编码器计数和软件测速状态
 *
 * @return void
 */
static void encoder_reset_impl(void)
{
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        (void) g_encoder_channels[i].p_timer->p_api->reset(g_encoder_channels[i].p_timer->p_ctrl);
        encoder_clear_channel(&g_encoder_channels[i]);
    }
}

/**
 * @brief  更新单路编码器速度
 *
 * @param p_channel    编码器通道状态结构体
 * @param interval_ms  两次测速之间的周期，单位 ms
 * @return fsp_err_t   FSP_SUCCESS 表示本次速度计算成功
 */
static fsp_err_t encoder_update_one(encoder_channel_t *p_channel, uint32_t interval_ms)
{
    timer_status_t status;
    fsp_err_t err = p_channel->p_timer->p_api->statusGet(p_channel->p_timer->p_ctrl, &status);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* 32 位 GPT 计数器可能回绕，先用无符号差值保留回绕结果，
     * 再强制转为 int32_t，得到带方向的计数差。
     */
    int32_t raw_delta = (int32_t)(status.counter - p_channel->last_counter);
    int32_t true_delta = raw_delta * (int32_t)p_channel->polarity;
    float interval_scale = 60000.0f / (float)interval_ms;

    p_channel->last_counter = status.counter;
    p_channel->last_delta = true_delta;
    p_channel->rpm = ((float)true_delta / CHASSIS_PULSES_PER_WHEEL_REV) * interval_scale;
    p_channel->speed_mps = (p_channel->rpm / 60.0f) * (CHASSIS_WHEEL_DIAMETER_M * CHASSIS_PI);

    return FSP_SUCCESS;
}

/**
 * @brief  更新四路编码器轮速
 *
 * @param interval_ms      两次测速之间的周期，单位 ms
 * @param wheel_speed_mps  输出四路轮速，单位 m/s，可传 NULL 表示只更新内部状态
 * @return fsp_err_t       FSP_SUCCESS 表示四路速度更新成功
 */
static fsp_err_t encoder_update_impl(uint32_t interval_ms, float wheel_speed_mps[CHASSIS_WHEEL_COUNT])
{
    fsp_err_t err = FSP_SUCCESS;

    if (0U == interval_ms)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        err = encoder_update_one(&g_encoder_channels[i], interval_ms);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        if (NULL != wheel_speed_mps)
        {
            wheel_speed_mps[i] = g_encoder_channels[i].speed_mps;
        }
    }

    return FSP_SUCCESS;
}

/**
 * @brief  获取指定电机当前轮速
 *
 * @param wheel  电机编号，取值为 CHASSIS_WHEEL_1 到 CHASSIS_WHEEL_4
 * @return float  当前轮速，单位 m/s
 */
static float encoder_get_speed_impl(chassis_wheel_t wheel)
{
    if ((uint32_t)wheel >= CHASSIS_WHEEL_COUNT)
    {
        return 0.0f;
    }

    return g_encoder_channels[wheel].speed_mps;
}

/**
 * @brief  获取指定电机当前转速
 *
 * @param wheel  电机编号，取值为 CHASSIS_WHEEL_1 到 CHASSIS_WHEEL_4
 * @return float  当前转速，单位 rpm
 */
static float encoder_get_rpm_impl(chassis_wheel_t wheel)
{
    if ((uint32_t)wheel >= CHASSIS_WHEEL_COUNT)
    {
        return 0.0f;
    }

    return g_encoder_channels[wheel].rpm;
}

/**
 * @brief  获取指定编码器的 GPT 溢出累计值
 *
 * @param wheel  电机编号，取值为 CHASSIS_WHEEL_1 到 CHASSIS_WHEEL_4
 * @return int32_t  溢出累计值，向上计数溢出加 1，向下计数溢出减 1
 */
static int32_t encoder_get_overflow_count_impl(chassis_wheel_t wheel)
{
    if ((uint32_t)wheel >= CHASSIS_WHEEL_COUNT)
    {
        return 0;
    }

    return g_encoder_channels[wheel].overflow_count;
}

const encoder_api_t encoder =
{
    .init = encoder_init_impl,
    .reset = encoder_reset_impl,
    .update = encoder_update_impl,
    .get_speed = encoder_get_speed_impl,
    .get_rpm = encoder_get_rpm_impl,
    .get_overflow_count = encoder_get_overflow_count_impl,
};

/**
 * @brief  GPT12 编码器溢出回调
 *
 * @param p_args  FSP 定时器回调参数
 * @return void
 */
void timer12_callback(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        encoder_handle_overflow(CHASSIS_WHEEL_4);
    }
}

/**
 * @brief  GPT11 编码器溢出回调
 *
 * @param p_args  FSP 定时器回调参数
 * @return void
 */
void timer11_callback(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        encoder_handle_overflow(CHASSIS_WHEEL_2);
    }
}

/**
 * @brief  GPT2 编码器溢出回调
 *
 * @param p_args  FSP 定时器回调参数
 * @return void
 */
void timer2_callback(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        encoder_handle_overflow(CHASSIS_WHEEL_3);
    }
}

/**
 * @brief  GPT0 编码器溢出回调
 *
 * @param p_args  FSP 定时器回调参数
 * @return void
 */
void timer0_callback(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        encoder_handle_overflow(CHASSIS_WHEEL_1);
    }
}
