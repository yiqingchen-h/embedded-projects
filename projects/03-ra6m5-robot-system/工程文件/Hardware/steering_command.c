#include "steering_command.h"

/**
 * @brief 舵机初始化函数
 * 开启PWM输出并让舵机回到设定的中点
 */
void Servo_Init(void)
{
    fsp_err_t err;

    /* 1. 打开定时器 */
    err = g_timer4.p_api->open(g_timer4.p_ctrl, g_timer4.p_cfg);
    if (FSP_SUCCESS != err)
    {
        // 错误处理
        return;
    }

    /* 2. 使能输出 */
    err = g_timer4.p_api->enable(g_timer4.p_ctrl);

		R_GPT_OutputEnable(g_timer4.p_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB); // 开启两路输出
		
    /* 3. 启动定时器 */
    err = g_timer4.p_api->start(g_timer4.p_ctrl);

    /* 4. 舵机归中 (输入0度，函数内部会自动加上 SERVO_OFFSET_ANGLE) */

    Servo_SetAngle(0);
}

/**
 * @brief 设置舵机角度
 * @param angle 目标角度，范围建议 -90 到 +90
 */
void Servo_SetAngle(int16_t angle)
{
    int16_t target_angle;
    uint32_t pulse_width_us;
    uint32_t duty_counts;
    uint32_t period_counts;

    /* 1. 加上机械中点偏移量 */
    target_angle = angle + SERVO_OFFSET_ANGLE;

    /* 2. 软件限位检测 (Clamping) */
    if (target_angle < SERVO_LIMIT_LEFT_MAX)
    {
        target_angle = SERVO_LIMIT_LEFT_MAX;
    }
    else if (target_angle > SERVO_LIMIT_RIGHT_MAX)
    {
        target_angle = SERVO_LIMIT_RIGHT_MAX;
    }

    /* 3. 将角度映射为脉宽 (us)
       计算公式：脉宽 = 中点 + (角度 / 90) * (范围/2)
       为了避免浮点运算，使用整数运算：
       假设范围是 +-90度 对应 +-1000us (500~2500)
       系数 = 1000us / 90度 ≈ 11.11 us/度
    */
    // 线性映射计算
    pulse_width_us = (uint32_t)(SERVO_PULSE_CENTER_US + (target_angle * (SERVO_PULSE_MAX_US - SERVO_PULSE_CENTER_US) / 90));

    /* 4. 获取当前定时器周期的总计数值 (Counts) */
    period_counts = g_timer4.p_cfg->period_counts;

    /* 5. 将脉宽时间转换为定时器计数值
       公式: DutyCounts = (PulseUS / PeriodUS) * PeriodCounts
       注意：使用 uint64_t 强转防止中间计算溢出
    */
    duty_counts = (uint32_t)(((uint64_t)pulse_width_us * period_counts) / SERVO_PERIOD_US);

    /* 6. 设置占空比 */
    g_timer4.p_api->dutyCycleSet(g_timer4.p_ctrl, duty_counts, SERVO_OUTPUT_PIN);
}
