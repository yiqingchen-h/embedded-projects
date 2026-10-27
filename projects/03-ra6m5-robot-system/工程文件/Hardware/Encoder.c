#include "Encoder.h"
// 111 -> E1A
// 112 -> E1B
// 113 -> E2A
// 114 -> E2B
// 003 -> ADC
/* 左轮编码器 (对应 g_timer2) */
Encoder_t g_left_encoder = {
    .p_timer_ctrl = NULL, // 初始化时赋值
    .overflow_count = 0,
    .polarity = -1, // 【修改这里】如果发现方向反了，改成 -1 或 1
    .current_rpm = 0.0f,
    .current_speed_mps = 0.0f};

/* 右轮编码器 (对应 g_timer3) */
Encoder_t g_right_encoder = {
    .p_timer_ctrl = NULL, // 初始化时赋值
    .overflow_count = 0,
    .polarity = 1, // 【修改这里】如果发现方向反了，改成 -1 或 1
    .current_rpm = 0.0f,
    .current_speed_mps = 0.0f};
/* 复位函数 */
void Encoder_Reset(Encoder_t *p_enc)
{
    R_GPT_Reset(p_enc->p_timer_ctrl);
    p_enc->overflow_count = 0;
    p_enc->current_rpm = 0;
    p_enc->current_speed_mps = 0;
    p_enc->last_hardware_counter = 0;
}

void Encoder_Update_Speed(Encoder_t *p_enc, uint32_t interval_ms)
{
    if (interval_ms == 0)
        return;

    timer_status_t status;

    /* 1. 直接读取当前的 32位 硬件计数值 */
    R_GPT_StatusGet(p_enc->p_timer_ctrl, &status);
    uint32_t current_hw_counter = status.counter;

    /* 2. 计算原始脉冲增量 (利用无符号整数溢出特性)
     * 魔法所在：
     * 如果 current = 5, last = 0xFFFFFFFE (-2)
     * 5 - 0xFFFFFFFE = 7 (在32位无符号运算下是正确的)
     * 强制转换为 int32_t 后，就是有符号的增量
     */
    int32_t raw_delta = (int32_t)(current_hw_counter - p_enc->last_hardware_counter);

    /* 3. 更新历史值 */
    p_enc->last_hardware_counter = current_hw_counter;

    /* 4. 应用极性修正 (方向) */
    int32_t true_delta = raw_delta * p_enc->polarity;

    /* === 异常保护 (可选，防止硬件干扰) === */
    /* 如果10ms内变化超过 10万脉冲(约等于10000RPM)，视为噪声丢弃 */
    if (true_delta > 100000 || true_delta < -100000)
    {
        return;
    }

    /* 5. 计算 RPM */
    float rpm = ((float)true_delta / PULSES_PER_WHEEL_REV) * (60000.0f / (float)interval_ms);

    /* 6. 计算线速度 m/s */
    float speed = (rpm / 60.0f) * (WHEEL_DIAMETER_M * PI);

    /* 7. 更新输出 */
    p_enc->current_rpm = rpm;
    p_enc->current_speed_mps = speed;

}

/* ================= 中断回调函数 ================= */

/* 内部辅助函数：处理溢出逻辑 */
static void Handle_Overflow_Interrupt(Encoder_t *p_enc)
{
    gpt_instance_ctrl_t *p_ctrl = (gpt_instance_ctrl_t *)p_enc->p_timer_ctrl;

    /* 读取硬件方向寄存器 (GTUDDTYC.UD) */
    /* 1 = Up, 0 = Down */
    if (p_ctrl->p_reg->GTUDDTYC_b.UD == 1)
    {
        /* 物理上是正转溢出，但如果 polarity 是 -1，我们在逻辑上视为反转 */
        /* 不过为了保持 overflow_count 与硬件计数器一致，这里只记录物理溢出 */
        /* 极性修正在 Get_Absolute_Position 里统一处理 */
        p_enc->overflow_count++;
    }
    else
    {
        p_enc->overflow_count--;
    }
}

/* Timer 3 中断 (左轮) */
void Encode_timer3_Callback(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        Handle_Overflow_Interrupt(&g_left_encoder);
    }
}

/* Timer 2 中断 (右轮) */
void Encode_timer2_Callback(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        Handle_Overflow_Interrupt(&g_right_encoder);
    }
}
