#include "GPT.h"
#include "Robot_Control.h"

CarState_t g_car_state = CAR_STATE_STOP; // 初始化为停止状态
int g_nav_target_speed = 0;              // 初始化导航目标速度为0

/* ================= 初始化函数 ================= */
void Init_All_Timers(void)
{
    /* 关联硬件控制指针 */
    g_left_encoder.p_timer_ctrl = g_timer3.p_ctrl;
    g_right_encoder.p_timer_ctrl = g_timer2.p_ctrl;

    /* 复位软件变量 */
    Encoder_Reset(&g_left_encoder);
    Encoder_Reset(&g_right_encoder);
    wheel_speed_init(); // 初始化轮速控制
    Navigation_Init();  // 导航参数初始化

    /* 设置初始占空比 */
    g_timer0.p_api->dutyCycleSet(g_timer0.p_ctrl, (g_timer0.p_cfg->period_counts) * 0 / 100, GPT_IO_PIN_GTIOCB);
    g_timer0.p_api->dutyCycleSet(g_timer0.p_ctrl, (g_timer0.p_cfg->period_counts) * 0 / 100, GPT_IO_PIN_GTIOCA);

    /* 打开并启动所有定时器 */
    // Timer 3 (右轮编码器)
    g_timer3.p_api->open(g_timer3.p_ctrl, g_timer3.p_cfg);
    g_timer3.p_api->enable(g_timer3.p_ctrl);
    g_timer3.p_api->start(g_timer3.p_ctrl);

    // Timer 2 (左轮编码器)
    g_timer2.p_api->open(g_timer2.p_ctrl, g_timer2.p_cfg);
    g_timer2.p_api->enable(g_timer2.p_ctrl);
    g_timer2.p_api->start(g_timer2.p_ctrl);

    // Timer 0, 4 (PWM电机驱动)
    g_timer0.p_api->open(g_timer0.p_ctrl, g_timer0.p_cfg);
    g_timer0.p_api->enable(g_timer0.p_ctrl);
    g_timer0.p_api->start(g_timer0.p_ctrl);

    wheel_stop();
    Servo_Init(); // 舵机初始化
    g_timer6.p_api->open(g_timer6.p_ctrl, g_timer6.p_cfg);
    g_timer6.p_api->enable(g_timer6.p_ctrl);
}

void timer_1ms_start(void)
{
    g_timer6.p_api->start(g_timer6.p_ctrl);
}

void timer_1ms_stop(void)
{
    g_timer6.p_api->stop(g_timer6.p_ctrl);
}

volatile bool get_voltage_mark = true;

/*
 * Timer 6 中断 (1ms 周期)
 *
 * 当前职责刻意保持很轻：
 * 1. 维护按键节拍。
 * 2. 每 10ms 只调用一次 RobotControl_TimerTask_10ms()。
 * 3. 维护电压采样标志位。
 *
 * 这样做的目的：
 * 1. 中断里不再堆叠大块业务代码。
 * 2. 真正的控制逻辑由 Robot_Control 统一协调。
 */
void gpt_timer6_callback(timer_callback_args_t *p_args)
{
    static uint8_t loop_cnt = 0;
    static uint16_t voltage_mark_cnt = 0;

    if ((NULL == p_args) || (TIMER_EVENT_CYCLE_END != p_args->event))
    {
        return;
    }

    Key_Tick_Handler(); // 按键时钟
    loop_cnt++;
    voltage_mark_cnt++; // ADC 读取标志位计数

    /* 每 10ms 进入一次协调层快任务入口 */
    if (loop_cnt >= 10U)
    {
        loop_cnt = 0U;
        RobotControl_TimerTask_10ms();
    }

    /* 每 1s 置位一次电压读取标志 */
    if (voltage_mark_cnt >= 10000U)
    {
        get_voltage_mark = true;
        voltage_mark_cnt = 0U;
    }
}
