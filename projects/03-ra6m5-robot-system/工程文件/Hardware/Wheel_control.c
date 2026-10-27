#include "Wheel_control.h"
Wheel_speed g_left_speed;
Wheel_speed g_right_speed;
/* ================= 内部静态函数声明 ================= */
static int pid_compute(Wheel_speed *wheel, int target, float actual);
// 临时调试使用
PID_Debug_t g_debug_left;
PID_Debug_t g_debug_right;

void wheel_speed_init(void)
{
    // ================= 左轮初始化 =================
    /* 硬件配置 */
    g_left_speed.g_ioport_ctrl = g_ioport.p_ctrl;
    g_left_speed.Pin1 = AIN1;
    g_left_speed.Pin2 = AIN2;
    g_left_speed.wheel_speed = 0;
    g_left_speed.g_ioport_ctrl = left_pwm.p_ctrl;
    g_left_speed.p_api = left_pwm.p_api;
    g_left_speed.p_cfg = left_pwm.p_cfg;
    g_left_speed.pin_pwm = GPT_IO_PIN_GTIOCA;

    /* PID 参数 */
    g_left_speed.pid.LPF_coefficient = LPF_COEFFICIENT;
    g_left_speed.pid.Kd = 0;
    g_left_speed.pid.Ki = 0;
    g_left_speed.pid.Kp = 0;
    g_left_speed.pid.Max_speed = 100;
    g_left_speed.pid.min_speed = -100;

    /*清零运行时状态变量 */
    g_left_speed.pid.present_speed = 0; // 必须清零，否则低通滤波会引入巨大的初始误差
    g_left_speed.pid.prev_error = 0;
    g_left_speed.pid.integral = 0;
    g_left_speed.pid.target_speed = 0;

    // ================= 右轮初始化 =================
    /* 硬件配置 */
    g_right_speed.g_ioport_ctrl = g_ioport.p_ctrl;
    g_right_speed.Pin1 = BIN1;
    g_right_speed.Pin2 = BIN2;
    g_right_speed.wheel_speed = 0;
    g_right_speed.g_ioport_ctrl = right_pwm.p_ctrl;
    g_right_speed.p_api = right_pwm.p_api;
    g_right_speed.p_cfg = right_pwm.p_cfg;
    g_right_speed.pin_pwm = GPT_IO_PIN_GTIOCB;

    // 开启驱动芯片
    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, STBY, 1);

    /* PID 参数 */
    g_right_speed.pid.LPF_coefficient = LPF_COEFFICIENT;
    g_right_speed.pid.Kd = 0;
    g_right_speed.pid.Ki = 0;
    g_right_speed.pid.Kp = 0;
    g_right_speed.pid.Max_speed = 100;
    g_right_speed.pid.min_speed = -100;

    /* 清零运行时状态变量 */
    g_right_speed.pid.present_speed = 0; // 必须清零！
    g_right_speed.pid.prev_error = 0;
    g_right_speed.pid.integral = 0;
    g_right_speed.pid.target_speed = 0;
}

/*
 * 函数功能：紧急停止或完全停止
 * 说明：将PWM设为0，并复位PID积分项，防止再次启动时暴冲
 */
void wheel_stop(void)
{
    // 1. 复位PID状态
    g_left_speed.pid.integral = 0;
    g_left_speed.pid.prev_error = 0;
    g_left_speed.pid.target_speed = 0;
    g_left_speed.pid.present_speed = 0;

    g_right_speed.pid.integral = 0;
    g_right_speed.pid.prev_error = 0;
    g_right_speed.pid.target_speed = 0;
    g_right_speed.pid.present_speed = 0;

    // 2. 硬件停止 (PWM归零，GPIO拉低)
    g_left_speed.p_api->dutyCycleSet(g_left_speed.g_ioport_ctrl, 0, g_left_speed.pin_pwm);
    g_right_speed.p_api->dutyCycleSet(g_right_speed.g_ioport_ctrl, 0, g_right_speed.pin_pwm);
    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, AIN1, 0);
    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, AIN2, 0);
    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BIN1, 0);
    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BIN2, 0);
}

/*
 * 函数功能：底层硬件驱动设置 (用户提供的函数)
 * 参数：
 *   left_speed:  左轮PWM占空比 (-100 到 100)
 *   right_speed: 右轮PWM占空比 (-100 到 100)
 */
void wheel_speed_set(int left_speed, int right_speed)
{
    // 左轮方向控制
    if (left_speed >= 0)
    {
        g_ioport.p_api->pinWrite(g_left_speed.g_ioport_ctrl, AIN1, 1);
        g_ioport.p_api->pinWrite(g_left_speed.g_ioport_ctrl, AIN2, 0);
    }
    else if (left_speed < 0)
    {
        g_ioport.p_api->pinWrite(g_left_speed.g_ioport_ctrl, AIN1, 0);
        g_ioport.p_api->pinWrite(g_left_speed.g_ioport_ctrl, AIN2, 1);
        left_speed = -left_speed; // 取绝对值用于PWM计算
    }
    // 右轮方向控制
    if (right_speed >= 0)
    {
        g_ioport.p_api->pinWrite(g_right_speed.g_ioport_ctrl, BIN1, 1);
        g_ioport.p_api->pinWrite(g_right_speed.g_ioport_ctrl, BIN2, 0);
    }
    else if (right_speed < 0)
    {
        g_ioport.p_api->pinWrite(g_right_speed.g_ioport_ctrl, BIN1, 0);
        g_ioport.p_api->pinWrite(g_right_speed.g_ioport_ctrl, BIN2, 1);
        right_speed = -right_speed; // 取绝对值用于PWM计算
    }

    // 设置PWM占空比
    // 注意：period_counts 代表周期计数值，乘以百分比得到高电平计数值
    g_left_speed.p_api->dutyCycleSet(g_left_speed.g_ioport_ctrl,
                                     (g_left_speed.p_cfg->period_counts) * (unsigned int)left_speed / 100,
                                     g_left_speed.pin_pwm);

    g_right_speed.p_api->dutyCycleSet(g_right_speed.g_ioport_ctrl,
                                      (g_right_speed.p_cfg->period_counts) * (unsigned int)right_speed / 100,
                                      g_right_speed.pin_pwm);
}

/*
 * [内部函数] 单轮 PID 计算逻辑
 * 参数：
 *   wheel:  轮子结构体指针
 *   target: 目标速度
 *   actual: 实际测量速度
 * 返回值：
 *   计算并限幅后的 PWM 值 (-100 到 100)
 */
static int pid_compute(Wheel_speed *wheel, int target, float actual)
{
    double error;
    double p_out, i_out, d_out;
    double output_f;

    // 1. 低通滤波 (Low Pass Filter)
    // 作用：平滑编码器读数，减少高频噪声对D项的影响
    // 公式: Y(n) = alpha * X(n) + (1 - alpha) * Y(n-1)
    wheel->pid.present_speed = (wheel->pid.LPF_coefficient * actual +
                                (1.0 - wheel->pid.LPF_coefficient) * wheel->pid.present_speed);

    // 2. 计算误差
    error = ((float)target - wheel->pid.present_speed);

    // 3. 比例项 (P)
    p_out = wheel->pid.Kp * error;

    // 4. 积分项 (I)
    wheel->pid.integral += wheel->pid.Ki * error;

    // 积分限幅 (Anti-windup) - 防止积分饱和导致响应迟滞
    if (wheel->pid.integral > wheel->pid.integral_max)
    {
        wheel->pid.integral = wheel->pid.integral_max;
    }
    else if (wheel->pid.integral < wheel->pid.integral_min)
    {
        wheel->pid.integral = wheel->pid.integral_min;
    }
    i_out = wheel->pid.integral;

    // 5. 微分项 (D)
    d_out = wheel->pid.Kd * (error - wheel->pid.prev_error);
    wheel->pid.prev_error = error;

    // 6. 总输出计算
    output_f = p_out + i_out + d_out;

    // 7. 输出限幅 (Saturation)
    // 必须限制在 -100 到 100 之间，以匹配 wheel_speed_set 的输入要求
    if (output_f > wheel->pid.Max_speed)
    {
        output_f = wheel->pid.Max_speed;
    }
    else if (output_f < wheel->pid.min_speed)
    {
        output_f = wheel->pid.min_speed;
    }

    // ================= 调试数据 =================
    // 通过指针地址判断当前计算的是左轮还是右轮
    if (wheel == &g_left_speed)
    {
        g_debug_left.error = (float)error;
        g_debug_left.p_term = (float)p_out;
        g_debug_left.i_term = (float)i_out;
        g_debug_left.d_term = (float)d_out;
        g_debug_left.output = (float)output_f;
    }
    else if (wheel == &g_right_speed)
    {
        g_debug_right.error = (float)error;
        g_debug_right.p_term = (float)p_out;
        g_debug_right.i_term = (float)i_out;
        g_debug_right.d_term = (float)d_out;
        g_debug_right.output = (float)output_f;
    }

    return (int)output_f;
}

/*
 * [核心接口函数] 周期性控制循环
 * 说明：应在定时器中断中调用 (例如每10ms或20ms)
 * 参数：
 *   left_target:  左轮目标速度
 *   left_actual:  左轮编码器读取到的实际速度
 *   right_target: 右轮目标速度
 *   right_actual: 右轮编码器读取到的实际速度
 */
void wheel_pid_control_loop(int left_target, float left_actual, int right_target, float right_actual)
{
    int left_pwm_out;
    int right_pwm_out;

    // --- 左轮 PID 计算 ---
    g_left_speed.pid.target_speed = left_target;

    // 特殊处理：如果目标速度为0，直接停止并清空积分，防止电机在0附近震荡
    if (left_target == 0)
    {
        g_left_speed.pid.integral = 0;
        g_left_speed.pid.prev_error = 0;
        left_pwm_out = 0;
        // 更新滤波值为当前值，避免下次启动时滤波滞后
        g_left_speed.pid.present_speed = left_actual;
    }
    else
    {
        left_pwm_out = pid_compute(&g_left_speed, left_target, left_actual);
    }

    // --- 右轮 PID 计算 ---
    g_right_speed.pid.target_speed = right_target;

    if (right_target == 0)
    {
        g_right_speed.pid.integral = 0;
        g_right_speed.pid.prev_error = 0;
        right_pwm_out = 0;
        g_right_speed.pid.present_speed = right_actual;
    }
    else
    {
        right_pwm_out = pid_compute(&g_right_speed, right_target, right_actual);
    }
    // --- 驱动硬件 ---
    // 将计算出的PWM值 (-100 ~ 100) 传递给底层驱动函数
    wheel_speed_set(left_pwm_out, right_pwm_out);
}

/*
 * 函数功能：设置PID参数
 * 说明：同时设置左右轮的参数，并自动计算积分限幅值
 */
void PID_coefficient_set(double kp, double kd, double ki)
{
    g_left_speed.pid.Kp = kp;
    g_left_speed.pid.Ki = ki;
    g_left_speed.pid.Kd = kd;

    g_right_speed.pid.Kp = kp;
    g_right_speed.pid.Ki = ki;
    g_right_speed.pid.Kd = kd;

    // 计算积分限幅值
    // 逻辑：积分项最大贡献设为总输出范围的60% (60)，防止积分项过大导致超调难恢复
    int output_range = g_left_speed.pid.Max_speed - g_left_speed.pid.min_speed;
    float integral_output_limit = (float)output_range * 60.0f / 200.0f; // 约等于 60
    // 或者直接理解为：最大PWM是100，积分项最多贡献80
    if (integral_output_limit > 80.0f)
        integral_output_limit = 80.0f;

    // 左轮积分限幅计算
    g_left_speed.pid.integral_max = integral_output_limit;
    g_left_speed.pid.integral_min = -integral_output_limit;

    // 右轮积分限幅计算
    g_right_speed.pid.integral_max = integral_output_limit;
    g_right_speed.pid.integral_min = -integral_output_limit;
}
