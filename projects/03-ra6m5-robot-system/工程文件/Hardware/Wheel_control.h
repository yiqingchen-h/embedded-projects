#ifndef WHEEL_CONTROL_H_
#define WHEEL_CONTROL_H_
#include "hal_data.h"
#include <stdio.h>
#include "SysTick.h"
#include <math.h> // 用于 fabs 等函数
// 414 -> PWMB	// 右轮
// 415 -> PWMA	// 左轮
// 600 -> AIN1
// 601 -> BIN2
// 602 -> AIN2
// 603 -> BIN1
// 604 -> STBY
#define left_pwm        g_timer0
#define right_pwm       g_timer0
#define AIN1            BSP_IO_PORT_06_PIN_00
#define AIN2            BSP_IO_PORT_06_PIN_02
#define BIN1            BSP_IO_PORT_06_PIN_03
#define BIN2            BSP_IO_PORT_06_PIN_01
#define STBY            BSP_IO_PORT_06_PIN_04

#define LPF_COEFFICIENT 0.7 // 低通滤波系数

/* ================= 结构体定义 ================= */
struct speed_pid {
    volatile int target_speed; // 目标速度
    float present_speed;       // 现在速度
    volatile double Kp;
    volatile double Kd;
    volatile double Ki;
    int Max_speed;
    int min_speed;
    double LPF_coefficient;  // 低通滤波系数
    double integral_min;     // 积分项最小值
    double integral_max;     // 积分项最大值
    double prev_error;       // 上次误差
    double integral;         // 积分累积
    double prev_measurement; // 上次测量值
};
typedef struct {
    /* 硬件相关 */
    timer_instance_t *p_timer_ctrl; // 指向 FSP 的控制结构体 (如 g_timer3.p_ctrl)
    timer_api_t const *p_api;
    timer_cfg_t const *p_cfg;
    ioport_instance_ctrl_t *g_ioport_ctrl; // 指向GPIO的控制结构体 (如 g_timer3.p_ctrl)
    bsp_io_port_pin_t Pin1;                // TB6612控制端口
    bsp_io_port_pin_t Pin2;
    uint8_t pin_pwm;
    /* 运行时数据 */
    volatile int16_t wheel_speed; // 溢出圈数
    struct speed_pid pid;

} Wheel_speed;

/* ================= 全局变量声明 ================= */
extern Wheel_speed g_left_speed;
extern Wheel_speed g_right_speed;

void wheel_speed_init(void);
void wheel_stop(void);
void wheel_speed_set(int left_speed, int right_speed);
// 传入目标速度和实际速度，执行PID并驱动电机
void wheel_pid_control_loop(int left_target, float left_actual, int right_target, float right_actual);
void PID_coefficient_set(double kp, double kd, double ki);

/* ================= 临时调试变量 (调试完删除) ================= */
typedef struct {
    float p_term; // P项贡献
    float i_term; // I项贡献
    float d_term; // D项贡献
    float output; // 最终输出
    float error;  // 当前误差
} PID_Debug_t;

extern PID_Debug_t g_debug_left;  // 左轮调试数据
extern PID_Debug_t g_debug_right; // 右轮调试数据

#endif