#ifndef ENCODER_H_
#define ENCODER_H_
#include "hal_data.h"
#include <stdio.h>
#include "SysTick.h"

/* ================= 物理参数定义 ================= */
#define ENCODER_PPR      500.0f // 编码器基础线数
#define GEAR_RATIO       28.0f  // 减速比 1:28
#define WHEEL_DIAMETER_M 0.065f // 车轮直径 (米)
#define PI               3.1415926535f

/*
 * 车轮转一圈的总脉冲数
 * 公式：线数 * 4倍频 * 减速比
 * 结果：500 * 4 * 28 = 56000
 */
#define PULSES_PER_WHEEL_REV (ENCODER_PPR * 4.0f * GEAR_RATIO)
/* ================= 结构体定义 ================= */
typedef struct {
    /* 硬件相关 */
    timer_ctrl_t *p_timer_ctrl;     // 指向 FSP 的控制结构体 (如 g_timer3.p_ctrl)
    uint32_t last_hardware_counter; /* 上一次读取的 32位 硬件计数器值 */
    /* 运行时数据 */
    volatile int32_t overflow_count; // 溢出圈数

    /* 配置项 */
    int8_t polarity; // 1: 正常, -1: 反向 (用于修正方向)

    /* 输出数据 */
    float current_rpm;       // 当前转速
    float current_speed_mps; // 当前线速度
} Encoder_t;

/* ================= 全局变量声明 ================= */
extern Encoder_t g_left_encoder;
extern Encoder_t g_right_encoder;
void Encoder_Reset(Encoder_t *p_enc);                              // 复位
void Encoder_Update_Speed(Encoder_t *p_enc, uint32_t interval_ms); // 速度计算

#endif