#ifndef KEY_H_
#define KEY_H_
#include "hal_data.h"
#include <stdint.h>
#include <stdbool.h>
// void GPIO_Pin_ON(ioport_instance_t GPIO_instance,bsp_io_port_pin_t pin);

// void GPIO_Pin_OFF(ioport_instance_t GPIO_instance,bsp_io_port_pin_t pin);

// unsigned char Key_State_Read(void);

// ================= 用户配置区 =================

#define KEY_COUNT     5  // 按键总数
#define KEY_FIFO_SIZE 20 // 事件队列深度

// --- 时间参数 (单位: ms) ---
#define KEY_SCAN_INTERVAL 5    // 按键扫描周期
#define TIME_DEBOUNCE     10   // 消抖时间
#define TIME_DOUBLE_WAIT  300  // 双击判定窗口
#define TIME_LONG_PRESS   700 // 长按判定时间

// --- 编码器引脚定义 ---
#define ENC_PIN_A BSP_IO_PORT_09_PIN_05
#define ENC_PIN_B BSP_IO_PORT_09_PIN_06

// --- 编码器高级配置 ---

// [配置1] 计数方向极性
//  1: 默认方向
// -1: 反向 (如果发现顺时针数值变小，改为 -1)
#define ENC_POLARITY (-1)

// [配置2] 编码器分辨率分频
// EC11 编码器通常转动一格(Detent)会产生 4 次电平跳变(双边沿检测下)
// 设为 4: 转一格 计数+1 (推荐)
// 设为 2: 转一格 计数+2
// 设为 1: 转一格 计数+4 (极高灵敏度)
#define ENC_PULSES_PER_STEP 4

// ===========================================

// 按键ID枚举
typedef enum {
    KEY_ID_NONE = 0,
    KEY_ID_1,
    KEY_ID_2,
    KEY_ID_3,
    KEY_ID_4,
    KEY_ID_5 // 编码器自带的按键
} key_id_t;

// 事件类型枚举
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT,  // 短按
    KEY_EVENT_LONG,   // 长按
    KEY_EVENT_DOUBLE, // 双击
    KEY_EVENT_ROT_CW, // 编码器顺时针 (Clockwise)
    KEY_EVENT_ROT_CCW // 编码器逆时针 (Counter-Clockwise)
} key_event_type_t;

// 事件结构体
typedef volatile struct {
    key_id_t id;           // 哪个按键/编码器
    key_event_type_t type; // 发生了什么事
    int32_t count;         // [核心] 事件发生时的编码器计数值 (快照)
} key_event_t;

// --- 函数声明 ---
void Key_Init(void);                      // 初始化
void Key_Tick_Handler(void);              // 定时器中断调用 (提供时间基准)
void Key_Scan(void);                      // 主循环调用 (处理普通按键)
bool Key_Get_Event(key_event_t *p_event); // 获取事件

// 编码器专用接口
int32_t Key_Get_Encoder_Count(void);       // 获取当前计数值
void Key_Set_Encoder_Count(int32_t count); // 设置/清零计数值
#endif
