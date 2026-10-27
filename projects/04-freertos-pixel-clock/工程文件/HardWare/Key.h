#ifndef __KEY_H
#define __KEY_H

#include "main.h"
#include <stdint.h>

/* ================= 用户配置区 ================= */

// 1: FreeRTOS模式, 0: 裸机模式
#define KEY_USE_FREERTOS 1

// 消抖时间 (ms)
#define KEY_DEBOUNCE_TIME 10

/* ============================================ */

// 按键ID定义
typedef enum
{
    KEY_NONE = 0,
    KEY_WKUP,    // PA0 (中断)
    KEY_PC4,     // (轮询)
    KEY_PC3,     // (轮询)
    KEY_PC2,     // (轮询)
    KEY_PC1,     // (轮询)
    KEY_ENC_BTN, // PE4 (轮询)
    KEY_ENC_CW,  // 编码器顺时针 (中断)
    KEY_ENC_CCW  // 编码器逆时针 (中断)
} Key_ID_t;

// 硬件初始化 (通用)
void BSP_Key_Init(void);
// 获取编码器原始计数值 (通用)
int32_t BSP_Encoder_GetCount(void);

/* ================= 接口分离定义 ================= */

#if KEY_USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern QueueHandle_t Key_QueueHandle;

// FreeRTOS 专用接口
// 1. 从队列获取按键值 (阻塞等待)
// 返回: pdTRUE (获取成功), pdFALSE (超时)
BaseType_t BSP_Key_GetMessage(Key_ID_t *key, uint32_t timeout);

// 2. 轮询任务函数 (需要在主函数创建任务时调用此函数作为入口，或者在任务中调用)
// 内部包含 while(1) 和 vTaskDelay
void BSP_Key_Poll_Task(void *argument);

#else
// 裸机 专用接口
// 1. 读取按键值 (非阻塞或阻塞消抖)
// 优先返回中断缓冲区的数据，若无则扫描GPIO
// 返回: KEY_NONE (无按键) 或 具体按键值
Key_ID_t BSP_Key_Read(void);

#endif

#endif 
