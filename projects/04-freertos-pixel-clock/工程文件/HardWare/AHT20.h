#ifndef __AHT20_H
#define __AHT20_H
#include "main.h"
/* AHT20 设备地址 (7-bit: 0x38, 左移一位后为 0x70) */
#define AHT20_ADDRESS (0x38 << 1)

/* AHT20 数据结构体 */
typedef struct
{
    float Temperature; // 温度 (摄氏度)
    float Humidity;    // 湿度 (%)
    uint8_t Alive;     // 0: 离线, 1: 在线
} AHT20_Data_t;

/* 函数声明 */
uint8_t AHT20_Init(void);
uint8_t AHT20_Read_Measure(AHT20_Data_t *pResult);
#endif
