#ifndef __RTC_CONFIG_H
#define __RTC_CONFIG_H

#include "main.h"
extern RTC_HandleTypeDef hrtc;
// 定义一个魔术字，写入备份寄存器，表示RTC已经配置过且时间有效
#define RTC_BKP_DR_VALID 0x32F2
// 初始化函数
void RTC_Config(void);


#endif
