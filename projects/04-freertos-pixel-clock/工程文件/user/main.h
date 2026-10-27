#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "Rcc.h"
#include "UART.h"
#include "My_FreeRTOS.h"
#include "IIC.h"
#include "LED.h"
#include "AHT20.h"
#include "24C512.h"
#include "ESP_12F.h"
#include "RTC.h"
#include "WS2812B.h"
#include "Key.h"
#include "Fan.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

// 0,不支持ucos
// 1,支持ucos
#define SYSTEM_SUPPORT_OS 0 // 定义系统文件夹是否支持UCOS

#endif /* __MAIN_H */
