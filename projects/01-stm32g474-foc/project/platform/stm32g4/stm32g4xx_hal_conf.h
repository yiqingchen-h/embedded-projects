#ifndef STM32G4XX_HAL_CONF_H
#define STM32G4XX_HAL_CONF_H

/* 最小模板仅启用基础系统、时钟、供电和 GPIO 模块。 */
#define HAL_UART_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED

/* 板载无源晶体频率，作为 HSE 和 PLL 输入，单位 Hz。 */
#ifndef HSE_VALUE
#define HSE_VALUE 24000000U
#endif
/* 内部高速时钟频率，单位 Hz。 */
#define HSI_VALUE 16000000U
/* 内部专用高速时钟频率，单位 Hz。 */
#define HSI48_VALUE 48000000U
/* 内部低速时钟标称频率，单位 Hz。 */
#define LSI_VALUE 32000U
/* 外部低速时钟定义，仅供 HAL 编译，不表示板上装有该晶体。 */
#define LSE_VALUE 32768U
/* HSE 启动等待上限，单位 ms。 */
#define HSE_STARTUP_TIMEOUT 100U
/* LSE 启动等待上限，单位 ms。 */
#define LSE_STARTUP_TIMEOUT 5000U
/* 外部音频时钟占位定义，当前未使用。 */
#define EXTERNAL_CLOCK_VALUE 48000U
/* 数字电源标称值，单位 mV。 */
#define VDD_VALUE 3300U
/* 毫秒时基使用最低中断优先级。 */
#define TICK_INT_PRIORITY 15U
/* 模板不引入操作系统。 */
#define USE_RTOS 0U
/* 不启用 Flash 预取。 */
#define PREFETCH_ENABLE 0U
/* 启用指令缓存。 */
#define INSTRUCTION_CACHE_ENABLE 1U
/* 启用数据缓存。 */
#define DATA_CACHE_ENABLE 1U

#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_pwr.h"

/* 发布模板不启用 HAL 参数断言。 */
#define assert_param(expr) ((void)0U)

#endif
