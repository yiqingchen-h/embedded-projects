#ifndef __LED_H
#define __LED_H
#include "main.h"
// 获取GPIO状态函数
// HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)

//配置GPIO口电平
// HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)

//翻转GPIO口电平
//HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)

void LED_Init(void);
void Toggle_LED(uint16_t LED_ID);
void boot_toggle_green_led(void);
#endif
