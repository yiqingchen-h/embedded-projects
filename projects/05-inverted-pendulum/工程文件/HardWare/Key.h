#ifndef __KEY_H
#define __KEY_H
// 获取GPIO状态函数
// HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)

// 配置GPIO口电平
//  HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)

// 翻转GPIO口电平
// HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)

void Key_Init(void);

unsigned char Key_Scan(void);

// 外部中断按键初始化
void Key_Init_It(void);

unsigned char Kry_Scan_It(void);

// 按键事件模式
void Key_Init_Evevt();

#endif
