#include "stm32g4xx_hal.h"

/**
 * @brief 为 HAL 维护 1 ms 时基。
 */
void
SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief 硬故障时拉低驱动使能并停止执行。
 */
void
HardFault_Handler(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    __disable_irq();
    while (1)
    {
    }
}
