#include "stm32g4xx_hal.h"
#include "board_test.h"

/**
 * @brief 为 HAL 维护 1 ms 时基。
 */
void
SysTick_Handler(void)
{
    HAL_IncTick();
    board_test_tick();
}

/**
 * @brief 硬故障时拉低驱动使能并停止执行。
 */
void
HardFault_Handler(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    board_trip();
    __disable_irq();
    while (1)
    {
    }
}

/** @brief 外部晶体失效时立即停机；保持锁存直到复位。 */
void
NMI_Handler(void)
{
    board_trip();
    while (1) { }
}

/** @brief 内存访问异常时立即关断。 */
void
MemManage_Handler(void) { HardFault_Handler(); }
/** @brief 总线访问异常时立即关断。 */
void
BusFault_Handler(void) { HardFault_Handler(); }
/** @brief 非法指令等用法异常时立即关断。 */
void
UsageFault_Handler(void) { HardFault_Handler(); }

/** @brief USART3接收入口，不阻塞、不打印。 */
void
USART3_IRQHandler(void) { board_uart_irq(); }

/** @brief 分发并清除本工程的EXTI挂起位。 @param mask 当前中断负责的位。 */
static void
dispatch_exti(uint32_t mask)
{
    uint32_t pending = EXTI->PR1 & mask;
    EXTI->PR1 = pending;
    board_test_exti(pending);
}

/** @brief 编码器Z边沿。 */
void
EXTI2_IRQHandler(void) { dispatch_exti(GPIO_PIN_2); }
/** @brief 编码器B边沿。 */
void
EXTI4_IRQHandler(void) { dispatch_exti(GPIO_PIN_4); }
/** @brief 编码器A边沿。 */
void
EXTI9_5_IRQHandler(void) { dispatch_exti(GPIO_PIN_5); }
/** @brief nFAULT下降沿，最高应用中断优先级。 */
void
EXTI15_10_IRQHandler(void) { dispatch_exti(GPIO_PIN_12); }
