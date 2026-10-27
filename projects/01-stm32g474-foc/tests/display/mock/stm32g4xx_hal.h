#ifndef DISPLAY_TEST_HAL_H
#define DISPLAY_TEST_HAL_H
#include <stddef.h>
#include <stdint.h>
/* 仅供主机测试；真实工程使用 ST HAL。 */
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct {
    /* 被配置的引脚及模式。 */
    uint32_t Pin, Mode, Pull, Speed;
} GPIO_InitTypeDef;
typedef struct {
    /* 模拟周期计数器和使能寄存器。 */
    uint32_t CYCCNT, CTRL;
} mock_dwt_t;
typedef struct {
    /* 模拟调试追踪使能。 */
    uint32_t DEMCR;
} mock_debug_t;
extern mock_dwt_t mock_dwt;
extern mock_debug_t mock_debug;
extern uint32_t SystemCoreClock;
#define DWT (&mock_dwt)
#define CoreDebug (&mock_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define GPIOB ((void *)1)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_MODE_OUTPUT_OD 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
void mock_nop(void);
#define __NOP() mock_nop()
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *gpio);
void HAL_GPIO_WritePin(void *port, uint16_t pins, GPIO_PinState value);
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
#endif
