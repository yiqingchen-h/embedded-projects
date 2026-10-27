#ifndef FOC_SELFTEST_HAL_MOCK_H
#define FOC_SELFTEST_HAL_MOCK_H
#include <stdint.h>
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t IDR; } GPIO_TypeDef;
typedef struct { uint32_t BDTR; } TIM_TypeDef;
typedef struct { uint32_t CSR; } RCC_TypeDef;
extern GPIO_TypeDef test_gpio_b;
extern TIM_TypeDef test_tim1;
extern RCC_TypeDef test_rcc;
#define GPIOB (&test_gpio_b)
#define TIM1 (&test_tim1)
#define RCC (&test_rcc)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_12 (1U << 12)
#define TIM_BDTR_MOE (1U << 15)
#define RCC_FLAG_HSERDY 1U
#define RCC_FLAG_PLLRDY 2U
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK 3U
#define __HAL_RCC_GET_FLAG(flag) (1U)
#define __HAL_RCC_GET_SYSCLK_SOURCE() RCC_SYSCLKSOURCE_STATUS_PLLCLK
#define __HAL_RCC_CLEAR_RESET_FLAGS() ((void)0)
#define __WFI() ((void)0)
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_GetDEVID(void);
void HAL_Delay(uint32_t ms);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
#endif
