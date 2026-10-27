#include "LED.h"
void LED_Init(void)
{
    // 开启APB2_GPIOA时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // 恢复GPIO默认状态
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5);
    // 配置GPIO模式
    GPIO_InitTypeDef GPIO_InitType;
    GPIO_InitType.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitType.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    // 输入模式有效
    // GPIO_InitType.Pull = GPIO_PULLUP;
    GPIO_InitType.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitType);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);
    // 配置GPIO模式
    GPIO_InitType.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitType);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void Toggle_LED(uint16_t LED_ID)
{
    HAL_GPIO_TogglePin(GPIOA, LED_ID);
}

void boot_toggle_green_led(void)
{
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
}
