#include "stm32f1xx_hal.h"
#include "RCC.h"
#include "OLED.h"
#include "Stepper_motor_drive.h"
/// 串口控制 tmc2209
#include "stm32f1xx_hal.h"

UART_HandleTypeDef UART1_HandleType;

// 传入参数为波特率
void U1_Init(unsigned int bandrate)
{
    UART1_HandleType.Instance        = USART1;
    UART1_HandleType.Init.BaudRate   = bandrate;
    UART1_HandleType.Init.WordLength = UART_WORDLENGTH_8B;
    UART1_HandleType.Init.StopBits   = UART_STOPBITS_1;
    UART1_HandleType.Init.Parity     = UART_PARITY_NONE;
    UART1_HandleType.Init.Mode       = UART_MODE_TX_RX;
    HAL_UART_Init(&UART1_HandleType);
}

// HAL_UART_Init--会调用下方的函数
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitType;
    if (huart->Instance == USART1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // 串口1时钟
        __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitType.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitType.Pin   = GPIO_PIN_9;
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);

        GPIO_InitType.Mode = GPIO_MODE_AF_INPUT;
        GPIO_InitType.Pin  = GPIO_PIN_10;
        GPIO_InitType.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);

        // 配置中断优先级
        HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
        // 使能
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

// CRC计算
uint8_t tmc_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? 0x07 : 0);
        }
    }
    return crc;
}

// 启用驱动器并配置UART模式
void TMC2209_Init(void)
{
    // GCONF: pdn_disable=1, motor_en=1
    uint8_t init_cmd[] = {0x05, 0x00, 0x80, 0x00, 0x00, 0x00, 0x48, 0x00};
    init_cmd[7]        = tmc_crc(init_cmd, 7);
    HAL_UART_Transmit(&UART1_HandleType, init_cmd, 8, 100);

    // // CHOPCONF: 微步1/256
    // uint8_t chopconf_cmd[] = {0x05, 0x00, 0xEC, 0x00, 0x00, 0x08, 0x00, 0x00};
    // chopconf_cmd[7]        = tmc_crc(chopconf_cmd, 7);
    // HAL_UART_Transmit(&UART1_HandleType, chopconf_cmd, 8, 100);
}

// 运动控制
void tmc_move(int32_t steps)
{
    uint8_t cmd[] = {
        0x05, 0x00, 0x2A | 0x80,
        (uint8_t)(steps >> 24) & 0xFF,
        (uint8_t)(steps >> 16) & 0xFF,
        (uint8_t)(steps >> 8) & 0xFF,
        (uint8_t)steps & 0xFF,
        0x00 // CRC placeholder
    };
    cmd[7] = tmc_crc(cmd, 7);
    HAL_UART_Transmit(&UART1_HandleType, cmd, sizeof(cmd), 100);
    OLED_ShouHexadecimal(0, 0, cmd[6], 2, OLED_8X16);
}

int main(void)
{
    HAL_Init();
    RccClock_HSE_Init(); // 需要实现时钟配置
    OLED_Init();
    /*串口驱动使用*/
    // U1_Init(115200);
    // TMC2209_Init();
    // while (1) {
    //     tmc_move(2000 * 256); // 正向200步
    //     HAL_Delay(2000);
    //     tmc_move(-2000 * 256); // 反向200步
    //     HAL_Delay(2000);
    // }
    Stepper_Init(0);
    Stepper_Move(90, GPIO_PIN_SET, subdivision_64);
    while (1) {
        OLED_ShowChar(0, 0, 'b', OLED_8X16);
    }
}