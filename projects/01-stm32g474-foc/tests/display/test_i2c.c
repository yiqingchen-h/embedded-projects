#include "board_i2c.h"
#include "stm32g4xx_hal.h"
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

/* 硬件寄存器测试替身。 */
mock_dwt_t mock_dwt;
mock_debug_t mock_debug;
/* 与板上配置一致，单位 Hz。 */
uint32_t SystemCoreClock = 170000000U;
/* 模拟输出锁存、时钟边沿、发送数据以及异常模式。 */
static uint16_t g_pins;
static unsigned g_edges, g_nops, g_mode;
static bool g_started;
static uint8_t g_bytes[3];
/* 避免旧代码无限循环导致整个测试进程挂住。 */
static jmp_buf g_timeout;

/** @brief 模拟时间推进；模式5让周期计数器停止。 */
void mock_nop(void)
{
    if (++g_nops > 100000U) { longjmp(g_timeout, 1); }
    if (g_mode != 5U) { mock_dwt.CYCCNT += 100U; }
}

/** @brief 检查测试代码只配置开漏。 @param port 模拟端口。 @param gpio 配置。 */
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *gpio)
{
    (void)port;
    if (gpio->Mode != GPIO_MODE_OUTPUT_OD) { longjmp(g_timeout, 1); }
}

/** @brief 模拟引脚锁存并记录数据。 @param port 端口。 @param pins 引脚。 @param value 电平。 */
void HAL_GPIO_WritePin(void *port, uint16_t pins, GPIO_PinState value)
{
    uint16_t old = g_pins;
    (void)port;
    if (value == GPIO_PIN_SET) { g_pins |= pins; } else { g_pins &= (uint16_t)~pins; }
    if (pins == GPIO_PIN_7 && (old & GPIO_PIN_6) != 0U && value == GPIO_PIN_RESET)
    {
        g_started = true;
    }
    if (g_started && (old & GPIO_PIN_6) == 0U && (g_pins & GPIO_PIN_6) != 0U)
    {
        unsigned position = g_edges % 9U;
        unsigned byte = g_edges / 9U;
        if (position < 8U && byte < 3U)
        {
            g_bytes[byte] = (uint8_t)((g_bytes[byte] << 1U) | ((g_pins & GPIO_PIN_7) != 0U));
        }
        ++g_edges;
    }
    mock_nop();
}

/** @brief 模拟 ACK、断线、短路和停止阶段故障。 @param port 端口。 @param pin 引脚。 @return 实际电平。 */
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    mock_nop();
    if (pin == GPIO_PIN_6)
    {
        if (g_mode == 2U || (g_mode == 4U && g_edges >= 28U)) { return GPIO_PIN_RESET; }
    }
    if (pin == GPIO_PIN_7)
    {
        if (g_mode == 3U || (g_mode == 6U && g_edges >= 28U)) { return GPIO_PIN_RESET; }
        if (g_started && g_edges != 0U && g_edges <= 27U && g_edges % 9U == 0U)
        {
            return g_mode == 1U ? GPIO_PIN_SET : GPIO_PIN_RESET;
        }
    }
    return (g_pins & pin) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/** @brief 执行正常传输及故障注入。 @return 失败用例数。 */
int main(void)
{
    unsigned failures = 0;
    const uint8_t data = 0xA5;
    for (unsigned mode = 0; mode < 7U; ++mode)
    {
        bool result = false;
        g_mode = mode;
        g_pins = GPIO_PIN_6 | GPIO_PIN_7;
        g_started = false;
        g_edges = g_nops = 0;
        memset(g_bytes, 0, sizeof(g_bytes));
        mock_dwt.CYCCNT = 0;
        if (setjmp(g_timeout) == 0)
        {
            board_i2c_init();
            result = board_i2c_write(0x3C, 0x40, &data, 1);
            if (result != (mode == 0U)) { ++failures; printf("FAIL result mode=%u\n", mode); }
            if (mode == 0U && (g_bytes[0] != 0x78 || g_bytes[1] != 0x40 || g_bytes[2] != 0xA5))
            { ++failures; puts("FAIL address/control/data bits"); }
            if (g_pins != (GPIO_PIN_6 | GPIO_PIN_7)) { ++failures; puts("FAIL bus not released"); }
        }
        else { ++failures; printf("FAIL non-terminating wait mode=%u\n", mode); }
    }
    g_mode = 0;
    if (board_i2c_write(0x80, 0, NULL, 0) || board_i2c_write(0x3C, 0, NULL, 1)) { ++failures; }
    printf("I2C cases=9 failures=%u\n", failures);
    return failures == 0U ? 0 : 1;
}
