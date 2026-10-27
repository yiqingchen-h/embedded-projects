#include "board_i2c.h"
#include "stm32g4xx_hal.h"

/* H1 的时钟和数据引脚，仅操作开漏输出，不主动输出高电平。 */
#define SCL GPIO_PIN_6
#define SDA GPIO_PIN_7

/** @brief 等待至少5us；计数器停止时也能有限次退出。 @return 延时完成为 true。 */
static bool
half_period(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = SystemCoreClock / 200000U;
    /* 170MHz 下只需850周期，4096次检查留足余量且避免 DWT 异常时死循环。 */
    for (uint32_t attempt = 0; attempt < 4096U; ++attempt)
    {
        if ((uint32_t)(DWT->CYCCNT - start) >= cycles && cycles != 0U) { return true; }
        __NOP();
    }
    return false;
}

/** @brief 释放时钟并等待从机，最多 1ms。 @return 时钟实际为高返回 true。 */
static bool
clock_high(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t attempts = SystemCoreClock / 1000U;
    HAL_GPIO_WritePin(GPIOB, SCL, GPIO_PIN_SET);
    while (HAL_GPIO_ReadPin(GPIOB, SCL) == GPIO_PIN_RESET)
    {
        if (attempts == 0U || (uint32_t)(DWT->CYCCNT - start) >= SystemCoreClock / 1000U) { return false; }
        --attempts;
    }
    return half_period();
}

/** @brief 产生停止条件，无论结果都释放两线。 @return STOP 完成且总线空闲为 true。 */
static bool
stop(void)
{
    bool ok;
    HAL_GPIO_WritePin(GPIOB, SCL | SDA, GPIO_PIN_RESET);
    ok = half_period();
    if (ok) { ok = clock_high(); }
    HAL_GPIO_WritePin(GPIOB, SDA, GPIO_PIN_SET);
    if (ok) { ok = half_period(); }
    HAL_GPIO_WritePin(GPIOB, SCL | SDA, GPIO_PIN_SET);
    return ok && HAL_GPIO_ReadPin(GPIOB, SCL) == GPIO_PIN_SET &&
           HAL_GPIO_ReadPin(GPIOB, SDA) == GPIO_PIN_SET;
}

/** @brief 发送字节并检查第九位应答。 @param value 发送值。 @return ACK 时为 true。 */
static bool
send_byte(uint8_t value)
{
    bool ack;
    for (uint8_t bit = 0; bit < 8U; ++bit)
    {
        HAL_GPIO_WritePin(GPIOB, SDA, (value & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        if (!half_period() || !clock_high()) { return false; }
        HAL_GPIO_WritePin(GPIOB, SCL, GPIO_PIN_RESET);
        value <<= 1U;
    }
    HAL_GPIO_WritePin(GPIOB, SDA, GPIO_PIN_SET);
    if (!half_period() || !clock_high()) { return false; }
    ack = (HAL_GPIO_ReadPin(GPIOB, SDA) == GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, SCL, GPIO_PIN_RESET);
    return half_period() && ack;
}

/** @brief 见 board_i2c.h。 */
void
board_i2c_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, SCL | SDA, GPIO_PIN_SET);
    gpio.Pin = SCL | SDA;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/** @brief 见 board_i2c.h。 */
bool
board_i2c_write(uint8_t address, uint8_t control, const uint8_t *data, uint16_t size)
{
    bool ok;
    if (address > 0x7FU || (size != 0U && data == NULL)) { return false; }
    HAL_GPIO_WritePin(GPIOB, SDA, GPIO_PIN_SET);
    if (!clock_high()) { (void)stop(); return false; }
    /* 仅在 SDA 被占用时提供最多九个恢复时钟，避免断线卡死。 */
    for (uint8_t pulse = 0; pulse < 9U && HAL_GPIO_ReadPin(GPIOB, SDA) == GPIO_PIN_RESET; ++pulse)
    {
        HAL_GPIO_WritePin(GPIOB, SCL, GPIO_PIN_RESET);
        if (!half_period() || !clock_high()) { (void)stop(); return false; }
    }
    if (HAL_GPIO_ReadPin(GPIOB, SDA) == GPIO_PIN_RESET) { (void)stop(); return false; }
    HAL_GPIO_WritePin(GPIOB, SDA, GPIO_PIN_RESET);
    if (!half_period()) { (void)stop(); return false; }
    HAL_GPIO_WritePin(GPIOB, SCL, GPIO_PIN_RESET);
    ok = send_byte((uint8_t)(address << 1U)) && send_byte(control);
    for (uint16_t index = 0; ok && index < size; ++index) { ok = send_byte(data[index]); }
    /* 即使数据已经 ACK，STOP 失败也不能宣称一次完整事务成功。 */
    bool stopped = stop();
    return ok && stopped;
}
