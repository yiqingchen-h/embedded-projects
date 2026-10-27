#include "board_test.h"
#include "stm32g4xx_hal.h"
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* SPI1 为编码器，SPI3 为栅极驱动。 */
static SPI_HandleTypeDef g_spi_enc, g_spi_drv;
/* 八路慢速轮询 ADC，不用于电机控制采样。 */
static ADC_HandleTypeDef g_adc;
/* 串口 TX 有界阻塞，RX 通过中断接收。 */
static UART_HandleTypeDef g_uart;
/* 中断与主循环共享状态。 */
static volatile board_inputs_t g_inputs;
/* 收发初始化标记、故障使能和停机锁存。 */
static volatile bool g_tick_ready, g_uart_ready, g_armed, g_trip;
/* 只有ADC完整初始化及校准成功才允许转换。 */
static bool g_adc_ready;
/* 负载单脉冲活动标记，毫秒中断负责在硬件结束后禁用DRV。 */
static volatile bool g_load_active;
/* 空载脉冲电流检查的有效性、零点与参考电压。 */
static volatile bool g_guard_ready;
static uint16_t g_guard_zero[3];
static uint32_t g_guard_vref;
/* 单生产者单消费者接收环形缓冲，256字节。 */
static volatile uint8_t g_rx[256];
/* 单字节下标自然回绕，IRQ 只写 head，主循环只写 tail。 */
static volatile uint8_t g_rx_head, g_rx_tail;
/* 负载期间的单条短响应，避免轮询发送阻塞1ms采样调度。 */
static char g_async_tx[128];
static volatile uint16_t g_async_count, g_async_position;
/* 方向触点需连续稳定20ms，按下和BOOT保留积分消抖。 */
static uint8_t g_nav_candidate, g_nav_filter, g_nav_state, g_key_filter, g_boot_filter;
/* 一次按压只累计一次事件。 */
static bool g_key_down, g_boot_down;
/* ADC 输入通道与硬件速查表一致。 */
static const uint32_t g_adc_channels[8] = {
    ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4,
    ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_VREFINT
};

/** @brief 用 DWT 有界延时。 @param us 微秒，测试内部不超过2000。 @return 计数正常为 true。 */
static bool
delay_us(uint32_t us)
{
    uint32_t begin = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    for (uint32_t limit = 0U; limit < cycles + 1000U; ++limit)
    {
        if ((uint32_t)(DWT->CYCCNT - begin) >= cycles) { return true; }
    }
    return false;
}

/** @brief 六个输入回到 GPIO 低；调用前必须先撤销 MOE。 */
static void
gates_low(void)
{
    GPIO_InitTypeDef gpio = {0};
    GPIOA->BSRR = (uint32_t)(GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10) << 16U;
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16U;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/** @brief 见 board_test.h。 */
void
board_trip(void)
{
    /* 不访问 SPI，不等待主循环或 HAL；TIM1 BKIN 还会独立撤销 MOE。 */
    GPIOB->BSRR = (uint32_t)GPIO_PIN_9 << 16U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    GPIOA->BSRR = (uint32_t)(GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10) << 16U;
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16U;
    g_trip = true;
    g_armed = false;
    g_guard_ready = false;
    g_load_active = false;
}

/** @brief 见 board_test.h。 */
bool
board_tripped(void) { return g_trip; }

/** @brief 见 board_test.h。 */
void
board_clear_trip(void)
{
    board_trip();
    gates_low();
    g_trip = false;
}

/** @brief 初始化硬件 SPI。 @param h 句柄。 @param instance 外设。 @param polarity 时钟闲置电平。 @param bits 数据位数。 @return HAL初始化成功为 true。 */
static bool
spi_init(SPI_HandleTypeDef *h, SPI_TypeDef *instance, uint32_t polarity, uint32_t bits)
{
    h->Instance = instance;
    h->Init.Mode = SPI_MODE_MASTER;
    h->Init.Direction = SPI_DIRECTION_2LINES;
    h->Init.DataSize = bits;
    h->Init.CLKPolarity = polarity;
    h->Init.CLKPhase = SPI_PHASE_2EDGE;
    h->Init.NSS = SPI_NSS_SOFT;
    h->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    h->Init.FirstBit = SPI_FIRSTBIT_MSB;
    h->Init.TIMode = SPI_TIMODE_DISABLE;
    h->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    h->Init.CRCPolynomial = 7U;
    h->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    h->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    bool ok = HAL_SPI_Init(h) == HAL_OK;
    /* 在片选下降前建立闲置时钟，尤其模式3不能在CS有效后才产生首次上升沿。 */
    if (ok) { __HAL_SPI_ENABLE(h); }
    return ok;
}

/** @brief 初始化 ADC1、独立模式和内部参考；GPIO 已在外层设置。 @return 校准成功为 true。 */
static bool
adc_init(void)
{
    RCC_PeriphCLKInitTypeDef clocks = {0};
    ADC_MultiModeTypeDef mode = {0};
    clocks.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    clocks.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&clocks) != HAL_OK) { return false; }
    __HAL_RCC_ADC12_CLK_ENABLE();
    g_adc.Instance = ADC1;
    g_adc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    g_adc.Init.Resolution = ADC_RESOLUTION_12B;
    g_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_adc.Init.GainCompensation = 0U;
    g_adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    g_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    g_adc.Init.LowPowerAutoWait = DISABLE;
    g_adc.Init.ContinuousConvMode = DISABLE;
    g_adc.Init.NbrOfConversion = 1U;
    g_adc.Init.DiscontinuousConvMode = DISABLE;
    g_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_adc.Init.DMAContinuousRequests = DISABLE;
    g_adc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    g_adc.Init.OversamplingMode = DISABLE;
    mode.Mode = ADC_MODE_INDEPENDENT;
    bool ok = HAL_ADC_Init(&g_adc) == HAL_OK &&
           HAL_ADCEx_MultiModeConfigChannel(&g_adc, &mode) == HAL_OK &&
           HAL_ADCEx_Calibration_Start(&g_adc, ADC_SINGLE_ENDED) == HAL_OK;
    if (ok)
    {
        /* 在首次转换前启用并稳定VREFINT，不能等到读该通道时才打开。 */
        ADC12_COMMON->CCR |= ADC_CCR_VREFEN;
        HAL_Delay(1U);
    }
    return ok;
}

/** @brief 读取指定通道，长采样适配分压阻抗。 @param channel 通道。 @param value 返回12位值。 @return 转换成功为 true。 */
static bool
adc_one(uint32_t channel, uint16_t *value)
{
    ADC_ChannelConfTypeDef config = {0};
    config.Channel = channel;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    if (HAL_ADC_ConfigChannel(&g_adc, &config) != HAL_OK || HAL_ADC_Start(&g_adc) != HAL_OK)
    {
        return false;
    }
    bool ok = HAL_ADC_PollForConversion(&g_adc, 2U) == HAL_OK;
    if (ok) { *value = (uint16_t)HAL_ADC_GetValue(&g_adc); }
    return HAL_ADC_Stop(&g_adc) == HAL_OK && ok;
}

/** @brief 见 board_test.h。 */
bool
board_adc_read(board_adc_t *sample)
{
    board_adc_t result = {0};
    if (NULL == sample || !g_adc_ready) { return false; }
    /* ES0430 2.7.9：长于1ms的转换间隔可能使首次结果错误；先读同一母线通道并丢弃该值。 */
    if (!adc_one(g_adc_channels[0], &result.bus_first_raw)) { return false; }
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        /* 相通道紧接着复测，区分切换后的首次转换与实际引脚电平。 */
        if (i >= 1U && i <= 3U && !adc_one(g_adc_channels[i], &result.phase_first_raw[i-1U])) { return false; }
        if (!adc_one(g_adc_channels[i], &result.raw[i])) { return false; }
        if (i >= 1U && i <= 3U)
        {
            result.phase_sq1[i-1U] = (uint8_t)((ADC1->SQR1 & ADC_SQR1_SQ1) >> ADC_SQR1_SQ1_Pos);
        }
    }
    result.pa_moder = (uint8_t)(GPIOA->MODER & 0xFFU);
    result.pa_pupdr = (uint8_t)(GPIOA->PUPDR & 0xFFU);
    if (result.raw[7] == 0U || result.raw[7] == 4095U) { return false; }
    result.vref_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE((uint32_t)result.raw[7], ADC_RESOLUTION_12B);
    if (result.vref_mv < 2700U || result.vref_mv > 3600U) { return false; }
    *sample = result;
    return true;
}

/** @brief 见 board_test.h。 */
board_test_init_t
board_test_init(void)
{
    board_test_init_t result = {0};
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* PC5 CAL_EN 低，不触发编码器自标定。 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOC, &gpio);
    result.adc = adc_init();
    g_adc_ready = result.adc;

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* 未接编码器时MISO稳定为高，使CRC失败而不是随机读到全零合法帧。 */
    gpio.Pin = GPIO_PIN_6;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &gpio);
    bool enc_ok = spi_init(&g_spi_enc, SPI1, SPI_POLARITY_HIGH, SPI_DATASIZE_8BIT);
    bool drv_ok = spi_init(&g_spi_drv, SPI3, SPI_POLARITY_LOW, SPI_DATASIZE_16BIT);
    result.spi = enc_ok && drv_ok;

    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Alternate = GPIO_AF7_USART3;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);
    g_uart.Instance = USART3;
    g_uart.Init.BaudRate = 115200U;
    g_uart.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart.Init.StopBits = UART_STOPBITS_1;
    g_uart.Init.Parity = UART_PARITY_NONE;
    g_uart.Init.Mode = UART_MODE_TX_RX;
    g_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    g_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    g_uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    result.uart = HAL_UART_Init(&g_uart) == HAL_OK;
    g_uart_ready = result.uart;
    if (result.uart)
    {
        __HAL_UART_ENABLE_IT(&g_uart, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(&g_uart, UART_IT_ERR);
        HAL_NVIC_SetPriority(USART3_IRQn, 3U, 0U);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }

    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    /* 编码器可能未接，弱下拉避免悬空输入造成连续中断。 */
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &gpio);
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    /* SPI探测承担在线判断；弱上拉只使未连接的nFAULT输入保持稳定。 */
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);
    /* 保留 EXTI12 路由，同时把 nFAULT 接到 TIM1 BKIN，形成硬件刹车。 */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pin = GPIO_PIN_8;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0U, 0U);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 4U, 0U);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 4U, 0U);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4U, 0U);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    g_tick_ready = true;
    return result;
}

/** @brief 见 board_test.h。 */
bool
board_drv_transfer(uint16_t tx, uint16_t *rx)
{
    if (NULL == rx) { return false; }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    bool ok = delay_us(2U) && HAL_SPI_TransmitReceive(&g_spi_drv, (uint8_t *)&tx, (uint8_t *)rx, 1U, 2U) == HAL_OK;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    return delay_us(2U) && ok;
}

/** @brief 见 board_test.h。 */
bool
board_enc_transfer(const uint8_t *tx, uint8_t *rx, uint16_t count)
{
    if (NULL == tx || NULL == rx || count == 0U || count > 6U) { return false; }
    uint8_t copy[6];
    memcpy(copy, tx, count);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    bool ok = delay_us(2U) && HAL_SPI_TransmitReceive(&g_spi_enc, copy, rx, count, 2U) == HAL_OK;
    /* 模式3的最后上升沿到CS释放至少半个SPI周期。 */
    ok = delay_us(2U) && ok;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    return delay_us(2U) && ok;
}

/** @brief 见 board_test.h。 */
void
board_driver_wake(void)
{
    g_guard_ready = false;
    gates_low();
    if ((GPIOA->IDR & (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)) != 0U ||
        (GPIOB->IDR & (GPIO_PIN_9 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)) != 0U)
    {
        board_trip();
        return;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!g_trip) { GPIOB->BSRR = GPIO_PIN_9; }
    __set_PRIMASK(mask);
}

/** @brief 见 board_test.h。 */
bool
board_driver_arm(void)
{
    g_armed = true;
    if ((GPIOB->IDR & (GPIO_PIN_2 | GPIO_PIN_12)) != (GPIO_PIN_2 | GPIO_PIN_12)) { board_trip(); }
    return !g_trip;
}

/** @brief 见 board_test.h。 */
bool
board_current_guard(const uint16_t zero[3], uint32_t vref_mv)
{
    g_guard_ready = false;
    if (NULL == zero || vref_mv < 2700U || vref_mv > 3600U) { return false; }
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (zero[i] < 1433U || zero[i] > 2662U) { return false; }
        g_guard_zero[i] = zero[i];
    }
    g_guard_vref = vref_mv;
    g_guard_ready = true;
    return true;
}

/** @brief 保存失败瞬间后关断并报告；只在主循环调用。 @param gate 桥臂编号。 @param stage 失败阶段。 @return 固定false。 */
static bool
pulse_failed(uint8_t gate, const char *stage)
{
    uint32_t pa = GPIOA->IDR, pb = GPIOB->IDR;
    uint32_t cr1 = TIM1->CR1, bdtr = TIM1->BDTR, cnt = TIM1->CNT;
    uint32_t channel = gate / 2U;
    uint32_t pin_index = ((gate & 1U) != 0U ? 13U : 8U) + channel;
    GPIO_TypeDef *port = (gate & 1U) != 0U ? GPIOB : GPIOA;
    uint32_t mode = (port->MODER >> (pin_index * 2U)) & 3U;
    uint32_t af = (port->AFR[pin_index / 8U] >> ((pin_index % 8U) * 4U)) & 15U;
    uint32_t type = (port->OTYPER >> pin_index) & 1U;
    uint32_t ccer = TIM1->CCER, ccmr1 = TIM1->CCMR1, ccmr2 = TIM1->CCMR2;
    uint32_t psc = TIM1->PSC, arr = TIM1->ARR;
    uint32_t ccr = channel == 0U ? TIM1->CCR1 : channel == 1U ? TIM1->CCR2 : TIM1->CCR3;
    board_trip();
    gates_low();
    char text[192];
    (void)snprintf(text, sizeof(text),
        "PWM FAIL GATE=%u STEP=%s PA=%08lX PB=%08lX CR1=%08lX BDTR=%08lX CNT=%lu\r\n",
        (unsigned)gate, stage, (unsigned long)pa, (unsigned long)pb,
        (unsigned long)cr1, (unsigned long)bdtr, (unsigned long)cnt);
    board_uart_write(text);
    (void)snprintf(text, sizeof(text),
        "PWM CFG MODE=%lu AF=%lu TYPE=%lu CCER=%08lX CCMR1=%08lX CCMR2=%08lX PSC=%lu ARR=%lu CCR=%lu\r\n",
        (unsigned long)mode, (unsigned long)af, (unsigned long)type, (unsigned long)ccer,
        (unsigned long)ccmr1, (unsigned long)ccmr2, (unsigned long)psc, (unsigned long)arr, (unsigned long)ccr);
    board_uart_write(text);
    return false;
}

/** @brief 见 board_test.h。 */
bool
board_gate_pulse(uint8_t gate, uint16_t *phase_raw, bool enabled)
{
    if (gate > 5U || NULL == phase_raw || g_trip || (enabled && (!g_armed || !g_guard_ready)) ||
        (!enabled && ((GPIOB->ODR | GPIOB->IDR) & GPIO_PIN_9) != 0U)) { return false; }
    uint32_t channel = gate / 2U;
    bool low = (gate & 1U) != 0U;
    GPIO_TypeDef *port = low ? GPIOB : GPIOA;
    uint16_t pin = (uint16_t)(1U << ((low ? 13U : 8U) + channel));
    GPIO_InitTypeDef gpio = {0};
    TIM1->BDTR = 0U;
    TIM1->CCER = 0U;
    if (!enabled)
    {
        /* 仅在ENABLE实测为低时隔离检查GPIO，功率测试不执行此软件脉冲。 */
        gates_low();
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        if (!delay_us(2U)) { return pulse_failed(gate, "GPIO_DELAY_HIGH"); }
        if (g_trip || (port->IDR & pin) == 0U) { return pulse_failed(gate, "GPIO_HIGH"); }
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        if (!delay_us(2U)) { return pulse_failed(gate, "GPIO_DELAY_LOW"); }
        if (g_trip || (port->IDR & pin) != 0U) { return pulse_failed(gate, "GPIO_LOW"); }
        char text[32];
        (void)snprintf(text, sizeof(text), "GPIO PASS GATE=%u\r\n", (unsigned)gate);
        board_uart_write(text);
    }
    TIM1->CR1 = TIM_CR1_OPM | TIM_CR1_ARPE;
    TIM1->CR2 = 0U;
    TIM1->CCER = 0U;
    TIM1->PSC = (SystemCoreClock / 1000000U) - 1U;
    /* 延迟200us后输出1000us；更新时CNT回零且硬件停止，PWM2自动回低。 */
    TIM1->ARR = 1199U;
    TIM1->CCR1 = 200U;
    TIM1->CCR2 = 200U;
    TIM1->CCR3 = 200U;
    TIM1->CCMR1 = 0x7878U;
    TIM1->CCMR2 = 0x0078U;
    TIM1->RCR = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    TIM1->CNT = 0U;
    /* RM0440 29.3.17：仅开启N输出时不反相，CCxNP必须为0才能得到正脉冲。 */
    TIM1->CCER = (low ? TIM_CCER_CC1NE : TIM_CCER_CC1E) << (channel * 4U);
    TIM1->AF1 = TIM1_AF1_BKINE;
    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = (gate == 5U) ? GPIO_AF4_TIM1 : GPIO_AF6_TIM1;
    HAL_GPIO_Init(port, &gpio);
    TIM1->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | (enabled ? TIM_BDTR_BKE : 0U);
    TIM1->BDTR |= TIM_BDTR_MOE;
    /* 同步输出使能后检查闲置低电平，极性/刹车异常时不启动计数器。 */
    if (!delay_us(2U)) { return pulse_failed(gate, "DELAY_IDLE"); }
    bool ok = !g_trip && (TIM1->BDTR & TIM_BDTR_MOE) != 0U &&
        (GPIOA->IDR & (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)) == 0U &&
        (GPIOB->IDR & (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)) == 0U;
    if (!ok) { return pulse_failed(gate, "IDLE"); }
    TIM1->CR1 |= TIM_CR1_CEN;
    if (!delay_us(450U)) { return pulse_failed(gate, "DELAY_HIGH"); }
    ok = !g_trip &&
        (GPIOA->IDR & (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)) == (low ? 0U : pin) &&
        (GPIOB->IDR & (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)) == (low ? pin : 0U);
    if (!ok) { return pulse_failed(gate, "HIGH"); }
    if (!adc_one(g_adc_channels[channel + 1U], phase_raw)) { return pulse_failed(gate, "ADC"); }
    if (enabled)
    {
        for (uint8_t i = 0U; i < 3U && ok; ++i)
        {
            uint16_t raw;
            ok = adc_one(g_adc_channels[4U+i], &raw);
            if (ok)
            {
                uint32_t delta = raw > g_guard_zero[i] ? raw - g_guard_zero[i] : g_guard_zero[i] - raw;
                /* 2A × 1mΩ × 20倍 = 40mV；先乘后比，避免整数截断放宽门限。 */
                ok = delta * g_guard_vref <= 4095U * 40U;
            }
        }
    }
    if (!ok) { return pulse_failed(gate, "CURRENT"); }
    /* 不依赖中断结束输出；软件等待失败也立即撤销输出。 */
    if (!delay_us(1000U)) { return pulse_failed(gate, "DELAY_END"); }
    if (g_trip || (TIM1->CR1 & TIM_CR1_CEN) != 0U || (port->IDR & pin) != 0U)
    {
        return pulse_failed(gate, "END");
    }
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER = 0U;
    gates_low();
    if (g_trip) { return pulse_failed(gate, "STOP"); }
    return true;
}

/** @brief 六步脉冲任一异常立即撤销输出并报告阶段。 @param step 六步状态编号。 @param stage 失败阶段。 @return 固定false。 */
static bool
six_step_failed(uint8_t step, const char *stage)
{
    uint32_t pa = GPIOA->IDR, pb = GPIOB->IDR;
    uint32_t ccer = TIM1->CCER, bdtr = TIM1->BDTR;
    board_trip();
    gates_low();
    char message[112];
    (void)snprintf(message, sizeof(message),
        "6STEP FAIL STEP=%u AT=%s PA=%08lX PB=%08lX CCER=%08lX BDTR=%08lX\r\n",
        (unsigned)step, stage, (unsigned long)pa, (unsigned long)pb,
        (unsigned long)ccer, (unsigned long)bdtr);
    board_uart_write(message);
    return false;
}

/** @brief 见 board_test.h。 */
bool
board_six_step_pulse(uint8_t step, board_six_step_sample_t *sample)
{
    /* 任何一步都只选不同相的一只上管和一只下管；其余四路保持低。 */
    static const uint8_t high_phase[6] = {0U, 0U, 1U, 1U, 2U, 2U};
    static const uint8_t low_phase[6] = {1U, 2U, 2U, 0U, 0U, 1U};
    if (step >= 6U || NULL == sample || g_trip || !g_armed || !g_guard_ready) { return false; }
    uint8_t high = high_phase[step], low = low_phase[step];
    uint8_t floating = (uint8_t)(3U - high - low);
    uint16_t high_pin = (uint16_t)(GPIO_PIN_8 << high);
    uint16_t low_pin = (uint16_t)(GPIO_PIN_13 << low);
    GPIO_InitTypeDef gpio = {0};
    board_six_step_sample_t measured = {0};
    if ((TIM1->CR1 & TIM_CR1_CEN) != 0U) { return six_step_failed(step, "PREVIOUS_TIMER"); }
    TIM1->BDTR = 0U;
    TIM1->CCER = 0U;
    gates_low();
    TIM1->CR1 = TIM_CR1_OPM | TIM_CR1_ARPE;
    TIM1->CR2 = 0U;
    TIM1->PSC = (SystemCoreClock / 1000000U) - 1U;
    TIM1->ARR = 1199U;
    TIM1->CCR1 = 200U;
    TIM1->CCR2 = 200U;
    TIM1->CCR3 = 200U;
    TIM1->CCMR1 = 0x7878U;
    TIM1->CCMR2 = 0x0078U;
    TIM1->RCR = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    TIM1->CNT = 0U;
    TIM1->CCER = (TIM_CCER_CC1E << (high * 4U)) | (TIM_CCER_CC1NE << (low * 4U));
    TIM1->AF1 = TIM1_AF1_BKINE;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    gpio.Pin = high_pin;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Alternate = low == 2U ? GPIO_AF4_TIM1 : GPIO_AF6_TIM1;
    gpio.Pin = low_pin;
    HAL_GPIO_Init(GPIOB, &gpio);
    TIM1->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_BKE;
    TIM1->BDTR |= TIM_BDTR_MOE;
    if (!delay_us(2U) || g_trip || (TIM1->BDTR & TIM_BDTR_MOE) == 0U ||
        (GPIOA->IDR & (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)) != 0U ||
        (GPIOB->IDR & (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)) != 0U)
    {
        return six_step_failed(step, "IDLE");
    }
    TIM1->CR1 |= TIM_CR1_CEN;
    if (!delay_us(450U) || g_trip ||
        (GPIOA->IDR & (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)) != high_pin ||
        (GPIOB->IDR & (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)) != low_pin)
    {
        return six_step_failed(step, "HIGH");
    }
    for (uint8_t pass = 0U; pass < 2U; ++pass)
    {
        if (pass != 0U && !delay_us(200U)) { return six_step_failed(step, "GUARD_DELAY"); }
        for (uint8_t i = 0U; i < 3U; ++i)
        {
            uint16_t raw;
            if (g_trip || !adc_one(g_adc_channels[4U + i], &raw)) { return six_step_failed(step, "CURRENT_ADC"); }
            uint32_t delta = raw > g_guard_zero[i] ? raw - g_guard_zero[i] : g_guard_zero[i] - raw;
            if (delta * g_guard_vref > 4095U * 40U) { return six_step_failed(step, "CURRENT_LIMIT"); }
        }
    }
    if (g_trip || (TIM1->CR1 & TIM_CR1_CEN) == 0U ||
        !adc_one(g_adc_channels[high + 1U], &measured.high_raw) ||
        !adc_one(g_adc_channels[low + 1U], &measured.low_raw) ||
        !adc_one(g_adc_channels[floating + 1U], &measured.floating_raw) ||
        (TIM1->CR1 & TIM_CR1_CEN) == 0U)
    {
        return six_step_failed(step, "PHASE_ADC");
    }
    /* 即使主循环随后失效，TIM1 单脉冲也在约1.2ms处撤销两路输入。 */
    if (!delay_us(1000U) || g_trip || (TIM1->CR1 & TIM_CR1_CEN) != 0U ||
        (GPIOA->IDR & (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)) != 0U ||
        (GPIOB->IDR & (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)) != 0U)
    {
        return six_step_failed(step, "END");
    }
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER = 0U;
    gates_low();
    *sample = measured;
    return true;
}

/** @brief 见 board_test.h。 */
void
board_uart_write(const char *text)
{
    if (!g_uart_ready || NULL == text) { return; }
    /* 旧的阻塞日志只在负载采样结束后调用，先让在途短响应完整发送。 */
    uint32_t began = HAL_GetTick();
    while (g_async_count != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - began) > 20U) { board_trip(); return; }
    }
    size_t length = strlen(text);
    if (length <= 512U && HAL_UART_Transmit(&g_uart, (uint8_t *)text, (uint16_t)length, 60U) != HAL_OK)
    {
        /* HAL发送超时路径会关闭接收中断，恢复独立的接收/紧急停止通道。 */
        __HAL_UART_ENABLE_IT(&g_uart, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(&g_uart, UART_IT_ERR);
    }
}

/** @brief 见 board_test.h。 */
bool
board_uart_try_write(const char *text)
{
    if (!g_uart_ready || NULL == text || g_async_count != 0U) { return false; }
    size_t length = strlen(text);
    if (length == 0U || length >= sizeof(g_async_tx)) { return false; }
    memcpy(g_async_tx, text, length);
    g_async_position = 0U;
    __DMB();
    g_async_count = (uint16_t)length;
    __HAL_UART_ENABLE_IT(&g_uart, UART_IT_TXE);
    return true;
}

/** @brief 见 board_test.h。 */
bool
board_uart_get(char *value)
{
    if (NULL == value || g_rx_tail == g_rx_head) { return false; }
    *value = (char)g_rx[g_rx_tail];
    g_rx_tail++;
    return true;
}

/** @brief 见 board_test.h。 */
void
board_uart_irq(void)
{
    uint32_t flags = USART3->ISR;
    if ((flags & USART_ISR_TXE_TXFNF) != 0U && (USART3->CR1 & USART_CR1_TXEIE_TXFNFIE) != 0U)
    {
        if (g_async_count != 0U)
        {
            USART3->TDR = (uint8_t)g_async_tx[g_async_position++];
            --g_async_count;
        }
        if (g_async_count == 0U) { __HAL_UART_DISABLE_IT(&g_uart, UART_IT_TXE); }
    }
    if ((flags & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0U)
    {
        USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
        ++g_inputs.rx_errors;
        board_trip();
    }
    if ((flags & USART_ISR_RXNE_RXFNE) != 0U)
    {
        uint8_t value = (uint8_t)USART3->RDR;
        ++g_inputs.rx_bytes;
        /* 紧急停止字节不等待行解析；即使接收缓冲溢出仍可停机。 */
        if (value == (uint8_t)'!')
        {
            board_trip();
            /* 使主循环丢弃当前残行，避免STOP之后拼接出新动作。 */
            ++g_inputs.rx_errors;
        }
        /* 硬件报错的字节不参与命令拼接。 */
        if ((flags & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0U) { return; }
        uint8_t next = (uint8_t)(g_rx_head + 1U);
        if (next == g_rx_tail) { ++g_inputs.rx_errors; board_trip(); }
        else { g_rx[g_rx_head] = value; __DMB(); g_rx_head = next; }
    }
}

/** @brief 见 board_test.h。 */
void
board_inputs_read(board_inputs_t *inputs)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *inputs = g_inputs;
    __set_PRIMASK(mask);
}

/** @brief 见 board_test.h。 */
void
board_test_tick(void)
{
    if (!g_tick_ready) { return; }
    /* 先记录截止边界的故障，不能先撤销武装而漏掉同一毫秒的PG/nFAULT。 */
    if (g_armed && (GPIOB->IDR & (GPIO_PIN_2 | GPIO_PIN_12)) != (GPIO_PIN_2 | GPIO_PIN_12)) { board_trip(); }
    if (g_load_active && (TIM1->SR & TIM_SR_BIF) != 0U) { board_trip(); }
    if (g_load_active && (TIM1->CR1 & TIM_CR1_CEN) == 0U)
    {
        /* OPM已使两路回低；中断再禁用驱动，不等待主循环恢复。 */
        GPIOB->BSRR = (uint32_t)GPIO_PIN_9 << 16U;
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        g_load_active = false;
        g_armed = false;
    }
    uint32_t pins = GPIOC->IDR;
    uint8_t now = (uint8_t)(((pins & GPIO_PIN_9) == 0U ? 1U : 0U) |
        ((pins & GPIO_PIN_8) == 0U ? 2U : 0U));
    g_inputs.switch_raw = (uint8_t)(now | ((pins & GPIO_PIN_7) == 0U ? 4U : 0U));
    if (now != g_nav_candidate) { g_nav_candidate = now; g_nav_filter = 1U; }
    else if (g_nav_filter < 20U) { ++g_nav_filter; }
    if (g_nav_filter == 20U && now != g_nav_state)
    {
        g_nav_state = now;
        /* 两方向同时有效时只报告状态，不生成翻页动作。回中也不计数。 */
        if (now == 1U)
        {
            ++g_inputs.up_presses;
            if (g_inputs.nav_steps < INT32_MAX) { ++g_inputs.nav_steps; }
        }
        else if (now == 2U)
        {
            ++g_inputs.down_presses;
            if (g_inputs.nav_steps > INT32_MIN) { --g_inputs.nav_steps; }
        }
    }
    if ((GPIOC->IDR & GPIO_PIN_7) == 0U) { if (g_key_filter < 20U) { ++g_key_filter; } }
    else if (g_key_filter != 0U) { --g_key_filter; }
    if (g_key_filter == 20U && !g_key_down) { g_key_down = true; ++g_inputs.presses; }
    if (g_key_filter == 0U) { g_key_down = false; g_inputs.held_ms = 0U; }
    if (g_key_down && g_inputs.held_ms < 60000U) { ++g_inputs.held_ms; }
    g_inputs.switch_state = (uint8_t)(g_nav_state | (g_key_down ? 4U : 0U));
    if ((GPIOB->IDR & GPIO_PIN_8) != 0U) { if (g_boot_filter < 20U) { ++g_boot_filter; } }
    else if (g_boot_filter != 0U) { --g_boot_filter; }
    if (g_boot_filter == 20U && !g_boot_down) { g_boot_down = true; ++g_inputs.boot_presses; }
    if (g_boot_filter == 0U) { g_boot_down = false; }
}

/** @brief 见 board_test.h。 */
void
board_test_exti(uint32_t pins)
{
    if ((pins & GPIO_PIN_5) != 0U) { ++g_inputs.encoder_edges[0]; }
    if ((pins & GPIO_PIN_4) != 0U) { ++g_inputs.encoder_edges[1]; }
    if ((pins & GPIO_PIN_2) != 0U) { ++g_inputs.encoder_edges[2]; }
    if ((pins & GPIO_PIN_12) != 0U)
    {
        ++g_inputs.fault_edges;
        if (g_armed) { board_trip(); }
    }
}

/** @brief 见 board_test.h。 */
bool
board_watchdog_init(void)
{
    /* LSI 标称32kHz，64分频、1000计数，约2s；实际范围受LSI误差影响。 */
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 4U;
    IWDG->RLR = 999U;
    /* ST HAL V1.2.3：同步最多5个分频后的LSI周期，按6周期和最大256分频留裕量。
       不能用10ms硬限值；64分频本身已需要约10ms，尚有LSI误差和启动时间。 */
    const uint32_t timeout_ms = (6U * 256U * 1000U) / LSI_VALUE + LSI_STARTUP_TIME / 1000U + 1U;
    for (uint32_t retry = 0U; IWDG->SR != 0U; ++retry)
    {
        if (retry >= timeout_ms)
        {
            board_trip(); board_uart_write("IWDG CONFIG TIMEOUT\r\n"); return false;
        }
        if (!delay_us(1000U))
        {
            board_trip(); board_uart_write("IWDG DELAY FAILED\r\n"); return false;
        }
    }
    IWDG->KR = 0xAAAAU;
    return true;
}

/** @brief 见 board_test.h。 */
void
board_watchdog_feed(void) { IWDG->KR = 0xAAAAU; }

/** @brief 见 board_test.h。 */
void
board_driver_disable(void)
{
    GPIOB->BSRR = (uint32_t)GPIO_PIN_9 << 16U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CCER = 0U;
    g_load_active = false;
    g_armed = false;
    g_guard_ready = false;
    gates_low();
}

/** @brief 见 board_test.h。 */
bool
board_load_start(void)
{
    if (g_trip || !g_armed || g_load_active || SystemCoreClock != 170000000U ||
        (TIM1->CR1 & TIM_CR1_CEN) != 0U || (GPIOB->IDR & GPIO_PIN_9) == 0U)
    {
        return false;
    }
    GPIO_InitTypeDef gpio = {0};
    TIM1->BDTR = 0U;
    TIM1->CCER = 0U;
    gates_low();
    TIM1->CR1 = TIM_CR1_OPM | TIM_CR1_ARPE;
    TIM1->CR2 = 0U;
    /* 10kHz计数：延迟一个计数，随后10000个计数恰好为1秒。 */
    TIM1->PSC = 16999U;
    TIM1->ARR = 10000U;
    TIM1->CCR1 = 1U;
    TIM1->CCR2 = 1U;
    TIM1->CCR3 = 1U;
    TIM1->CCMR1 = 0x7878U;
    TIM1->CCMR2 = 0x0078U;
    TIM1->RCR = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    TIM1->CNT = 0U;
    /* CH1和CH2N属于不同桥臂；单独N输出不反相，禁止使能同相另一管。 */
    TIM1->CCER = TIM_CCER_CC1E | (TIM_CCER_CC1NE << 4U);
    TIM1->AF1 = TIM1_AF1_BKINE;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    gpio.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio);
    TIM1->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_BKE;
    /* 检查与启动原子化；中断期间发生的急停不能被随后置MOE覆盖。 */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool ok = !g_trip && g_armed &&
        (GPIOB->IDR & (GPIO_PIN_2 | GPIO_PIN_12)) == (GPIO_PIN_2 | GPIO_PIN_12);
    if (ok)
    {
        g_guard_ready = false;
        g_load_active = true;
        TIM1->BDTR |= TIM_BDTR_MOE;
        TIM1->CR1 |= TIM_CR1_CEN;
    }
    __set_PRIMASK(mask);
    if (!ok) { board_trip(); }
    return ok;
}

/** @brief 见 board_test.h。 */
bool
board_load_running(void)
{
    if (g_load_active && ((TIM1->SR & TIM_SR_BIF) != 0U ||
        ((TIM1->CR1 & TIM_CR1_CEN) != 0U && (TIM1->BDTR & TIM_BDTR_MOE) == 0U)))
    {
        board_trip();
    }
    return g_load_active && (TIM1->CR1 & TIM_CR1_CEN) != 0U;
}
