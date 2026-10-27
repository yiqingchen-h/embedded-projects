#include "board_encoder.h"
#include "mt6835.h"
#include "motor_config.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

/* 编码器CAL输入与外部ADC标记共用PC5，编译期禁止同时启用。 */
_Static_assert(MOTOR_ADC_TRACE_ENC_CAL == 0U, "Encoder CAL pin must not output ADC trace");
/* SPI1为170MHz/64=2.65625MHz，模式3、8位；不占用DRV的SPI3。 */
#define ENCODER_SPI_CR1 (SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR_2 | SPI_CR1_BR_0 | SPI_CR1_SPE)
/* 单次事务的墙钟超时，主循环等待期间保持全部中断开放。 */
#define ENCODER_TRANSFER_TIMEOUT_US 250U
/* 芯片上电64ms后探测；掉线后每秒有限重试。 */
#define ENCODER_POWERUP_MS 70U
#define ENCODER_REPROBE_MS 1000U

/* 编码器仅主循环持有，不向电机反馈或修改现有参数。 */
static encoder_readout_t g_encoder;
/* 采集调度与最近探测时刻，全部以HAL毫秒时基计算。 */
static uint32_t g_encoder_next_probe, g_encoder_last_poll, g_encoder_gap_max;
/* 事务耗时含被高优先级中断抢占的时间，单位DWT周期。 */
static uint32_t g_encoder_transfer_max;
/* 连续通信错误触发重新探测，不把芯片状态告警当作断线。 */
static uint8_t g_encoder_failures;
static bool g_encoder_probed, g_encoder_polled;
/* 主循环探测时双读并缓存；诊断不在DMA控制期间插入SPI事务。 */
static mt6835_config_t g_encoder_sensor_config;
static bool g_encoder_sensor_config_valid;
/* 控制采集独占SPI1；TIM仅按50us边界推进，DMA硬件传输6字节，不产生新中断。 */
static volatile bool g_encoder_control_enabled;
/* 0建立片选、1启动DMA、2核对完成、3保持结束并释放片选；20kHz峰顶下5kHz采集。 */
static uint8_t g_encoder_control_phase;
static uint32_t g_encoder_control_phase_tick, g_encoder_control_started, g_encoder_observed_sequence;
/* DMA1通道2/3及DMAMUX1通道1/2专属SPI1；ADC保留通道1。 */
static const uint8_t g_encoder_control_tx[6] = {0xA0U, 0x03U, 0U, 0U, 0U, 0U};
static volatile uint8_t g_encoder_control_rx[6];
/* TIM为唯一写者；主循环复制时短临界区保护，禁止边更新边显示。 */
static volatile encoder_six_feedback_t g_encoder_control_feedback;
static uint8_t g_encoder_control_status;
/* 空闲时主循环写，DMA控制时仅采集ISR写；切换时保持累计机械圈数。 */
static encoder_position_t g_encoder_position;
static bool g_encoder_position_bridge;

/** @brief 重置编码器SPI1，清除超时或溢出的FIFO，不影响其他SPI。
 * @details 只在片选释放后调用，保持模式3闲置时钟为高。 */
static void
encoder_spi_reset(void)
{
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    SPI1->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;
    SPI1->CR1 = ENCODER_SPI_CR1;
}

/** @brief 片选建立和保持各等待1us，满足MT6835的100ns及半周期要求。 */
static void
encoder_cs_delay(void)
{
    uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < SystemCoreClock / 1000000U) { __NOP(); }
}

/** @brief 在共享墙钟截止内等待SPI标志，不屏蔽中断。
 * @param mask 状态位。 @param set 期望置位。 @param started 事务起始周期。
 * @return 及时达到要求为true。 */
static bool
encoder_wait(uint32_t mask, bool set, uint32_t started)
{
    while (((SPI1->SR & mask) != 0U) != set)
    {
        if (DWT->CYCCNT - started >= ENCODER_TRANSFER_TIMEOUT_US * (SystemCoreClock / 1000000U))
        {
            return false;
        }
    }
    return true;
}

/** @brief 模式3的完整事务，超时后释放片选并只复位SPI1。
 * @param tx 请求。 @param rx 响应。 @param count 字节数，限1～6。
 * @return 传输完整且无SPI错误为true。 */
static bool
encoder_transfer(const uint8_t *tx, uint8_t *rx, uint16_t count)
{
    if (NULL == tx || NULL == rx || count == 0U || count > 6U) { return false; }
    uint32_t started = DWT->CYCCNT;
    bool ok = true;
    GPIOC->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
    encoder_cs_delay();
    for (uint16_t i = 0U; i < count; ++i)
    {
        if (!encoder_wait(SPI_SR_TXE, true, started)) { ok = false; break; }
        *(volatile uint8_t *)&SPI1->DR = tx[i];
        if (!encoder_wait(SPI_SR_RXNE, true, started)) { ok = false; break; }
        rx[i] = *(volatile uint8_t *)&SPI1->DR;
    }
    if (ok) { ok = encoder_wait(SPI_SR_BSY, false, started); }
    if ((SPI1->SR & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_FRE)) != 0U) { ok = false; }
    encoder_cs_delay();
    GPIOC->BSRR = GPIO_PIN_4;
    uint32_t elapsed = DWT->CYCCNT - started;
    if (elapsed > g_encoder_transfer_max) { g_encoder_transfer_max = elapsed; }
    if (!ok) { encoder_spi_reset(); }
    return ok;
}

/** @brief 见board_encoder.h。 */
void MOTOR_BACKGROUND_CODE
board_encoder_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* 先置片选高、CAL低，再切GPIO模式，避免错误标定或选通。 */
    GPIOC->BSRR = GPIO_PIN_4 | ((uint32_t)GPIO_PIN_5 << 16U);
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* MISO弱上拉令断开的输入得到CRC错误，避免全零合法帧。 */
    gpio.Pin = GPIO_PIN_6;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* 角度采集使用SPI，ABZ保持输入且不开中断。 */
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &gpio);
    encoder_spi_reset();
    encoder_readout_init(&g_encoder, SystemCoreClock);
    g_encoder_next_probe = HAL_GetTick() + ENCODER_POWERUP_MS;
    g_encoder_last_poll = HAL_GetTick();
    g_encoder_gap_max = 0U;
    g_encoder_transfer_max = 0U;
    g_encoder_failures = 0U;
    g_encoder_probed = false;
    g_encoder_polled = false;
    g_encoder_sensor_config_valid=false;
    g_encoder_position=(encoder_position_t){0};
}

/** @brief 见board_encoder.h。 */
void
board_encoder_service(void)
{
    uint32_t now = HAL_GetTick();
    if (g_encoder_control_enabled)
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        encoder_six_feedback_t feedback = g_encoder_control_feedback;
        uint8_t status = g_encoder_control_status;
        __set_PRIMASK(mask);
        if (feedback.sequence != g_encoder_observed_sequence)
        {
            g_encoder_observed_sequence = feedback.sequence;
            if (feedback.valid) { encoder_readout_sample(&g_encoder, feedback.raw, status, now, feedback.tick); }
            else { encoder_readout_error(&g_encoder, ENCODER_IO_ERROR); }
        }
        return;
    }
    if (!g_encoder_probed)
    {
        if ((int32_t)(now - g_encoder_next_probe) < 0) { return; }
        g_encoder_next_probe = now + ENCODER_REPROBE_MS;
        g_encoder_probed = mt6835_probe(encoder_transfer);
        if (!g_encoder_probed)
        {
            encoder_readout_error(&g_encoder, ENCODER_NO_DEVICE);
            return;
        }
        g_encoder_failures = 0U;
        g_encoder_polled = false;
        g_encoder_sensor_config_valid=mt6835_read_config(&g_encoder_sensor_config);
    }
    uint32_t gap = now - g_encoder_last_poll;
    /* 空闲WFI会暂停DWT，主循环丢采样还需按真实HAL毫秒撤销多圈资格。 */
    if(g_encoder_polled && gap>15U) { g_encoder_position.zero_valid=false; }
    if (g_encoder_polled && gap == 0U) { return; }
    if (g_encoder_polled && gap > g_encoder_gap_max) { g_encoder_gap_max = gap; }
    g_encoder_last_poll = now;
    g_encoder_polled = true;
    mt6835_sample_t sample;
    uint32_t captured = DWT->CYCCNT;
    mt6835_result_t result = mt6835_read(&sample);
    if (result == MT6835_OK)
    {
        encoder_readout_sample(&g_encoder, sample.angle, sample.status, now, captured);
        encoder_position_sample(&g_encoder_position,sample.angle,captured,sample.status==0U,false);
        g_encoder_failures = 0U;
    }
    else
    {
        encoder_position_sample(&g_encoder_position,0U,captured,false,false);
        encoder_readout_error(&g_encoder, result == MT6835_CRC_ERROR ? ENCODER_CRC_ERROR : ENCODER_IO_ERROR);
        if (++g_encoder_failures >= 8U)
        {
            g_encoder_probed = false;
            g_encoder_next_probe = now + ENCODER_REPROBE_MS;
        }
    }
}

/** @brief 见board_encoder.h。 */
encoder_snapshot_t
board_encoder_snapshot(uint32_t now_ms)
{
    return encoder_readout_snapshot(&g_encoder, now_ms);
}

/** @brief 见board_encoder.h；调用者另行保证功率关闭/轴静止/REMOTE。 */
bool
board_encoder_hysteresis(uint8_t code)
{
    if(g_encoder_control_enabled || !g_encoder_probed) { return false; }
    if(!g_encoder_sensor_config_valid)
    { g_encoder_sensor_config_valid=mt6835_read_config(&g_encoder_sensor_config); }
    if(!g_encoder_sensor_config_valid) { return false; }
    bool changed=mt6835_set_hysteresis(code);
    g_encoder_sensor_config_valid=mt6835_read_config(&g_encoder_sensor_config);
    return changed && g_encoder_sensor_config_valid && (g_encoder_sensor_config.hysteresis&7U)==code;
}

/** @brief 见board_encoder.h。 */
void
board_encoder_diagnostics(char *out, size_t size, uint32_t now_ms)
{
    encoder_snapshot_t s = board_encoder_snapshot(now_ms);
    char angle[16] = "NA", speed[16] = "NA";
    if (s.angle_valid) { snprintf(angle, sizeof(angle), "%lu", (unsigned long)s.angle_mdeg); }
    if (s.speed_valid) { snprintf(speed, sizeof(speed), "%ld", (long)s.rpm); }
    /* Rev1.3第36页迟滞窗表；只有CONF_VALID=1时才解释配置字段。 */
    static const unsigned hyst_mdeg[8]={22U,44U,88U,176U,0U,3U,6U,11U};
    snprintf(out, size, "OK ENCODER CHIP=MT6835 STATE=%s COMM=%s PROBE_OK=%u STATUS=%u ANGLE_VALID=%u SPEED_VALID=%u RAW=%lu ANGLE_MDEG=%s RPM=%s AGE_MS=%lu SAMPLES=%lu IO_ERRORS=%lu CRC_ERRORS=%lu GAP_MAX_MS=%lu XFER_MAX_US=%lu CAL=LOW CONTROL=%s CONF_VALID=%u HYST_REG=%u HYST_CODE=%u HYST_MDEG=%u BW_REG=%u BW_CODE=%u",
        encoder_state_name(s.state), g_encoder_probed && s.state != ENCODER_IO_ERROR && s.state != ENCODER_CRC_ERROR ? "OK" : "ERROR",
        g_encoder_probed ? 1U : 0U, s.status, s.angle_valid ? 1U : 0U, s.speed_valid ? 1U : 0U,
        (unsigned long)s.raw, angle, speed, (unsigned long)s.age_ms, (unsigned long)s.samples,
        (unsigned long)s.io_errors, (unsigned long)s.crc_errors, (unsigned long)g_encoder_gap_max,
        (unsigned long)((g_encoder_transfer_max + SystemCoreClock / 1000000U - 1U) / (SystemCoreClock / 1000000U)),
        g_encoder_control_enabled ? "ENCODER_6STEP" : "OBSERVE",
        g_encoder_sensor_config_valid?1U:0U,(unsigned)g_encoder_sensor_config.hysteresis,
        g_encoder_sensor_config.hysteresis&7U,hyst_mdeg[g_encoder_sensor_config.hysteresis&7U],
        (unsigned)g_encoder_sensor_config.bandwidth,g_encoder_sensor_config.bandwidth&7U);
}

/** @brief 无等待撤销两路DMA请求并释放片选。 */
static void MOTOR_FAST_CODE
encoder_control_release(void)
{
    SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    DMA1_Channel2->CCR = 0U;
    DMA1_Channel3->CCR = 0U;
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
    GPIOC->BSRR = GPIO_PIN_4;
}

/** @brief 见board_encoder.h。 */
bool
board_encoder_control_begin(void)
{
    if (g_encoder_control_enabled || !g_encoder_probed) { return false; }
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    encoder_control_release();
    encoder_spi_reset();
    g_encoder_control_feedback = (encoder_six_feedback_t){0};
    g_encoder_control_phase = 0U;
    g_encoder_observed_sequence = 0U;
    g_encoder_control_status = 0U;
    /* START已确认轴静止；仅首帧跨过功率禁用期间的ADC初始化间隔。 */
    g_encoder_position_bridge = true;
    g_encoder_control_enabled = true;
    return true;
}

/** @brief 见board_encoder.h。 */
encoder_six_feedback_t MOTOR_FAST_CODE
board_encoder_control_tick(void)
{
    uint32_t now = DWT->CYCCNT;
    if (!g_encoder_control_enabled) { return (encoder_six_feedback_t){0}; }
    if (g_encoder_control_phase == 0U)
    {
        GPIOC->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
        g_encoder_control_phase_tick = now;
        g_encoder_control_phase = 1U;
    }
    else if (g_encoder_control_phase == 1U)
    {
        /* 片选提前一整个50us控制周期建立，覆盖100ns要求；本分支从不忙等。 */
        if (now - g_encoder_control_phase_tick < SystemCoreClock / 1000000U) { return g_encoder_control_feedback; }
        DMAMUX1_Channel1->CCR = DMA_REQUEST_SPI1_RX;
        DMAMUX1_Channel2->CCR = DMA_REQUEST_SPI1_TX;
        DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
        DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
        DMA1_Channel2->CMAR = (uint32_t)g_encoder_control_rx;
        DMA1_Channel2->CNDTR = 6U;
        DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
        DMA1_Channel3->CMAR = (uint32_t)g_encoder_control_tx;
        DMA1_Channel3->CNDTR = 6U;
        DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_EN;
        DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;
        g_encoder_control_started = now;
        SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
        g_encoder_control_phase = 2U;
    }
    else if (g_encoder_control_phase == 2U)
    {
        uint32_t flags = DMA1->ISR;
        bool error = (flags & (DMA_ISR_TEIF2 | DMA_ISR_TEIF3)) != 0U ||
            (SPI1->SR & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_FRE)) != 0U;
        bool complete = (flags & (DMA_ISR_TCIF2 | DMA_ISR_TCIF3)) == (DMA_ISR_TCIF2 | DMA_ISR_TCIF3) &&
            DMA1_Channel2->CNDTR == 0U && DMA1_Channel3->CNDTR == 0U && (SPI1->SR & SPI_SR_BSY) == 0U;
        if (!error && !complete && now - g_encoder_control_started < SystemCoreClock / 5000U) { return g_encoder_control_feedback; }
        uint8_t data[3] = {g_encoder_control_rx[2], g_encoder_control_rx[3], g_encoder_control_rx[4]};
        bool valid = !error && complete && mt6835_crc(data) == g_encoder_control_rx[5] && (data[2] & 7U) == 0U;
        uint32_t raw = ((uint32_t)data[0] << 13U) | ((uint32_t)data[1] << 5U) | (data[2] >> 3U);
        g_encoder_control_status = data[2] & 7U;
        g_encoder_control_feedback = (encoder_six_feedback_t){raw, g_encoder_control_started,
            g_encoder_control_feedback.sequence + 1U, valid};
        encoder_position_sample(&g_encoder_position,raw,g_encoder_control_started,valid,g_encoder_position_bridge);
        g_encoder_position_bridge=false;
        /* 完成后再保留一个周期的CS低电平，满足最后时钟到CS的保持；下周期才释放。 */
        SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
        g_encoder_control_phase = 3U;
    }
    else
    {
        encoder_control_release();
        g_encoder_control_phase = 0U;
    }
    return g_encoder_control_feedback;
}

/** @brief 见board_encoder.h。 */
void
board_encoder_control_end(void)
{
    if (!g_encoder_control_enabled) { return; }
    g_encoder_control_enabled = false;
    encoder_control_release();
    encoder_spi_reset();
    g_encoder_polled = false;
}

/** @brief 见board_encoder.h。 */
encoder_position_t
board_encoder_position(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    encoder_position_t value=g_encoder_position;
    __set_PRIMASK(mask);
    return value;
}

/** @brief 见board_encoder.h；功率关闭时只有主循环写坐标。 */
bool
board_encoder_position_zero(int8_t direction)
{
    return !g_encoder_control_enabled && encoder_position_zero(&g_encoder_position,direction);
}

/** @brief 见board_encoder.h。 */
void
board_encoder_position_invalidate(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    g_encoder_position.zero_valid=false;
    ++g_encoder_position.epoch;
    __set_PRIMASK(mask);
}
