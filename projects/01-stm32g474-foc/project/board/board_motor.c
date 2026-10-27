#include "board_motor.h"
#include "motor_config.h"
#if MOTOR_ENCODER_ENABLED
#include "board_encoder.h"
#endif
#if MOTOR_FOC_ENABLED
#include "board_foc.h"
#endif
#include "six_step.h"
#include "drv8353f.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* 所有时间基于已确认的 170 MHz TIM1/HCLK，ADC 异步 SYSCLK/4。 */
#define TIMER_HZ 170000000UL
#define TIMER_TICKS_US (TIMER_HZ / 1000000U)
#define PWM_ARR (TIMER_HZ / (2U * MOTOR_PWM_HZ))
/* TIM1 CKD=1，200 ns 精确对应 DTG=34；DRV 数字死区 100 ns 在芯片驱动配置。 */
#define TIMER_DEADTIME_NS 200U
#define DRIVER_DEADTIME_NS 100U
/* 栅极翻转到电流输入稳定的初始工程估计，需示波器复核，不是 CPU 延时。 */
#define SWITCH_SETTLE_NS 500U
/* 在下降计数距谷底 1 us 处触发，Rev Z 双转换也在谷底前完成。 */
#define SAMPLE_COMPARE TIMER_TICKS_US
/* 启动寄存器写入及更高优先级故障处理的时间预算，不延长 ADC 采样。 */
#define REGULAR_LAUNCH_TICKS (2U * TIMER_TICKS_US)
/* 六个慢速量：SOC、母线、三相电压、VREFINT；Rev Z 每项两次。 */
#define REGULAR_ITEMS 6U
#define REGULAR_MAX_RANKS (REGULAR_ITEMS * 2U)
/* 慢速采样的最短间隔，不是必须命中的触发时刻。 */
#define REGULAR_INTERVAL_TICKS (MOTOR_PWM_HZ / 1000U)
/* 初始化等待有界；运行 ISR 不使用等待。 */
#define INIT_TIMEOUT_US 2000U
#define ADC_IDLE_WARMUP_CYCLES (TIMER_HZ / 1000U)
/* ST 工厂 VREFINT 标定地址及标定电压。 */
#define VREFINT_CAL_ADDRESS 0x1FFF75AAUL
#define VREFINT_CAL_MV 3000U
/* ADC1 的 DMAMUX 请求编号，来自 STM32G474 请求映射。 */
#define ADC1_DMA_REQUEST 5U
/* ADC2 的 DMAMUX 请求编号；停机零点标定时复用同一 DMA 通道。 */
#define ADC2_DMA_REQUEST 36U
/* 相节点有1nF；实测允许12.5周期采样，加最坏2.5周期同步，ADC:TIMER=1:4。 */
#define BEMF_HOLD_TICKS 60U
/* 保持加12.5周期转换，浮相与电流同时由OC4导通窗口触发。 */
#define BEMF_CONVERSION_TICKS 110U
#define BEMF_EDGE_GUARD_TICKS (2U * TIMER_TICKS_US)
/* 每个完整 PWM 周期的整数微秒数，当前支持 10/20 kHz。 */
#define PWM_PERIOD_US (1000000U / MOTOR_PWM_HZ)
/* SMPR 编码：1=6.5、2=12.5、3=24.5 周期；改这一项即可做采样时间对比。 */
#define CURRENT_SAMPLE_SETTING 1U

/* ISR 发布的状态，主循环只在短临界区复制。 */
static volatile board_motor_status_t g_motor_status;
/* 当前实际输出及下周期候选；仅 TIM1/ADC ISR 和停机初始化使用。 */
static board_motor_output_t g_motor_output, g_motor_next;
static board_motor_sample_fn g_motor_sample;
/* 循环 DMA 缓冲：TC 后复制数据，下一次机会才重新启动 Regular。 */
static volatile uint16_t g_motor_dma[REGULAR_MAX_RANKS];
static volatile bool g_motor_regular_busy;
/* 修订版条件和预先算好的 JSQR，不在每周期判断静态硬件参数。 */
static uint32_t g_motor_jsqr[3], g_motor_regular_ticks;
static unsigned g_motor_repeat;
static uint32_t g_motor_next_regular, g_motor_last_conversion, g_motor_last_fast, g_motor_idle_ms;
static bool g_motor_initialized, g_motor_adc_active;
/* JEOS 消耗一个上下文，下一峰顶补一个，不根据 JSQR 未定义位判断空队列。 */
static volatile bool g_motor_need_context;
/* 仅停机切换采样拓扑；无感模式电流使用 ADC2。 */
static bool g_motor_bemf_enabled;
/* BEMF 发布值及独立采样看门时间，ISR 生产、快速回调/主循环读取。 */
static volatile board_motor_observation_t g_motor_observation;
static uint32_t g_motor_bemf_jsqr[3], g_motor_last_bemf, g_motor_last_current_conversion;
/* 导通窗口与本周期脉冲关联；換相退磁由ADC检测器消隐。 */
static bool g_motor_bemf_window, g_motor_bemf_need_context;
/* 将 DWT 周期增量累积为运行微秒，避免约 25 秒 DWT 回绕造成时间倒退。 */
static volatile uint32_t g_motor_clock_cycle, g_motor_clock_us, g_motor_clock_remainder;
/* 当前实际峰顶的运行相对时间及其后的 BEMF 采样保持时刻。 */
static volatile uint32_t g_motor_peak_us;
static uint32_t g_motor_bemf_sample_us;
/* 只在相组合实际改变时更新，不能被每周期重复 COM 或限流跳脉冲刷新。 */
static uint32_t g_motor_committed_us;
static uint8_t g_motor_sector;
/* 无感ADC直接检测；速度回调在TIM峰顶提交后按1ms分频。 */
static board_motor_bemf_fn g_motor_bemf_callback;
static board_motor_speed_fn g_motor_speed_callback;
static uint32_t g_motor_speed_divider;
#define SPEED_LOOP_DIVIDER (MOTOR_PWM_HZ / MOTOR_SPEED_LOOP_HZ)
_Static_assert(MOTOR_PWM_HZ % MOTOR_SPEED_LOOP_HZ == 0U, "PWM/speed divider");
/* 停机设置的轻量峰顶回调；预约仅在短临界区完整发布。 */
static board_motor_peak_fn g_motor_peak_callback;
/* 有感关闭留窗只存实际CCER清零时刻；下一峰顶仍使用新的编码器反馈。 */
static bool g_motor_encoder_blanked;
static uint32_t g_motor_encoder_off_cycle;
static volatile bool g_motor_controlled, g_motor_schedule_pending;
static volatile uint8_t g_motor_scheduled_sector;
static volatile uint16_t g_motor_control_duty;
static volatile uint32_t g_motor_schedule_us;
/** @brief 读取TIM独占更新的时钟锚点，不让后台修改ISR时基。
 * @return 本次运行微秒；短临界区仅复制锚点。 */
static uint32_t MOTOR_FAST_CODE
motor_time_us(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t cycle = DWT->CYCCNT - g_motor_clock_cycle;
    uint32_t now = g_motor_clock_us;
    uint32_t remainder = g_motor_clock_remainder;
    __set_PRIMASK(mask);
    return now + (cycle + remainder) / TIMER_TICKS_US;
}

/** @brief 只由TIM每PWM更新锚点，避免DWT约25秒回绕；发布仅几个对齐写入。 */
static void MOTOR_FAST_CODE
motor_clock_update(void)
{
    uint32_t cycle = DWT->CYCCNT;
    uint32_t elapsed = cycle - g_motor_clock_cycle + g_motor_clock_remainder;
    uint32_t now = g_motor_clock_us + elapsed / TIMER_TICKS_US;
    uint32_t remainder = elapsed % TIMER_TICKS_US;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    g_motor_clock_cycle = cycle;
    g_motor_clock_us = now;
    g_motor_clock_remainder = remainder;
    __set_PRIMASK(mask);
}

/** @brief 建立当前 ADC 通道、触发和扫描预算；仅在两路 ADC 已停机时调用。 */
static void
sampling_configure(void)
{
    TIM1->CR2 = TIM_CR2_CCPC | (g_motor_bemf_enabled ?
        ((7U << TIM_CR2_MMS_Pos) | (7U << TIM_CR2_MMS2_Pos)) : (7U << TIM_CR2_MMS2_Pos));
    /* CH5 只产生内部 OC5REF，CCR5 的 GC5C1/2/3 始终为零。 */
    TIM1->CCMR3 = g_motor_bemf_enabled ? (6U << TIM_CCMR3_OC5M_Pos) : 0U;
    for (unsigned i = 0; i < 3; ++i)
    {
        g_motor_jsqr[i] = (g_motor_repeat - 1U) | ADC_JSQR_JEXTEN_0 |
            (g_motor_bemf_enabled ? 0U : (8U << ADC_JSQR_JEXTSEL_Pos)) |
            ((i + 6U) << ADC_JSQR_JSQ1_Pos) | ((i + 6U) << ADC_JSQR_JSQ2_Pos);
        g_motor_bemf_jsqr[i] = ADC_JSQR_JEXTEN_0 |
            ((i + 2U) << ADC_JSQR_JSQ1_Pos);
    }
    static const uint16_t conversion_cycles[8] = {15, 19, 25, 37, 60, 105, 260, 653};
    g_motor_regular_ticks = ((g_motor_bemf_enabled ? 2U * 260U :
        5U * 260U + conversion_cycles[CURRENT_SAMPLE_SETTING]) * g_motor_repeat * 4U) + 10U;
}

/** @brief 仅初始化时按 DWT 等待。 @param us 微秒。 */
static void
delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < us * TIMER_TICKS_US) { __NOP(); }
}

/** @brief 等待硬件位达到目标，超时退出。 @param reg 寄存器。 @param mask 位。 @param set 目标。 @return 达到目标。 */
static bool
wait_bits(volatile uint32_t *reg, uint32_t mask, bool set)
{
    uint32_t start = DWT->CYCCNT;
    while (((*reg & mask) != 0U) != set)
    {
        if (DWT->CYCCNT - start > INIT_TIMEOUT_US * TIMER_TICKS_US) { return false; }
    }
    return true;
}

/** @brief DRV mode 1、16 bit SPI 事务，片选管理和超时只在主循环。 @param tx 发送帧。 @param rx 接收帧。 @return 完整事务成功。 */
static bool
driver_transfer(uint16_t tx, uint16_t *rx)
{
    GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16U;
    delay_us(1);
    bool ok = wait_bits(&SPI3->SR, SPI_SR_TXE, true);
    if (ok)
    {
        *(__IO uint16_t *)&SPI3->DR = tx;
        ok = wait_bits(&SPI3->SR, SPI_SR_RXNE, true);
    }
    if (ok)
    {
        *rx = *(__IO uint16_t *)&SPI3->DR;
        ok = wait_bits(&SPI3->SR, SPI_SR_BSY, false);
    }
    GPIOA->BSRR = GPIO_PIN_15;
    delay_us(1);
    return ok;
}

/** @brief 立即禁用功率；中断内不发 SPI、不等待转换结束。 */
void MOTOR_FAST_CODE
board_motor_stop(const char *reason)
{
#if MOTOR_FOC_ENABLED
    board_foc_abort(reason);
#endif
    GPIOB->BSRR = (uint32_t)GPIO_PIN_9 << 16U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->DIER = 0;
    ADC1->IER = 0;
    ADC2->IER = 0;
    g_motor_observation.valid = false;
    g_motor_bemf_window = false;
    memset((void *)g_motor_status.phase_valid, 0, sizeof(g_motor_status.phase_valid));
    g_motor_status.running = false;
    g_motor_schedule_pending = false;
    g_motor_controlled = false;
    g_motor_status.calibrated = false;
    if (NULL != reason) { g_motor_status.fault = reason; }
}

/** @brief 停机回收 ADC；Rev Y 软件 STOP 后执行 disable/enable 消除 2.7.10 条件。 @return ADC 已停止。 */
static bool
adc_quiesce(void)
{
    ADC_TypeDef *const converters[2] = {ADC1, ADC2};
    for (unsigned i = 0; i < 2U; ++i)
    {
        ADC_TypeDef *converter = converters[i];
        if (converter->CR & ADC_CR_JADSTART) { converter->CR |= ADC_CR_JADSTP; }
        if (converter->CR & ADC_CR_ADSTART) { converter->CR |= ADC_CR_ADSTP; }
        if (!wait_bits(&converter->CR, ADC_CR_JADSTART | ADC_CR_ADSTART, false)) { return false; }
        if (converter->CR & ADC_CR_ADEN)
        {
            converter->CR |= ADC_CR_ADDIS;
            if (!wait_bits(&converter->CR, ADC_CR_ADEN, false)) { return false; }
        }
    }
    DMA1_Channel1->CCR = 0;
    g_motor_regular_busy = false;
    g_motor_adc_active = false;
    return true;
}

/** @brief 使能已校准 ADC，调用前所有转换停止。 @return ADRDY 到达。 */
static bool
adc_enable(void)
{
    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    g_motor_adc_active = wait_bits(&ADC1->ISR, ADC_ISR_ADRDY, true);
    if (g_motor_adc_active && g_motor_bemf_enabled)
    {
        ADC2->ISR = ADC_ISR_ADRDY;
        ADC2->CR |= ADC_CR_ADEN;
        if (!wait_bits(&ADC2->ISR, ADC_ISR_ADRDY, true)) { return false; }
    }
    return g_motor_adc_active;
}

/** @brief 保存刚完成的实际寄存器回读供诊断显示，调用处仍需核对期望值。 @return 五个寄存器均成功读取。 */
static bool
driver_capture(void)
{
    for (unsigned i = 0; i < 5; ++i)
    {
        uint16_t value;
        if (!drv8353f_read((uint8_t)(i + 2U), &value)) { return false; }
        g_motor_status.driver_registers[i] = value;
    }
    g_motor_status.driver_verified = true;
    return true;
}

/** @brief 停机软件触发单个输入；Rev Z 双转换保留第一结果。 @param channel 输入。 @param value 结果。 @return 完整序列完成。 */
static bool
adc_sequence(uint32_t channel, uint16_t *value)
{
    bool current = g_motor_bemf_enabled && channel >= 6U && channel <= 8U;
    ADC_TypeDef *converter = current ? ADC2 : ADC1;
    /* 两份结果均由 DMA 搬走；即使主循环被抢占，也不会将第二份误作第一份。 */
    volatile uint16_t raw[2] = {0};
    DMA1_Channel1->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMAMUX1_Channel0->CCR = current ? ADC2_DMA_REQUEST : ADC1_DMA_REQUEST;
    DMA1_Channel1->CPAR = (uint32_t)&converter->DR;
    DMA1_Channel1->CMAR = (uint32_t)raw;
    DMA1_Channel1->CNDTR = g_motor_repeat;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_EN;
    converter->CFGR |= ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
    converter->SQR1 = (g_motor_repeat - 1U) | (channel << ADC_SQR1_SQ1_Pos) | (channel << ADC_SQR1_SQ2_Pos);
    converter->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    converter->CR |= ADC_CR_ADSTART;
    bool ok = wait_bits(&DMA1->ISR, DMA_ISR_TCIF1, true) && wait_bits(&converter->CR, ADC_CR_ADSTART, false);
    DMA1_Channel1->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    converter->CFGR &= ~(ADC_CFGR_DMAEN | ADC_CFGR_DMACFG);
    if (!ok) { return false; }
    *value = raw[0];
    if (current) { g_motor_last_current_conversion = DWT->CYCCNT; }
    else { g_motor_last_conversion = DWT->CYCCNT; }
    return true;
}

/** @brief 超过 1 ms 空闲后丢弃整个首序列，Rev Z 再保留下一序列第一结果。 @param channel 输入。 @param value 结果。 @return 成功。 */
static bool
adc_read(uint32_t channel, uint16_t *value)
{
    uint16_t discarded;
    uint32_t last = g_motor_bemf_enabled && channel >= 6U && channel <= 8U ?
        g_motor_last_current_conversion : g_motor_last_conversion;
    if (DWT->CYCCNT - last >= ADC_IDLE_WARMUP_CYCLES && !adc_sequence(channel, &discarded)) { return false; }
    return adc_sequence(channel, value);
}

/** @brief 发布完整慢速采样，时间戳对应实际转换完成时刻。 @param values SOC、母线、三相电压和VREFINT。 @param now 完成时刻，ms。 @return 参考有效且运行母线未越界。 */
static bool
publish_regular(const uint16_t values[REGULAR_ITEMS], uint32_t now)
{
    if (values[5] == 0) { board_motor_stop("VREF ADC"); return false; }
    uint32_t vdda = VREFINT_CAL_MV * *(const uint16_t *)VREFINT_CAL_ADDRESS / values[5];
    if (vdda < 2700U || vdda > 3600U) { board_motor_stop("VDDA"); return false; }
    g_motor_status.vdda_mv = (uint16_t)vdda;
    if (!g_motor_bemf_enabled) { g_motor_status.c_raw = values[0]; }
    g_motor_status.bus_mv = (uint16_t)((uint32_t)values[1] * vdda * 13U / 4095U);
    if (!g_motor_bemf_enabled)
    {
        for (unsigned i = 0; i < 3; ++i)
        {
            g_motor_status.phase_mv[i] = (uint16_t)((uint32_t)values[i + 2U] * vdda * 13U / 4095U);
            g_motor_status.phase_valid[i] = true;
        }
    }
    g_motor_status.slow_ms = now;
    ++g_motor_status.regular_count;
    if (g_motor_status.running && (g_motor_status.bus_mv < MOTOR_BUS_MIN_MV || g_motor_status.bus_mv > MOTOR_BUS_MAX_MV))
    {
        board_motor_stop("BUS VOLTAGE");
        return false;
    }
    return true;
}

/** @brief 功率禁用、Injected停止且ADC已使能时，软件采集完整慢速量。 @return 实际采样和发布成功；失败不更新旧时间戳。 */
static bool
regular_read(void)
{
    static const uint32_t channels[REGULAR_ITEMS] = {8, 1, 2, 3, 4, 18};
    uint16_t values[REGULAR_ITEMS] = {0};
    for (unsigned i = 0; i < REGULAR_ITEMS; ++i)
    {
        if (g_motor_bemf_enabled && i != 1U && i != 5U) { continue; }
        if (!adc_read(channels[i], &values[i])) { return false; }
    }
    return publish_regular(values, HAL_GetTick());
}

/** @brief TIM1 COM 原子切换两条有效桥臂；调用处位于 PWM 关闭的峰顶。 @param output 下周期输出。 */
static void MOTOR_FAST_CODE
apply_output(const board_motor_output_t *output)
{
    bool changed = output->high != g_motor_output.high || output->low != g_motor_output.low;
    if (g_motor_bemf_enabled)
    {
        /* 导通期抬高中性点；首个新扇区样本仍记录，由算法使用实际COM做退磁消隐。 */
        g_motor_bemf_window = output->pulse;
        TIM1->CCR5 = SAMPLE_COMPARE;
    }
    uint32_t mode[3] = {4U, 4U, 4U};
    uint32_t enables = 0;
    if (output->pulse)
    {
        mode[output->high] = 6U; /* PWM1 只启用主输出，高侧斩波。 */
        mode[output->low] = 5U;  /* 仅启用 N 时不反相，强制 OCREF=1 使低侧导通。 */
        enables = (TIM_CCER_CC1E << (4U * output->high)) | (TIM_CCER_CC1NE << (4U * output->low));
    }
    TIM1->CCR1 = output->duty_ticks;
    TIM1->CCR2 = output->duty_ticks;
    TIM1->CCR3 = output->duty_ticks;
    TIM1->CCMR1 = (mode[0] << TIM_CCMR1_OC1M_Pos) | (mode[1] << TIM_CCMR1_OC2M_Pos);
    TIM1->CCMR2 = (mode[2] << TIM_CCMR2_OC3M_Pos) | (6U << TIM_CCMR2_OC4M_Pos);
    /* 临时 CH4N 仅输出触发参考；跳脉冲时也保留标记，不改变三相桥臂使能。 */
#if MOTOR_ADC_TRACE_ENC_CAL
    enables |= TIM_CCER_CC4NE;
#endif
    TIM1->CCER = enables;
    TIM1->EGR = TIM_EGR_COMG;
    if (g_motor_bemf_enabled && changed)
    {
        static const uint8_t sectors[9] = {0, 0, 1, 3, 0, 2, 4, 5, 0};
        g_motor_sector = sectors[output->high * 3U + output->low];
        g_motor_committed_us = motor_time_us();
    }
    g_motor_output = *output;
}

/** @brief 有感高占空比同相只改CCR；换相缺少自然关闭窗时先全关一拍。
 * @param output 本次新鲜反馈决定的输出。 @return 已安全提交或留窗，超时则停机。 */
static bool MOTOR_FAST_CODE
encoder_apply_output(const board_motor_output_t *output)
{
    uint32_t count = TIM1->CNT;
    /* 保留有界计算截止；95%不能成为CPU迟到的豁免。 */
    if (!(TIM1->CR1 & TIM_CR1_DIR) || count <= PWM_ARR / 2U + BEMF_EDGE_GUARD_TICKS)
    {
        board_motor_stop("COM DEADLINE");
        return false;
    }
    if (g_motor_encoder_blanked)
    {
        if (output->pulse)
        {
            /* 实际六路CCER已清零，关闭时间不少于原2us余量；不重开MOE或ENABLE。 */
            if (DWT->CYCCNT - g_motor_encoder_off_cycle < BEMF_EDGE_GUARD_TICKS)
            {
                board_motor_stop("COM DEADLINE");
                return false;
            }
            apply_output(output);
            g_motor_encoder_blanked = false;
        }
        return true;
    }
    uint32_t previous = g_motor_output.pulse ? g_motor_output.duty_ticks : 0U;
    uint32_t maximum = previous > output->duty_ticks ? previous : output->duty_ticks;
    if (count > maximum + BEMF_EDGE_GUARD_TICKS)
    {
        apply_output(output);
        return true;
    }
    if (output->pulse && g_motor_output.pulse && output->high == g_motor_output.high && output->low == g_motor_output.low)
    {
        /* 无桥臂切换，不提交CCER/模式；CCR变化只调整同一高侧脉宽。 */
        TIM1->CCR1 = output->duty_ticks;
        TIM1->CCR2 = output->duty_ticks;
        TIM1->CCR3 = output->duty_ticks;
        g_motor_output = *output;
        return true;
    }
    board_motor_output_t off = g_motor_output;
    off.pulse = false;
    off.duty_ticks = 0U;
    apply_output(&off);
    g_motor_encoder_off_cycle = DWT->CYCCNT;
    g_motor_encoder_blanked = true;
    ++g_motor_status.encoder_blank_count;
    return true;
}

/** @brief 计算稳定时间之后的最短有效导通宽度，不固定插入 6 us 空白。 */
uint16_t
board_motor_min_duty(uint32_t shunt_mohm)
{
    uint32_t csa_ns = shunt_mohm == 50U ? 500U : 2000U;
    uint32_t settle = TIMER_DEADTIME_NS + DRIVER_DEADTIME_NS + SWITCH_SETTLE_NS + csa_ns;
    return (uint16_t)(SAMPLE_COMPARE + (settle * (TIMER_HZ / 1000000U) + 999U) / 1000U);
}

/** @brief 千分比占空比映射到中心对齐比较值。 */
uint16_t MOTOR_FAST_CODE
board_motor_duty_ticks(uint32_t duty) { return (uint16_t)(PWM_ARR * duty / 1000U); }

/** @brief 初始化实际引脚、SPI、定时器与 ADC；ENABLE 始终保持低。 */
bool MOTOR_BACKGROUND_CODE
board_motor_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    board_motor_stop(NULL);
    memset((void *)&g_motor_status, 0, sizeof(g_motor_status));
    g_motor_status.fault = "NONE";
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOC, &gpio);
    GPIOA->BSRR = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_SPI3;
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &gpio);
#if MOTOR_ADC_TRACE_ENC_CAL
    /* PC5/AF6=TIM1_CH4N，编码器接口 FPC2.5；仅开 N 输出时与 OC4REF 同相。 */
    gpio.Alternate = GPIO_AF6_TIM1;
    gpio.Pin = GPIO_PIN_5;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pull = GPIO_NOPULL;
#endif
    SPI3->CR2 = SPI_CR2_DS;
    SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_BR | SPI_CR1_SPE;
    gpio.Alternate = GPIO_AF6_TIM1;
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Alternate = GPIO_AF4_TIM1;
    gpio.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Alternate = GPIO_AF6_TIM1;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &gpio);
    TIM1->PSC = 0;
    TIM1->ARR = PWM_ARR;
    TIM1->CR1 = TIM_CR1_CMS_0;
    TIM1->CR2 = TIM_CR2_CCPC | (7U << TIM_CR2_MMS2_Pos); /* TRGO2=OC4REF，下降计数时仅一个上升沿。 */
    TIM1->CCR4 = SAMPLE_COMPARE;
    TIM1->AF1 = TIM1_AF1_BKINE;
    TIM1->BDTR = TIM_BDTR_BKE | TIM_BDTR_OSSI | TIM_BDTR_OSSR |
        (TIMER_DEADTIME_NS * (TIMER_HZ / 1000000U) / 1000U);
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0;
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_ADC12SEL, RCC_CCIPR_ADC12SEL_1);
    ADC12_COMMON->CCR = ADC_CCR_VREFEN | (2U << ADC_CCR_PRESC_Pos);
    ADC1->CR = ADC_CR_ADVREGEN;
    ADC2->CR = ADC_CR_ADVREGEN;
    delay_us(20);
    ADC1->CR |= ADC_CR_ADCAL;
    ADC2->CR |= ADC_CR_ADCAL;
    if (!wait_bits(&ADC1->CR, ADC_CR_ADCAL, false) || !wait_bits(&ADC2->CR, ADC_CR_ADCAL, false)) { return false; }
    delay_us(1);
    g_motor_status.revision = (uint16_t)(DBGMCU->IDCODE >> 16U);
    g_motor_repeat = (g_motor_status.revision == 0x2002U || g_motor_status.revision == 0x2003U) ? 1U : 2U;
    /* 电压分压源阻抗约 9.23 kΩ，选 247.5 周期；CSA 直连选 6.5 周期。 */
    ADC1->SMPR1 = (6U << 3U) | (6U << 6U) | (6U << 9U) | (6U << 12U) |
        (CURRENT_SAMPLE_SETTING << 18U) | (CURRENT_SAMPLE_SETTING << 21U) | (CURRENT_SAMPLE_SETTING << 24U);
    ADC1->SMPR2 = 6U << ADC_SMPR2_SMP18_Pos;
    ADC1->CFGR = ADC_CFGR_JQDIS;
    ADC2->SMPR1 = (CURRENT_SAMPLE_SETTING << 18U) | (CURRENT_SAMPLE_SETTING << 21U) |
        (CURRENT_SAMPLE_SETTING << 24U);
    ADC2->CFGR = ADC_CFGR_JQDIS;
    g_motor_last_conversion = DWT->CYCCNT - ADC_IDLE_WARMUP_CYCLES;
    g_motor_last_current_conversion = g_motor_last_conversion;
    sampling_configure();
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 2, 0);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    g_motor_initialized = true;
    return true;
}

/** @brief 选择采样拓扑，完整契约见头文件。 */
bool
board_motor_set_bemf(bool enabled)
{
#if MOTOR_FOC_ENABLED
    if (board_foc_active()) { return false; }
#endif
    if (!g_motor_initialized || g_motor_status.running || (GPIOB->ODR & GPIO_PIN_9) ||
        (TIM1->BDTR & TIM_BDTR_MOE)) { return false; }
    if (!adc_quiesce()) { board_motor_stop("ADC STOP"); return false; }
    g_motor_bemf_enabled = enabled;
    g_motor_status.bemf_enabled = enabled;
    g_motor_status.calibrated = false;
    memset((void *)&g_motor_observation, 0, sizeof(g_motor_observation));
    memset((void *)g_motor_status.phase_valid, 0, sizeof(g_motor_status.phase_valid));
    sampling_configure();
    /* 母线/VREFINT保留长采样；只有三相改为12.5周期。 */
    MODIFY_REG(ADC1->SMPR1, (7U << 6U) | (7U << 9U) | (7U << 12U),
        enabled ? (2U << 6U) | (2U << 9U) | (2U << 12U) : (6U << 6U) | (6U << 9U) | (6U << 12U));
    return true;
}

/** @brief 获取运行中的同一份观测帧和当前时刻，完整契约见头文件。 */
board_motor_observation_t
board_motor_observation(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    board_motor_observation_t result = g_motor_observation;
    result.next_commit_us = g_motor_peak_us + PWM_PERIOD_US;
    if (!g_motor_bemf_enabled || !g_motor_status.running) { result.valid = false; }
    __set_PRIMASK(mask);
    result.now_us = motor_time_us();
    return result;
}

/** @brief 设置峰顶回调，参见头文件；只允许停机装入。 */
void
board_motor_set_peak_callback(board_motor_peak_fn callback)
{
    if (!g_motor_status.running) { g_motor_peak_callback = callback; }
}

/** @brief 装入实时回调，完整契约见头文件。 */
void
board_motor_set_control_callbacks(board_motor_bemf_fn bemf, board_motor_speed_fn speed)
{
    if (!g_motor_status.running)
    {
        g_motor_bemf_callback = bemf;
        g_motor_speed_callback = speed;
    }
}

/** @brief 单次读取运行标志，参见头文件。 */
bool
board_motor_running(void)
{
#if MOTOR_FOC_ENABLED
    if (board_foc_active()) { return true; }
#endif
    return g_motor_status.running;
}

/** @brief 临界区仅用于少量快照字段，参见头文件。 */
uint32_t MOTOR_FAST_CODE
board_motor_control_lock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

/** @brief 恢复进入前的PRIMASK，参见头文件。 */
void MOTOR_FAST_CODE
board_motor_control_unlock(uint32_t mask)
{
    __DMB();
    __set_PRIMASK(mask);
}

/** @brief 完整预约在短临界区发布；TIM只能看到旧预约或完整新预约。 */
bool MOTOR_FAST_CODE
board_motor_schedule(uint8_t sector, uint32_t deadline_us, uint16_t duty, uint32_t *commit_us)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = motor_time_us();
    bool ok = g_motor_status.running && !g_motor_schedule_pending &&
        sector == (uint8_t)((g_motor_sector + 1U) % 6U) && (int32_t)(deadline_us - now) > 0;
    if (ok)
    {
        uint32_t peak = g_motor_peak_us + PWM_PERIOD_US;
        if ((int32_t)(deadline_us - peak) > 0)
        {
            peak += ((deadline_us - peak + PWM_PERIOD_US - 1U) / PWM_PERIOD_US) * PWM_PERIOD_US;
        }
        g_motor_scheduled_sector = sector;
        g_motor_control_duty = duty;
        g_motor_schedule_us = peak;
        g_motor_schedule_pending = true;
        g_motor_controlled = true;
        *commit_us = peak;
    }
    else if (g_motor_status.running) { board_motor_stop("BEMF SCHEDULE"); }
    __set_PRIMASK(mask);
    return ok;
}

/** @brief 更新PI幅度；单个对齐写入，峰顶回调负责单位换算。 */
void MOTOR_FAST_CODE
board_motor_set_control_duty(uint16_t duty)
{
    g_motor_control_duty = duty;
}

/** @brief 读取周期计数，参见头文件。 */
uint32_t MOTOR_FAST_CODE
board_motor_cycles(void) { return DWT->CYCCNT; }

/** @brief 见board_motor.h；原六步默认增益保持。 */
bool
board_motor_calibrate(uint32_t shunt_mohm)
{
    return board_motor_calibrate_gain(shunt_mohm,shunt_mohm==50U?10U:40U);
}

/** @brief 见board_motor.h；在开启输出前完成指定增益和零点的实际回读。 */
bool MOTOR_BACKGROUND_CODE
board_motor_calibrate_gain(uint32_t shunt_mohm,uint8_t gain)
{
    if (!g_motor_initialized || !((shunt_mohm==1U && gain==40U) ||
        (shunt_mohm==50U && (gain==10U || gain==20U)))) { return false; }
    g_motor_status.driver_verified = false;
    board_motor_stop(NULL);
    if (!adc_quiesce()) { board_motor_stop("ADC STOP"); return false; }
    ADC1->CFGR = ADC_CFGR_JQDIS;
    ADC2->CFGR = ADC_CFGR_JQDIS;
    if (!adc_enable()) { board_motor_stop("ADC ENABLE"); return false; }
    board_motor_output_t off = {0};
    apply_output(&off);
    /* 必须完成休眠复位再唤醒，避免重试落在 ENABLE 短脉冲复位区间。 */
    delay_us(MOTOR_DRIVER_SLEEP_US);
    GPIOB->BSRR = GPIO_PIN_9;
    delay_us(MOTOR_DRIVER_WAKE_US);
    bool ok = drv8353f_init_gain(driver_transfer,gain) && drv8353f_calibrate(true);
    if (ok) { delay_us(MOTOR_CSA_SETTLE_US); ok = drv8353f_calibrate(false); }
    if (ok) { delay_us(MOTOR_CSA_SETTLE_US); }
    for (unsigned phase = 0; ok && phase < 3; ++phase)
    {
        uint32_t sum = 0;
        uint16_t minimum = 4095, maximum = 0;
        for (unsigned i = 0; ok && i < MOTOR_ZERO_SAMPLES; ++i)
        {
            uint16_t raw;
            ok = adc_read(phase + 6U, &raw);
            if (!ok) { break; }
            sum += raw;
            if (raw < minimum) { minimum = raw; }
            if (raw > maximum) { maximum = raw; }
        }
        g_motor_status.zero[phase] = (uint16_t)(sum / MOTOR_ZERO_SAMPLES);
        ok = ok && minimum > 1024U && maximum < 3072U && (uint32_t)(maximum - minimum) < 100U;
    }
    uint16_t f1 = 0, f2 = 0;
    ok = ok && drv8353f_verify(true) && driver_capture() && drv8353f_faults(&f1, &f2) && f1 == 0 && f2 == 0 &&
        (GPIOB->IDR & GPIO_PIN_12) != 0;
    /* 100 ms唤醒期间慢速采样暂停；先取得新母线和VDDA，再允许进入RUN。 */
    if (ok) { ok = regular_read(); }
    g_motor_status.calibrated = ok;
    g_motor_status.fault = ok ? "NONE" : "DRV/ADC CAL";
    if (!ok) { board_motor_stop("DRV/ADC CAL"); }
    return ok;
}

/** @brief 建立循环 DMA 序列，保留 Rev Z 每对中的第一个结果。 */
static void
regular_configure(void)
{
    static const uint32_t channels[REGULAR_ITEMS] = {8, 1, 2, 3, 4, 18};
    unsigned items = g_motor_bemf_enabled ? 2U : REGULAR_ITEMS;
    uint32_t sqr[4] = {items * g_motor_repeat - 1U, 0, 0, 0};
    for (unsigned rank = 0; rank < items * g_motor_repeat; ++rank)
    {
        unsigned reg = rank < 4 ? 0 : rank < 9 ? 1 : 2;
        unsigned pos = rank < 4 ? 6U + rank * 6U : rank < 9 ? (rank - 4U) * 6U : (rank - 9U) * 6U;
        unsigned item = rank / g_motor_repeat;
        uint32_t channel = g_motor_bemf_enabled ? (item == 0U ? 1U : 18U) : channels[item];
        sqr[reg] |= channel << pos;
    }
    ADC1->SQR1 = sqr[0]; ADC1->SQR2 = sqr[1]; ADC1->SQR3 = sqr[2]; ADC1->SQR4 = 0;
    DMA1_Channel1->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMAMUX1_Channel0->CCR = ADC1_DMA_REQUEST;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)g_motor_dma;
    DMA1_Channel1->CNDTR = items * g_motor_repeat;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
        DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN;
}

/** @brief 功率禁用时启动计数器，在首次实际峰顶UEV对齐共用PWM周期更新。 @return 在有界等待内确认峰顶；失败停机。 */
static bool
timer_prime_peak(void)
{
    TIM1->CR1 = 0;
    TIM1->RCR = 0;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_CMS_0;
    TIM1->CNT = 0;
    TIM1->SR = 0;
    /* RM0440 29.3.5：计数器启动后再写奇数RCR，将UEV锁定到峰顶。 */
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->RCR = 1;
    if (!wait_bits(&TIM1->SR, TIM_SR_UIF, true) || !(TIM1->CR1 & TIM_CR1_DIR))
    {
        board_motor_stop("TIM PHASE");
        return false;
    }
    TIM1->SR = 0;
    return true;
}

/** @brief 启动前排除整段慢速扫描无法容纳的时基，避免通电后必然母线采样过期。
 * @return 最大一个PWM周期仍有扫描及启动余量为true，否则禁用功率并记录ADC TIMING。 */
static bool
regular_timing_ready(void)
{
    /* 首版无感采样只验收单次转换修订版，不在导通窗口猜测配对时序。 */
    if (g_motor_bemf_enabled && g_motor_repeat != 1U)
    {
        board_motor_stop("BEMF REV");
        return false;
    }
    /* 只做静态必要条件检查；运行仍按真实CNT窗口决定是否启动Regular。
     * 20 kHz单次扫描约31.1 us可容纳，Rev Z/未知修订的62.1 us配对扫描拒绝启动。 */
    if (six_step_regular_fits(2U * PWM_ARR, g_motor_regular_ticks, REGULAR_LAUNCH_TICKS))
    {
        return true;
    }
    board_motor_stop("ADC TIMING");
    return false;
}

/** @brief 功率禁用时验证、预热并同步峰顶，再开启Injected及输出。 @param output 初始扇区。 @param sample 快速采样回调。 @return 同步和启动均成功。 */
bool
board_motor_start(const board_motor_output_t *output, board_motor_sample_fn sample)
{
    if (NULL == output || NULL == sample || output->high >= 3U || output->low >= 3U ||
        output->high == output->low || output->duty_ticks > PWM_ARR)
    {
        board_motor_stop("OUTPUT CONFIG");
        return false;
    }
    if (!regular_timing_ready()) { return false; }
    if (!g_motor_status.calibrated) { board_motor_stop("NOT CALIBRATED"); return false; }
    if (!drv8353f_verify(true)) { board_motor_stop("DRV READBACK"); return false; }
    if (g_motor_bemf_enabled && output->duty_ticks + BEMF_CONVERSION_TICKS + BEMF_EDGE_GUARD_TICKS >= PWM_ARR)
    {
        board_motor_stop("BEMF TIMING");
        return false;
    }
    uint16_t dummy;
    /* 使用 Regular 预热整个 ADC1；不是每次换相都重做预热。 */
    if (!adc_read(6U + output->low, &dummy)) { board_motor_stop("ADC WARMUP"); return false; }
    if (g_motor_bemf_enabled && !adc_read(5U - output->high - output->low, &dummy))
    {
        board_motor_stop("BEMF WARMUP");
        return false;
    }
    if (!adc_quiesce()) { board_motor_stop("ADC STOP"); return false; }
    ADC1->CFGR = ADC_CFGR_JQM | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
    ADC2->CFGR = ADC_CFGR_JQM;
    if (!adc_enable()) { board_motor_stop("ADC ENABLE"); return false; }
    regular_configure();
    g_motor_sample = sample;
    g_motor_need_context = false;
    g_motor_bemf_need_context = false;
    g_motor_next = *output;
    apply_output(output);
    if (!drv8353f_coast(false) || !drv8353f_verify(false) || !driver_capture()) { board_motor_stop("DRV READBACK"); return false; }
    g_motor_status.update_count = 0;
    g_motor_status.peak_count = 0;
    g_motor_status.bus_age_ms = 0;
    g_motor_speed_divider = 0U;
    g_motor_status.speed_count = 0U;
    g_motor_status.encoder_blank_count = 0U;
    g_motor_encoder_blanked = false;
    g_motor_encoder_off_cycle = 0U;
    g_motor_status.peak_max_cycles = g_motor_status.speed_max_cycles = g_motor_status.tim_total_max_cycles = 0U;
    /* 此时MOE仍为0、Injected未启动；先确认真实峰顶相位再开放功率。 */
    if (!timer_prime_peak()) { return false; }
    /* SPI 回读可能跨过 1 ms；此时还未输出，用软件 Regular 首转换预热。 */
    uint32_t current_last = g_motor_bemf_enabled ? g_motor_last_current_conversion : g_motor_last_conversion;
    if (DWT->CYCCNT - current_last >= ADC_IDLE_WARMUP_CYCLES ||
        (g_motor_bemf_enabled && DWT->CYCCNT - g_motor_last_conversion >= ADC_IDLE_WARMUP_CYCLES))
    {
        DMA1_Channel1->CCR = 0;
        ADC1->CFGR &= ~(ADC_CFGR_DMAEN | ADC_CFGR_DMACFG);
        if (!adc_read(6U + output->low, &dummy)) { board_motor_stop("ADC WARMUP"); return false; }
        if (g_motor_bemf_enabled && !adc_read(5U - output->high - output->low, &dummy))
        {
            board_motor_stop("BEMF WARMUP");
            return false;
        }
        ADC1->CFGR |= ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
        regular_configure();
    }
    /* 双 ADC 的停采预热可能跨过首个导通触发，重新对齐后再同时开放。 */
    if (g_motor_bemf_enabled && !timer_prime_peak()) { return false; }
    /* 原子开放两路队列与running，避免首个JEOS先于软件运行态而被丢弃。
     * 等待和SPI均已结束；这里只屏蔽一段寄存器写入，不等待硬件。 */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (g_motor_bemf_enabled && (!(TIM1->CR1 & TIM_CR1_DIR) ||
        TIM1->CNT <= SAMPLE_COMPARE + REGULAR_LAUNCH_TICKS))
    {
        board_motor_stop("TIM START");
        __set_PRIMASK(mask);
        return false;
    }
    TIM1->SR = 0;
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JQOVF | ADC_ISR_OVR;
    ADC1->JSQR = g_motor_bemf_enabled ? g_motor_bemf_jsqr[3U - output->high - output->low] : g_motor_jsqr[output->low];
    ADC1->CR |= ADC_CR_JADSTART;
    ADC1->IER = ADC_IER_JEOSIE | ADC_IER_JQOVFIE | ADC_IER_OVRIE;
    if (g_motor_bemf_enabled)
    {
        ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JQOVF | ADC_ISR_OVR;
        ADC2->JSQR = g_motor_jsqr[output->low];
        ADC2->CR |= ADC_CR_JADSTART;
        ADC2->IER = ADC_IER_JEOSIE | ADC_IER_JQOVFIE | ADC_IER_OVRIE;
        memset((void *)&g_motor_observation, 0, sizeof(g_motor_observation));
        g_motor_speed_divider = 0U;
        g_motor_controlled = g_motor_schedule_pending = false;
        for (uint8_t sector = 0U; sector < 6U; ++sector)
        {
            uint8_t high, low;
            six_step_phases(sector, &high, &low);
            if (high == output->high && low == output->low) { g_motor_sector = sector; break; }
        }
        g_motor_status.irq_max_cycles = g_motor_status.current_max_cycles = g_motor_status.bemf_max_cycles = 0U;
        g_motor_status.peak_max_cycles = g_motor_status.speed_max_cycles = g_motor_status.speed_count = 0U;
        g_motor_status.tim_total_max_cycles = 0U;
        g_motor_status.sample_age_max_us = 0U;
        memset((void *)g_motor_status.phase_valid, 0, sizeof(g_motor_status.phase_valid));
        g_motor_clock_cycle = DWT->CYCCNT;
        /* 当前 CNT 保留首峰顶后经过的时间，不用 ISR 进入时刻冒充峰顶。 */
        uint32_t peak_ticks = PWM_ARR - TIM1->CNT;
        g_motor_clock_us = peak_ticks / TIMER_TICKS_US;
        g_motor_clock_remainder = peak_ticks % TIMER_TICKS_US;
        g_motor_peak_us = 0;
        g_motor_committed_us = g_motor_clock_us;
        g_motor_bemf_window = output->pulse;
        g_motor_bemf_sample_us = (PWM_ARR - SAMPLE_COMPARE + BEMF_HOLD_TICKS) / TIMER_TICKS_US;
    }
    g_motor_next_regular = g_motor_status.fast_count;
    g_motor_last_fast = DWT->CYCCNT;
    g_motor_last_bemf = g_motor_last_fast;
    g_motor_status.running = true;
    TIM1->DIER = TIM_DIER_UIE | TIM_DIER_BIE;
    TIM1->BDTR |= TIM_BDTR_MOE;
    __set_PRIMASK(mask);
    return true;
}

/** @brief 启动机会来自 JEOS，先检查真实剩余时间；不足时不启动也不抢占高速转换。 */
static void MOTOR_FAST_CODE
regular_opportunity(void)
{
    if (g_motor_regular_busy || (int32_t)(g_motor_status.fast_count - g_motor_next_regular) < 0) { return; }
    uint32_t cnt = TIM1->CNT;
    uint32_t compare = SAMPLE_COMPARE;
    uint32_t remaining = (TIM1->CR1 & TIM_CR1_DIR) ?
        (cnt >= compare ? cnt - compare : cnt + 2U * PWM_ARR - compare) :
        2U * PWM_ARR - cnt - compare;
    if (!six_step_regular_fits(remaining, g_motor_regular_ticks, REGULAR_LAUNCH_TICKS))
    {
        ++g_motor_status.regular_skipped;
        return;
    }
    g_motor_regular_busy = true;
    ADC1->ISR = ADC_ISR_EOS | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADSTART;
    g_motor_next_regular = g_motor_status.fast_count + REGULAR_INTERVAL_TICKS;
}

/** @brief 读取同步浮相样本后立即回调轻量检测，不依赖后台消费。
 * @param started ISR 开始的 DWT 周期，供独立采样看门。 */
static void MOTOR_FAST_CODE
bemf_publish(uint32_t started)
{
    uint8_t floating = (uint8_t)(3U - g_motor_output.high - g_motor_output.low);
    g_motor_last_bemf = g_motor_last_conversion = started;
    g_motor_bemf_need_context = true;
    board_motor_observation_t frame = {
        .raw=(uint16_t)ADC1->JDR1, .floating=floating, .sector=g_motor_sector,
        .valid=g_motor_bemf_window, .sequence=g_motor_observation.sequence + 1U,
        .sample_us=g_motor_bemf_sample_us, .committed_us=g_motor_committed_us,
        .next_commit_us=g_motor_peak_us + PWM_PERIOD_US, .now_us=motor_time_us()
    };
    g_motor_observation = frame;
    uint32_t age = frame.now_us - frame.sample_us;
    if (age > g_motor_status.sample_age_max_us) { g_motor_status.sample_age_max_us = age; }
    g_motor_status.phase_valid[floating] = frame.valid;
    g_motor_status.phase_sample_us[floating] = frame.sample_us;
    if (g_motor_bemf_callback != NULL) { g_motor_bemf_callback(&frame); }
}

/** @brief 电流保护优先，浮相执行精简检测/事件预约；速度PI在TIM低频分支，诊断换算不在本IRQ。 */
void MOTOR_FAST_CODE
ADC1_2_IRQHandler(void)
{
#if MOTOR_FOC_ENABLED
    /* 覆盖分派、FOC计算和采样复制的整个软件处理段；原保护条件不变。 */
    uint32_t foc_started=DWT->CYCCNT;
    if (board_foc_active())
    {
        board_foc_adc_irq();
        uint32_t foc_elapsed=DWT->CYCCNT-foc_started;
        if(foc_elapsed>g_motor_status.irq_max_cycles) { g_motor_status.irq_max_cycles=foc_elapsed; }
        return;
    }
#endif
    uint32_t started = DWT->CYCCNT;
    uint32_t voltage_started = started;
    bool voltage_processed = false;
    ADC_TypeDef *current = g_motor_bemf_enabled ? ADC2 : ADC1;
    uint32_t flags = current->ISR;
    current->ISR = flags & (ADC_ISR_JEOS | ADC_ISR_JEOC | ADC_ISR_OVR | ADC_ISR_JQOVF);
    if (flags & (ADC_ISR_OVR | ADC_ISR_JQOVF)) { board_motor_stop("ADC OVERRUN"); goto adc_finish; }
    if (!g_motor_status.running) { goto adc_finish; }
    uint32_t voltage_flags = 0U;
    if (g_motor_bemf_enabled)
    {
        voltage_flags = ADC1->ISR;
        ADC1->ISR = voltage_flags & (ADC_ISR_JEOS | ADC_ISR_JEOC | ADC_ISR_OVR | ADC_ISR_JQOVF);
        if (voltage_flags & (ADC_ISR_OVR | ADC_ISR_JQOVF)) { board_motor_stop("BEMF ADC"); goto adc_finish; }
    }
    if (flags & ADC_ISR_JEOS)
    {
        g_motor_last_fast = started;
        if (g_motor_bemf_enabled) { g_motor_last_current_conversion = started; }
        else { g_motor_last_conversion = started; }
        g_motor_need_context = true;
        ++g_motor_status.fast_count;
        g_motor_sample((uint16_t)current->JDR1, g_motor_output.low, g_motor_output.pulse, &g_motor_next);
        uint32_t elapsed = DWT->CYCCNT - started;
        if (elapsed > g_motor_status.current_max_cycles) { g_motor_status.current_max_cycles = elapsed; }
        if (!g_motor_status.running) { goto adc_finish; }
        if (!g_motor_bemf_enabled) { regular_opportunity(); }
    }
    if (voltage_flags & ADC_ISR_JEOS)
    {
        voltage_started = DWT->CYCCNT;
        voltage_processed = true;
        bemf_publish(started);
        if (!g_motor_status.running) { goto adc_finish; }
        regular_opportunity();
    }
adc_finish:
    if (voltage_processed)
    {
        uint32_t voltage_elapsed = DWT->CYCCNT - voltage_started;
        if (voltage_elapsed > g_motor_status.bemf_max_cycles) { g_motor_status.bemf_max_cycles = voltage_elapsed; }
    }
    if (g_motor_status.running && g_motor_bemf_enabled && (int32_t)(motor_time_us() - (g_motor_peak_us + PWM_PERIOD_US)) >= 0)
    {
        /* ADC只生成远处换相预约；保护/检测不得阻塞到下一峰顶。 */
        board_motor_stop("CONTROL DEADLINE");
    }
    uint32_t elapsed = DWT->CYCCNT - started;
    if (elapsed > g_motor_status.irq_max_cycles) { g_motor_status.irq_max_cycles = elapsed; }
}

/** @brief 峰顶时高侧 PWM 已低，提交下周期扇区；谷底更新不换相。 */
void MOTOR_FAST_CODE
TIM1_UP_TIM16_IRQHandler(void)
{
#if MOTOR_FOC_ENABLED
    if (board_foc_active()) { board_foc_update_irq(); return; }
#endif
    uint32_t started = DWT->CYCCNT;
    uint32_t fast_elapsed = 0U;
    bool fast_complete = false;
    TIM1->SR = ~TIM_SR_UIF;
    if (!g_motor_status.running) { goto peak_finish; }
    ++g_motor_status.update_count;
    if (!(TIM1->CR1 & TIM_CR1_DIR))
    {
        /* 谷底不能切换桥臂；也不能静默丢弃全部UEV使ADC队列停在首样本。 */
        board_motor_stop("TIM PHASE");
        goto peak_finish;
    }
    ++g_motor_status.peak_count;
    if (DWT->CYCCNT - g_motor_last_fast > ADC_IDLE_WARMUP_CYCLES)
    {
        board_motor_stop("ADC TIMEOUT");
        goto peak_finish;
    }
    if (g_motor_bemf_enabled)
    {
        if (DWT->CYCCNT - g_motor_last_bemf > ADC_IDLE_WARMUP_CYCLES)
        {
            board_motor_stop("BEMF TIMEOUT");
            goto peak_finish;
        }
        motor_clock_update();
        uint32_t now = motor_time_us();
        g_motor_peak_us = now - (PWM_ARR - TIM1->CNT) / TIMER_TICKS_US;
        /* us整数量化允许1us误差，不因锚点取整晚整整一个PWM。 */
        if (g_motor_schedule_pending && (int32_t)(g_motor_peak_us + 1U - g_motor_schedule_us) >= 0)
        {
            if ((int32_t)(g_motor_peak_us - (g_motor_schedule_us + PWM_PERIOD_US - 1U)) >= 0)
            {
                board_motor_stop("BEMF LATE");
                goto peak_finish;
            }
            six_step_phases(g_motor_scheduled_sector, &g_motor_next.high, &g_motor_next.low);
            g_motor_schedule_pending = false;
        }
        if (g_motor_peak_callback != NULL)
        {
            uint8_t sector = g_motor_controlled ?
                (uint8_t)((g_motor_next.high == g_motor_output.high && g_motor_next.low == g_motor_output.low) ?
                    g_motor_sector : g_motor_scheduled_sector) : g_motor_sector;
            g_motor_peak_callback(g_motor_controlled, sector, g_motor_control_duty, &g_motor_next);
            if (!g_motor_status.running) { goto peak_finish; }
        }
    }
    else if (g_motor_peak_callback != NULL)
    {
        /* 编码器换相使用原电流采样，峰顶只推进非阻塞DMA和角度计算。 */
        g_motor_peak_callback(false,g_motor_sector,g_motor_control_duty,&g_motor_next);
        if (!g_motor_status.running) { goto peak_finish; }
    }
    /* 无感保持原关闭窗；有感可在高占空比扇区变化时主动全关一拍。 */
    if (!g_motor_bemf_enabled && g_motor_peak_callback != NULL)
    {
        if (!encoder_apply_output(&g_motor_next)) { goto peak_finish; }
    }
    else
    {
        if (g_motor_bemf_enabled && (!(TIM1->CR1 & TIM_CR1_DIR) ||
            TIM1->CNT <= g_motor_next.duty_ticks + BEMF_EDGE_GUARD_TICKS))
        {
            board_motor_stop("COM DEADLINE");
            goto peak_finish;
        }
        apply_output(&g_motor_next);
    }
    /* 启动已排入首上下文；每次JEOS消耗后仅补一个，避免队列重复。 */
    if (g_motor_need_context)
    {
        if (g_motor_bemf_enabled) { ADC2->JSQR = g_motor_jsqr[g_motor_output.low]; }
        else { ADC1->JSQR = g_motor_jsqr[g_motor_output.low]; }
        g_motor_need_context = false;
    }
    if (g_motor_bemf_enabled)
    {
        /* 同一峰顶提交新扇区及比较值；保持时间含异步同步最坏 2.5 周期，误差小于 0.06 us。 */
        g_motor_bemf_sample_us = g_motor_peak_us + (PWM_ARR - SAMPLE_COMPARE + BEMF_HOLD_TICKS) / TIMER_TICKS_US;
        if (g_motor_bemf_need_context)
        {
            ADC1->JSQR = g_motor_bemf_jsqr[3U - g_motor_output.high - g_motor_output.low];
            g_motor_bemf_need_context = false;
        }
    }
    fast_elapsed = DWT->CYCCNT - started;
    fast_complete = true;
    if ((g_motor_bemf_enabled || g_motor_peak_callback != NULL) && ++g_motor_speed_divider >= SPEED_LOOP_DIVIDER)
    {
        g_motor_speed_divider = 0U;
        /* 慢速量仍以1ms检查数据新鲜度，主循环停摆不应取消母线停采保护。 */
        g_motor_status.bus_age_ms = HAL_GetTick() - g_motor_status.slow_ms;
        if (g_motor_status.bus_age_ms > MOTOR_BUS_STALE_MS)
        {
            board_motor_stop("BUS STALE");
            goto peak_finish;
        }
        uint32_t speed_started = DWT->CYCCNT;
        if (g_motor_speed_callback != NULL)
        {
            ++g_motor_status.speed_count;
            g_motor_speed_callback();
        }
        uint32_t speed_elapsed = DWT->CYCCNT - speed_started;
        if (speed_elapsed > g_motor_status.speed_max_cycles) { g_motor_status.speed_max_cycles = speed_elapsed; }
    }
peak_finish:
    if (!fast_complete) { fast_elapsed = DWT->CYCCNT - started; }
    if (fast_elapsed > g_motor_status.peak_max_cycles) { g_motor_status.peak_max_cycles = fast_elapsed; }
    uint32_t total = DWT->CYCCNT - started;
    if (total > g_motor_status.tim_total_max_cycles) { g_motor_status.tim_total_max_cycles = total; }
}

/** @brief BKIN 硬件已撤销 MOE；软件同步锁存故障并拉低 ENABLE。 */
void
TIM1_BRK_TIM15_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_BIF;
    board_motor_stop("DRV NFAULT");
}

/** @brief 循环 DMA 完成后发布慢速量；错误时停机，不让丢数据继续运行。 */
void
DMA1_Channel1_IRQHandler(void)
{
#if MOTOR_FOC_ENABLED
    if (board_foc_active()) { board_foc_dma_irq(); return; }
#endif
    uint32_t flags = DMA1->ISR;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    if (flags & DMA_ISR_TEIF1) { board_motor_stop("ADC DMA"); return; }
    if (flags & DMA_ISR_TCIF1)
    {
        uint16_t values[REGULAR_ITEMS] = {0};
        if (g_motor_bemf_enabled)
        {
            values[1] = g_motor_dma[0];
            values[5] = g_motor_dma[g_motor_repeat];
        }
        else
        {
            for (unsigned i = 0; i < REGULAR_ITEMS; ++i) { values[i] = g_motor_dma[i * g_motor_repeat]; }
        }
        publish_regular(values, HAL_GetTick());
        g_motor_last_conversion = DWT->CYCCNT;
        g_motor_regular_busy = false;
    }
}

/** @brief 停机每10 ms实测一次；运行超时用当前时钟，避免DMA比调度时间更新一毫秒时误停。 @param now 主循环调度参考，ms。 */
void
board_motor_service(uint32_t now)
{
#if MOTOR_FOC_ENABLED
    if (board_foc_active()) { return; }
#endif
    if (!g_motor_initialized) { return; }
    if (g_motor_status.running)
    {
        /* 无感运行的母线新鲜度由TIM低频分支检查，后台只服务停机与交互。 */
        if (g_motor_bemf_enabled) { return; }
        /* 先取已发布时间，再读当前Tick，不能拿旧调度时间减去新DMA时间戳。 */
        uint32_t measured_ms = g_motor_status.slow_ms;
        uint32_t age_ms = HAL_GetTick() - measured_ms;
        g_motor_status.bus_age_ms = age_ms;
        if (age_ms > MOTOR_BUS_STALE_MS) { board_motor_stop("BUS STALE"); }
        return;
    }
    if (g_motor_adc_active && !adc_quiesce()) { g_motor_status.fault = "ADC STOP"; return; }
    if (now - g_motor_idle_ms < 10U) { return; }
    g_motor_idle_ms = now;
    ADC1->CFGR = ADC_CFGR_JQDIS;
    ADC2->CFGR = ADC_CFGR_JQDIS;
    if (!adc_enable()) { g_motor_status.fault = "ADC ENABLE"; return; }
    if (!regular_read()) { g_motor_status.fault = "ADC READ"; }
    (void)adc_quiesce();
}

/** @brief 启动前检查低反电动势，MOE/ENABLE始终关闭；所有等待仅在主循环。 */
bool
board_motor_standstill(void)
{
    g_motor_status.rest_scans=0U;
    g_motor_status.rest_line_ptp_mv=0U;
    g_motor_status.rest_line_raw_ptp_mv=0U;
    if(!board_motor_storage_safe() || !board_motor_set_bemf(false)) { return false; }
    ADC1->CFGR=ADC_CFGR_JQDIS;
    ADC2->CFGR=ADC_CFGR_JQDIS;
    if(!adc_enable()) { (void)adc_quiesce(); return false; }
    bool still=true;
    int32_t minimum[3]={0},maximum[3]={0};
    int32_t raw_minimum[3]={0},raw_maximum[3]={0};
    for(unsigned n=0;n<128U;++n)
    {
        uint32_t sum[3]={0};
        /* 只在输出关闭的主循环做短组平均，减小停机ADC噪声；不改变运行电流采样。
         * 8次扫描小于约0.3ms，远短于需要拒绝的50/100Hz旋转反电动势周期。 */
        for(unsigned repeat=0;repeat<8U;++repeat)
        {
            if(!regular_read()) { still=false; break; }
            for(unsigned i=0;i<3U;++i)
            {
                sum[i]+=g_motor_status.phase_mv[i];
                int32_t raw=(int32_t)g_motor_status.phase_mv[i]-g_motor_status.phase_mv[(i+1U)%3U];
                if(n==0U&&repeat==0U) { raw_minimum[i]=raw_maximum[i]=raw; }
                if(raw<raw_minimum[i]) { raw_minimum[i]=raw; }
                if(raw>raw_maximum[i]) { raw_maximum[i]=raw; }
                uint32_t span=(uint32_t)(raw_maximum[i]-raw_minimum[i]);
                if(span>g_motor_status.rest_line_raw_ptp_mv) { g_motor_status.rest_line_raw_ptp_mv=span; }
            }
        }
        if(!still) { break; }
        ++g_motor_status.rest_scans;
        /* 实测静止通道有约100mV固定差值，瞬时压差不能当作反电动势。
         * 保持80mV门限，检查整个静止窗口内AB/BC/CA各自的时间变化，固定偏置自行消除。 */
        for(unsigned i=0;i<3U;++i)
        {
            int32_t value=(int32_t)((sum[i]+4U)/8U)-(int32_t)((sum[(i+1U)%3U]+4U)/8U);
            if(n==0U) { minimum[i]=maximum[i]=value; }
            if(value<minimum[i]) { minimum[i]=value; }
            if(value>maximum[i]) { maximum[i]=value; }
            uint32_t span=(uint32_t)(maximum[i]-minimum[i]);
            if(span>g_motor_status.rest_line_ptp_mv) { g_motor_status.rest_line_ptp_mv=span; }
            if(span>80U) { still=false; }
        }
        if(!still) { break; }
        /* 静止预检约0.2秒；在主循环继续读编码器，避免机械坐标因预检自身丢采样。 */
#if MOTOR_ENCODER_ENABLED
        board_encoder_service();
#endif
        /* 0.9/1.1ms交替避免原10ms停机扫描与50/100Hz电频率同相混叠。 */
        delay_us(n%2U?1100U:900U);
    }
    return adc_quiesce() && still;
}

/** @brief 短暂保护状态副本，期间不进行任何格式化或总线操作。 */
board_motor_status_t
board_motor_status(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    board_motor_status_t result = g_motor_status;
    __set_PRIMASK(mask);
    if (!result.running) { memset(result.phase_valid, 0, sizeof(result.phase_valid)); }
    return result;
}

/** @brief 保存前确认 ENABLE 低、MOE 低及两类转换均已停止。 */
bool
board_motor_storage_safe(void)
{
    return !g_motor_status.running && !(GPIOB->ODR & GPIO_PIN_9) && !(TIM1->BDTR & TIM_BDTR_MOE) &&
        !(ADC1->CR & (ADC_CR_JADSTART | ADC_CR_ADSTART)) &&
        !(ADC2->CR & (ADC_CR_JADSTART | ADC_CR_ADSTART));
}

/** @brief 恢复独立六步外设配置；不更改保存参数和原算法采样选择。 */
bool
board_motor_restore_profile(void)
{
    if (g_motor_status.running || (GPIOB->ODR & GPIO_PIN_9) || !adc_quiesce()) { return false; }
    MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_DUAL, 0U);
    TIM1->CR1 = TIM_CR1_CMS_0;
    TIM1->ARR = PWM_ARR;
    TIM1->CCR4 = SAMPLE_COMPARE;
    TIM1->CCMR1 = 0U;
    TIM1->CCMR2 = 6U << TIM_CCMR2_OC4M_Pos;
    TIM1->CCER = 0U;
    TIM1->RCR = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    ADC1->CFGR = ADC_CFGR_JQDIS;
    ADC2->CFGR = ADC_CFGR_JQDIS;
    sampling_configure();
    return true;
}
