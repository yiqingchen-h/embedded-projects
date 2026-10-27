#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "board_test.h"

/* 寄存器替身只用于主机测试；位定义对应RM0440，不参与固件构建。 */
typedef struct { uint32_t IDR, ODR, MODER, AFR[2], OTYPER, BSRR; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct
{
    uint32_t BDTR, CR1, CR2, CCER, PSC, ARR, CCR1, CCR2, CCR3;
    uint32_t CCMR1, CCMR2, RCR, EGR, SR, CNT, AF1;
} TIM_TypeDef;
typedef struct { uint32_t KR, PR, RLR, SR; } IWDG_TypeDef;
/* 模拟GPIO、定时器和独立看门狗。 */
static GPIO_TypeDef gpio_a, gpio_b, gpio_c;
static TIM_TypeDef tim1;
static IWDG_TypeDef iwdg;
#define GPIOA (&gpio_a)
#define GPIOB (&gpio_b)
#define GPIOC (&gpio_c)
#define TIM1 (&tim1)
#define IWDG (&iwdg)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_14 (1U << 14)
#define GPIO_PIN_15 (1U << 15)
#define TIM_CR1_CEN 1U
#define TIM_CR1_OPM (1U << 3)
#define TIM_CR1_ARPE (1U << 7)
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC1NE 4U
#define TIM_CCER_CC1NP 8U
#define TIM_EGR_UG 1U
#define TIM_SR_BIF (1U << 7)
#define TIM1_AF1_BKINE 1U
#define TIM_BDTR_OSSI (1U << 10)
#define TIM_BDTR_OSSR (1U << 11)
#define TIM_BDTR_BKE (1U << 12)
#define TIM_BDTR_MOE (1U << 15)
#define GPIO_MODE_AF_PP 2U
#define GPIO_SPEED_FREQ_HIGH 2U
#define GPIO_AF4_TIM1 4U
#define GPIO_AF6_TIM1 6U
#define LSI_VALUE 32000U
#define LSI_STARTUP_TIME 130U
/* 与被测板卡函数同名的内部状态。 */
static bool g_trip, g_armed, g_guard_ready, g_load_active;
/* 中断屏蔽模型，仅验证恢复原屏蔽状态。 */
static uint32_t primask;
/** @brief 读取模拟PRIMASK。 @return 当前屏蔽位。 */
static uint32_t __get_PRIMASK(void) { return primask; }
/** @brief 屏蔽模拟中断。 */
static void __disable_irq(void) { primask=1U; }
/** @brief 恢复模拟中断。 @param value 屏蔽位。 */
static void __set_PRIMASK(uint32_t value) { primask=value; }
/* 与实际输入采样函数共享名称，注入毫秒级触点序列。 */
static bool g_tick_ready, g_key_down, g_boot_down;
static uint8_t g_nav_candidate, g_nav_filter, g_nav_state, g_key_filter, g_boot_filter;
static volatile board_inputs_t g_inputs;
static uint16_t g_guard_zero[3];
static uint32_t g_guard_vref, SystemCoreClock = 170000000U;
static const uint32_t g_adc_channels[8] = {1, 2, 3, 4, 6, 7, 8, 18};
/* 故障注入、波形计数与所选复用引脚。 */
static bool adc_failure, overcurrent, irq_trip, hardware_break, timer_stuck, delay_failure;
static bool watchdog_mode, watchdog_stuck;
static unsigned checks, high_us, pair_high_us, adc_calls, watchdog_waits, af;
static uint32_t adc_delay_us = 8U;
/* 分频后的计数步长，长脉冲不再按1MHz误建模。 */
static uint32_t prescaler_us;
/* 独立时钟域可能超过10ms才同步完成。 */
static unsigned watchdog_ready_after;
static GPIO_TypeDef *selected_port;
static uint32_t selected_pin;
static uint32_t configured_high_pin, configured_low_pin, active_ccer;
/* 模拟控制板引脚短路及统计GPIO诊断，仅禁用功率时允许。 */
static bool gpio_stuck_low;
static unsigned gpio_high_writes;
/* 捕获实际故障诊断文本。 */
static char diagnostic[1024];

/** @brief 保存日志替身。 @param text 板卡诊断。 */
void
board_uart_write(const char *text)
{
    size_t used = strlen(diagnostic);
    (void)snprintf(diagnostic + used, sizeof(diagnostic) - used, "%s", text);
}

/** @brief 模拟GPIO输出和对地短路。 @param port 端口。 @param pin 引脚。 @param state 电平。 */
static void
HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET)
    {
        ++gpio_high_writes;
        port->ODR |= pin;
        if (!gpio_stuck_low) { port->IDR |= pin; }
    }
    else { port->ODR &= ~(uint32_t)pin; port->IDR &= ~(uint32_t)pin; }
}

/** @brief 断言并累计检查数。 @param ok 条件。 @param line 调用行号。 */
static void
check(bool ok, unsigned line)
{
    ++checks;
    if (!ok) { fprintf(stderr, "board check failed at line %u\n", line); exit(1); }
}
#define CHECK(value) check((value), __LINE__)

/** @brief 模拟中断紧急关断，与真实输出波形模型独立。 */
void
board_trip(void)
{
    g_trip = true;
    g_armed = false;
    g_guard_ready = false;
    g_load_active = false;
    GPIOB->IDR &= ~GPIO_PIN_9;
    GPIOB->ODR &= ~GPIO_PIN_9;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

/** @brief 模拟恢复六路GPIO低电平。 */
static void
gates_low(void)
{
    GPIOA->IDR = 0U;
    GPIOB->IDR &= GPIO_PIN_9 | GPIO_PIN_2 | GPIO_PIN_12;
    GPIOA->MODER = (1U << 16U) | (1U << 18U) | (1U << 20U);
    GPIOB->MODER = (1U << 26U) | (1U << 28U) | (1U << 30U);
    selected_port = NULL;
    configured_high_pin = configured_low_pin = 0U;
}

/** @brief 记录被测函数实际选择的引脚。 @param port 端口。 @param init GPIO配置。 */
static void
HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *init)
{
    selected_port = port;
    selected_pin = init->Pin;
    af = init->Alternate;
    if (port == GPIOA) { configured_high_pin |= init->Pin; }
    if (port == GPIOB) { configured_low_pin |= init->Pin; }
    for (uint32_t i = 0U; i < 16U; ++i)
    {
        if ((init->Pin & (1U << i)) != 0U)
        {
            port->MODER = (port->MODER & ~(3U << (i * 2U))) | (init->Mode << (i * 2U));
            port->AFR[i / 8U] = (port->AFR[i / 8U] & ~(15U << ((i % 8U) * 4U))) | (af << ((i % 8U) * 4U));
        }
    }
}

/** @brief 按RM0440 29.3.17及表282计算所选引脚，而非假定N通道始终反相。 */
static void
update_output(void)
{
    if (selected_port == NULL) { return; }
    for (uint32_t channel = 0U; channel < 3U; ++channel)
    {
        uint32_t enable = (TIM1->CCER >> (channel * 4U)) & 15U;
        uint32_t compare = channel == 0U ? TIM1->CCR1 : channel == 1U ? TIM1->CCR2 : TIM1->CCR3;
        uint32_t mode = channel == 2U ? (TIM1->CCMR2 >> 4U) & 7U : (TIM1->CCMR1 >> (channel * 8U + 4U)) & 7U;
        bool reference = mode == 7U && TIM1->CNT >= compare;
        uint32_t high_pin = GPIO_PIN_8 << channel;
        uint32_t low_pin = GPIO_PIN_13 << channel;
        if ((configured_high_pin & high_pin) != 0U)
        {
            bool output = reference != ((enable & 2U) != 0U);
            if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U || (enable & 1U) == 0U) { output = false; }
            GPIOA->IDR = (GPIOA->IDR & ~high_pin) | (output ? high_pin : 0U);
        }
        if ((configured_low_pin & low_pin) != 0U)
        {
            bool output = ((enable & 1U) != 0U ? !reference : reference) != ((enable & 8U) != 0U);
            if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U || (enable & 4U) == 0U) { output = false; }
            GPIOB->IDR = (GPIOB->IDR & ~low_pin) | (output ? low_pin : 0U);
        }
    }
}

/** @brief 推进定时器并注入刹车/延时故障；IWDG同步用独立分支。 @param us 微秒。 @return 延时模型成功。 */
static bool
delay_us(uint32_t us)
{
    if (delay_failure) { return false; }
    if (watchdog_mode)
    {
        ++watchdog_waits;
        if (!watchdog_stuck && watchdog_waits == watchdog_ready_after) { IWDG->SR = 0U; }
        return true;
    }
    if (us == 450U) { active_ccer = TIM1->CCER; }
    if (us == 450U && irq_trip) { board_trip(); }
    if (us == 450U && hardware_break && (TIM1->BDTR & TIM_BDTR_BKE) != 0U)
    {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
    for (uint32_t i = 0U; i < us; ++i)
    {
        update_output();
        if (selected_port != NULL && (selected_port->IDR & selected_pin) != 0U) { ++high_us; }
        if (configured_high_pin != 0U && configured_low_pin != 0U &&
            (GPIOA->IDR & configured_high_pin) != 0U && (GPIOB->IDR & configured_low_pin) != 0U)
        {
            ++pair_high_us;
        }
        if ((TIM1->CR1 & TIM_CR1_CEN) != 0U && !timer_stuck)
        {
            if (++prescaler_us >= (TIM1->PSC + 1U) / (SystemCoreClock / 1000000U))
            {
                prescaler_us=0U;
                if (++TIM1->CNT > TIM1->ARR)
                {
                    TIM1->CNT = 0U;
                    if ((TIM1->CR1 & TIM_CR1_OPM) != 0U) { TIM1->CR1 &= ~TIM_CR1_CEN; }
                }
            }
        }
    }
    update_output();
    return true;
}

/** @brief 模拟相电压和静态电流采样。 @param channel ADC通道。 @param value 结果。 @return 成功或注入的失败。 */
static bool
adc_one(uint32_t channel, uint16_t *value)
{
    ++adc_calls;
    (void)delay_us(adc_delay_us);
    *value = channel < 6U ? 1234U : (uint16_t)(2048U + (overcurrent ? 60U : 0U));
    return !adc_failure;
}

/* run.ps1从实际源码提取完整函数，避免复制实现导致测试与固件脱节。 */
#include "board_under_test.inc"

/** @brief 清理替身并设置合法脉冲前置条件。 @param gate 所选桥臂。 @param enabled 是否唤醒驱动。 */
static void
reset_test(uint8_t gate, bool enabled)
{
    (void)gate;
    memset(&tim1, 0, sizeof(tim1));
    memset(&iwdg, 0, sizeof(iwdg));
    memset(&gpio_a, 0, sizeof(gpio_a));
    memset(&gpio_b, 0, sizeof(gpio_b));
    selected_port = NULL;
    configured_high_pin = configured_low_pin = active_ccer = 0U;
    g_trip = false;
    g_load_active = false;
    primask=0U; prescaler_us=0U;
    g_armed = enabled;
    g_guard_ready = true;
    g_guard_vref = 3300U;
    for (unsigned i = 0U; i < 3U; ++i) { g_guard_zero[i] = 2048U; }
    if (enabled) { GPIOB->IDR = GPIO_PIN_9; GPIOB->ODR = GPIO_PIN_9; }
    adc_failure = overcurrent = irq_trip = hardware_break = timer_stuck = delay_failure = false;
    watchdog_mode = watchdog_stuck = false;
    high_us = pair_high_us = adc_calls = watchdog_waits = af = 0U;
    adc_delay_us = 8U;
    watchdog_ready_after = 2U;
    diagnostic[0] = '\0';
    gpio_stuck_low = false;
    gpio_high_writes = 0U;
}

/** @brief 运行实际板卡函数的寄存器模型回归，不能代替示波器实测。 @return 全部通过为0。 */
int
main(void)
{
    g_tick_ready = true;
    GPIOC->IDR = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    /* 上拨前19ms无事件，稳定20ms计一次；持续保持不连发。 */
    GPIOC->IDR &= ~GPIO_PIN_9;
    for (unsigned i = 0U; i < 19U; ++i) { board_test_tick(); }
    CHECK(g_inputs.up_presses == 0U && g_inputs.switch_raw == 1U);
    board_test_tick();
    CHECK(g_inputs.up_presses == 1U && g_inputs.nav_steps == 1 && g_inputs.switch_state == 1U);
    for (unsigned i = 0U; i < 1000U; ++i) { board_test_tick(); }
    CHECK(g_inputs.up_presses == 1U);
    /* 短暂松动不重新计数，稳定回中不生成反向事件。 */
    GPIOC->IDR |= GPIO_PIN_9;
    for (unsigned i = 0U; i < 10U; ++i) { board_test_tick(); }
    GPIOC->IDR &= ~GPIO_PIN_9;
    for (unsigned i = 0U; i < 30U; ++i) { board_test_tick(); }
    CHECK(g_inputs.up_presses == 1U && g_inputs.down_presses == 0U);
    GPIOC->IDR |= GPIO_PIN_9;
    for (unsigned i = 0U; i < 20U; ++i) { board_test_tick(); }
    CHECK(g_inputs.switch_state == 0U && g_inputs.up_presses == 1U);
    GPIOC->IDR &= ~GPIO_PIN_8;
    for (unsigned i = 0U; i < 20U; ++i) { board_test_tick(); }
    CHECK(g_inputs.down_presses == 1U && g_inputs.nav_steps == 0 && g_inputs.switch_state == 2U);
    GPIOC->IDR &= ~GPIO_PIN_9;
    for (unsigned i = 0U; i < 20U; ++i) { board_test_tick(); }
    CHECK(g_inputs.switch_state == 3U && g_inputs.down_presses == 1U && g_inputs.up_presses == 1U);
    GPIOC->IDR = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    for (unsigned i = 0U; i < 20U; ++i) { board_test_tick(); }
    /* 按下触点独立计数，长按仍只产生一个按下事件，松开清除保持时间。 */
    GPIOC->IDR &= ~GPIO_PIN_7;
    for (unsigned i = 0U; i < 19U; ++i) { board_test_tick(); }
    CHECK(g_inputs.presses == 0U && g_inputs.switch_raw == 4U);
    board_test_tick();
    CHECK(g_inputs.presses == 1U && g_inputs.switch_state == 4U);
    for (unsigned i = 0U; i < 3100U; ++i) { board_test_tick(); }
    CHECK(g_inputs.presses == 1U && g_inputs.held_ms >= 3000U);
    GPIOC->IDR |= GPIO_PIN_7;
    for (unsigned i = 0U; i < 20U; ++i) { board_test_tick(); }
    CHECK(g_inputs.held_ms == 0U && g_inputs.switch_state == 0U);
    GPIOC->IDR &= ~GPIO_PIN_9;
    g_inputs.nav_steps = INT32_MAX;
    for (unsigned i = 0U; i < 20U; ++i) { board_test_tick(); }
    CHECK(g_inputs.up_presses == 2U && g_inputs.nav_steps == INT32_MAX);
    GPIOC->IDR = GPIO_PIN_7 | GPIO_PIN_9;
    g_inputs.nav_steps = INT32_MIN;
    for (unsigned i = 0U; i < 20U; ++i) { board_test_tick(); }
    CHECK(g_inputs.down_presses == 2U && g_inputs.nav_steps == INT32_MIN);
    uint16_t raw;
    for (uint8_t gate = 0U; gate < 6U; ++gate)
    {
        for (unsigned power = 0U; power < 2U; ++power)
        {
            reset_test(gate, power != 0U);
            CHECK(board_gate_pulse(gate, &raw, power != 0U));
            CHECK(high_us == 1000U && raw == 1234U && !g_trip);
            CHECK(af == (gate == 5U ? 4U : 6U) && TIM1->PSC == 169U);
            CHECK((TIM1->CR1 & TIM_CR1_CEN) == 0U && TIM1->CCER == 0U && (TIM1->BDTR & TIM_BDTR_MOE) == 0U);
            CHECK(adc_calls == (power != 0U ? 4U : 1U));
            CHECK(gpio_high_writes == (power != 0U ? 0U : 1U));
        }
    }
    static const uint32_t expected_ccer[6] = {0x41U, 0x401U, 0x410U, 0x14U, 0x104U, 0x140U};
    for (uint8_t step = 0U; step < 6U; ++step)
    {
        board_six_step_sample_t sample;
        reset_test(step, true);
        CHECK(board_six_step_pulse(step, &sample));
        CHECK(pair_high_us == 1000U && high_us == 1000U && !g_trip);
        CHECK(active_ccer == expected_ccer[step] && TIM1->CCER == 0U && (TIM1->BDTR & TIM_BDTR_MOE) == 0U);
        CHECK(adc_calls == 9U && sample.high_raw == 1234U && sample.low_raw == 1234U);
        CHECK(af == (step == 1U || step == 2U ? 4U : 6U));
    }
    board_six_step_sample_t step_sample;
    reset_test(0U, true); overcurrent = true;
    CHECK(!board_six_step_pulse(0U, &step_sample) && g_trip && pair_high_us > 0U);
    CHECK(strstr(diagnostic, "6STEP FAIL STEP=0 AT=CURRENT_LIMIT") != NULL);
    reset_test(1U, true); hardware_break = true;
    CHECK(!board_six_step_pulse(1U, &step_sample) && g_trip && pair_high_us == 0U);
    reset_test(2U, true); timer_stuck = true;
    CHECK(!board_six_step_pulse(2U, &step_sample) && g_trip);
    reset_test(0U, true); adc_delay_us = 100U;
    CHECK(!board_six_step_pulse(0U, &step_sample) && g_trip);
    CHECK(strstr(diagnostic, "6STEP FAIL STEP=0 AT=PHASE_ADC") != NULL);
    reset_test(3U, true); g_guard_ready = false;
    CHECK(!board_six_step_pulse(3U, &step_sample) && pair_high_us == 0U);
    reset_test(4U, true);
    CHECK(!board_six_step_pulse(6U, &step_sample) && !board_six_step_pulse(4U, NULL));
    reset_test(0U, true); overcurrent = true;
    CHECK(!board_gate_pulse(0U, &raw, true) && g_trip && adc_calls == 2U);
    CHECK(strstr(diagnostic, "GATE=0 STEP=CURRENT") != NULL);
    reset_test(1U, true); adc_failure = true;
    CHECK(!board_gate_pulse(1U, &raw, true) && g_trip);
    reset_test(2U, true); irq_trip = true;
    CHECK(!board_gate_pulse(2U, &raw, true) && g_trip && high_us == 0U);
    reset_test(3U, true); hardware_break = true;
    CHECK(!board_gate_pulse(3U, &raw, true) && g_trip && high_us == 0U);
    reset_test(4U, false); timer_stuck = true;
    CHECK(!board_gate_pulse(4U, &raw, false) && g_trip);
    reset_test(5U, true); delay_failure = true;
    CHECK(!board_gate_pulse(5U, &raw, true) && g_trip && adc_calls == 0U);
    CHECK(strstr(diagnostic, "GATE=5 STEP=DELAY_IDLE") != NULL);
    CHECK(strstr(diagnostic, "MODE=2 AF=4 TYPE=0 CCER=00000400") != NULL);
    reset_test(1U, false); gpio_stuck_low = true;
    CHECK(!board_gate_pulse(1U, &raw, false) && g_trip && adc_calls == 0U);
    CHECK(strstr(diagnostic, "GATE=1 STEP=GPIO_HIGH") != NULL);
    CHECK(strstr(diagnostic, "PWM CFG MODE=1") != NULL);
    reset_test(0U, true); g_guard_ready = false;
    CHECK(!board_gate_pulse(0U, &raw, true) && high_us == 0U);
    reset_test(0U, true);
    CHECK(!board_gate_pulse(0U, &raw, false) && high_us == 0U);
    reset_test(0U, false);
    CHECK(!board_gate_pulse(6U, &raw, false) && !board_gate_pulse(0U, NULL, false));
    reset_test(0U, false); watchdog_mode = true; IWDG->SR = 3U;
    CHECK(board_watchdog_init() && watchdog_waits == 2U && IWDG->KR == 0xAAAAU);
    CHECK(IWDG->PR == 4U && IWDG->RLR == 999U);
    reset_test(0U, false); watchdog_mode = true; IWDG->SR = 3U; watchdog_ready_after = 12U;
    CHECK(board_watchdog_init() && watchdog_waits == 12U && IWDG->KR == 0xAAAAU);
    reset_test(0U, false); watchdog_mode = true; watchdog_stuck = true; IWDG->SR = 3U;
    CHECK(!board_watchdog_init() && g_trip && watchdog_waits == 49U && IWDG->KR != 0xAAAAU);
    CHECK(strstr(diagnostic, "IWDG CONFIG TIMEOUT") != NULL);
    reset_test(0U, false); watchdog_mode = true; delay_failure = true; IWDG->SR = 3U;
    CHECK(!board_watchdog_init() && g_trip && IWDG->KR != 0xAAAAU);
    CHECK(strstr(diagnostic, "IWDG DELAY FAILED") != NULL);
    reset_test(0U, true); GPIOB->IDR |= GPIO_PIN_2 | GPIO_PIN_12;
    CHECK(board_load_start());
    CHECK(TIM1->CCER==0x41U && TIM1->PSC==16999U && TIM1->ARR==10000U);
    CHECK(TIM1->CCR1==1U && TIM1->CCR2==1U && (TIM1->BDTR & TIM_BDTR_BKE)!=0U);
    CHECK(primask==0U && board_load_running());
    CHECK(delay_us(1000100U));
    CHECK(pair_high_us==1000000U && !board_load_running());
    board_test_tick();
    CHECK(!g_load_active && !g_armed && (TIM1->BDTR & TIM_BDTR_MOE)==0U);
    CHECK(GPIOB->BSRR==((uint32_t)GPIO_PIN_9<<16U));
    reset_test(0U, true); GPIOB->IDR |= GPIO_PIN_2 | GPIO_PIN_12;
    CHECK(board_load_start()); TIM1->BDTR &= ~TIM_BDTR_MOE;
    CHECK(!board_load_running() && g_trip);
    reset_test(0U, true); GPIOB->IDR |= GPIO_PIN_2 | GPIO_PIN_12;
    g_trip=true; CHECK(!board_load_start() && TIM1->CCER==0U);
    reset_test(0U, true); GPIOB->IDR |= GPIO_PIN_2;
    CHECK(!board_load_start() && g_trip);
    reset_test(0U, true); GPIOB->IDR |= GPIO_PIN_2 | GPIO_PIN_12;
    CHECK(board_load_start()); TIM1->CR1 &= ~TIM_CR1_CEN; TIM1->SR |= TIM_SR_BIF;
    CHECK(!board_load_running() && g_trip);
    reset_test(0U, true); GPIOB->IDR |= GPIO_PIN_2 | GPIO_PIN_12;
    CHECK(board_load_start()); TIM1->CR1 &= ~TIM_CR1_CEN; GPIOB->IDR &= ~GPIO_PIN_2;
    board_test_tick(); CHECK(g_trip && !g_load_active);
    printf("Board timer/watchdog: %u checks passed\n", checks);
    return 0;
}
