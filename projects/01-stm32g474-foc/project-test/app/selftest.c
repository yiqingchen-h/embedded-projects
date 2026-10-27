#include "selftest.h"
#include "test_logic.h"
#include "load_test.h"
#include "board_i2c.h"
#include "drv8353f.h"
#include "mt6835.h"
#include "ssd1306.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

typedef enum
{
    RESULT_WAIT, RESULT_PASS, RESULT_FAIL, RESULT_MISSING,
    RESULT_WARN, RESULT_MANUAL, RESULT_BLOCKED
} result_t;
typedef struct
{
    /* 最新通道与实际参考电压；adc!=PASS时数据只供历史定位。 */
    board_adc_t sample;
    /* 最近编码器角度与状态。 */
    mt6835_sample_t encoder_sample;
    /* 各测试结果，PASS仅限对应的自检项目。 */
    result_t clock, adc, supply, driver, encoder, csa, pwm, power, sequence;
    /* 脉冲中六路相电压ADC及分别判定的结果。 */
    uint16_t gate_raw[6];
    result_t gate_result[6];
    /* 驱动最近一次可读取的故障和各路零点极差。 */
    uint16_t fault1, fault2, zero[3], noise[3];
    /* 母线电压mV、成功CRC帧及失败帧计数。 */
    uint32_t bus_mv, encoder_frames, encoder_errors;
    /* OLED事务状态，不代表肉眼验证已完成。 */
    bool display;
    /* 目前是否允许读取唤醒状态的驱动及CSA。 */
    bool awake;
    /* 启动时捕获的复位原因寄存器。 */
    uint32_t reset_flags;
} selftest_state_t;

/* 调试器可直接查看；写入只发生在主循环。 */
volatile selftest_state_t g_selftest;
/* HAL外设初始化结果，不能当作外部芯片在线证据。 */
static board_test_init_t g_initialized;
/* 页面、功率测试选项、绘图时间与命令行状态。 */
static uint8_t g_page, g_power_choice, g_pattern;
static uint32_t g_sample_time, g_draw_time, g_log_time, g_probe_time, g_pattern_end;
/* 页面保持静止，按键短按选测试，长按才允许功率输出。 */
static bool g_log_enabled = true, g_pattern_active, g_long_consumed;
/* 上一次输入快照的交互字段。 */
static uint32_t g_last_up, g_last_down, g_last_presses, g_last_held;
/* 行命令溢出或UART丢字节时整行丢弃，禁止执行截断命令。 */
static char g_command[64];
static uint8_t g_command_size;
static bool g_command_discard;
static uint32_t g_last_rx_errors;
/* 双向探测通过后才接受CRC帧，避免短路为零形成假健康。 */
static bool g_encoder_probed;
/* OLED最多21字符，文本与数值使用统一有界缓冲。 */
static char g_line[22];

static void run_sequence(void);

/** @brief 转换显示状态。 @param result 状态。 @return 静态名称。 */
static const char *
name(result_t result)
{
    static const char *const names[] = {"WAIT", "PASS", "FAIL", "MISSING", "WARN", "MEASURE", "BLOCKED"};
    return result <= RESULT_BLOCKED ? names[result] : "FAIL";
}

/** @brief 主循环输出一行日志。 @param format 格式。 @param ... 可变参数。 */
static void
log_line(const char *format, ...)
{
    char text[384];
    va_list args;
    va_start(args, format);
    (void)vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    board_uart_write(text);
}

/** @brief 刷新ADC结果，失败时禁止功率输出。 @return 本次有效为true。 */
static bool
sample_adc(void)
{
    board_adc_t sample;
    if (!g_initialized.adc || !board_adc_read(&sample))
    {
        g_selftest.adc = RESULT_FAIL;
        g_selftest.supply = RESULT_FAIL;
        if (g_selftest.awake) { board_trip(); }
        return false;
    }
    g_selftest.sample = sample;
    g_selftest.bus_mv = test_voltage_mv(sample.raw[0], sample.vref_mv);
    g_selftest.adc = RESULT_PASS;
    /* PG只监测降压器，VREFINT对3.3V支路做粗筛；5V仍需万用表。 */
    g_selftest.supply = (sample.vref_mv >= 3135U && sample.vref_mv <= 3465U &&
        HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) == GPIO_PIN_SET) ? RESULT_PASS : RESULT_WARN;
    return true;
}

/** @brief 在全部桥臂关断时采集16组零点与噪声。 @return 三路范围均合理为true。 */
static bool
measure_zero(void)
{
    uint32_t sums[3] = {0};
    uint16_t minimum[3] = {4095U,4095U,4095U};
    uint16_t maximum[3] = {0};
    for (uint8_t count = 0U; count < 16U; ++count)
    {
        if (!sample_adc() || board_tripped()) { return false; }
        for (uint8_t i = 0U; i < 3U; ++i)
        {
            uint16_t raw = g_selftest.sample.raw[4U + i];
            sums[i] += raw;
            if (raw < minimum[i]) { minimum[i] = raw; }
            if (raw > maximum[i]) { maximum[i] = raw; }
        }
        HAL_Delay(1U);
    }
    bool ok = true;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        g_selftest.zero[i] = (uint16_t)(sums[i] / 16U);
        g_selftest.noise[i] = maximum[i] - minimum[i];
        if (!test_csa_valid(g_selftest.zero[i], g_selftest.noise[i])) { ok = false; }
    }
    return ok;
}

/** @brief 核对SPI状态与保护配置，故障立即锁存停机。 @return 驱动健康为true。 */
static bool
check_driver(void)
{
    uint16_t f1 = 0U, f2 = 0U;
    bool ok = !board_tripped() && drv8353f_faults(&f1, &f2) && drv8353f_verify(true);
    if (ok) { g_selftest.fault1 = f1; g_selftest.fault2 = f2; }
    if (!ok || f1 != 0U || f2 != 0U)
    {
        board_trip();
        g_selftest.driver = RESULT_FAIL;
        g_selftest.awake = false;
        return false;
    }
    return true;
}

/** @brief 编码器通信与CRC检查；缺磁仅报告WARN。 @param probe 是否重新做双向寄存器验证。 */
static void
check_encoder(bool probe)
{
    mt6835_sample_t sample;
    if (probe) { g_encoder_probed = g_initialized.spi && mt6835_probe(board_enc_transfer); }
    bool ok = g_encoder_probed;
    if (ok) { ok = mt6835_read(&sample); }
    if (ok)
    {
        g_selftest.encoder_sample = sample;
        ++g_selftest.encoder_frames;
        g_selftest.encoder = sample.status == 0U ? RESULT_PASS : RESULT_WARN;
    }
    else
    {
        g_encoder_probed = false;
        ++g_selftest.encoder_errors;
        /* 仅从通信失败不能区分断线和损坏，因此提示检查连接。 */
        g_selftest.encoder = RESULT_MISSING;
    }
}

/** @brief 自动自检；禁止负载输出，只在ENABLE低时测试六路MCU脉冲。 */
static void
run_checks(void)
{
    board_trip();
    g_selftest.awake = false;
    HAL_Delay(5U);
    board_clear_trip();
    g_selftest.driver = RESULT_WAIT;
    g_selftest.csa = RESULT_WAIT;
    g_selftest.pwm = RESULT_WAIT;
    g_selftest.power = RESULT_WAIT;
    g_selftest.sequence = RESULT_WAIT;
    g_selftest.fault1 = 0U;
    g_selftest.fault2 = 0U;
    for (uint8_t i = 0U; i < 6U; ++i) { g_selftest.gate_result[i] = RESULT_WAIT; }
    g_selftest.clock = (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) &&
        __HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) && HAL_RCC_GetSysClockFreq() == 170000000U &&
        __HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) ? RESULT_PASS : RESULT_FAIL;
    check_encoder(true);
    if (!sample_adc()) { g_selftest.driver = RESULT_BLOCKED; return; }
    bool pwm = true;
    for (uint8_t i = 0U; i < 6U && pwm; ++i)
    {
        uint16_t unused;
        pwm = board_gate_pulse(i, &unused, false);
    }
    g_selftest.pwm = pwm ? RESULT_PASS : RESULT_FAIL;
    if (!pwm || g_selftest.clock != RESULT_PASS || g_selftest.supply != RESULT_PASS || !g_initialized.spi ||
        g_selftest.bus_mv < 10000U || g_selftest.bus_mv > 14000U ||
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) != GPIO_PIN_SET)
    {
        g_selftest.driver = RESULT_BLOCKED;
        return;
    }
    /* 六输入低，唤醒后留足手册1ms准备时间；只开放SPI和CSA。 */
    board_driver_wake();
    HAL_Delay(5U);
    if (board_tripped() || !drv8353f_init(board_drv_transfer))
    {
        g_selftest.driver = RESULT_MISSING;
        board_trip();
        return;
    }
    g_selftest.awake = true;
    if (!board_driver_arm() || !check_driver())
    {
        g_selftest.driver = RESULT_FAIL;
        g_selftest.awake = false;
        board_trip();
        return;
    }
    g_selftest.driver = RESULT_PASS;
    HAL_Delay(10U);
    g_selftest.csa = measure_zero() ? RESULT_PASS : RESULT_FAIL;
    if (g_selftest.csa != RESULT_PASS) { board_trip(); g_selftest.awake = false; }
}

/** @brief 显式手动触发的空载单管测试，任一异常即停止剩余步骤。 */
static void
run_power(void)
{
    if (load_test_mode()) { board_uart_write("POWER BLOCKED: LOAD MODE; DISCONNECT LOAD BEFORE RETEST\r\n"); return; }
    for (uint8_t i = 0U; i < 6U; ++i) { g_selftest.gate_result[i] = RESULT_WAIT; }
    g_selftest.sequence = RESULT_WAIT;
    if (!sample_adc() || !test_power_allowed(g_selftest.adc == RESULT_PASS && g_selftest.supply == RESULT_PASS,
        g_selftest.awake && g_selftest.driver == RESULT_PASS && g_selftest.pwm == RESULT_PASS,
        g_selftest.csa == RESULT_PASS, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET,
        board_tripped(), g_selftest.bus_mv) || !check_driver())
    {
        g_selftest.power = RESULT_BLOCKED;
        board_trip();
        g_selftest.awake = false;
        board_uart_write("POWER BLOCKED: CHECK ADC/PG/DRV/CSA/10-14V; RETEST REQUIRED\r\n");
        return;
    }
    g_selftest.csa = measure_zero() ? RESULT_PASS : RESULT_FAIL;
    if (g_selftest.csa != RESULT_PASS || board_tripped())
    {
        board_trip(); g_selftest.awake = false; g_selftest.power = RESULT_BLOCKED;
        board_uart_write("POWER BLOCKED: CSA ZERO INVALID\r\n");
        return;
    }
    /* 高阻相可被DRV微小漏电流抬至母线；记录预脉冲高电平，避免把后续ADC读数误判为高侧导通。 */
    bool phase_pre_high[3] = {false, false, false};
    bool any_pre_high = false;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        phase_pre_high[i] = (uint32_t)g_selftest.sample.raw[i + 1U] * 5U >= g_selftest.sample.raw[0];
        any_pre_high = any_pre_high || phase_pre_high[i];
    }
    if (any_pre_high)
    {
        log_line("POWER PREHIGH: A=%u B=%u C=%u; HIGH SIDE ADC REQUIRES VGS CHECK\r\n",
            g_selftest.sample.raw[1], g_selftest.sample.raw[2], g_selftest.sample.raw[3]);
    }
    uint16_t zero[3] = {g_selftest.zero[0],g_selftest.zero[1],g_selftest.zero[2]};
    if (!board_current_guard(zero,g_selftest.sample.vref_mv))
    {
        board_trip(); g_selftest.awake=false; g_selftest.power=RESULT_BLOCKED;
        return;
    }
    board_uart_write("POWER START: NO MOTOR/LOAD; SIX SINGLE 1MS PULSES\r\n");
    bool ok = true;
    for (uint8_t gate = 0U; gate < 6U && ok; ++gate)
    {
        uint16_t raw = 0U;
        ok = !board_tripped() && check_driver() && sample_adc() &&
             test_power_allowed(g_selftest.supply == RESULT_PASS, true, true, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET,
                                board_tripped(), g_selftest.bus_mv);
        if (ok) { ok = drv8353f_coast(false) && drv8353f_verify(false); }
        if (ok) { ok = board_gate_pulse(gate, &raw, true); }
        /* 无论成功与否优先使输出回高阻；通信失败则硬件ENABLE关断。 */
        bool coast_ok = !board_tripped() && drv8353f_coast(true);
        ok = ok && coast_ok;
        g_selftest.gate_raw[gate] = raw;
        if (ok && (gate & 1U) == 0U)
        {
            ok = test_high_side_valid(raw, g_selftest.sample.raw[0]);
            g_selftest.gate_result[gate] = ok ? (phase_pre_high[gate / 2U] ? RESULT_MANUAL : RESULT_PASS) : RESULT_FAIL;
        }
        else if (ok)
        {
            ok = (uint32_t)raw * 5U < g_selftest.sample.raw[0];
            /* 分压电阻本身能把悬空相拉低，零电压不能证明下管导通。 */
            g_selftest.gate_result[gate] = ok ? RESULT_MANUAL : RESULT_FAIL;
        }
        else { g_selftest.gate_result[gate] = RESULT_FAIL; }
        log_line("GATE %u: %s PHASE=%luMV RAW=%u\r\n", gate,
            name(g_selftest.gate_result[gate]), (unsigned long)test_voltage_mv(raw, g_selftest.sample.vref_mv), raw);
        if (!ok) { board_trip(); g_selftest.awake = false; }
        HAL_Delay(10U);
    }
    if (ok) { ok = check_driver(); }
    g_selftest.power = ok ? RESULT_MANUAL : RESULT_FAIL;
    if (!ok) { board_uart_write("POWER FAILED: DISABLED; RETEST REQUIRED\r\n"); }
    else if (any_pre_high) { board_uart_write("POWER: PULSES COMPLETE; PREHIGH PHASES AND LOW SIDES REQUIRE VGS MEASUREMENT\r\n"); }
    else { board_uart_write("POWER: HIGH SIDES PASS; LOW SIDES REQUIRE VGS MEASUREMENT\r\n"); }
}

/** @brief 在保守脉冲测试完成后执行有界空载六步波形，异常时锁存关断。 */
static void
run_sequence(void)
{
    if (load_test_mode()) { board_uart_write("6STEP BLOCKED: LOAD MODE; DISCONNECT LOAD BEFORE RETEST\r\n"); return; }
    if (g_selftest.power != RESULT_MANUAL)
    {
        g_selftest.sequence = RESULT_BLOCKED;
        board_uart_write("6STEP BLOCKED: RUN POWER NOLOAD FIRST; NO OUTPUT\r\n");
        return;
    }
    if (!sample_adc() || !test_power_allowed(g_selftest.adc == RESULT_PASS && g_selftest.supply == RESULT_PASS,
        g_selftest.awake && g_selftest.driver == RESULT_PASS && g_selftest.pwm == RESULT_PASS,
        g_selftest.csa == RESULT_PASS, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET,
        board_tripped(), g_selftest.bus_mv) || !check_driver())
    {
        g_selftest.sequence = RESULT_BLOCKED;
        g_selftest.power = RESULT_BLOCKED;
        board_trip();
        g_selftest.awake = false;
        board_uart_write("6STEP BLOCKED: CHECK ADC/PG/DRV/CSA/10-14V; RETEST REQUIRED\r\n");
        return;
    }
    g_selftest.csa = measure_zero() ? RESULT_PASS : RESULT_FAIL;
    uint16_t zero[3] = {g_selftest.zero[0], g_selftest.zero[1], g_selftest.zero[2]};
    if (g_selftest.csa != RESULT_PASS || board_tripped() ||
        !board_current_guard(zero, g_selftest.sample.vref_mv))
    {
        g_selftest.sequence = RESULT_BLOCKED;
        g_selftest.power = RESULT_BLOCKED;
        board_trip();
        g_selftest.awake = false;
        board_uart_write("6STEP BLOCKED: CSA ZERO/CURRENT GUARD INVALID\r\n");
        return;
    }
    board_uart_write("6STEP START: NO MOTOR/LOAD; 20 CYCLES, 1MS ON PER STEP, ALL OFF BETWEEN STEPS\r\n");
    bool ok = drv8353f_coast(false) && drv8353f_verify(false);
    uint32_t begin = HAL_GetTick();
    uint16_t completed = 0U;
    for (uint8_t cycle = 0U; cycle < 20U && ok; ++cycle)
    {
        for (uint8_t step = 0U; step < 6U && ok; ++step)
        {
            board_six_step_sample_t measured = {0};
            ok = !board_tripped() && (uint32_t)(HAL_GetTick() - begin) < 1000U && sample_adc() &&
                test_power_allowed(g_selftest.supply == RESULT_PASS, true, true,
                    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET, board_tripped(), g_selftest.bus_mv);
            if (ok) { ok = board_six_step_pulse(step, &measured); }
            if (ok)
            {
                /* 空载预高不能证明上管，但低相必须接近地；异常电压阻止继续切步。 */
                ok = test_high_side_valid(measured.high_raw, g_selftest.sample.raw[0]) &&
                    (uint32_t)measured.low_raw * 5U < g_selftest.sample.raw[0];
                if (!ok)
                {
                    log_line("6STEP PHASE FAIL STEP=%u HIGH=%u LOW=%u BUS=%u\r\n",
                        step, measured.high_raw, measured.low_raw, g_selftest.sample.raw[0]);
                }
            }
            if (ok) { ++completed; HAL_Delay(1U); }
        }
        if (ok)
        {
            uint16_t f1 = 0U, f2 = 0U;
            ok = !board_tripped() && drv8353f_faults(&f1, &f2) && f1 == 0U && f2 == 0U &&
                drv8353f_verify(false);
            g_selftest.fault1 = f1;
            g_selftest.fault2 = f2;
        }
    }
    if (ok && !board_tripped()) { ok = drv8353f_coast(true) && check_driver(); }
    if (!ok || board_tripped())
    {
        board_trip();
        g_selftest.awake = false;
        g_selftest.driver = RESULT_FAIL;
        g_selftest.power = RESULT_BLOCKED;
        g_selftest.sequence = RESULT_FAIL;
        log_line("6STEP STOPPED: %u/120 STEPS; ENABLE LOW; RETEST REQUIRED\r\n", completed);
        return;
    }
    g_selftest.sequence = RESULT_MANUAL;
    log_line("6STEP COMPLETE: %u STEPS IN %luMS; COAST RESTORED; MEASURE OUTPUT\r\n",
        completed, (unsigned long)(HAL_GetTick() - begin));
}

/** @brief 输出全部重要测量值和交互计数。 */
static void
report(void)
{
    board_inputs_t inputs;
    board_inputs_read(&inputs);
    log_line("STATUS CLK=%s ADC=%s SUPPLY=%s DRV=%s ENC=%s CSA=%s PWM=%s POWER=%s OLED=%s STOP=%u\r\n",
        name(g_selftest.clock), name(g_selftest.adc), name(g_selftest.supply), name(g_selftest.driver), name(g_selftest.encoder),
        name(g_selftest.csa), name(g_selftest.pwm), name(g_selftest.power), g_selftest.display ? "ACK" : "MISSING", board_tripped());
    log_line("ADC VREF=%luMV BUS=%luMV RAW=%u,%u,%u,%u,%u,%u,%u,%u BUS_FIRST_RAW=%u PG=%u NF=%u FAULT=%03X,%03X\r\n",
        (unsigned long)g_selftest.sample.vref_mv, (unsigned long)g_selftest.bus_mv,
        g_selftest.sample.raw[0],g_selftest.sample.raw[1],g_selftest.sample.raw[2],g_selftest.sample.raw[3],
        g_selftest.sample.raw[4],g_selftest.sample.raw[5],g_selftest.sample.raw[6],g_selftest.sample.raw[7],
        g_selftest.sample.bus_first_raw,
        HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2),HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12),g_selftest.fault1,g_selftest.fault2);
    log_line("ADC_DIAG PHASE_FIRST_RAW=%u,%u,%u PHASE_SQ1=%u,%u,%u PA_MODER=%02X PA_PUPDR=%02X\r\n",
        g_selftest.sample.phase_first_raw[0],g_selftest.sample.phase_first_raw[1],g_selftest.sample.phase_first_raw[2],
        g_selftest.sample.phase_sq1[0],g_selftest.sample.phase_sq1[1],g_selftest.sample.phase_sq1[2],
        g_selftest.sample.pa_moder,g_selftest.sample.pa_pupdr);
    log_line("ENC ANGLE=%lu STATUS=%u CRC_OK=%lu ERR=%lu ABZ=%lu,%lu,%lu BOOT=%lu RX=%lu RXERR=%lu\r\n",
        (unsigned long)g_selftest.encoder_sample.angle,g_selftest.encoder_sample.status,
        (unsigned long)g_selftest.encoder_frames,(unsigned long)g_selftest.encoder_errors,
        (unsigned long)inputs.encoder_edges[0],(unsigned long)inputs.encoder_edges[1],(unsigned long)inputs.encoder_edges[2],
        (unsigned long)inputs.boot_presses,
        (unsigned long)inputs.rx_bytes,(unsigned long)inputs.rx_errors);
    log_line("SWITCH UP=%lu DOWN=%lu KEY=%lu NAV=%ld RAW=%u,%u,%u ACTIVE=%u,%u,%u HOLD=%lu PAGE=%u\r\n",
        (unsigned long)inputs.up_presses,(unsigned long)inputs.down_presses,(unsigned long)inputs.presses,
        (long)inputs.nav_steps,inputs.switch_raw & 1U,(inputs.switch_raw >> 1U) & 1U,(inputs.switch_raw >> 2U) & 1U,
        inputs.switch_state & 1U,(inputs.switch_state >> 1U) & 1U,(inputs.switch_state >> 2U) & 1U,
        (unsigned long)inputs.held_ms,g_page);
    log_line("CSA ZERO=%u,%u,%u NOISE_PP=%u,%u,%u POWER_RAW=%u,%u,%u,%u,%u,%u\r\n",
        g_selftest.zero[0],g_selftest.zero[1],g_selftest.zero[2],g_selftest.noise[0],g_selftest.noise[1],g_selftest.noise[2],
        g_selftest.gate_raw[0],g_selftest.gate_raw[1],g_selftest.gate_raw[2],g_selftest.gate_raw[3],g_selftest.gate_raw[4],g_selftest.gate_raw[5]);
    log_line("6STEP=%s\r\n",name(g_selftest.sequence));
}

/** @brief 处理完整串口行，危险动作仅接受完整的明确命令。 @param command 大写行。 */
static void
execute(const char *command)
{
    /* 负载采样期间不执行其他阻塞业务，短响应使用TXE中断发送。 */
    if (load_test_busy() && strcmp(command,"STOP") != 0 && strcmp(command,"!") != 0)
    {
        if (strcmp(command,"LOAD STATUS") == 0 || strcmp(command,"STATUS") == 0) { load_test_report(); }
        else { (void)board_uart_try_write("LOAD BUSY; COMMAND REJECTED\r\n"); }
        return;
    }
    if (strcmp(command,"LOAD CHECK") == 0)
    {
        bool allowed = g_initialized.adc && g_initialized.spi && g_selftest.clock == RESULT_PASS &&
            g_selftest.pwm == RESULT_PASS && g_selftest.driver == RESULT_PASS;
        bool accepted = load_test_check(allowed, HAL_GetTick());
        if (accepted) { g_selftest.awake = false; }
        board_uart_write(accepted ? "LOAD CHECK ACCEPTED; WAIT READY\r\n" : "LOAD CHECK BLOCKED; RETEST REQUIRED\r\n");
        return;
    }
    if (strcmp(command,"LOAD RUN") == 0)
    {
        board_uart_write(load_test_run(HAL_GetTick()) ? "LOAD RUN ACCEPTED; AH+BL ONE SECOND\r\n" :
            "LOAD RUN BLOCKED; CHECK READY AND COOL_MS\r\n");
        return;
    }
    if (strcmp(command,"LOAD STATUS") == 0) { load_test_report(); return; }
    if (strcmp(command,"HELP") == 0)
    {
        board_uart_write("HELP VERSION STATUS PING RETEST STOP/! PAGE 0..2 OLED LOG ON LOG OFF TRIP WATCHDOG POWER NOLOAD POWER 6STEP NOLOAD\r\n");
        board_uart_write("LOAD CHECK / LOAD RUN / LOAD STATUS: 24.5OHM A-B, C OPEN, NO MOTOR\r\n");
    }
    else if (strcmp(command,"PING") == 0) { board_uart_write("PONG USART3 PB10/PB11 115200 8N1\r\n"); }
    else if (strcmp(command,"VERSION") == 0) { board_uart_write("PCB SELFTEST V1.8.1 LOAD_R24500_GAIN40\r\n"); }
    else if (strcmp(command,"STATUS") == 0) { if (load_test_mode()) { load_test_report(); } else { report(); } }
    else if (strcmp(command,"RETEST") == 0) { load_test_exit(); run_checks(); report(); }
    else if (strcmp(command,"STOP") == 0 || strcmp(command,"!") == 0)
    {
        board_trip(); g_selftest.awake = false; g_selftest.driver = RESULT_BLOCKED;
        g_selftest.power = RESULT_BLOCKED; g_selftest.sequence = RESULT_BLOCKED;
        board_uart_write("STOPPED; RETEST REQUIRED\r\n");
    }
    else if (strcmp(command,"POWER NOLOAD") == 0) { run_power(); }
    else if (strcmp(command,"POWER 6STEP NOLOAD") == 0) { run_sequence(); }
    else if (strcmp(command,"OLED") == 0)
    {
        g_pattern = 0U; g_pattern_active = true; g_pattern_end = HAL_GetTick() + 1500U;
        board_uart_write("OLED: BLACK THEN WHITE THEN CHECKER; VERIFY BY EYE\r\n");
    }
    else if (strcmp(command,"LOG ON") == 0) { g_log_enabled = true; }
    else if (strcmp(command,"LOG OFF") == 0) { g_log_enabled = false; }
    else if (strncmp(command,"PAGE ",5U) == 0 && command[5] >= '0' && command[5] <= '2' && command[6] == '\0')
    {
        g_page = (uint8_t)(command[5] - '0');
    }
    else if (strcmp(command,"TRIP") == 0)
    {
        board_trip(); g_selftest.awake = false; g_selftest.driver = RESULT_BLOCKED;
        g_selftest.power = RESULT_BLOCKED; g_selftest.sequence = RESULT_BLOCKED;
        log_line("SOFTWARE STOP ENABLE_READ=%u MOE=%u; NOT AN OCP TEST\r\n",
            HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9), (TIM1->BDTR & TIM_BDTR_MOE) != 0U);
    }
    else if (strcmp(command,"WATCHDOG") == 0)
    {
        board_trip(); board_uart_write("WATCHDOG RESET TEST; OUTPUT DISABLED\r\n");
        while (1) { __WFI(); }
    }
    else { board_uart_write("UNKNOWN COMMAND; HELP\r\n"); }
}

/** @brief 接收有限数量字节，不让串口洪泛阻塞采样。 */
static void
receive(void)
{
    char value;
    board_inputs_t inputs;
    board_inputs_read(&inputs);
    if (inputs.rx_errors != g_last_rx_errors)
    {
        g_last_rx_errors = inputs.rx_errors;
        g_command_discard = true;
        board_trip();
    }
    for (uint8_t count = 0U; count < 64U && board_uart_get(&value); ++count)
    {
        if (value == '\r' || value == '\n')
        {
            /* 解析期间也可能发生接收错误，执行动作前再检查一次。 */
            board_inputs_read(&inputs);
            if (inputs.rx_errors != g_last_rx_errors)
            {
                g_last_rx_errors = inputs.rx_errors;
                g_command_discard = true;
                board_trip();
            }
            if (g_command_discard) { if (!load_test_busy()) { board_uart_write("RX ERROR: LINE DISCARDED\r\n"); } }
            else if (g_command_size != 0U) { g_command[g_command_size] = '\0'; execute(g_command); }
            g_command_size = 0U; g_command_discard = false;
        }
        else if (value == '!')
        {
            board_trip();
            g_command_discard = true;
            g_command_size = 0U;
        }
        else if (value == '\b' || value == 127)
        {
            if (g_command_size != 0U) { --g_command_size; }
        }
        else if (value >= 32 && value <= 126)
        {
            if (value >= 'a' && value <= 'z') { value = (char)(value - 'a' + 'A'); }
            if (g_command_size < sizeof(g_command) - 1U) { g_command[g_command_size++] = value; }
            else { g_command_discard = true; }
        }
        else { g_command_discard = true; }
    }
}

/** @brief 格式化OLED一行。 @param row 行。 @param format 格式。 @param ... 参数。 */
static void
draw_line(uint8_t row, const char *format, ...)
{
    va_list args;
    va_start(args,format);
    (void)vsnprintf(g_line,sizeof(g_line),format,args);
    va_end(args);
    ssd1306_text(row,g_line);
}

/**
 * @brief 只绘制需要关注的自检状态，通过项不占屏幕行。
 * @param row 下一可用行，最多绘到第六行。
 * @param issues 非通过项目计数。
 * @param label 项目名称。
 * @param result 当前结果。
 */
static void
draw_issue(uint8_t *row, uint8_t *issues, const char *label, result_t result)
{
    if (result == RESULT_PASS) { return; }
    ++*issues;
    if (*row <= 6U) { draw_line((*row)++, "%s:%s", label, name(result)); }
}

/** @brief 绘制简洁状态、两个功率测试选项及按需查看的测量页。 */
static void
draw(void)
{
    if (!g_selftest.display) { g_selftest.display = ssd1306_init(board_i2c_write); }
    if (!g_selftest.display) { return; }
    if (g_pattern_active) { ssd1306_pattern(g_pattern); }
    else
    {
        board_inputs_t inputs;
        board_inputs_read(&inputs);
        ssd1306_clear();
        if (g_page == 0U)
        {
            uint8_t row = 2U, issues = 0U;
            ssd1306_text(0,"PCB SELFTEST");
            draw_line(1,"BUS:%luMV STOP:%u",(unsigned long)g_selftest.bus_mv,board_tripped());
            draw_issue(&row,&issues,"CLK",g_selftest.clock);
            draw_issue(&row,&issues,"ADC",g_selftest.adc);
            draw_issue(&row,&issues,"SUPPLY",g_selftest.supply);
            draw_issue(&row,&issues,"DRV",g_selftest.driver);
            draw_issue(&row,&issues,"CSA",g_selftest.csa);
            draw_issue(&row,&issues,"PWM",g_selftest.pwm);
            draw_issue(&row,&issues,"ENC",g_selftest.encoder);
            if (issues == 0U) { ssd1306_text(2,"NO CHECK ISSUES"); }
            else if (issues > 5U) { draw_line(6,"+%u MORE:UART",(unsigned)(issues-4U)); }
            ssd1306_text(7,"KEY:POWER UP:DETAIL");
        }
        else if (g_page == 1U)
        {
            ssd1306_text(0,"POWER TEST");
            draw_line(1,"BUS:%luMV STOP:%u",(unsigned long)g_selftest.bus_mv,board_tripped());
            draw_line(2,"SINGLE:%s",name(g_selftest.power));
            draw_line(3,"6STEP:%s",name(g_selftest.sequence));
            ssd1306_text(4,g_power_choice == 0U ? "> SINGLE PULSE" : "  SINGLE PULSE");
            ssd1306_text(5,g_power_choice == 1U ? "> 6STEP BURST" : "  6STEP BURST");
            ssd1306_text(6,"KEY:SELECT HOLD 3S");
            ssd1306_text(7,"NO MOTOR  !:STOP");
        }
        else
        {
            ssd1306_text(0,"LIVE MEASUREMENTS");
            draw_line(1,"VREF:%luMV",(unsigned long)g_selftest.sample.vref_mv);
            for (uint8_t i = 0U; i < 3U; ++i)
            {
                draw_line((uint8_t)(2U+i),"%c:%luMV",(int)('A'+i),
                    (unsigned long)test_voltage_mv(g_selftest.sample.raw[i+1U],g_selftest.sample.vref_mv));
            }
            draw_line(5,"FAULT:%03X %03X",g_selftest.fault1,g_selftest.fault2);
            draw_line(6,"RXERR:%lu",(unsigned long)inputs.rx_errors);
            ssd1306_text(7,"KEY:PIXEL TEST");
        }
    }
    g_selftest.display = ssd1306_refresh();
}

/** @brief 见 selftest.h。 */
void
selftest_init(board_test_init_t initialized)
{
    load_test_init();
    g_initialized = initialized;
    g_selftest.reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    HAL_Delay(100U);
    board_uart_write("\r\nPCB SELFTEST V1.8.1 - NO MOTOR - TYPE HELP\r\n");
    log_line("RESET=%08lX DEVID=%03lX HAL ADC=%u SPI=%u UART=%u\r\n",
        (unsigned long)g_selftest.reset_flags,(unsigned long)HAL_GetDEVID(),initialized.adc,initialized.spi,initialized.uart);
    run_checks();
    draw();
    report();
}

/** @brief 见 selftest.h。 */
void
selftest_poll(void)
{
    uint32_t now = HAL_GetTick();
    board_inputs_t inputs;
    board_inputs_read(&inputs);
    receive();
    now = HAL_GetTick();
    board_inputs_read(&inputs);
    if (load_test_mode())
    {
        /* 模式拥有ADC与DRV，原200ms采样、编码器、OLED及按键功率入口全部隔离。 */
        load_test_poll(now);
        g_last_up = inputs.up_presses;
        g_last_down = inputs.down_presses;
        g_last_presses = inputs.presses;
        g_last_held = inputs.held_ms;
        g_long_consumed = true;
        return;
    }
    if (board_tripped() && g_selftest.awake)
    {
        g_selftest.awake = false; g_selftest.driver = RESULT_FAIL;
        g_selftest.power = RESULT_BLOCKED; g_selftest.sequence = RESULT_BLOCKED;
        board_uart_write("FAULT/STOP: ENABLE LOW; LAST FAULT REGISTERS MAY PRECEDE EVENT\r\n");
    }
    if ((uint32_t)(now - g_sample_time) >= 200U)
    {
        g_sample_time = now;
        (void)sample_adc();
        check_encoder((uint32_t)(now - g_probe_time) >= 2000U);
        if ((uint32_t)(now - g_probe_time) >= 2000U) { g_probe_time = now; }
        if (g_selftest.awake)
        {
            if (g_selftest.bus_mv < 10000U || g_selftest.bus_mv > 14000U || g_selftest.supply != RESULT_PASS) { board_trip(); }
            (void)check_driver();
        }
    }
    if (inputs.up_presses != g_last_up || inputs.down_presses != g_last_down)
    {
        /* 用动作次数处理两次轮询之间的多次拨动；按住确认键时丢弃翻页动作。 */
        uint32_t up = inputs.up_presses - g_last_up, down = inputs.down_presses - g_last_down;
        if (inputs.held_ms == 0U)
        {
            g_page = (uint8_t)((g_page + 3U - up % 3U + down % 3U) % 3U);
        }
        g_last_up = inputs.up_presses; g_last_down = inputs.down_presses;
    }
    /* 短按在松开时处理，避免长按功率键先跳走页面。 */
    bool new_press = inputs.presses != g_last_presses;
    if (new_press) { g_last_presses = inputs.presses; g_long_consumed = false; }
    if (inputs.held_ms >= 3000U && !g_long_consumed && g_page == 1U && g_selftest.display && !g_pattern_active)
    {
        g_long_consumed = true;
        if (g_power_choice == 0U) { run_power(); }
        else { run_sequence(); }
    }
    if (inputs.held_ms == 0U && (g_last_held != 0U || new_press) && !g_long_consumed)
    {
        if (g_page == 1U) { g_power_choice ^= 1U; }
        else if (g_page == 2U) { execute("OLED"); }
        else { g_page = 1U; }
    }
    g_last_held = inputs.held_ms;
    if (g_pattern_active && (int32_t)(now - g_pattern_end) >= 0)
    {
        ++g_pattern; g_pattern_end = now + 1500U;
        if (g_pattern > 2U) { g_pattern_active = false; }
    }
    if ((uint32_t)(now - g_draw_time) >= (g_selftest.display ? 500U : 1500U))
    {
        g_draw_time = now; draw();
    }
    if (g_log_enabled && (uint32_t)(now - g_log_time) >= 2000U)
    {
        g_log_time = now; report();
    }
}
