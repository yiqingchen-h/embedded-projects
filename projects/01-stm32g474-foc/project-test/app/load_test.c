#include "load_test.h"
#include "board_test.h"
#include "drv8353f.h"
#include "test_logic.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 本诊断仅支持现有1mΩ、40倍、24.5Ω负载，不接电机。 */
#define LOAD_GAIN 40U
/* 冷却从输出关闭时起计时，RETEST不清除。 */
#define LOAD_COOLDOWN_MS 10000U
/* 基线及恢复阶段每毫秒一次，共256次。 */
#define LOAD_ZERO_COUNT 256U

typedef enum
{
    LOAD_OFF, LOAD_OFF_WAIT, LOAD_WAKE_WAIT, LOAD_CAL_WAIT, LOAD_CAL_SAMPLE,
    LOAD_ZERO_WAIT, LOAD_ZERO_SAMPLE, LOAD_READY, LOAD_BASELINE, LOAD_OUTPUT,
    LOAD_RECOVER_OFF, LOAD_RECOVER_WAKE, LOAD_RECOVER_WAIT, LOAD_RECOVER_SAMPLE,
    LOAD_DONE, LOAD_FAULT
} load_state_t;
/* 阶段名称同时用于状态查询与故障采样，保持与枚举顺序一致。 */
static const char *const g_load_state_names[] = {
    "OFF", "OFF_WAIT", "WAKE_WAIT", "CAL_WAIT", "CAL_SAMPLE", "ZERO_WAIT", "ZERO_SAMPLE",
    "READY", "BASELINE", "OUTPUT", "RECOVER_OFF", "RECOVER_WAKE", "RECOVER_WAIT", "RECOVER_SAMPLE",
    "DONE", "FAULT"
};
typedef struct
{
    /* 样本数量、和、平方和，保留小信号的分数码均值。 */
    uint32_t count, sum;
    uint64_t square_sum;
    /* 该组最小值和最大值。 */
    uint16_t minimum, maximum;
} load_stats_t;
/* 单一测试上下文，只由主循环访问。 */
static struct
{
    /* 状态、阶段时间、最近轮询及DRV检查时间。 */
    load_state_t state;
    uint32_t stage_at, last_tick, driver_at;
    /* 测试编号、最后关断时间与是否已发生过输出。 */
    uint32_t sequence, stopped_at;
    bool has_output, cooling;
    /* 模式互锁及延迟报告请求。 */
    bool mode, report_pending;
    /* 停止原因保留到下一次CHECK。 */
    const char *reason;
    /* 校准、正常零点、输出前、输出中和恢复后统计。 */
    load_stats_t calibration[3], zero[3], before, during, after;
    /* 三路正常零点，仅用于原始码到电流换算和保护。 */
    uint16_t offsets[3];
    /* 本次CHECK是否已完成正常通路零点，避免在校准失败时解释旧零点。 */
    bool offsets_valid;
    /* 输出期间的母线、A/B电压、参考电压累积与调度最大间隔。 */
    uint64_t bus_sum, a_sum, b_sum, vref_sum;
    uint32_t gap_max;
    /* 引起采样类故障的完整帧，属于关断前采集的数据而非关断后重读。 */
    board_adc_t fault_sample;
    /* 故障帧是否存在；无完整帧时不把历史数据当作触发值。 */
    bool fault_sample_valid;
    /* 故障前的阶段及距该阶段开始的毫秒数。 */
    load_state_t fault_stage;
    uint32_t fault_elapsed;
} g_load;

/** @brief 重置一组统计。 @param stats 待清空对象。 */
static void
stats_reset(load_stats_t *stats)
{
    memset(stats, 0, sizeof(*stats));
    stats->minimum = 4095U;
}

/** @brief 累积原始码。 @param stats 统计对象。 @param raw ADC码。 */
static void
stats_add(load_stats_t *stats, uint16_t raw)
{
    ++stats->count;
    stats->sum += raw;
    stats->square_sum += (uint64_t)raw * raw;
    if (raw < stats->minimum) { stats->minimum = raw; }
    if (raw > stats->maximum) { stats->maximum = raw; }
}

/** @brief 返回千分之一码均值，避免对约24码信号过早取整。 @param stats 统计对象。 @return 原始码乘1000。 */
static uint32_t
stats_mean(const load_stats_t *stats)
{
    return stats->count == 0U ? 0U : (uint32_t)((uint64_t)stats->sum * 1000U / stats->count);
}

/** @brief 查询冷却余量，使用无符号差值支持毫秒回绕。 @param now 毫秒。 @return 剩余毫秒。 */
static uint32_t
cooldown(uint32_t now)
{
    uint32_t elapsed = now - g_load.stopped_at;
    if (g_load.cooling && elapsed >= LOAD_COOLDOWN_MS) { g_load.cooling = false; }
    return g_load.cooling ? LOAD_COOLDOWN_MS - elapsed : 0U;
}

/** @brief 切换阶段。 @param state 新阶段。 @param now 毫秒。 */
static void
set_state(load_state_t state, uint32_t now)
{
    g_load.state = state;
    g_load.stage_at = now;
}

/** @brief 立即撤销输出并记下冷却起点；重复调用不延长冷却。 @param now 毫秒。 */
static void
disable_output(uint32_t now)
{
    board_driver_disable();
    if (g_load.has_output)
    {
        g_load.has_output = false;
        g_load.cooling = true;
        g_load.stopped_at = now;
    }
}

/** @brief 先关断再锁存失败，报告留给输出关闭后的主循环。 @param reason 原因。 @param now 毫秒。 */
static void
fail(const char *reason, uint32_t now)
{
    disable_output(now);
    board_trip();
    g_load.fault_sample_valid = false;
    g_load.reason = reason;
    set_state(LOAD_FAULT, now);
    g_load.report_pending = true;
}

/** @brief 先关断，再保存导致故障的现成ADC帧；不增加SPI或ADC读数。 @param reason 原因。 @param sample 已完整采集的帧。 @param now 当前毫秒。 */
static void
fail_sample(const char *reason, const board_adc_t *sample, uint32_t now)
{
    load_state_t stage = g_load.state;
    uint32_t elapsed = now - g_load.stage_at;
    fail(reason, now);
    g_load.fault_sample = *sample;
    g_load.fault_stage = stage;
    g_load.fault_elapsed = elapsed;
    g_load.fault_sample_valid = true;
}

/** @brief 读取一帧并核对供电及饱和，故障立即关断。 @param sample 输出帧。 @param now 毫秒。 @return 新鲜数据有效。 */
static bool
read_sample(board_adc_t *sample, uint32_t now)
{
    if (!board_adc_read(sample)) { fail("ADC", now); return false; }
    /* 采样跨过硬件截止时，CSA可能已随ENABLE进入休眠，该帧不能用于保护或统计。 */
    if (g_load.state == LOAD_OUTPUT && !board_load_running()) { return false; }
    uint32_t bus = test_voltage_mv(sample->raw[0], sample->vref_mv);
    if (bus < 10800U || bus > 13200U || sample->vref_mv < 3135U || sample->vref_mv > 3465U)
    {
        fail_sample("SUPPLY", sample, now); return false;
    }
    for (uint8_t i = 4U; i < 7U; ++i)
    {
        if (sample->raw[i] == 0U || sample->raw[i] >= 4095U)
        {
            fail_sample("SATURATION", sample, now); return false;
        }
    }
    if (board_tripped()) { fail("STOP_FAULT", now); return false; }
    return true;
}

/** @brief 校验CSA和正常零点范围及噪声。 @param stats 三组统计。 @return 三路均有效。 */
static bool
zero_valid(const load_stats_t stats[3])
{
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (stats[i].count != LOAD_ZERO_COUNT ||
            !test_csa_valid((uint16_t)(stats[i].sum / stats[i].count), stats[i].maximum - stats[i].minimum))
        {
            return false;
        }
    }
    return true;
}

/** @brief 运行时按实际40倍增益执行三路1.5A估算停止门限。 @param sample 最新帧。 @return 未超限。 */
static bool
current_valid(const board_adc_t *sample)
{
    if (!g_load.offsets_valid) { return false; }
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        int32_t delta = (int32_t)sample->raw[4U + i] - g_load.offsets[i];
        uint32_t magnitude = (uint32_t)(delta < 0 ? -delta : delta);
        /* 1.5A×1mΩ×40=60mV，先乘后比避免门限被整数取整放宽。 */
        if (magnitude * sample->vref_mv > 4095U * 60U) { return false; }
    }
    return true;
}

/** @brief 见 load_test.h。 */
void
load_test_init(void)
{
    memset(&g_load, 0, sizeof(g_load));
    g_load.reason = "NONE";
}

/** @brief 见 load_test.h。 */
bool
load_test_mode(void) { return g_load.mode; }

/** @brief 见 load_test.h。 */
bool
load_test_busy(void)
{
    return g_load.mode && g_load.state != LOAD_OFF && g_load.state != LOAD_READY &&
        g_load.state != LOAD_DONE && g_load.state != LOAD_FAULT;
}

/** @brief 见 load_test.h。 */
bool
load_test_check(bool allowed, uint32_t now)
{
    if (!allowed || load_test_busy() || board_tripped()) { return false; }
    g_load.mode = true;
    disable_output(now);
    g_load.offsets_valid = false;
    memset(g_load.offsets, 0, sizeof(g_load.offsets));
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        stats_reset(&g_load.calibration[i]);
        stats_reset(&g_load.zero[i]);
    }
    stats_reset(&g_load.before);
    stats_reset(&g_load.during);
    stats_reset(&g_load.after);
    g_load.bus_sum = g_load.a_sum = g_load.b_sum = g_load.vref_sum = 0U;
    g_load.gap_max = 0U;
    g_load.fault_sample_valid = false;
    g_load.reason = "PREPARING";
    g_load.last_tick = now;
    set_state(LOAD_OFF_WAIT, now);
    return true;
}

/** @brief 见 load_test.h。 */
bool
load_test_run(uint32_t now)
{
    if (!g_load.mode || g_load.state != LOAD_READY || board_tripped() || cooldown(now) != 0U)
    {
        return false;
    }
    ++g_load.sequence;
    stats_reset(&g_load.before);
    stats_reset(&g_load.during);
    stats_reset(&g_load.after);
    g_load.bus_sum = g_load.a_sum = g_load.b_sum = g_load.vref_sum = 0U;
    g_load.gap_max = 0U;
    g_load.reason = "MEASURING";
    g_load.fault_sample_valid = false;
    g_load.last_tick = g_load.driver_at = now;
    set_state(LOAD_BASELINE, now);
    return true;
}

/** @brief 见 load_test.h。 */
void
load_test_exit(void)
{
    board_driver_disable();
    g_load.mode = false;
    g_load.state = LOAD_OFF;
    g_load.report_pending = false;
}

/** @brief 见 load_test.h。 */
void
load_test_poll(uint32_t now)
{
    (void)cooldown(now);
    if (!g_load.mode) { return; }
    if ((load_test_busy() || g_load.state == LOAD_READY) && board_tripped()) { fail("STOP_FAULT", now); }
    if (!load_test_busy())
    {
        g_load.last_tick = now;
        if (g_load.report_pending) { load_test_report(); }
        return;
    }
    uint32_t gap = now - g_load.last_tick;
    if (gap == 0U) { return; }
    g_load.last_tick = now;
    uint32_t elapsed = now - g_load.stage_at;
    if (elapsed > 3000U) { fail("STAGE_TIMEOUT", now); return; }
    if (g_load.state == LOAD_OUTPUT)
    {
        if (gap > g_load.gap_max) { g_load.gap_max = gap; }
        if (!board_load_running())
        {
            if (board_tripped()) { fail("STOP_FAULT", now); return; }
            disable_output(now);
            if (elapsed < 1000U || g_load.during.count < 900U)
            {
                fail("EARLY_END_OR_SAMPLES", now); return;
            }
            set_state(LOAD_RECOVER_OFF, now);
            return;
        }
        if (elapsed > 1005U || gap > 5U) { fail("SAMPLE_TIMEOUT", now); return; }
    }
    if (g_load.state == LOAD_OFF_WAIT || g_load.state == LOAD_RECOVER_OFF)
    {
        if (elapsed >= 5U)
        {
            board_adc_t voltage;
            /* 唤醒前只验证ADC与电源，此时CSA可能休眠，不能检查电流码。 */
            if (!board_adc_read(&voltage) || voltage.vref_mv < 3135U || voltage.vref_mv > 3465U ||
                test_voltage_mv(voltage.raw[0], voltage.vref_mv) < 10800U ||
                test_voltage_mv(voltage.raw[0], voltage.vref_mv) > 13200U)
            {
                fail("SUPPLY_ADC", now); return;
            }
            bool recovery = g_load.state == LOAD_RECOVER_OFF;
            board_driver_wake();
            set_state(recovery ? LOAD_RECOVER_WAKE : LOAD_WAKE_WAIT, now);
        }
        return;
    }
    if (g_load.state == LOAD_WAKE_WAIT || g_load.state == LOAD_RECOVER_WAKE)
    {
        if (elapsed >= 5U)
        {
            bool recovery = g_load.state == LOAD_RECOVER_WAKE;
            if (!drv8353f_init_gain(board_drv_transfer, LOAD_GAIN) || !board_driver_arm() ||
                (!recovery && !drv8353f_calibrate(true)))
            {
                fail("DRV_CONFIG", now); return;
            }
            g_load.driver_at = now;
            set_state(recovery ? LOAD_RECOVER_WAIT : LOAD_CAL_WAIT, now);
        }
        return;
    }
    if ((uint32_t)(now - g_load.driver_at) >= 10U)
    {
        uint16_t f1, f2;
        g_load.driver_at = now;
        if (!drv8353f_faults(&f1, &f2) || f1 != 0U || f2 != 0U ||
            !drv8353f_verify(g_load.state != LOAD_OUTPUT))
        {
            fail("DRV_FAULT_CONFIG", now); return;
        }
    }
    if (g_load.state == LOAD_CAL_WAIT || g_load.state == LOAD_ZERO_WAIT || g_load.state == LOAD_RECOVER_WAIT)
    {
        if (elapsed >= 10U)
        {
            set_state(g_load.state == LOAD_CAL_WAIT ? LOAD_CAL_SAMPLE :
                (g_load.state == LOAD_ZERO_WAIT ? LOAD_ZERO_SAMPLE : LOAD_RECOVER_SAMPLE), now);
        }
        return;
    }
    board_adc_t sample;
    if (!read_sample(&sample, now)) { return; }
    if (g_load.state == LOAD_CAL_SAMPLE || g_load.state == LOAD_ZERO_SAMPLE)
    {
        bool calibration = g_load.state == LOAD_CAL_SAMPLE;
        load_stats_t *stats = calibration ? g_load.calibration : g_load.zero;
        for (uint8_t i = 0U; i < 3U; ++i) { stats_add(&stats[i], sample.raw[4U + i]); }
        if (stats[0].count == LOAD_ZERO_COUNT)
        {
            if (!zero_valid(stats)) { fail("ZERO_NOISE", now); return; }
            if (calibration)
            {
                if (!drv8353f_calibrate(false)) { fail("CAL_EXIT", now); return; }
                set_state(LOAD_ZERO_WAIT, now);
            }
            else
            {
                for (uint8_t i = 0U; i < 3U; ++i)
                {
                    g_load.offsets[i] = (uint16_t)(stats[i].sum / LOAD_ZERO_COUNT);
                }
                g_load.offsets_valid = true;
                set_state(LOAD_READY, now);
                g_load.reason = "READY";
                g_load.report_pending = true;
            }
        }
    }
    else if (g_load.state == LOAD_BASELINE)
    {
        if (!current_valid(&sample)) { fail_sample("BASELINE_CURRENT", &sample, now); return; }
        stats_add(&g_load.before, sample.raw[5]);
        if (g_load.before.count == LOAD_ZERO_COUNT)
        {
            if (!test_csa_valid((uint16_t)(g_load.before.sum / LOAD_ZERO_COUNT),
                g_load.before.maximum - g_load.before.minimum)) { fail("BASELINE_NOISE", now); return; }
            uint16_t f1, f2;
            if (!board_driver_arm() || !drv8353f_faults(&f1, &f2) || f1 != 0U || f2 != 0U ||
                !drv8353f_verify(true) || !drv8353f_coast(false))
            {
                fail("START_DRV", now); return;
            }
            /* 失败的启动也按可能曾输出计入冷却，不能立即重复尝试。 */
            g_load.has_output = true;
            if (!board_load_start()) { fail("START_TIMER", now); return; }
            set_state(LOAD_OUTPUT, now);
        }
    }
    else if (g_load.state == LOAD_OUTPUT)
    {
        if (!current_valid(&sample)) { fail_sample("CURRENT_LIMIT", &sample, now); return; }
        /* 帧采集可能跨过硬件结束点，结束后的帧不得混入导通统计。 */
        if (elapsed >= 10U && board_load_running())
        {
            stats_add(&g_load.during, sample.raw[5]);
            g_load.bus_sum += test_voltage_mv(sample.raw[0], sample.vref_mv);
            g_load.a_sum += test_voltage_mv(sample.raw[1], sample.vref_mv);
            g_load.b_sum += test_voltage_mv(sample.raw[2], sample.vref_mv);
            g_load.vref_sum += sample.vref_mv;
        }
    }
    else if (g_load.state == LOAD_RECOVER_SAMPLE)
    {
        stats_add(&g_load.after, sample.raw[5]);
        if (g_load.after.count == LOAD_ZERO_COUNT)
        {
            disable_output(now);
            g_load.reason = "MEASURE_NOT_PRECISION_PASS";
            set_state(LOAD_DONE, now);
            g_load.report_pending = true;
        }
    }
}

/** @brief 打印一组统计，标准差为总体标准差，单位千分之一码。 @param name 阶段。 @param stats 样本统计。 */
static void
print_stats(const char *name, const load_stats_t *stats)
{
    char line[160];
    uint32_t sd = 0U;
    if (stats->count != 0U)
    {
        double mean = (double)stats->sum / stats->count;
        double variance = (double)stats->square_sum / stats->count - mean * mean;
        if (variance > 0.0) { sd = (uint32_t)(sqrt(variance) * 1000.0); }
    }
    (void)snprintf(line, sizeof(line), "LOAD %s N=%lu MEAN_X1000=%lu MIN=%u MAX=%u SD_X1000=%lu\r\n",
        name, (unsigned long)stats->count, (unsigned long)stats_mean(stats),
        stats->count == 0U ? 0U : (unsigned)stats->minimum, (unsigned)stats->maximum, (unsigned long)sd);
    board_uart_write(line);
}

/** @brief 输出触发帧及三相门限关系，只在功率关闭后调用。 */
static void
print_fault_sample(void)
{
    if (!g_load.fault_sample_valid)
    {
        board_uart_write("LOAD TRIP SAMPLE_VALID=0\r\n");
        return;
    }
    const board_adc_t *sample = &g_load.fault_sample;
    int32_t delta[3], current[3];
    unsigned mask = 0U;
    char line[256];
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        delta[i] = g_load.offsets_valid ? (int32_t)sample->raw[4U + i] - g_load.offsets[i] : 0;
        uint32_t magnitude = (uint32_t)(delta[i] < 0 ? -delta[i] : delta[i]);
        if (magnitude * sample->vref_mv > 4095U * 60U) { mask |= 1U << i; }
        current[i] = (int32_t)((int64_t)delta[i] * sample->vref_mv * 1000 / (4095 * (int32_t)LOAD_GAIN));
    }
    (void)snprintf(line, sizeof(line), "LOAD TRIP SAMPLE_VALID=1 STAGE=%s ELAPSED_MS=%lu OVER_MASK=%u ZERO_VALID=%u LIMIT_MA=1500 VDDA_MV=%lu\r\n",
        g_load_state_names[g_load.fault_stage], (unsigned long)g_load.fault_elapsed, mask,
        g_load.offsets_valid ? 1U : 0U, (unsigned long)sample->vref_mv);
    board_uart_write(line);
    (void)snprintf(line, sizeof(line), "LOAD TRIP RAW=%u,%u,%u ZERO=%u,%u,%u DELTA=%ld,%ld,%ld EST_MA=%ld,%ld,%ld\r\n",
        (unsigned)sample->raw[4], (unsigned)sample->raw[5], (unsigned)sample->raw[6],
        (unsigned)g_load.offsets[0], (unsigned)g_load.offsets[1], (unsigned)g_load.offsets[2],
        (long)delta[0], (long)delta[1], (long)delta[2], (long)current[0], (long)current[1], (long)current[2]);
    board_uart_write(line);
    uint32_t a = test_voltage_mv(sample->raw[1], sample->vref_mv);
    uint32_t b = test_voltage_mv(sample->raw[2], sample->vref_mv);
    (void)snprintf(line, sizeof(line), "LOAD TRIP VRAW=%u,%u,%u,%u BUS_MV=%lu A_MV=%lu B_MV=%lu C_MV=%lu EXPECTED_AB_MA=%ld\r\n",
        (unsigned)sample->raw[0], (unsigned)sample->raw[1], (unsigned)sample->raw[2], (unsigned)sample->raw[3],
        (unsigned long)test_voltage_mv(sample->raw[0], sample->vref_mv), (unsigned long)a, (unsigned long)b,
        (unsigned long)test_voltage_mv(sample->raw[3], sample->vref_mv), (long)(((int32_t)a - (int32_t)b) * 1000 / 24500));
    board_uart_write(line);
}

/** @brief 见 load_test.h。 */
void
load_test_report(void)
{
    char line[256];
    if (load_test_busy())
    {
        (void)snprintf(line, sizeof(line), "LOAD STATE=%s ID=%lu COOL_MS=%lu\r\n",
            g_load_state_names[g_load.state], (unsigned long)g_load.sequence, (unsigned long)cooldown(g_load.last_tick));
        if (!board_uart_try_write(line)) { g_load.report_pending = true; }
        return;
    }
    g_load.report_pending = false;
    (void)snprintf(line, sizeof(line), "LOAD STATE=%s ID=%lu REASON=%s GAIN=40 RSENSE_UOHM=1000 LOAD_MOHM=24500 COOL_MS=%lu\r\n",
        g_load_state_names[g_load.state], (unsigned long)g_load.sequence, g_load.reason,
        (unsigned long)cooldown(g_load.last_tick));
    board_uart_write(line);
    if (g_load.state == LOAD_READY)
    {
        print_stats("CAL_A", &g_load.calibration[0]); print_stats("CAL_B", &g_load.calibration[1]);
        print_stats("CAL_C", &g_load.calibration[2]); print_stats("ZERO_A", &g_load.zero[0]);
        print_stats("ZERO_B", &g_load.zero[1]); print_stats("ZERO_C", &g_load.zero[2]);
    }
    else if (g_load.state == LOAD_DONE || g_load.state == LOAD_FAULT)
    {
        if (g_load.state == LOAD_FAULT) { print_fault_sample(); }
        print_stats("BEFORE", &g_load.before); print_stats("DURING", &g_load.during); print_stats("AFTER", &g_load.after);
        uint32_t n = g_load.during.count;
        uint32_t bus = n == 0U ? 0U : (uint32_t)(g_load.bus_sum / n);
        uint32_t a = n == 0U ? 0U : (uint32_t)(g_load.a_sum / n);
        uint32_t b = n == 0U ? 0U : (uint32_t)(g_load.b_sum / n);
        uint32_t vref = n == 0U ? 0U : (uint32_t)(g_load.vref_sum / n);
        int32_t delta = n == 0U ? 0 : (int32_t)stats_mean(&g_load.during) - (int32_t)stats_mean(&g_load.before);
        int32_t current = (int32_t)((int64_t)delta * vref / (4095 * (int32_t)LOAD_GAIN));
        int32_t expected = ((int32_t)a - (int32_t)b) * 1000 / 24500;
        int32_t drift = g_load.after.count == 0U ? 0 : (int32_t)stats_mean(&g_load.after) - (int32_t)stats_mean(&g_load.before);
        (void)snprintf(line, sizeof(line), "LOAD RESULT VALID=%u BUS_MV=%lu A_MV=%lu B_MV=%lu VDDA_MV=%lu DELTA_X1000=%ld B_MA=%ld EXPECTED_AB_MA=%ld RETURN_X1000=%ld GAP_MAX_MS=%lu\r\n",
            g_load.state == LOAD_DONE ? 1U : 0U, (unsigned long)bus, (unsigned long)a, (unsigned long)b,
            (unsigned long)vref, (long)delta, (long)current, (long)expected, (long)drift, (unsigned long)g_load.gap_max);
        board_uart_write(line);
    }
}
