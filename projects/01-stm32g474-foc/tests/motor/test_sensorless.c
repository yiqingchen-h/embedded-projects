#include "sensorless_six_step.h"
#include "motor_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 主机断言失败立即报告位置，不把模拟结果冒充实物标定。 */
#define CHECK(condition) do { if (!(condition)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); exit(1); } } while (0)
/* 本套测试使用实际控制周期和真实六步启动模块。 */
#define PERIOD_US (1000000U / MOTOR_CONTROL_HZ)

typedef struct {
    /* 算法和模拟硬件时间；origin 用于构造跨 uint32 回绕的同一转子波形。 */
    sensorless_six_step_t state;
    /* 异步入口测试单独持有TIM启动器，不能与后台共享可写状态。 */
    six_step_t startup;
    bool background;
    /* 精简中断模型的PI积分及低频执行次数，与ADC观测器分开。 */
    sensorless_six_step_pi_t pi;
    uint32_t pi_updates;
    uint32_t now, origin, sequence, committed_us, next_apply_us;
    /* 硬件只在未来峰顶采用候选，相序检查独立于算法内部计数。 */
    uint8_t sector, pending_sector;
    bool pending;
    /* 机械指令、独立转子电频率、采样相位偏移以及可注入故障。 */
    uint32_t target_rpm, rotor_mhz, commits, reservations;
    int32_t waveform_delay_us;
    /* 两种边沿可有不同RC延迟；COM偏差是注入的真实执行时间偏差。 */
    int32_t rising_wave_delay_us, falling_wave_delay_us, commit_delay_us;
    bool constant_wave, reverse_edge, wrong_float, invalid, duplicate, current_limited;
    /* 板卡每次换相首周期没有OFF有效窗口；默认模拟此真实行为。 */
    bool invalid_first_pwm, first_pwm, omit_commit, alternate_pwm;
} fixture_t;

/** @brief 创建已验证静态参数，电机为 3 极对，模拟空载转速 1200 RPM。
 * @param observe 仅观察模式。
 * @return 可传入算法启动的测试配置。 */
static sensorless_six_step_config_t
configuration(bool observe)
{
    sensorless_six_step_config_t config;
    sensorless_six_step_defaults(&config);
    config.startup = (six_step_config_t){60000U, 60000U, 20000U, 0U, MOTOR_CONTROL_HZ, 200U, 0U};
    config.pole_pairs = 3U;
    config.kp_milli = 20U;
    config.ki_milli = 20U;
    config.min_duty = 100U;
    /* 既有边界用例保持200千分比；独立带载用例另设250验证运行余量。 */
    config.max_duty = config.startup.duty;
    config.observe_only = observe;
    return config;
}

/** @brief 初始化无 HAL 的算法和未来峰顶模拟器。
 * @param f 模拟器。 @param config 静态配置。 @param origin 起始 us 时间。 */
static void
fixture_start(fixture_t *f, const sensorless_six_step_config_t *config, uint32_t origin)
{
    memset(f, 0, sizeof(*f));
    sensorless_six_step_start(&f->state, config);
    six_step_start(&f->startup, &config->startup);
    f->now = f->origin = f->committed_us = origin;
    f->target_rpm = config->startup.target_mhz * 60U / (1000U * config->pole_pairs);
    f->rotor_mhz = config->startup.target_mhz;
    f->invalid_first_pwm = true;
}

/** @brief 从独立于换相指令的匀速转子产生浮空相过零波形。
 * @param f 模拟器。
 * @return 以 32 码为过零门限的 ADC 值。 */
static uint16_t
sample_wave(const fixture_t *f)
{
    if (f->constant_wave)
    {
        return 48U;
    }
    uint32_t relative = f->now - f->origin;
    int64_t cycle = (int64_t)((uint64_t)relative * f->rotor_mhz / 1000000000ULL);
    int64_t event = ((cycle * 12LL + (int64_t)f->sector * 2LL + 1LL) * 500000000LL) /
        ((int64_t)f->rotor_mhz * 6LL);
    int64_t revolution = 1000000000LL / f->rotor_mhz;
    if (event - relative > revolution / 2)
    {
        event -= revolution;
    }
    else if ((int64_t)relative - event > revolution / 2)
    {
        event += revolution;
    }
    bool rising = (f->sector & 1U) != 0U;
    event += f->waveform_delay_us + (rising ? f->rising_wave_delay_us : f->falling_wave_delay_us);
    if (f->reverse_edge)
    {
        rising = !rising;
    }
    int64_t value = 32LL + (rising ? 1LL : -1LL) * ((int64_t)relative - event);
    if (value < 0)
    {
        value = 0;
    }
    if (value > 64)
    {
        value = 64;
    }
    return (uint16_t)value;
}

/** @brief 执行一个真实控制周期，样本归属旧硬件扇区，候选只在下次峰顶采用。
 * @param f 模拟器。 */
static void
fixture_tick(fixture_t *f)
{
    f->now += PERIOD_US;
    f->first_pwm = false;
    uint32_t actual_apply_us = f->next_apply_us + (uint32_t)(f->commit_delay_us > 0 ? f->commit_delay_us : 0);
    if (f->pending && !f->omit_commit && (int32_t)(f->now - actual_apply_us) >= 0)
    {
        CHECK(f->pending_sector == (uint8_t)((f->sector + 1U) % 6U));
        f->sector = f->pending_sector;
        f->committed_us = f->next_apply_us + (uint32_t)f->commit_delay_us;
        f->pending = false;
        f->first_pwm = true;
        ++f->commits;
    }
    if (!f->duplicate)
    {
        ++f->sequence;
    }
    sensorless_six_step_input_t input = {
        .raw = sample_wave(f), .floating = (uint8_t)(2U - f->sector % 3U),
        .sector = f->sector, .valid = !f->invalid && !(f->invalid_first_pwm && f->first_pwm) &&
            !(f->alternate_pwm && (f->sequence&1U)), .sequence = f->sequence,
        .sample_us = f->now, .committed_us = f->committed_us,
        .next_commit_us = f->now + PERIOD_US / 2U, .now_us = f->now + 2U,
        .target_rpm = f->target_rpm, .current_limited = f->current_limited
    };
    if (f->wrong_float)
    {
        input.floating = (uint8_t)((input.floating + 1U) % 3U);
    }
    if (f->background)
    {
        if (!f->state.synchronizing && !f->state.locked) { six_step_tick(&f->startup); }
        else if (++f->startup.ticks >= f->startup.config.run_ticks) { f->startup.running=false; }
        if (f->state.config.irq_control)
        {
            sensorless_six_step_progress_irq(&f->state, f->startup.ticks, f->startup.frequency_mhz,
                f->startup.sector, f->startup.duty, f->startup.stage, input.now_us);
            sensorless_six_step_sample_irq(&f->state, &input);
        }
        else
        {
            sensorless_six_step_progress(&f->state, &f->startup, input.now_us);
            sensorless_six_step_sample(&f->state, &input);
        }
        uint8_t sector;
        uint32_t deadline;
        if (sensorless_six_step_schedule(&f->state, &sector, &deadline))
        {
            uint32_t peak=input.next_commit_us;
            if ((int32_t)(deadline-peak)>0) { peak+=((deadline-peak+PERIOD_US-1U)/PERIOD_US)*PERIOD_US; }
            CHECK((int32_t)(peak-deadline)>=0 && peak-deadline<PERIOD_US);
            CHECK(sector==(uint8_t)((f->sector+1U)%6U));
            sensorless_six_step_schedule_ack(&f->state,peak);
        }
        if (f->state.config.irq_control)
        {
            if (f->startup.ticks % (MOTOR_CONTROL_HZ/MOTOR_SPEED_LOOP_HZ) == 0U)
            {
                sensorless_six_step_check_time(&f->state,input.now_us);
                if (f->state.running && f->state.locked)
                {
                    f->state.duty=sensorless_six_step_pi_update(&f->pi,&f->state.config,
                        input.target_rpm,f->state.estimated_rpm,f->state.duty,input.current_limited);
                    ++f->pi_updates;
                }
            }
        }
        else { sensorless_six_step_service(&f->state,input.now_us,input.target_rpm,input.current_limited); }
    }
    else { sensorless_six_step_tick(&f->state, &input); }
    if (f->state.running && f->state.desired_sector != f->sector)
    {
        if (f->pending)
        {
            CHECK(f->pending_sector == f->state.desired_sector);
        }
        else
        {
            f->pending = true;
            f->pending_sector = f->state.desired_sector;
            f->next_apply_us = f->background && (f->state.locked || f->state.synchronizing) ?
                f->state.submitted_us : input.next_commit_us;
            if (f->state.locked && (int32_t)(f->state.next_commutation_us - input.now_us) > 0)
            {
                ++f->reservations;
            }
        }
    }
}

/** @brief 按真实时长运行模拟器，故障或总时限结束时提前返回。
 * @param f 模拟器。 @param duration_us 最大推进时间 us。 */
static void
run_for(fixture_t *f, uint32_t duration_us)
{
    for (uint32_t tick = 0U; tick < duration_us / PERIOD_US && f->state.running; ++tick)
    {
        fixture_tick(f);
    }
}

/** @brief 使用真实十二事件资格获取闭环初态，避免手工伪造观测器历史。
 * @param f 模拟器。 */
static void
run_to_lock(fixture_t *f)
{
    for (unsigned tick = 0U; tick < MOTOR_CONTROL_HZ / 4U && !f->state.locked && f->state.running; ++tick)
    {
        fixture_tick(f);
    }
    CHECK(f->state.locked);
    CHECK(f->state.consecutive_events >= 12U);
    CHECK(f->state.qualified && f->state.estimate_valid && f->state.fault == NULL);
}

/** @brief 注入同一扇区的原始元数据，独立检验坏样本不能拼接成过零。
 * @param state 已启动的测试状态。 @param sequence 样本序号。
 * @param sample_us 样本时间。 @param raw ADC 码。 @param valid 样本有效性。 */
static void
feed_raw(sensorless_six_step_t *state, uint32_t sequence, uint32_t sample_us, uint16_t raw, bool valid)
{
    sensorless_six_step_input_t input = {
        .raw = raw, .floating = 2U, .sector = 0U, .valid = valid, .sequence = sequence,
        .sample_us = sample_us, .committed_us = 0U, .next_commit_us = sample_us + PERIOD_US,
        .now_us = sample_us + 1U, .target_rpm = 1200U, .current_limited = false
    };
    sensorless_six_step_tick(state, &input);
}

/** @brief 默认值只允许观察，启动与截止逐周期等价于原六步模块。 */
static void
test_defaults_and_startup(void)
{
    sensorless_six_step_config_t defaults;
    sensorless_six_step_defaults(&defaults);
    CHECK(defaults.observe_only && defaults.blank_us == 150U && defaults.lock_count == 12U);
    CHECK(defaults.min_rpm == 1000U && defaults.kp_milli == 0U && defaults.ki_milli == 0U);
    CHECK(defaults.rising_delay_us == 0U && defaults.falling_delay_us == 0U);
    sensorless_six_step_config_t config = configuration(true);
    config.startup.align_ticks = MOTOR_CONTROL_HZ / 5U;
    config.startup.pullin_steps = 6U;
    config.startup.start_mhz = 50000U;
    config.startup.run_ticks = MOTOR_CONTROL_HZ / 2U;
    fixture_t f;
    fixture_start(&f, &config, 0U);
    f.invalid = true;
    six_step_t reference;
    six_step_start(&reference, &config.startup);
    for (uint32_t tick = 0U; tick < config.startup.run_ticks; ++tick)
    {
        fixture_tick(&f);
        six_step_tick(&reference);
        CHECK(f.state.desired_sector == reference.sector);
        CHECK(f.state.duty == reference.duty && f.state.ticks == reference.ticks);
        CHECK(f.state.open_loop.frequency_mhz == reference.frequency_mhz);
    }
    CHECK(!f.state.running && !f.state.locked && f.state.fault == NULL);
    CHECK(f.state.stage == SENSORLESS_SIX_STEP_STOPPED);
    sensorless_six_step_t stopped = f.state;
    fixture_tick(&f);
    CHECK(memcmp(&stopped, &f.state, sizeof(stopped)) == 0);
}

/** @brief 观察模式可形成真实资格，但不会改由过零换相；估速须来自整周事件。 */
static void
test_observe_qualification(void)
{
    sensorless_six_step_config_t config = configuration(true);
    fixture_t f;
    fixture_start(&f, &config, 0U);
    run_for(&f, 16000U);
    CHECK(f.state.event_count <= 6U && !f.state.estimate_valid && !f.state.qualified);
    run_for(&f, 84000U);
    CHECK(f.state.event_count >= 30U && f.state.qualified && f.state.estimate_valid);
    CHECK(f.state.invalid_count >= f.commits);
    CHECK(!f.state.locked && f.state.stage == SENSORLESS_SIX_STEP_QUALIFIED);
    CHECK(f.state.estimated_rpm >= 1195U && f.state.estimated_rpm <= 1205U);
    CHECK(f.state.blanked_count > 0U && f.state.fault == NULL);
    f.target_rpm = 1400U;
    fixture_tick(&f);
    CHECK(f.state.open_loop.config.target_mhz == 70000U);
    CHECK(f.state.target_rpm == 1400U);
    fixture_tick(&f);
    CHECK(f.state.target_rpm == 1400U && f.state.open_loop.config.target_mhz == 70000U);
}

/** @brief 错相、错边沿、消隐内事件及越过扇区的相位不能积累成十二个同步事件。 */
static void
test_rejection_paths(void)
{
    sensorless_six_step_config_t config = configuration(true);
    fixture_t f;
    fixture_start(&f, &config, 0U);
    f.wrong_float = true;
    run_for(&f, 100000U);
    CHECK(f.state.event_count == 0U && f.state.rejected_count > 0U && !f.state.qualified);
    fixture_start(&f, &config, 0U);
    f.reverse_edge = true;
    run_for(&f, 100000U);
    CHECK(f.state.event_count == 0U && !f.state.qualified);
    fixture_start(&f, &config, 0U);
    f.waveform_delay_us = 1800;
    run_for(&f, 100000U);
    CHECK(f.state.rejected_count > 0U && !f.state.qualified && !f.state.estimate_valid);
    fixture_start(&f, &config, 0U);
    f.waveform_delay_us = -1300;
    run_for(&f, 100000U);
    CHECK(f.state.event_count == 0U && !f.state.qualified);
    fixture_start(&f, &config, 0U);
    fixture_tick(&f);
    f.duplicate = true;
    run_for(&f, 100000U);
    CHECK(f.state.event_count == 0U && f.state.duplicate_count > 0U && !f.state.qualified);
}

/** @brief 已有资格遇到无效采样长间隙须撤销，但观察模式不触发算法停机。 */
static void
test_observe_gap(void)
{
    sensorless_six_step_config_t config = configuration(true);
    fixture_t f;
    fixture_start(&f, &config, 0U);
    run_for(&f, 100000U);
    CHECK(f.state.qualified);
    f.invalid = true;
    run_for(&f, 3000U);
    CHECK(!f.state.qualified && !f.state.estimate_valid && f.state.running && f.state.fault == NULL);
    f.invalid = false;
    run_for(&f, 60000U);
    CHECK(f.state.qualified && f.state.estimate_valid);
}

/** @brief 同时间戳的新序号和重复序号的无效窗口均不能被当作连续有效边沿。 */
static void
test_sample_metadata(void)
{
    sensorless_six_step_config_t config = configuration(true);
    sensorless_six_step_t state;
    sensorless_six_step_start(&state, &config);
    feed_raw(&state, 1U, 1300U, 64U, true);
    feed_raw(&state, 2U, 1300U, 0U, true);
    CHECK(state.event_count == 0U && state.rejected_count == 1U);
    feed_raw(&state, 3U, 1400U, 0U, true);
    CHECK(state.event_count == 0U);
    sensorless_six_step_start(&state, &config);
    feed_raw(&state, 1U, 1300U, 64U, true);
    feed_raw(&state, 1U, 1350U, 64U, false);
    feed_raw(&state, 2U, 1400U, 0U, true);
    CHECK(state.event_count == 0U && state.invalid_count == 1U && state.duplicate_count == 1U);
    sensorless_six_step_start(&state, &config);
    feed_raw(&state, 1U, 1300U, 64U, true);
    feed_raw(&state, 1U, 1350U, 0U, true);
    feed_raw(&state, 2U, 1400U, 0U, true);
    CHECK(state.event_count == 0U);
    feed_raw(&state, 3U, 1450U, 0U, true);
    CHECK(state.event_count == 1U && state.duplicate_count == 1U);
}

/** @brief 单样本尖峰和中间落回回差区均不能生成事件；连续两个样本确认时保留首次过阈时刻。 */
static void
test_continuous_sample_confirmation(void)
{
    sensorless_six_step_config_t config = configuration(true);
    sensorless_six_step_t state;
    sensorless_six_step_start(&state, &config);
    feed_raw(&state, 1U, 1100U, 64U, true);
    feed_raw(&state, 2U, 1150U, 0U, true);
    CHECK(state.event_count == 0U);
    feed_raw(&state, 3U, 1200U, 64U, true);
    feed_raw(&state, 4U, 1300U, 0U, true);
    feed_raw(&state, 5U, 1350U, 30U, true);
    CHECK(state.event_count == 0U);
    feed_raw(&state, 6U, 1400U, 0U, true);
    CHECK(state.event_count == 0U);
    feed_raw(&state, 7U, 1450U, 0U, true);
    CHECK(state.event_count == 1U && state.last_event_us == 1250U);
}

/** @brief 单个跳脉冲保留资格；有效反馈长间隙或缺失事件撤销，避免旧估速粘滞。 */
static void
test_quality_expires(void)
{
    sensorless_six_step_config_t config = configuration(true);
    fixture_t f;
    fixture_start(&f, &config, 0U);
    run_for(&f, 100000U);
    CHECK(f.state.qualified);
    while (f.pending || f.now - f.committed_us < f.state.blanking_us)
    {
        fixture_tick(&f);
    }
    f.invalid = true;
    fixture_tick(&f);
    CHECK(f.state.qualified && f.state.estimate_valid && f.state.running && f.state.fault == NULL);
    run_for(&f,3000U);
    CHECK(!f.state.qualified && !f.state.estimate_valid && f.state.running && f.state.fault == NULL);
    f.invalid = false;
    run_for(&f, 60000U);
    CHECK(f.state.qualified);
    f.constant_wave = true;
    run_for(&f, 10000U);
    CHECK(!f.state.qualified && !f.state.estimate_valid && f.state.running && f.state.fault == NULL);
}

/** @brief 开环过零偏离中点仍能同步，但首次预约COM实际提交前绝不报告LOCKED；含时间回绕。 */
static void
test_phase_offset_synchronization(void)
{
    for (int offset = -500; offset <= 1100; offset += 1600)
    {
        sensorless_six_step_config_t config = configuration(false);
        config.startup.start_mhz = config.startup.target_mhz = 50000U;
        fixture_t f;
        fixture_start(&f, &config, UINT32_MAX - 20000U);
        f.waveform_delay_us = offset;
        for (unsigned i = 0; i < MOTOR_CONTROL_HZ / 4U && !f.state.synchronizing; ++i)
        {
            fixture_tick(&f);
            CHECK(f.state.running && !f.state.locked);
        }
        CHECK(f.state.synchronizing && f.state.qualified && f.state.consecutive_events >= 12U);
        CHECK(f.state.stage == SENSORLESS_SIX_STEP_SYNCHRONIZE && f.state.estimated_rpm == 1000U);
        /* 开环积分量化与未来峰顶各有不足一周期的误差，仍保留所注入的机械相位偏移。 */
        CHECK(f.state.phase_error_us >= offset - 2*(int)PERIOD_US && f.state.phase_error_us <= offset + 2*(int)PERIOD_US);
        CHECK(f.state.next_commutation_us - f.state.last_event_us == f.state.sector_period_us / 2U);
        uint32_t startup_ticks = f.state.open_loop.ticks;
        uint8_t old_sector = f.sector;
        while (!f.pending)
        {
            fixture_tick(&f);
            CHECK(f.state.running && !f.state.locked && f.state.open_loop.ticks == startup_ticks);
        }
        CHECK(f.sector == old_sector && f.state.desired_sector == (uint8_t)((old_sector + 1U) % 6U));
        CHECK((int32_t)(f.next_apply_us - f.state.next_commutation_us) >= 0);
        CHECK(f.next_apply_us - f.state.next_commutation_us < PERIOD_US);
        fixture_tick(&f);
        CHECK(f.state.locked && !f.state.synchronizing && f.state.stage == SENSORLESS_SIX_STEP_CLOSED);
        CHECK(f.sector == f.state.desired_sector && f.committed_us == f.state.submitted_us);
        run_for(&f, 100000U);
        CHECK(f.state.running && f.state.locked && f.state.fault == NULL && f.state.estimated_rpm == 1000U);
    }
    sensorless_six_step_config_t config = configuration(false);
    fixture_t f;
    fixture_start(&f, &config, 0U);
    while (!f.state.synchronizing) { fixture_tick(&f); CHECK(f.state.running); }
    f.omit_commit = true;
    run_for(&f, 10000U);
    CHECK(!f.state.running && !f.state.locked && strcmp(f.state.fault, "BEMF COMMIT") == 0);
}

/** @brief 动态退磁窗口拒绝150us以后但20%扇区以内的振铃，正常中点边沿仍确认一次。 */
static void
test_adaptive_demagnetization(void)
{
    sensorless_six_step_config_t config = configuration(true);
    sensorless_six_step_t state;
    sensorless_six_step_start(&state, &config);
    feed_raw(&state, 1U, 200U, 64U, true);
    feed_raw(&state, 2U, 300U, 0U, true);
    feed_raw(&state, 3U, 400U, 0U, true);
    feed_raw(&state, 4U, 650U, 0U, true);
    CHECK(state.blanking_us == 555U && state.event_count == 0U && !state.qualified);
    feed_raw(&state, 5U, 1300U, 64U, true);
    feed_raw(&state, 6U, 1400U, 0U, true);
    feed_raw(&state, 7U, 1450U, 0U, true);
    CHECK(state.event_count == 1U && state.last_event_us == 1350U);
}

/** @brief 注入越界浮相、ADC、旧COM及未来/过期采样元数据；观察撤资格，已接管时按故障停止。 */
static void
test_invalid_contexts(void)
{
    for (unsigned closed = 0U; closed < 2U; ++closed)
    {
        for (unsigned error = 0U; error < 8U; ++error)
        {
            sensorless_six_step_config_t config = configuration(closed == 0U);
            fixture_t f;
            fixture_start(&f, &config, 0U);
            if (closed != 0U)
            {
                run_to_lock(&f);
            }
            else
            {
                run_for(&f, 100000U);
            }
            CHECK(f.state.qualified);
            sensorless_six_step_input_t input = {
                .raw = 48U, .floating = (uint8_t)(2U - f.sector % 3U), .sector = f.sector,
                .valid = true, .sequence = f.sequence + 1U, .sample_us = f.now + PERIOD_US,
                .committed_us = f.committed_us, .next_commit_us = f.now + 2U * PERIOD_US,
                .now_us = f.now + PERIOD_US + 1U, .target_rpm = f.target_rpm
            };
            switch (error)
            {
            case 0: input.sector = 6U; break;
            case 1: input.floating = (uint8_t)((input.floating + 1U) % 3U); break;
            case 2: input.sample_us = input.committed_us - 1U; break;
            case 3: input.sample_us = input.now_us + 1U; break;
            case 4: input.now_us = input.sample_us + 2U * PERIOD_US + 1U; break;
            case 5: input.sample_us = f.state.last_valid_sample_us; break;
            case 6: input.raw = 4096U; break;
            case 7: input.committed_us = f.state.observed_committed_us - 1U; break;
            default: CHECK(false); break;
            }
            sensorless_six_step_tick(&f.state, &input);
            CHECK(!f.state.qualified && !f.state.estimate_valid);
            if (closed != 0U)
            {
                CHECK(!f.state.running && f.state.fault != NULL && strcmp(f.state.fault, "BEMF SAMPLE") == 0);
            }
            else
            {
                CHECK(f.state.running && f.state.fault == NULL);
            }
        }
    }
}

/** @brief 5→50 Hz升频按20 Hz/s需要2.25 s，交接的500 ms计时不能从低频RAMP开始。 */
static void
test_acquisition_timeout_after_ramp(void)
{
    sensorless_six_step_config_t config = configuration(false);
    config.startup.start_mhz = 5000U;
    config.startup.target_mhz = 50000U;
    config.startup.align_ticks = MOTOR_CONTROL_HZ / 5U;
    config.startup.pullin_steps = 6U;
    config.startup.run_ticks = 4U * MOTOR_CONTROL_HZ;
    fixture_t f;
    fixture_start(&f, &config, UINT32_MAX - 1000000U);
    f.constant_wave = true;
    for (unsigned tick = 0U; tick < 3U * MOTOR_CONTROL_HZ && !f.state.acquisition_started; ++tick)
    {
        fixture_tick(&f);
        CHECK(f.state.running && f.state.fault == NULL);
        if (f.state.open_loop.frequency_mhz < 50000U)
        {
            CHECK(!f.state.acquisition_started);
        }
    }
    CHECK(f.state.acquisition_started && f.state.open_loop.frequency_mhz == 50000U);
    CHECK(f.state.acquisition_started_us - f.origin >= 2650000U);
    run_for(&f, 499000U);
    CHECK(f.state.running && f.state.fault == NULL);
    run_for(&f, 1000U);
    CHECK(!f.state.running && strcmp(f.state.fault, "BEMF STARTUP") == 0);
    CHECK(f.now + 2U - f.state.acquisition_started_us == config.timeout_us);
    config.startup.run_ticks = 2U * MOTOR_CONTROL_HZ;
    fixture_start(&f, &config, 0U);
    f.constant_wave = true;
    run_for(&f, 3000000U);
    CHECK(!f.state.running && f.state.fault == NULL && !f.state.acquisition_started);
    CHECK(f.state.ticks == config.startup.run_ticks);
    config.startup.start_mhz = 50000U;
    config.startup.target_mhz = 70000U;
    config.startup.align_ticks = 0U;
    config.startup.pullin_steps = 0U;
    config.handoff_rpm = 1200U;
    fixture_start(&f, &config, 0U);
    f.constant_wave = true;
    run_for(&f, 499000U);
    CHECK(f.state.running && !f.state.acquisition_started);
    run_for(&f, 1000U);
    CHECK(f.state.acquisition_started && f.state.acquisition_started_us >= 500000U);
}

/** @brief 非3倍极对数的1000RPM电频率向下取整后仍可启动timeout，并由独立转子波形正常确认接管。 */
static void
test_minimum_speed_frequency_quantization(void)
{
    for (uint32_t poles = 1U; poles <= 2U; ++poles)
    {
        for (unsigned explicit_handoff = 0U; explicit_handoff < 2U; ++explicit_handoff)
        {
            sensorless_six_step_config_t config = configuration(false);
            config.pole_pairs = poles;
            config.handoff_rpm = explicit_handoff != 0U ? 1000U : 0U;
            uint32_t frequency = 1000U * poles * 1000U / 60U;
            config.startup.start_mhz = config.startup.target_mhz = frequency;
            fixture_t f;
            fixture_start(&f, &config, 0U);
            f.target_rpm = 1000U;
            f.constant_wave = true;
            fixture_tick(&f);
            CHECK(f.state.acquisition_started && f.state.acquisition_frequency_mhz == frequency);
            CHECK(f.state.open_loop.frequency_mhz == frequency && f.state.fault == NULL);
            run_for(&f, 499000U);
            CHECK(f.state.running && f.state.fault == NULL);
            run_for(&f, 1000U);
            CHECK(!f.state.running && strcmp(f.state.fault, "BEMF STARTUP") == 0);
            fixture_start(&f, &config, UINT32_MAX - 20000U);
            f.target_rpm = 1000U;
            run_to_lock(&f);
            CHECK(f.state.estimated_rpm == 1000U && f.state.qualified && f.state.estimate_valid);
            run_for(&f, 100000U);
            CHECK(f.state.running && f.state.locked && f.state.estimated_rpm == 1000U && f.state.fault == NULL);
        }
    }
}

/** @brief 连续合格事件的估速跌破交接门槛时及时撤销旧资格，不能凭历史值接管。 */
static void
test_handoff_speed_falls(void)
{
    sensorless_six_step_config_t config = configuration(true);
    config.handoff_rpm = 1190U;
    fixture_t f;
    fixture_start(&f, &config, 0U);
    run_for(&f, 100000U);
    CHECK(f.state.qualified && f.state.consecutive_events >= 12U);
    f.rotor_mhz = 59000U;
    f.waveform_delay_us = -(int32_t)((uint64_t)(f.now - f.origin) * 1000U / 59000U);
    run_for(&f, 20000U);
    CHECK(f.state.consecutive_events >= 12U && f.state.estimated_rpm < config.handoff_rpm);
    CHECK(!f.state.qualified && !f.state.estimate_valid && f.state.running);
}

/** @brief 闭环接管不跳扇区，在理想截止前预约未来峰顶，且总预算不能被接管重置。 */
static void
test_closed_loop_and_budget(void)
{
    sensorless_six_step_config_t config = configuration(false);
    config.startup.run_ticks = MOTOR_CONTROL_HZ / 5U;
    fixture_t f;
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    CHECK(f.state.duty == config.startup.duty);
    CHECK(f.state.stage == SENSORLESS_SIX_STEP_CLOSED);
    CHECK(f.state.desired_sector == f.sector);
    run_for(&f, 100000U);
    CHECK(f.state.running && f.state.locked && f.state.fault == NULL);
    CHECK(f.reservations > 0U && f.commits > 30U);
    CHECK(f.state.estimated_rpm >= 1195U && f.state.estimated_rpm <= 1205U);
    run_for(&f, 200000U);
    CHECK(!f.state.running && f.state.fault == NULL && f.state.ticks == config.startup.run_ticks);
}

/** @brief 交接超时、丢采样和缺过零都停止闭环，不自动回退为开环。 */
static void
test_closed_failures(void)
{
    sensorless_six_step_config_t config = configuration(false);
    fixture_t f;
    fixture_start(&f, &config, 0U);
    f.constant_wave = true;
    run_for(&f, 600000U);
    CHECK(!f.state.running && strcmp(f.state.fault, "BEMF STARTUP") == 0);
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    f.invalid = true;
    run_for(&f, 3000U);
    CHECK(!f.state.running && strcmp(f.state.fault, "BEMF GAP") == 0);
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    f.constant_wave = true;
    run_for(&f, 10000U);
    CHECK(!f.state.running && strcmp(f.state.fault, "BEMF LOST") == 0);
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    /* 实际首次COM确认后尚无下个过零预约；先等待真实事件再注入迟到的截止。 */
    while (!f.state.deadline_valid) { fixture_tick(&f); CHECK(f.state.running); }
    f.state.next_commutation_us = f.now - 2U * PERIOD_US;
    fixture_tick(&f);
    CHECK(!f.state.running && strcmp(f.state.fault, "BEMF LATE") == 0 && f.state.late_count == 1U);
}

/** @brief 真实COM允许同一预约峰顶内的执行延迟；提前整周期、迟到下一周期或未提交都须故障停机。 */
static void
test_real_commit_times(void)
{
    sensorless_six_step_config_t config = configuration(false);
    fixture_t f;
    fixture_start(&f, &config, UINT32_MAX - 20000U);
    run_to_lock(&f);
    f.commit_delay_us = (int32_t)(PERIOD_US / 4U);
    run_for(&f, 100000U);
    CHECK(f.state.running && f.state.locked && f.commits > 30U && f.state.fault == NULL);
    CHECK(f.state.observed_committed_us == f.committed_us);
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    f.commit_delay_us = (int32_t)PERIOD_US;
    run_for(&f, 10000U);
    CHECK(!f.state.running && strcmp(f.state.fault, "BEMF COMMIT") == 0);
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    f.commit_delay_us = -(int32_t)PERIOD_US;
    run_for(&f, 10000U);
    CHECK(!f.state.running && strcmp(f.state.fault, "BEMF COMMIT") == 0);
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    f.omit_commit = true;
    run_for(&f, 10000U);
    CHECK(!f.state.running && strcmp(f.state.fault, "BEMF COMMIT") == 0);
}

/** @brief 推进到下一个1 kHz速度更新，只通过真实控制周期驱动算法。
 * @param f 已经锁定的模拟器。 */
static void
next_pi_update(fixture_t *f)
{
    do
    {
        fixture_tick(f);
        CHECK(f->state.running && f->state.locked);
    }
    while (f->state.pi_ticks != 0U);
}

/** @brief PI交接无阶跃，速度指令改变及大误差每毫秒最多变化一个千分点，积分不追赶限速输出。 */
static void
test_pi_anti_windup(void)
{
    sensorless_six_step_config_t config = configuration(false);
    config.kp_milli = 1000U;
    config.ki_milli = 100000U;
    config.startup.run_ticks = MOTOR_CONTROL_HZ;
    fixture_t f;
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    uint16_t before = f.state.duty;
    int64_t expected = (int64_t)before * 1000000LL -
        ((int64_t)f.target_rpm - f.state.estimated_rpm) * config.kp_milli * 1000LL;
    CHECK(f.state.integral_micro == expected && before == 200U);
    f.target_rpm = 1000U;
    int64_t integral = f.state.integral_micro;
    for (unsigned update = 0U; update < 110U; ++update)
    {
        uint16_t previous = f.state.duty;
        uint32_t updates_start = f.now;
        next_pi_update(&f);
        CHECK(f.state.duty <= previous && previous - f.state.duty <= 1);
        CHECK(f.state.integral_micro == integral);
        if (update != 0U)
        {
            CHECK(f.now - updates_start == 1000U);
        }
    }
    CHECK(f.state.integral_micro == integral && f.state.duty == config.min_duty);
    f.target_rpm = 1600U;
    for (unsigned update = 0U; update < 110U; ++update)
    {
        uint16_t previous = f.state.duty;
        next_pi_update(&f);
        CHECK(f.state.duty >= previous && f.state.duty - previous <= 1);
        CHECK(f.state.integral_micro == integral);
    }
    CHECK(f.state.duty == config.startup.duty);
}

/** @brief 小误差积分保留小数；一个控制周期限流也使整个1ms窗口主动退幅，不丢限流标志。 */
static void
test_pi_fraction_and_limit_window(void)
{
    sensorless_six_step_config_t config = configuration(false);
    config.kp_milli = 0U;
    config.ki_milli = 1U;
    fixture_t f;
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    f.target_rpm = 1190U;
    int64_t integral = f.state.integral_micro;
    f.current_limited = true;
    fixture_tick(&f);
    f.current_limited = false;
    next_pi_update(&f);
    CHECK(f.state.integral_micro < integral && f.state.duty == config.startup.duty - 1U);
    integral = f.state.integral_micro;
    next_pi_update(&f);
    CHECK(f.state.integral_micro < integral && integral - f.state.integral_micro < 100LL);
    integral = f.state.integral_micro;
    f.current_limited = true;
    next_pi_update(&f);
    CHECK(f.state.integral_micro < integral);
    integral = f.state.integral_micro;
    f.current_limited = false;
    f.target_rpm = 1600U;
    next_pi_update(&f);
    CHECK(f.state.integral_micro > integral && f.state.duty < config.startup.duty);
    integral = f.state.integral_micro;
    next_pi_update(&f);
    CHECK(f.state.integral_micro > integral && f.state.duty < config.startup.duty);
}

/** @brief 小P大I每毫秒都可利用部分积分走一步；上下界无windup，清除误差保持积分且反向误差立即生效。 */
static void
test_pi_integral_rate_limited_progress(void)
{
    sensorless_six_step_config_t config = configuration(false);
    config.startup.start_mhz = config.startup.target_mhz = 62500U;
    config.kp_milli = 1U;
    config.ki_milli = 100000U;
    fixture_t f;
    fixture_start(&f, &config, 0U);
    run_to_lock(&f);
    CHECK(f.state.estimated_rpm == 1250U && f.state.duty == 200U);
    f.target_rpm = 1000U;
    for (unsigned update = 0U; update < 100U; ++update)
    {
        uint16_t previous = f.state.duty;
        next_pi_update(&f);
        CHECK(f.state.estimated_rpm == 1250U && f.state.duty + 1U == previous);
        CHECK(f.state.integral_micro >= 99000000LL && f.state.integral_micro <= 201000000LL);
    }
    CHECK(f.state.duty == config.min_duty);
    int64_t integral = f.state.integral_micro;
    for (unsigned update = 0U; update < 20U; ++update)
    {
        next_pi_update(&f);
        CHECK(f.state.duty == config.min_duty && f.state.integral_micro == integral);
    }
    f.target_rpm = 1250U;
    for (unsigned update = 0U; update < 20U; ++update)
    {
        next_pi_update(&f);
        CHECK(f.state.duty == config.min_duty && f.state.integral_micro == integral);
    }
    f.target_rpm = 1500U;
    for (unsigned update = 0U; update < 100U; ++update)
    {
        uint16_t previous = f.state.duty;
        next_pi_update(&f);
        CHECK(f.state.duty == previous + 1U);
        CHECK(f.state.integral_micro >= 99000000LL && f.state.integral_micro <= 201000000LL);
    }
    CHECK(f.state.duty == config.startup.duty);
    integral = f.state.integral_micro;
    for (unsigned update = 0U; update < 20U; ++update)
    {
        next_pi_update(&f);
        CHECK(f.state.duty == config.startup.duty && f.state.integral_micro == integral);
    }
    f.target_rpm = 1250U;
    next_pi_update(&f);
    uint16_t settled = f.state.duty;
    CHECK(config.startup.duty - settled <= 1 && f.state.integral_micro == integral);
    for (unsigned update = 0U; update < 20U; ++update)
    {
        next_pi_update(&f);
        CHECK(f.state.duty == settled && f.state.integral_micro == integral);
    }
    f.target_rpm = 1000U;
    next_pi_update(&f);
    CHECK(f.state.duty + 1U == settled && f.state.integral_micro < integral);
}

/** @brief 方向延迟补偿、正提前角以及自然时间/序号回绕均保持正确事件归属。 */
static void
test_timing_and_wrap(void)
{
    sensorless_six_step_config_t config = configuration(false);
    config.comm_timing_decideg = 90;
    config.rising_delay_us = config.falling_delay_us = 100U;
    fixture_t f;
    fixture_start(&f, &config, UINT32_MAX - 20000U);
    f.waveform_delay_us = 100;
    f.sequence = UINT32_MAX - 10U;
    run_to_lock(&f);
    CHECK(f.state.next_commutation_us - f.state.last_event_us ==
        (uint32_t)((uint64_t)f.state.sector_period_us * 210U / 600U));
    run_for(&f, 100000U);
    CHECK(f.state.running && f.state.locked && f.state.estimate_valid);
    CHECK(f.state.duplicate_count == 0U && f.state.late_count == 0U);
    CHECK(strcmp(sensorless_six_step_stage_name(f.state.stage), "CLOSED") == 0);
    CHECK(strcmp(sensorless_six_step_stage_name((sensorless_six_step_stage_t)99), "NONE") == 0);
}

/** @brief 双方向不同滤波延迟仍按理想过零估速；允许的正负20度修正都须保持真实扇区/事件一致。 */
static void
test_direction_delays_and_timing_limits(void)
{
    for (int timing = -200; timing <= 200; timing += 200)
    {
        sensorless_six_step_config_t config = configuration(false);
        config.comm_timing_decideg = (int16_t)timing;
        config.rising_delay_us = 100U;
        config.falling_delay_us = 200U;
        fixture_t f;
        fixture_start(&f, &config, UINT32_MAX - 20000U);
        f.rising_wave_delay_us = 100;
        f.falling_wave_delay_us = 200;
        run_to_lock(&f);
        /* 本次资格事件的理想时间比第二次确认样本早至少其方向标定延迟。 */
        uint32_t delay = (f.state.last_event_sector & 1U) != 0U ? 100U : 200U;
        CHECK(f.now - f.state.last_event_us >= delay + PERIOD_US);
        run_for(&f, 100000U);
        CHECK(f.state.running && f.state.locked && f.state.qualified && f.state.estimate_valid);
        CHECK(f.state.estimated_rpm >= 1195U && f.state.estimated_rpm <= 1205U);
        CHECK(f.state.rejected_count == 0U && f.state.late_count == 0U);
    }
}

/** @brief 运行真实算法的主机回归。
 * @return 全部通过为 0，任一断言失败立即退出。 */
/** @brief 正式异步API的接管、绝对PI时基、失锁及实际COM缺失回归。 */
static void
test_background_pipeline(void)
{
    fixture_t f;
    sensorless_six_step_config_t config=configuration(false);
    fixture_start(&f,&config,0U); f.background=true;
    run_for(&f,200000U);
    CHECK(f.state.running && f.state.locked && f.state.qualified && f.commits>30U);
    CHECK(f.state.ticks==200000U/PERIOD_US && f.state.event_count>=12U);
    uint16_t duty=f.state.duty;
    int64_t integral=f.state.integral_micro;
    for (unsigned i=0U;i<100U;++i)
    {
        sensorless_six_step_service(&f.state,f.now+2U,f.target_rpm,false);
    }
    CHECK(f.state.duty==duty && f.state.integral_micro==integral);
    f.constant_wave=true;
    run_for(&f,20000U);
    CHECK(!f.state.running && f.state.fault!=NULL && !f.state.locked);
    fixture_start(&f,&config,0U); f.background=true;
    while (!f.state.synchronizing) { fixture_tick(&f); CHECK(f.state.running); }
    CHECK(!f.state.locked && f.state.commutation_pending);
    f.omit_commit=true;
    run_for(&f,20000U);
    CHECK(!f.state.running && strcmp(f.state.fault,"BEMF COMMIT")==0);
    fixture_start(&f,&config,0U); f.background=true;
    fixture_tick(&f);
    sensorless_six_step_input_t stale={.raw=48U,.sector=0U,.floating=2U,.valid=true,.sequence=++f.sequence,
        .sample_us=f.now,.now_us=f.now+2U*PERIOD_US+1U,.committed_us=0U,.target_rpm=f.target_rpm};
    sensorless_six_step_sample(&f.state,&stale);
    CHECK(f.state.event_count==0U && f.state.rejected_count>0U);
}

/** @brief 精简IRQ检测独立于后台，验证回绕、实际COM、失锁及精确1ms速度更新。 */
static void
test_irq_pipeline(void)
{
    sensorless_six_step_config_t config=configuration(false);
    config.irq_control=true;
    fixture_t f;
    fixture_start(&f,&config,UINT32_MAX-50000U); f.background=true;
    run_for(&f,200000U);
    CHECK(f.state.running && f.state.locked && f.state.qualified && f.commits>30U);
    CHECK(f.pi_updates>100U && f.pi_updates<200U);
    uint32_t before=f.pi_updates;
    run_for(&f,10000U);
    CHECK(f.pi_updates-before==10U);
    CHECK(f.state.estimated_rpm>=1195U && f.state.estimated_rpm<=1205U);
    f.constant_wave=true;
    run_for(&f,20000U);
    CHECK(!f.state.running && f.state.fault!=NULL);
    fixture_start(&f,&config,0U); f.background=true;
    while (!f.state.synchronizing) { fixture_tick(&f); CHECK(f.state.running); }
    CHECK(!f.state.locked);
    f.omit_commit=true;
    run_for(&f,20000U);
    CHECK(!f.state.running && !f.state.locked && f.state.fault!=NULL);
}

/** @brief 回放2026-10-06加摩擦失锁前浮相样本；旧20%窗口漏掉620us穿越。
 * 只核对采样/消隐机制，不把门限穿越冒充已标定的真实电角过零。 */
static void
test_loaded_early_crossing_capture(void)
{
    static const uint16_t raw[] = {
        347U,0U,0U,0U,0U,0U,0U,0U,0U,0U,10U,19U,33U,41U,51U,64U,76U
    };
    sensorless_six_step_config_t config=configuration(false);
    config.irq_control=true;
    config.startup.start_mhz=config.startup.target_mhz=50000U;
    config.falling_threshold=300U;
    sensorless_six_step_t state;
    sensorless_six_step_start(&state,&config);
    state.locked=true;
    state.open_loop.stage=SIX_STEP_RUN;
    state.sector_period_us=3775U;
    state.have_sector=true;
    state.observed_sector=0U;
    state.observed_committed_us=1075804U;
    state.sector_event=true;
    state.have_event=true;
    state.last_event_sector=0U;
    state.last_event_us=1076974U;
    state.commutation_pending=true;
    state.desired_sector=1U;
    state.submitted_us=1079454U;
    for(unsigned i=0U;i<sizeof(raw)/sizeof(raw[0]);++i)
    {
        sensorless_six_step_input_t input={.sector=1U,.raw=raw[i],.valid=true,
            .committed_us=1079454U,.sample_us=1079454U+20U+i*50U};
        input.now_us=input.sample_us+3U;
        sensorless_six_step_sample_irq(&state,&input);
        CHECK(state.running && state.locked && state.fault==NULL);
        if(i<12U) { CHECK(state.event_count==0U); }
    }
    CHECK(state.event_count==1U && state.sector_event && state.last_event_us==1080070U);
    CHECK(state.blanking_us>=config.blank_us && state.blanking_us<570U);
    CHECK(state.deadline_valid && state.next_commutation_us>state.last_event_us);
    /* 同样的长时间无后续过零仍必须关断，不延长缺反馈保护。 */
    uint32_t expired=state.last_event_us+2U*state.sector_period_us+1U;
    sensorless_six_step_input_t held={.sector=1U,.raw=76U,.valid=true,
        .committed_us=1079454U,.sample_us=expired,.now_us=expired};
    sensorless_six_step_sample_irq(&state,&held);
    sensorless_six_step_check_time(&state,expired);
    CHECK(!state.running && strcmp(state.fault,"BEMF LOST")==0);
}

/** @brief 回放2026-10-07原22%失败轮PULLIN期间真实BA样本，观测应开始而接管条件保持。
 * @note 定位阶段须忽略相同波形；换相尖峰与666us消隐仍过滤，单事件不能取得12次资格。 */
static void
test_pullin_observation_capture(void)
{
    /* 原始记录original22_fine_after20，COM210005us，+19us起每50us记录。 */
    static const uint16_t raw[] = {488U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,3U,8U,17U,20U,29U,39U,43U,52U,57U,65U,73U,78U,85U,94U,101U,109U,116U,122U,128U,135U,141U,146U,152U,158U,166U,173U,177U,182U,188U,192U,197U,203U,202U,207U,209U,209U,211U,214U,216U,218U,219U,221U,226U,227U,229U,232U,232U,236U,235U,238U,240U};
    for (unsigned pulling = 0U; pulling < 2U; ++pulling)
    {
        sensorless_six_step_config_t config = configuration(true);
        config.irq_control = true;
        config.startup.start_mhz = config.startup.target_mhz = 50000U;
        config.rising_threshold = 32U;
        config.falling_threshold = 280U;
        sensorless_six_step_t state;
        sensorless_six_step_start(&state, &config);
        const uint32_t commit = 210005U;
        six_step_stage_t stage = pulling != 0U ? SIX_STEP_PULLIN : SIX_STEP_ALIGN_HOLD;
        sensorless_six_step_progress_irq(&state, 4200U, 50000U, 3U, 220U, stage, commit);
        for (unsigned i = 0U; i < sizeof(raw) / sizeof(raw[0]); ++i)
        {
            sensorless_six_step_input_t input = {
                .sector = 3U, .raw = raw[i], .valid = true, .committed_us = commit,
                .sample_us = commit + 19U + i * 50U, .target_rpm = 1000U
            };
            input.now_us = input.sample_us + 3U;
            sensorless_six_step_sample_irq(&state, &input);
            CHECK(state.running && state.fault == NULL && !state.qualified && !state.locked);
            CHECK(!state.acquisition_started);
            if (i < 22U) { CHECK(state.event_count == 0U); }
        }
        if (pulling != 0U)
        {
            CHECK(state.blanking_us == 666U && state.event_count == 1U && state.sector_event);
            CHECK(state.last_event_us == 211039U);
        }
        else { CHECK(state.event_count == 0U); }
    }
}

/** @brief 隔周期合法跳脉冲仍可取得十二事件并闭环；真正持续缺反馈必须关断。 */
static void
test_density_sample_windows(void)
{
    for(unsigned irq=0U;irq<2U;++irq)
    {
        sensorless_six_step_config_t config=configuration(false);
        config.irq_control=irq!=0U;
        fixture_t f; fixture_start(&f,&config,UINT32_MAX-50000U);
        f.background=true; f.alternate_pwm=true;
        run_for(&f,200000U);
        CHECK(f.state.running && f.state.locked && f.state.qualified && f.commits>30U);
        CHECK(f.state.event_count>=12U && f.state.invalid_count>500U);
        /* 隔周期有效样本的整电周期时间误差至多2PWM，按真实时间分辨率检查估速。 */
        uint32_t speed_tolerance=(1200U*2U*PERIOD_US+16665U)/16666U;
        CHECK(f.state.estimated_rpm>=1200U-speed_tolerance && f.state.estimated_rpm<=1200U+speed_tolerance);
        f.invalid=true; run_for(&f,4000U);
        CHECK(!f.state.running && !f.state.locked && f.state.fault!=NULL);
    }
}

/** @brief 接管后交替边沿偏置缓慢校相；局部间隔超25%不误停，持续缺事件仍须失锁。 */
static void
test_phase_interval_tolerance(void)
{
    sensorless_six_step_config_t config=configuration(false);
    config.irq_control=true;
    config.startup.start_mhz=config.startup.target_mhz=50000U;
    fixture_t f; fixture_start(&f,&config,0U); f.background=true;
    run_for(&f,100000U);
    CHECK(f.state.running && f.state.locked);
    f.rising_wave_delay_us=350; f.falling_wave_delay_us=-350;
    run_for(&f,100000U);
    CHECK(f.state.running && f.state.locked && f.state.rejected_count==0U);
    f.rising_wave_delay_us=500; f.falling_wave_delay_us=-500;
    run_for(&f,20000U);
    CHECK(f.state.running && f.state.locked && f.state.rejected_count==0U);
    f.constant_wave=true;
    run_for(&f,20000U);
    CHECK(!f.state.running && !f.state.locked && f.state.fault!=NULL);
    fixture_start(&f,&config,0U); f.background=true;
    run_for(&f,100000U);
    CHECK(f.state.running && f.state.locked && f.state.history_count==6U);
    uint32_t accepted=f.state.event_count;
    /* 注入整周历史失真，拒绝的事件不能记成接受或更新最后有效反馈。 */
    f.state.event_history[f.state.history_index]-=50000U;
    run_for(&f,10000U);
    CHECK(!f.state.running && strcmp(f.state.fault,"BEMF PERIOD")==0);
    CHECK(f.state.event_count==accepted);
}

/** @brief 独立运行上限与限流退幅，不依赖启动DUTY；零误差时接管保持连续。 */
static void
test_pi_running_headroom(void)
{
    sensorless_six_step_config_t c=configuration(false);
    c.max_duty=250;c.min_duty=92;
    c.kp_milli=MOTOR_SENSORLESS_KP_DEFAULT_MILLI;
    c.ki_milli=MOTOR_SENSORLESS_KI_DEFAULT_MILLI;
    sensorless_six_step_pi_t pi={0};
    CHECK(sensorless_six_step_pi_update(&pi,&c,1000,1000,200,false)==200);
    for(unsigned k=0;k<500;++k)
    {
        uint16_t old=pi.duty;
        sensorless_six_step_pi_update(&pi,&c,1000,900,200,false);
        CHECK(pi.duty<=c.max_duty && pi.duty>=old && pi.duty-old<=1);
    }
    CHECK(pi.duty==250 && pi.duty>c.startup.duty);
    int64_t saturated=pi.integral_micro;
    for(unsigned k=0;k<100;++k)
    {
        sensorless_six_step_pi_update(&pi,&c,1000,900,200,false);
        CHECK(pi.duty==250 && pi.integral_micro==saturated);
    }
    for(unsigned k=0;k<200;++k)
    {
        uint16_t old=pi.duty;
        sensorless_six_step_pi_update(&pi,&c,1000,900,200,true);
        CHECK(pi.duty>=c.min_duty && pi.duty<=old && old-pi.duty<=1);
    }
    CHECK(pi.duty==c.min_duty);
    sensorless_six_step_pi_update(&pi,&c,1000,900,200,false);
    CHECK(pi.duty<=c.min_duty+1U);
}

/** @brief 速度PI与简化电压/转矩机械模型耦合，验证加卸负载下的执行量余量。
 * @note 不含退磁及过零检测，不代替整条六步链带载验收；惯量与负载均未实物标定。 */
static void
test_pi_load_plant(void)
{
    sensorless_six_step_config_t c=configuration(false);
    c.max_duty=250;c.min_duty=92;
    c.kp_milli=MOTOR_SENSORLESS_KP_DEFAULT_MILLI;
    c.ki_milli=MOTOR_SENSORLESS_KI_DEFAULT_MILLI;
    sensorless_six_step_pi_t pi={0};
    float omega=104.719755f,minimum=1000,maximum=1000;
    uint16_t peak_duty=0;
    for(unsigned k=0;k<5000;++k)
    {
        uint32_t rpm=(uint32_t)(omega*9.5492966f+.5f);
        uint16_t duty=sensorless_six_step_pi_update(&pi,&c,1000,rpm,200,false);
        CHECK(duty>=c.min_duty && duty<=c.max_duty);
        if(duty>peak_duty) { peak_duty=duty; }
        for(unsigned sub=0;sub<10;++sub)
        {
            /* 采用电气准稳态线电流与10个机械子步；由转矩独立求速度，不预设转子波形。 */
            float current=(12*duty*.001f-.021246f*omega)/1.58f;
            float torque=.0189f*current-.00002f*omega-(k>=2000 && k<3000?.004f:0);
            omega+=torque/.000003f*.0001f;
        }
        float actual=omega*9.5492966f;
        if(k>=2000 && k<3000 && actual<minimum) { minimum=actual; }
        if(k>=3000 && actual>maximum) { maximum=actual; }
    }
    CHECK(peak_duty>c.startup.duty && minimum>800 && maximum<1200);
    CHECK(omega*9.5492966f>980 && omega*9.5492966f<1020);
}

int
main(void)
{
    test_defaults_and_startup();
    test_observe_qualification();
    test_rejection_paths();
    test_phase_offset_synchronization();
    test_adaptive_demagnetization();
    test_observe_gap();
    test_sample_metadata();
    test_continuous_sample_confirmation();
    test_quality_expires();
    test_invalid_contexts();
    test_acquisition_timeout_after_ramp();
    test_minimum_speed_frequency_quantization();
    test_handoff_speed_falls();
    test_closed_loop_and_budget();
    test_closed_failures();
    test_real_commit_times();
    test_pi_anti_windup();
    test_pi_fraction_and_limit_window();
    test_pi_integral_rate_limited_progress();
    test_pi_running_headroom();
    test_pi_load_plant();
    test_timing_and_wrap();
    test_direction_delays_and_timing_limits();
    test_background_pipeline();
    test_irq_pipeline();
    test_loaded_early_crossing_capture();
    test_pullin_observation_capture();
    test_density_sample_windows();
    test_phase_interval_tolerance();
    puts("sensorless six-step tests passed");
    return 0;
}
