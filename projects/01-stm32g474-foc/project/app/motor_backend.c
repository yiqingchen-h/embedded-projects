#include "motor_backend.h"
#include "motor_config.h"
#if MOTOR_FOC_ENABLED
#include "board_foc.h"
#endif
#include "settings.h"
#include "board_motor.h"
#include "board_storage.h"
#include "six_step.h"
#include "sensorless_six_step.h"
#if MOTOR_ENCODER_ENABLED
#include "board_encoder.h"
#include "encoder_six_step.h"
#endif
#include <stdio.h>
#include <string.h>

/* OPEN/STEP由电流ISR推进；无感启动器由TIM峰顶独占，停机后才重新装载。 */
static six_step_t g_backend_step;
/* 观测器、资格及预约由ADC独占；PI积分由TIM的1kHz分支独占。 */
static sensorless_six_step_t g_backend_sensorless;
/* 当前运行采用无感采样配置；仅在功率禁用时修改。 */
static bool g_backend_using_sensorless;
/* 有感状态只由TIM持有；主循环仅在停机后验证并保存标定结果。 */
static bool g_backend_using_encoder;
#if MOTOR_ENCODER_ENABLED
static encoder_six_step_t g_backend_encoder;
/* 标定资格仅本次上电有效，不把默认零点或导入预设当成已标定。 */
static bool g_backend_encoder_calibrated, g_backend_encoder_cal_pending, g_backend_encoder_cal_request;
static int32_t g_backend_encoder_saved_direction, g_backend_encoder_saved_zero;
static uint32_t g_backend_encoder_saved_poles, g_backend_encoder_now, g_backend_encoder_changes;
static uint32_t g_backend_encoder_limit_hits;
/* 停机会话选项；正常有感启动时锁存，不写Flash且不影响标定资格。 */
static bool g_backend_encoder_quiet;

/** @brief 校准身份必须与当前极对数、方向和电角零点完全一致。 @return 本次上电校准可用。 */
static bool
encoder_calibration_valid(void)
{
    const int32_t *p=settings_get()->parameter;
    return g_backend_encoder_calibrated &&
        p[CONTROL_PARAM_ENCODER_DIR]==g_backend_encoder_saved_direction &&
        p[CONTROL_PARAM_ELEC_ZERO]==g_backend_encoder_saved_zero &&
        (uint32_t)p[CONTROL_PARAM_POLES]==g_backend_encoder_saved_poles;
}
#endif
/* 在线速度目标为对齐32位单次写，快速回调每周期读取。 */
static volatile uint32_t g_backend_target_rpm;
/* 目标电频率在主循环换算，峰顶只读取；限流标志由ADC写、峰顶读。 */
static volatile uint32_t g_backend_target_mhz;
static volatile bool g_backend_limited;
/* 速度中断累计限流命中快照，保证两次PI服务之间的短暂限流也冻结积分。 */
static uint32_t g_backend_seen_limit_hits;
/* ADC事件及低频检查分段计时，TIM速度计时包含ADC抢占；不格式化日志。 */
static uint32_t g_backend_next_check_us, g_backend_sample_max_cycles, g_backend_event_max_cycles, g_backend_pi_max_cycles;
/* 格式化墙钟时间和实际接管证据在停机后保留，不能用当前LOCKED=0覆盖历史。 */
static uint32_t g_backend_confirmed_com, g_backend_first_lock_us, g_backend_last_confirmed_us;
static volatile bool g_backend_lock_seen;
/* TIM写非活动槽后发布索引；ADC可抢占TIM，始终只读已发布的完整进度。 */
typedef struct {
    uint32_t ticks, frequency_mhz;
    uint8_t sector;
    uint16_t duty;
    six_step_stage_t stage;
} startup_progress_t;
static volatile startup_progress_t g_backend_progress[2];
static volatile uint32_t g_backend_progress_slot;
/* PI积分只在TIM低频分支使用，不与ADC共享64位可写状态。 */
static sensorless_six_step_pi_t g_backend_pi;
static uint32_t g_backend_pi_updates;
/* 单次测试配置仅在功率禁用时装入，运行期间只由 ADC ISR 读取。 */
static control_step_test_t g_backend_test;
/* 每步固定保持的控制周期数；0 表示普通开环。 */
static uint32_t g_backend_test_dwell_ticks;
/* ADC 码门限在启动前预计算，运行时不除法换算电流。 */
static uint16_t g_backend_zero[3], g_backend_trip_raw, g_backend_limit_raw;
/* 当前占空指令换算的PWM比较值和可采样的最小比较值。 */
static uint16_t g_backend_duty, g_backend_min_duty;
/* 小于可采样脉宽时使用整脉冲密度调制。 */
static uint32_t g_backend_pulse_accumulator;
/* 每次启动重置非零随机状态；仅正常有感RUN锁存分散模式，标定与旧六步保持原序列。 */
static uint32_t g_backend_pulse_noise;
static bool g_backend_pulse_spread;
/* 一次 RUN 的原始统计；高速路径只累加/比较，电流换算仅在主循环。 */
typedef struct {
    /* 实际回调数、有效/无效低侧样本数。 */
    uint32_t ticks, valid, invalid;
    /* 软限流命中与纯密度预算不足的输出决策次数，两者分别统计。 */
    uint32_t limit_hits, density_skip;
    /* 三相有效样本数及各相最大绝对 ADC 偏差；零样本没有可用峰值。 */
    uint32_t phase_samples[3], peak_delta[3];
    /* 最近有效样本，low 为 0..2；valid=0 时这些字段没有测量含义。 */
    int32_t last_delta;
    uint32_t last_raw, last_low;
    /* 最近完成回调的算法状态，频率 mHz、占空比千分比；不是实测转速。 */
    uint32_t sector, frequency_mhz, duty;
    /* 普通开环启动阶段；描述指令进程，不表示转子已同步。 */
    six_step_stage_t stage;
    /* 该次RUN的量程快照；失败的后续启动或停机换档不能改写历史峰值换算。 */
    uint32_t vdda_mv, gain, shunt_mohm;
    /* 最近运行类型：0 未运行、1 普通开环、2 单步、3 单轮六步。 */
    uint32_t step_mode;
    /* 测试步长 ms、反向标志和已完整保持的扇区数。 */
    uint32_t dwell_ms, reverse, completed;
    /* 本次是否无感运行，以及观察器的完整状态快照。 */
    bool sensorless, estimated_valid, qualified, locked;
    sensorless_six_step_stage_t sensorless_stage;
    uint32_t estimated_rpm, events, rejected, consecutive, invalid_bemf, blanked, duplicates, late;
    /* 反电动势原始样本、窗口有效性及时间，全部与实际已提交扇区关联。 */
    uint16_t bemf_raw;
    uint8_t floating, sampled_sector;
    bool bemf_valid;
    uint32_t sample_us, sample_age_us, sector_period_us, next_commutation_us;
    /* 补偿后过零相对开环扇区中点的偏差，单位us。 */
    int32_t phase_error_us;
} motor_run_stats_t;
/* 电流/观测字段ADC写，启动/输出字段TIM写；主循环只读取，不推进控制。 */
static volatile motor_run_stats_t g_backend_run;
/* 奇数表示写入中；全部共享统计为 volatile，主循环只接受序号一致的副本。 */
static volatile uint32_t g_backend_run_sequence;
static bool g_backend_initialized;
static control_algorithm_t g_backend_algorithm;
static control_mode_t g_backend_mode;
static uint32_t g_backend_gain, g_backend_shunt, g_backend_vdda;
static char g_backend_last_fault[24];
/* UI 快照最多 50 Hz；请求、选择、运行状态和故障变化立即发布。 */
#if MOTOR_SENSORLESS_TRACE
/* 1024项仅覆盖最后51.2ms原始波形，ADC写，停机读；溢出覆盖旧记录，不导致停机。 */
typedef struct {
    uint32_t sample_us, committed_us;
    uint16_t raw;
    uint8_t sector, flags;
} bemf_trace_sample_t;
/* 事件快照仅在接受/拒绝/实际COM确认时记录。 */
typedef struct {
    uint32_t sample_us, committed_us, crossing_us, period_us;
    int32_t phase_error_us;
    uint16_t raw;
    uint8_t sector, flags;
} bemf_trace_event_t;
static bemf_trace_sample_t g_backend_trace[1024];
static bemf_trace_event_t g_backend_event_trace[64];
/* 总写入序号停机后稳定，不与主循环进行控制交接。 */
static uint32_t g_backend_trace_written, g_backend_event_trace_written;
#endif
static uint32_t g_backend_published_ms;
static bool g_backend_publish_pending;

/** @brief 读取完整 RUN 原始统计，不在复制期间屏蔽中断。 @return 序号一致的统计副本。 */
static motor_run_stats_t
run_snapshot(void)
{
    for (;;)
    {
        uint32_t before = g_backend_run_sequence;
        if (before & 1U) { continue; }
        motor_run_stats_t result = g_backend_run;
        if (before == g_backend_run_sequence)
        {
            /* 人读电频率仅在后台换算，闭环执行始终使用过零周期us。 */
            if (result.sensorless && g_backend_lock_seen)
            {
                result.frequency_mhz = result.estimated_rpm * g_backend_sensorless.config.pole_pairs * 1000U / 60U;
            }
            return result;
        }
    }
}

/** @brief 在主循环将普通开环阶段转为可读指令状态。
 * @param stage 最近生成输出的阶段。
 * @return 阶段名称，不表示转子位置或同步结果。 */
static const char *
startup_stage_name(six_step_stage_t stage)
{
    switch (stage)
    {
    case SIX_STEP_ALIGN_RAMP: return "ALIGN_RAMP";
    case SIX_STEP_ALIGN_HOLD: return "ALIGN_HOLD";
    case SIX_STEP_PULLIN: return "PULLIN";
    case SIX_STEP_RAMP: return "RAMP";
    case SIX_STEP_RUN: return "RUN";
    default: return "NONE";
    }
}

/** @brief 按限流和密度预算生成首周期或下周期输出，不额外赠送最小宽脉冲。 @param limited 当前有效样本已达到软限流。 @param next 输出候选；相序由调用者填写。 */
static void MOTOR_FAST_CODE
prepare_pulse(bool limited, board_motor_output_t *next)
{
    next->duty_ticks = g_backend_duty > g_backend_min_duty ? g_backend_duty : g_backend_min_duty;
    next->pulse = six_step_pulse_density(&g_backend_pulse_accumulator,&g_backend_pulse_noise,
        g_backend_duty,g_backend_min_duty,limited,g_backend_pulse_spread);
    if (g_backend_duty < g_backend_min_duty && !limited && !next->pulse)
    {
        ++g_backend_run.density_skip;
    }
}

#if MOTOR_ENCODER_ENABLED
/** @brief 峰顶非阻塞获取完整编码器帧并提交有感扇区；ADC仍独立执行电流保护。
 * @param controlled 无感预约标记，本模式忽略。 @param sector 原扇区，本模式忽略。
 * @param duty 无感幅度，本模式忽略。 @param next 下周期功率输出。 */
static void MOTOR_FAST_CODE
encoder_peak(bool controlled, uint8_t sector, uint16_t duty, board_motor_output_t *next)
{
    (void)controlled; (void)sector; (void)duty;
    encoder_six_feedback_t feedback=board_encoder_control_tick();
    encoder_six_step_tick(&g_backend_encoder,&feedback,board_motor_cycles());
    ++g_backend_run_sequence;
    if (g_backend_run.sector != g_backend_encoder.sector) { ++g_backend_encoder_changes; }
    g_backend_run.ticks=g_backend_encoder.ticks;
    g_backend_run.sector=g_backend_encoder.sector;
    g_backend_run.duty=g_backend_encoder.duty;
    g_backend_run.estimated_rpm=g_backend_encoder.rpm>0?(uint32_t)g_backend_encoder.rpm:0U;
    g_backend_run.estimated_valid=g_backend_encoder.speed_valid;
    g_backend_run.frequency_mhz=g_backend_run.estimated_rpm*g_backend_encoder.config.pole_pairs*1000U/60U;
    g_backend_run.stage=g_backend_encoder.calibration?SIX_STEP_ALIGN_HOLD:SIX_STEP_RUN;
    if (!g_backend_encoder.running)
    {
        board_motor_stop(g_backend_encoder.fault);
        ++g_backend_run_sequence;
        return;
    }
    g_backend_duty=board_motor_duty_ticks(g_backend_encoder.duty);
    six_step_phases(g_backend_encoder.sector,&next->high,&next->low);
    prepare_pulse(g_backend_limited,next);
    ++g_backend_run_sequence;
}

/** @brief 实际COM之后执行1kHz有感速度PI，累计限流命中冻结积分。 */
static void MOTOR_FAST_CODE
encoder_speed_irq(void)
{
    uint32_t hits=g_backend_run.limit_hits;
    encoder_six_step_speed(&g_backend_encoder,g_backend_target_rpm,hits!=g_backend_encoder_limit_hits);
    g_backend_encoder_limit_hits=hits;
    if (!g_backend_encoder.running) { board_motor_stop(g_backend_encoder.fault); }
}
#endif

/** @brief 单步/单轮测试按固定周期推进，普通开环与直接连续六步使用电角积分。
 * @return 本次调度完成的测试扇区数；电角积分运行返回0。 */
static uint32_t
advance_step(void)
{
    if (g_backend_test_dwell_ticks == 0U)
    {
        six_step_tick(&g_backend_step);
        return 0U;
    }
    ++g_backend_step.ticks;
    uint32_t completed = g_backend_step.ticks / g_backend_test_dwell_ticks;
    if (g_backend_step.ticks >= g_backend_step.config.run_ticks)
    {
        /* 满一轮直接关断，不额外提交第七个扇区，也不返回普通开环。 */
        g_backend_step.running = false;
    }
    else if (g_backend_step.ticks % g_backend_test_dwell_ticks == 0U)
    {
        g_backend_step.sector = (uint8_t)((g_backend_step.sector + (g_backend_test.reverse ? 5U : 1U)) % 6U);
    }
    return completed;
}

/** @brief ADC只发布原始控制量及计数，单位换算留在后台。
 * @param frame 当前同步原始帧。 */
static void MOTOR_FAST_CODE
publish_sensorless(const board_motor_observation_t *frame)
{
    g_backend_run.sensorless_stage = g_backend_sensorless.stage;
    g_backend_run.estimated_rpm = g_backend_sensorless.estimated_rpm;
    g_backend_run.estimated_valid = g_backend_sensorless.estimate_valid;
    g_backend_run.qualified = g_backend_sensorless.qualified;
    g_backend_run.locked = g_backend_sensorless.locked;
    g_backend_run.events = g_backend_sensorless.event_count;
    g_backend_run.rejected = g_backend_sensorless.rejected_count;
    g_backend_run.consecutive = g_backend_sensorless.consecutive_events;
    g_backend_run.invalid_bemf = g_backend_sensorless.invalid_count;
    g_backend_run.blanked = g_backend_sensorless.blanked_count;
    g_backend_run.duplicates = g_backend_sensorless.duplicate_count;
    g_backend_run.late = g_backend_sensorless.late_count;
    g_backend_run.phase_error_us = g_backend_sensorless.phase_error_us;
    g_backend_run.bemf_raw = frame->raw;
    g_backend_run.floating = frame->floating;
    g_backend_run.sampled_sector = frame->sector;
    g_backend_run.bemf_valid = frame->valid;
    g_backend_run.sample_us = frame->sample_us;
    g_backend_run.sample_age_us = frame->now_us - frame->sample_us;
    g_backend_run.sector_period_us = g_backend_sensorless.sector_period_us;
    g_backend_run.next_commutation_us = g_backend_sensorless.next_commutation_us;
}

/** @brief 无感峰顶独占启动推进，预约阶段冻结电角积分；电流保护结果独立跳脉冲。
 * @param controlled ADC已经完整发布预约。 @param sector 板卡选定的实际下一扇区。
 * @param duty PI占空比千分比。 @param next 本峰顶输出。 */
static void MOTOR_FAST_CODE
sensorless_peak(bool controlled, uint8_t sector, uint16_t duty, board_motor_output_t *next)
{
    ++g_backend_run_sequence;
    if (!controlled)
    {
        g_backend_step.config.target_mhz = g_backend_target_mhz;
        six_step_tick(&g_backend_step);
    }
    else
    {
        if (++g_backend_step.ticks >= g_backend_step.config.run_ticks) { g_backend_step.running = false; }
        g_backend_step.sector = sector;
        g_backend_step.duty = duty;
    }
    g_backend_run.ticks = g_backend_step.ticks;
    if (!g_backend_step.running)
    {
        board_motor_stop(NULL);
        ++g_backend_run_sequence;
        return;
    }
    if (g_backend_run.duty != g_backend_step.duty) { g_backend_duty = board_motor_duty_ticks(g_backend_step.duty); }
    g_backend_run.sector = g_backend_step.sector;
    g_backend_run.frequency_mhz = g_backend_step.frequency_mhz;
    g_backend_run.duty = g_backend_step.duty;
    g_backend_run.stage = g_backend_step.stage;
    six_step_phases(g_backend_step.sector, &next->high, &next->low);
    prepare_pulse(g_backend_limited, next);
    uint32_t slot = g_backend_progress_slot ^ 1U;
    g_backend_progress[slot] = (startup_progress_t){g_backend_step.ticks,
        g_backend_step.frequency_mhz, g_backend_step.sector, g_backend_step.duty, g_backend_step.stage};
    g_backend_progress_slot = slot;
    ++g_backend_run_sequence;
}

/** @brief ADC同步检测及事件预约，不经过主循环；普通样本不复制观测器。
 * @param frame 真实硬件采样及COM上下文。 */
static void MOTOR_FAST_CODE
sensorless_bemf_irq(const board_motor_observation_t *frame)
{
    if (!g_backend_sensorless.running) { return; }
    uint32_t started = board_motor_cycles();
    ++g_backend_run_sequence;
    startup_progress_t startup = g_backend_progress[g_backend_progress_slot];
    sensorless_six_step_progress_irq(&g_backend_sensorless, startup.ticks,
        startup.frequency_mhz, startup.sector, startup.duty, startup.stage, frame->now_us);
    sensorless_six_step_input_t input = {
        .raw=frame->raw, .sector=frame->sector, .valid=frame->valid,
        .sample_us=frame->sample_us, .committed_us=frame->committed_us,
        .target_rpm=g_backend_target_rpm
    };
    uint32_t events = g_backend_sensorless.event_count;
    bool locked = g_backend_sensorless.locked;
#if MOTOR_SENSORLESS_TRACE
    uint32_t rejected = g_backend_sensorless.rejected_count;
    bool synchronizing = g_backend_sensorless.synchronizing;
#endif
    sensorless_six_step_sample_irq(&g_backend_sensorless, &input);
    bool due = (int32_t)(frame->now_us - g_backend_next_check_us) >= 0;
    if (due)
    {
        g_backend_next_check_us = frame->now_us + 1000U;
        sensorless_six_step_check_time(&g_backend_sensorless, frame->now_us);
    }
    if (!g_backend_sensorless.running)
    {
        board_motor_stop(g_backend_sensorless.fault);
    }
    else
    {
        uint8_t sector;
        uint32_t deadline, commit;
        if (sensorless_six_step_schedule(&g_backend_sensorless, &sector, &deadline) &&
            board_motor_schedule(sector, deadline, g_backend_sensorless.duty, &commit))
        {
            sensorless_six_step_schedule_ack(&g_backend_sensorless, commit);
        }
        if (g_backend_sensorless.locked &&
            (!g_backend_lock_seen || frame->committed_us != g_backend_last_confirmed_us))
        {
            if (!g_backend_lock_seen) { g_backend_first_lock_us = frame->committed_us; }
            g_backend_lock_seen = true;
            g_backend_last_confirmed_us = frame->committed_us;
            ++g_backend_confirmed_com;
        }
    }
    if (due || events != g_backend_sensorless.event_count || locked != g_backend_sensorless.locked ||
        !board_motor_running()) { publish_sensorless(frame); }
#if MOTOR_SENSORLESS_TRACE
    /* 几个原始字段直接写SRAM；与控制结果无依赖，禁止在此格式化。 */
    uint8_t flags = (frame->valid ? 1U : 0U) | (locked ? 2U : 0U) |
        (synchronizing ? 4U : 0U) | (events != g_backend_sensorless.event_count ? 8U : 0U) |
        (rejected != g_backend_sensorless.rejected_count ? 16U : 0U) |
        (g_backend_sensorless.locked ? 32U : 0U);
    g_backend_trace[g_backend_trace_written++ & 1023U] =
        (bemf_trace_sample_t){frame->sample_us, frame->committed_us, frame->raw, frame->sector, flags};
    if ((flags & 24U) != 0U || locked != g_backend_sensorless.locked)
    {
        g_backend_event_trace[g_backend_event_trace_written++ & 63U] =
            (bemf_trace_event_t){frame->sample_us, frame->committed_us, g_backend_sensorless.crossing_us,
                g_backend_sensorless.sector_period_us, g_backend_sensorless.phase_error_us,
                frame->raw, frame->sector, flags};
    }
#endif
    uint32_t elapsed = board_motor_cycles() - started;
    if (elapsed > g_backend_sample_max_cycles) { g_backend_sample_max_cycles = elapsed; }
    if (events != g_backend_sensorless.event_count && elapsed > g_backend_event_max_cycles)
    {
        g_backend_event_max_cycles = elapsed;
    }
    ++g_backend_run_sequence;
}

/** @brief TIM提交完成后每1ms运行PI；ADC可抢占计算，快照/提交各只屏蔽数个字段。 */
static void MOTOR_FAST_CODE
sensorless_speed_irq(void)
{
    uint32_t mask = board_motor_control_lock();
    bool active = board_motor_running() && g_backend_sensorless.locked;
    uint32_t rpm = g_backend_sensorless.estimated_rpm;
    uint32_t target = g_backend_target_rpm;
    uint32_t hits = g_backend_run.limit_hits;
    uint16_t duty = g_backend_sensorless.duty;
    board_motor_control_unlock(mask);
    if (!active) { return; }
    uint32_t started = board_motor_cycles();
    uint16_t next = sensorless_six_step_pi_update(&g_backend_pi, &g_backend_sensorless.config,
        target, rpm, duty, hits != g_backend_seen_limit_hits);
    g_backend_seen_limit_hits = hits;
    mask = board_motor_control_lock();
    /* ADC或BKIN可在计算中关断；提交幅度不能改变关断后的功率状态。 */
    if (board_motor_running() && g_backend_sensorless.locked)
    {
        g_backend_sensorless.duty = next;
        board_motor_set_control_duty(next);
        ++g_backend_pi_updates;
    }
    board_motor_control_unlock(mask);
    uint32_t elapsed = board_motor_cycles() - started;
    if (elapsed > g_backend_pi_max_cycles) { g_backend_pi_max_cycles = elapsed; }
}

/** @brief 每周期读取有效低侧电流，原始码比较后生成下周期扇区和脉冲。 @param raw ADC 结果。 @param low 有效低侧相。 @param valid 本周期有导通窗口。 @param next 下周期输出。 */
static void MOTOR_FAST_CODE
fast_sample(uint16_t raw, uint8_t low, bool valid, board_motor_output_t *next)
{
    ++g_backend_run_sequence;
    if (!g_backend_using_sensorless && !g_backend_using_encoder) { ++g_backend_run.ticks; }
    bool limited = false;
    if (valid)
    {
        int32_t delta = (int32_t)raw - g_backend_zero[low];
        uint32_t magnitude = (uint32_t)(delta < 0 ? -delta : delta);
        g_backend_run.last_delta = delta;
        g_backend_run.last_raw = raw;
        g_backend_run.last_low = low;
        ++g_backend_run.valid;
        ++g_backend_run.phase_samples[low];
        if (magnitude > g_backend_run.peak_delta[low]) { g_backend_run.peak_delta[low] = magnitude; }
        limited = magnitude >= g_backend_limit_raw;
        if (limited) { ++g_backend_run.limit_hits; }
        if (magnitude >= g_backend_trip_raw)
        {
            board_motor_stop("PHASE OVERCURRENT");
            ++g_backend_run_sequence;
            return;
        }
    }
    else
    {
        ++g_backend_run.invalid;
    }
    if (g_backend_using_sensorless || g_backend_using_encoder)
    {
        g_backend_limited = limited;
        ++g_backend_run_sequence;
        return;
    }
    else
    {
        g_backend_run.completed = advance_step();
    }
    g_backend_run.sector = g_backend_step.sector;
    g_backend_run.frequency_mhz = g_backend_step.frequency_mhz;
    if (!g_backend_step.running)
    {
        board_motor_stop(g_backend_using_sensorless ? g_backend_sensorless.fault : NULL);
        ++g_backend_run_sequence;
        return;
    }
    if (g_backend_test_dwell_ticks == 0U && g_backend_run.duty != g_backend_step.duty)
    {
        /* 幅度仅在千分比指令改变时换算，低于采样下限仍走整脉冲密度预算。 */
        g_backend_duty = board_motor_duty_ticks(g_backend_step.duty);
    }
    g_backend_run.duty = g_backend_step.duty;
    g_backend_run.stage = g_backend_step.stage;
    six_step_phases(g_backend_step.sector, &next->high, &next->low);
    prepare_pulse(limited, next);
    ++g_backend_run_sequence;
}

/** @brief 由配置转换电频率，机械目标不冒充实测速度。 @param rpm 机械转速。 @return 电频率 mHz。 */
static uint32_t
electrical_frequency(uint32_t rpm)
{
    return rpm * (uint32_t)settings_get()->parameter[CONTROL_PARAM_POLES] * 1000U / 60U;
}

/** @brief 在后台检查电角度修正是否给退磁和下一峰顶排程保留有效窗口。
 * @param frequency_mhz 目标电频率，非零mHz。
 * @param timing 电角度修正，单位0.1度。
 * @param blank_us 退磁消隐，us。
 * @return 闭环配置具有可观测与可提交窗口。 */
static bool
sensorless_timing_valid(uint32_t frequency_mhz, int16_t timing, uint32_t blank_us)
{
    if (timing < -200 || timing > 200) { return false; }
    uint32_t period = 1000000000U / (frequency_mhz * 6U);
    uint32_t margin = 2U * (1000000U / MOTOR_CONTROL_HZ);
    uint32_t crossing = period * (uint32_t)(300 + timing) / 600U;
    uint32_t delay = period * (uint32_t)(300 - timing) / 600U;
    return crossing > blank_us + margin && delay > margin;
}

/** @brief 检查静态启动条件并校准；这些条件不在快速控制 ISR 重复检查。
 * @param test NULL 为连续六步，否则为有界单步或单轮测试；不改写持久参数。
 * @return NULL 成功，其他为可见原因。 */
static const char * MOTOR_BACKGROUND_CODE
start_motor(const control_step_test_t *test)
{
    const int32_t *p = settings_get()->parameter;
    board_motor_status_t status = board_motor_status();
    if (!g_backend_initialized || !board_storage_layout_valid()) { return "HARDWARE/LAYOUT"; }
#if MOTOR_FOC_ENABLED
    if(control_model_algorithm()==CONTROL_SENSORLESS_FOC ||
        (MOTOR_ENCODER_ENABLED && control_model_algorithm()==CONTROL_ENCODER_FOC))
    {
        bool iq_mode=control_model_algorithm()==CONTROL_ENCODER_FOC && control_model_mode()==CONTROL_CURRENT;
        bool position_mode=control_model_algorithm()==CONTROL_ENCODER_FOC && control_model_mode()==CONTROL_POSITION;
        bool haptic_mode=control_model_algorithm()==CONTROL_ENCODER_FOC && control_model_mode()==CONTROL_HAPTIC;
        if(test || (control_model_mode()!=CONTROL_SPEED && !iq_mode && !position_mode && !haptic_mode)) { return "FOC MODE"; }
        if(status.running || !board_motor_storage_safe()) { return "POWER ACTIVE"; }
        if(strcmp(status.fault,"NONE")!=0) { return "CLEAR FAULT FIRST"; }
        if(p[CONTROL_PARAM_SHUNT]!=50 || p[CONTROL_PARAM_POLES]!=MOTOR_POLE_PAIRS) { return "FOC SHUNT/POLE PROFILE"; }
        const settings_foc_t *f=&settings_get()->foc;
        foc_drive_config_t config={.resistance_ohm=f->resistance_mohm*.001f,.inductance_h=f->inductance_uh*.000001f,
            .voltage_error_bound_v=.12f,
            .current_limit_a=(haptic_mode?settings_get()->haptic.current_limit_ma:iq_mode?500:f->current_limit_ma)*.001f,.current_bandwidth_hz=(float)f->bandwidth_hz,
            .target_rpm=(uint32_t)f->target_rpm,.pole_pairs=MOTOR_POLE_PAIRS,.extended_speed_range=true,.speed_kp=f->speed_kp_micro*.000001f,
            .speed_ki=f->speed_ki_micro*.000001f,.speed_ramp_rpm_s=f->speed_ramp_millirpm_s*.001f};
#if MOTOR_ENCODER_ENABLED
        if(control_model_algorithm()==CONTROL_ENCODER_FOC)
        {
            if(!encoder_calibration_valid()) { return "ENCODER CAL REQUIRED"; }
            encoder_snapshot_t position=board_encoder_snapshot(g_backend_encoder_now);
            if(!position.angle_valid || !position.speed_valid || position.rpm<-10 || position.rpm>10)
            { return "ENCODER NOT STILL"; }
            encoder_foc_config_t observed={.drive=config,.direction=(int8_t)p[CONTROL_PARAM_ENCODER_DIR],
                .zero_decideg=(uint16_t)p[CONTROL_PARAM_ELEC_ZERO],.current_mode=iq_mode,.target_iq_a=f->target_iq_ma*.001f,
                .position_mode=position_mode,.haptic_mode=haptic_mode,.haptic=settings_get()->haptic.config};
            if(position_mode)
            {
                encoder_position_t mechanical=board_encoder_position();
                if(!mechanical.valid || !mechanical.zero_valid || mechanical.direction!=p[CONTROL_PARAM_ENCODER_DIR])
                { return "POSITION ZERO REQUIRED"; }
                const settings_position_t *q=&settings_get()->position;
                observed.drive.speed_kp=q->speed_kp_micro*.000001f;
                observed.drive.speed_ki=q->speed_ki_micro*.000001f;
                observed.drive.speed_ramp_rpm_s=q->acceleration_millirpm_s*.001f;
                observed.target_position_mdeg=settings_value(CONTROL_PARAM_TARGET);
                observed.position_tolerance_mdeg=q->tolerance_mdeg;
                observed.position_kp=q->kp_milli*.001f;
                observed.position_speed_limit_rpm=(float)q->speed_limit_rpm;
            }
            return board_foc_encoder_start(&observed,(uint32_t)(haptic_mode?settings_get()->haptic.run_ms:f->run_ms))?NULL:"ENCODER FOC REST/CAL/CONFIG";
        }
#endif
        return board_foc_speed_start(&config,(uint32_t)f->run_ms)?NULL:"FOC REST/CAL/CONFIG";
    }
#endif
    bool sensorless = control_model_algorithm() == CONTROL_SENSORLESS_6STEP;
    bool encoder = false;
#if MOTOR_ENCODER_ENABLED
    encoder = control_model_algorithm() == CONTROL_ENCODER_6STEP;
    if (encoder && !g_backend_encoder_cal_request && !encoder_calibration_valid()) { return "ENCODER CAL REQUIRED"; }
    if (encoder && (p[CONTROL_PARAM_POLES] != MOTOR_POLE_PAIRS || p[CONTROL_PARAM_SHUNT] != 50)) { return "ENCODER PROFILE"; }
    if (encoder && control_model_mode()!=CONTROL_SPEED) { return "ENCODER SPEED ONLY"; }
#endif
    if (control_model_algorithm() != CONTROL_OPEN_6STEP && !sensorless && !encoder) { return "NOT IMPLEMENTED"; }
    if ((sensorless || encoder) && NULL != test) { return "STEP OPEN ONLY"; }
    if (status.running || !board_motor_storage_safe()) { return "POWER ACTIVE"; }
    if (strcmp(status.fault, "NONE") != 0) { return "CLEAR FAULT FIRST"; }
    uint32_t frequency = 0U;
    uint32_t pullin_steps = 0U;
    if (encoder)
    {
        if (p[CONTROL_PARAM_TARGET] <= 0 || p[CONTROL_PARAM_TARGET] > p[CONTROL_PARAM_SPEED_LIMIT]) { return "TARGET/SPEED LIMIT"; }
#if MOTOR_ENCODER_ENABLED
        encoder_snapshot_t position=board_encoder_snapshot(g_backend_encoder_now);
        if (!position.angle_valid || !position.speed_valid || position.rpm < -10 || position.rpm > 10) { return "ENCODER NOT STILL"; }
        if (!board_motor_standstill()) { return "ENCODER NOT STILL"; }
#endif
        frequency=electrical_frequency((uint32_t)p[CONTROL_PARAM_TARGET]);
    }
    else if (NULL == test)
    {
        if (p[CONTROL_PARAM_POLES] == 0) { return "SET POLE PAIRS"; }
        if (p[CONTROL_PARAM_TARGET] <= 0 || p[CONTROL_PARAM_TARGET] > p[CONTROL_PARAM_SPEED_LIMIT]) { return "TARGET/SPEED LIMIT"; }
        frequency = electrical_frequency((uint32_t)p[CONTROL_PARAM_TARGET]);
        /* 每周期最多换一个扇区；限制最高电频率为控制频率的 1/12。 */
        if (frequency > MOTOR_CONTROL_HZ * 1000U / 12U) { return "ELECTRICAL SPEED"; }
        if (frequency < (uint32_t)p[CONTROL_PARAM_START_HZ]) { return "ELECTRICAL SPEED"; }
        if (p[CONTROL_PARAM_RUN_MS] <= p[CONTROL_PARAM_ALIGN_MS]) { return "RUN/ALIGN TIME"; }
        pullin_steps = 6U * MOTOR_START_PULLIN_CYCLES;
        /* 普通启动必须为定位后的完整牵引段留出运行时间。 */
        uint32_t pullin_denominator = (uint32_t)p[CONTROL_PARAM_START_HZ] * 6U;
        uint32_t pullin_ticks = (uint32_t)(((uint64_t)pullin_steps * MOTOR_CONTROL_HZ * 1000U +
            pullin_denominator - 1U) / pullin_denominator);
        uint32_t available_ticks = (uint32_t)(p[CONTROL_PARAM_RUN_MS] - p[CONTROL_PARAM_ALIGN_MS]) *
            (MOTOR_CONTROL_HZ / 1000U);
        if (available_ticks <= pullin_ticks) { return "RUN/START TIME"; }
        if (sensorless)
        {
            sensorless_six_step_config_t limits;
            sensorless_six_step_defaults(&limits);
            if ((uint32_t)p[CONTROL_PARAM_TARGET] < limits.min_rpm) { return "BEMF SPEED RANGE"; }
            if (p[CONTROL_PARAM_HANDOFF] > p[CONTROL_PARAM_TARGET]) { return "HANDOFF/TARGET"; }
            if (frequency > MOTOR_CONTROL_HZ * 1000U / 60U) { return "BEMF SAMPLE RATE"; }
            if (MOTOR_SENSORLESS_CLOSED_LOOP && !sensorless_timing_valid(frequency,
                (int16_t)p[CONTROL_PARAM_COMM_TIMING], limits.blank_us)) { return "BEMF TIMING RANGE"; }
        }
    }
    else
    {
        if (test->sector >= 6U || test->dwell_ms < 20U || test->dwell_ms > 1000U ||
            (test->cycle && test->sector != 0U) || (!test->cycle && test->reverse)) { return "STEP RANGE"; }
        /* 固定时长六步的电频率指令；单个静态扇区没有换向频率。 */
        if (test->cycle) { frequency = 1000000U / (6U * test->dwell_ms); }
    }
    g_backend_shunt = (uint32_t)p[CONTROL_PARAM_SHUNT];
    g_backend_gain = g_backend_shunt == 50U ? 10U : 40U;
    if (!board_motor_set_bemf(sensorless)) { return "ADC PROFILE"; }
    if (!board_motor_calibrate(g_backend_shunt)) { return "DRV/ADC CAL"; }
    status = board_motor_status();
    /* 使用长等待完成后的实测电压校验启动，不沿用校准前的旧值。 */
    if (status.bus_mv < MOTOR_BUS_MIN_MV || status.bus_mv > MOTOR_BUS_MAX_MV)
    {
        board_motor_stop("BUS VOLTAGE");
        return "BUS VOLTAGE";
    }
    g_backend_vdda = status.vdda_mv;
    if (g_backend_vdda < 2700U || g_backend_vdda > 3600U) { board_motor_stop("VDDA"); return "VDDA"; }
    memcpy(g_backend_zero, status.zero, sizeof(g_backend_zero));
    g_backend_trip_raw = (uint16_t)((MOTOR_TRIP_MA * g_backend_gain * g_backend_shunt * 4095UL) / (g_backend_vdda * 1000UL));
    uint32_t rail = (MOTOR_CSA_LINEAR_MARGIN_MV * 4095U + g_backend_vdda - 1U) / g_backend_vdda;
    for (unsigned i = 0; i < 3; ++i)
    {
        /* 只在启动检查，保证正负 2 A 都在线性范围内，不依赖饱和后的电流读数。 */
        if (g_backend_zero[i] < rail + g_backend_trip_raw || g_backend_zero[i] > 4095U - rail - g_backend_trip_raw)
        {
            board_motor_stop("CSA RANGE");
            return "CSA RANGE";
        }
    }
    g_backend_limit_raw = (uint16_t)(((uint32_t)p[CONTROL_PARAM_CURRENT_LIMIT] * g_backend_gain * g_backend_shunt * 4095UL) / (g_backend_vdda * 1000UL));
    g_backend_min_duty = board_motor_min_duty(g_backend_shunt);
    g_backend_pulse_accumulator = 0;
    g_backend_pulse_noise = 0x6D2B79F5U;
    g_backend_pulse_spread = false;
    six_step_config_t config = {
        frequency, (uint32_t)p[CONTROL_PARAM_START_HZ], (uint32_t)p[CONTROL_PARAM_RAMP],
        (uint32_t)p[CONTROL_PARAM_ALIGN_MS] * (MOTOR_CONTROL_HZ / 1000U),
        (uint32_t)p[CONTROL_PARAM_RUN_MS] * (MOTOR_CONTROL_HZ / 1000U), (uint16_t)p[CONTROL_PARAM_DUTY],
        (uint16_t)pullin_steps
    };
    g_backend_test_dwell_ticks = 0U;
    if (NULL != test)
    {
        g_backend_test = *test;
        g_backend_test_dwell_ticks = test->dwell_ms * (MOTOR_CONTROL_HZ / 1000U);
        config.target_mhz = config.start_mhz = frequency;
        config.align_ticks = 0U;
        config.run_ticks = g_backend_test_dwell_ticks * (test->cycle ? 6U : 1U);
    }
    six_step_start(&g_backend_step, &config);
    g_backend_using_sensorless = sensorless;
    g_backend_using_encoder = encoder;
    g_backend_target_rpm = (uint32_t)p[CONTROL_PARAM_TARGET];
    g_backend_target_mhz = frequency;
    g_backend_limited = false;
    g_backend_seen_limit_hits = 0U;
#if MOTOR_SENSORLESS_TRACE
    g_backend_trace_written = g_backend_event_trace_written = 0U;
#endif
    g_backend_next_check_us = g_backend_sample_max_cycles = g_backend_event_max_cycles = g_backend_pi_max_cycles = 0U;
    g_backend_confirmed_com = g_backend_first_lock_us = g_backend_last_confirmed_us = 0U;
    g_backend_lock_seen = false;
    memset(&g_backend_pi, 0, sizeof(g_backend_pi));
    g_backend_pi_updates = 0U;
    g_backend_progress_slot = 0U;
    g_backend_progress[0] = (startup_progress_t){g_backend_step.ticks, g_backend_step.frequency_mhz,
        g_backend_step.sector, g_backend_step.duty, g_backend_step.stage};
    board_motor_set_peak_callback(sensorless ? sensorless_peak : NULL);
    board_motor_set_control_callbacks(sensorless ? sensorless_bemf_irq : NULL, sensorless ? sensorless_speed_irq : NULL);
#if MOTOR_ENCODER_ENABLED
    if (encoder)
    {
        g_backend_pulse_spread = g_backend_encoder_quiet && !g_backend_encoder_cal_request;
        encoder_six_config_t feedback_config={.tick_hz=170000000U,.pwm_hz=MOTOR_CONTROL_HZ,
            .pole_pairs=(uint32_t)p[CONTROL_PARAM_POLES],.run_ticks=config.run_ticks,
            .target_rpm=(uint32_t)p[CONTROL_PARAM_TARGET],
            .ramp_millirpm_s=(uint32_t)p[CONTROL_PARAM_RAMP]*60U/(uint32_t)p[CONTROL_PARAM_POLES],
            .kp_milli=p[CONTROL_PARAM_SPEED_KP]==0?MOTOR_SENSORLESS_KP_DEFAULT_MILLI:(uint32_t)p[CONTROL_PARAM_SPEED_KP],
            .ki_milli=p[CONTROL_PARAM_SPEED_KI]==0?MOTOR_SENSORLESS_KI_DEFAULT_MILLI:(uint32_t)p[CONTROL_PARAM_SPEED_KI],
            .max_duty=(uint16_t)p[CONTROL_PARAM_DUTY],
            /* 标定跟随已校验DUTY（G档最高95%），原相电流限幅及异常保护继续生效。 */
            .calibration_duty=(uint16_t)p[CONTROL_PARAM_DUTY],
            .direction=(int8_t)p[CONTROL_PARAM_ENCODER_DIR],.zero_decideg=(uint16_t)p[CONTROL_PARAM_ELEC_ZERO],
            .startup_boost=MOTOR_PROFILE_G2208!=0U,
            .hard_rpm=MOTOR_FOC_HARD_MAX_RPM};
        if (!encoder_six_step_start(&g_backend_encoder,&feedback_config,g_backend_encoder_cal_request)) { return "ENCODER CONFIG"; }
        if (!board_encoder_control_begin()) { g_backend_encoder.running=false; return "ENCODER OFFLINE"; }
        g_backend_encoder_changes=g_backend_encoder_limit_hits=0U;
        g_backend_step.duty=0U;
        g_backend_step.stage=g_backend_encoder_cal_request?SIX_STEP_ALIGN_HOLD:SIX_STEP_RUN;
        g_backend_encoder_cal_pending=g_backend_encoder_cal_request;
        board_motor_set_peak_callback(encoder_peak);
        board_motor_set_control_callbacks(NULL,encoder_speed_irq);
    }
#endif
    if (sensorless)
    {
        sensorless_six_step_config_t observed;
        sensorless_six_step_defaults(&observed);
        observed.startup = config;
        observed.pole_pairs = (uint32_t)p[CONTROL_PARAM_POLES];
        observed.handoff_rpm = (uint32_t)p[CONTROL_PARAM_HANDOFF];
        /* 保留已有保存值，0选择固件默认增益，不写回Flash；非零值按用户配置生效。 */
        observed.kp_milli = p[CONTROL_PARAM_SPEED_KP] == 0 ? MOTOR_SENSORLESS_KP_DEFAULT_MILLI : (uint32_t)p[CONTROL_PARAM_SPEED_KP];
        observed.ki_milli = p[CONTROL_PARAM_SPEED_KI] == 0 ? MOTOR_SENSORLESS_KI_DEFAULT_MILLI : (uint32_t)p[CONTROL_PARAM_SPEED_KI];
        observed.comm_timing_decideg = (int16_t)p[CONTROL_PARAM_COMM_TIMING];
        observed.observe_only = MOTOR_SENSORLESS_CLOSED_LOOP == 0U;
        observed.irq_control = true;
        /* 12V/50Hz/20%实测下降300码过早，280码回放恢复连续事件并完成短轮实际COM。
         * 仍是本工况候选，不是跨母线/占空/转速通用的已标定真实过零电平。 */
        observed.rising_threshold = 32U;
        observed.falling_threshold = 280U;
        uint32_t full_scale = board_motor_duty_ticks(1000U);
        observed.min_duty = (uint16_t)((g_backend_min_duty * 1000U + full_scale - 1U) / full_scale);
        if (config.duty < observed.min_duty || config.duty > observed.max_duty)
        {
            board_motor_stop(NULL);
            return "BEMF DUTY RANGE";
        }
        sensorless_six_step_start(&g_backend_sensorless, &observed);
    }
    if (NULL != test) { g_backend_step.sector = test->sector; }
    g_backend_duty = board_motor_duty_ticks(g_backend_step.duty);
    ++g_backend_run_sequence;
    memset((void *)&g_backend_run, 0, sizeof(g_backend_run));
    g_backend_run.frequency_mhz = g_backend_step.frequency_mhz;
    g_backend_run.duty = g_backend_step.duty;
    g_backend_run.stage = g_backend_step.stage;
    g_backend_run.vdda_mv = g_backend_vdda;
    g_backend_run.gain = g_backend_gain;
    g_backend_run.shunt_mohm = g_backend_shunt;
    g_backend_run.sector = g_backend_step.sector;
    g_backend_run.step_mode = encoder ? 4U : NULL == test ? 1U : test->cycle ? 3U : 2U;
    g_backend_run.sensorless = sensorless;
    if (sensorless) { g_backend_run.sensorless_stage = g_backend_sensorless.stage; }
    if (NULL != test)
    {
        g_backend_run.dwell_ms = test->dwell_ms;
        g_backend_run.reverse = test->reverse ? 1U : 0U;
    }
    board_motor_output_t first = {0, 1, 0, false};
    six_step_phases(g_backend_step.sector, &first.high, &first.low);
    prepare_pulse(false, &first);
    ++g_backend_run_sequence;
    if (!board_motor_start(&first, fast_sample))
    {
        /* 保留板卡记录的启动失败原因，例如TIM峰顶同步失败。 */
        const char *reason = board_motor_status().fault;
        if (strcmp(reason, "NONE") == 0) { reason = "START FAILED"; }
        board_motor_stop(reason);
        return reason;
    }
    return NULL;
}

/** @brief 见motor_backend.h。 */
const char *
motor_backend_encoder_calibrate(void)
{
#if MOTOR_ENCODER_ENABLED
    if (control_model_algorithm()!=CONTROL_ENCODER_6STEP || control_model_mode()!=CONTROL_SPEED) { return "SELECT ENCODER_6STEP"; }
    if (settings_get()->source!=CONTROL_REMOTE) { return "REMOTE ONLY"; }
    if (control_model_pending()!=NULL || board_motor_running() || !board_motor_storage_safe()) { return "BUSY"; }
    g_backend_encoder_cal_request=true;
    const char *reason=start_motor(NULL);
    g_backend_encoder_cal_request=false;
    if (reason==NULL)
    {
        board_encoder_position_invalidate();
        settings_position_reset_target();
        g_backend_encoder_calibrated=false; g_backend_publish_pending=true;
    }
    return reason;
#else
    return "UNAVAILABLE";
#endif
}

/** @brief 见motor_backend.h。 */
const char *
motor_backend_encoder_quiet(bool enabled)
{
#if MOTOR_ENCODER_ENABLED
    if (control_model_algorithm()!=CONTROL_ENCODER_6STEP || control_model_mode()!=CONTROL_SPEED) { return "SELECT ENCODER_6STEP"; }
    if (settings_get()->source!=CONTROL_REMOTE) { return "REMOTE ONLY"; }
    if (control_model_pending()!=NULL || board_motor_running() || !board_motor_storage_safe()) { return "BUSY"; }
    g_backend_encoder_quiet=enabled;
    return NULL;
#else
    (void)enabled;
    return "UNAVAILABLE";
#endif
}

/** @brief 见motor_backend.h；禁止运行中干扰SPI1 DMA，恢复影子也须真实停机。 */
const char *
motor_backend_encoder_hysteresis(uint8_t code)
{
#if MOTOR_ENCODER_ENABLED
    if(code<4U || code>7U) { return "RANGE"; }
    if(settings_get()->source!=CONTROL_REMOTE) { return "REMOTE ONLY"; }
    if(control_model_pending()!=NULL || board_motor_running() || !board_motor_storage_safe()) { return "BUSY"; }
    if(!board_motor_standstill()) { return "NOT STILL"; }
    return board_encoder_hysteresis(code)?NULL:"HYSTERESIS IO";
#else
    (void)code;
    return "UNAVAILABLE";
#endif
}

/** @brief 见motor_backend.h。 */
void MOTOR_BACKGROUND_CODE
motor_backend_encoder_diagnostics(char *out,size_t size)
{
#if MOTOR_ENCODER_ENABLED
    /* 只复制会被TIM修改的少量对齐字段；格式化和单位换算均在临界区之外。 */
    uint32_t mask=board_motor_control_lock();
    uint32_t ticks=g_backend_encoder.ticks,raw=g_backend_encoder.raw;
    uint32_t sequence=g_backend_encoder.feedback_sequence,age=g_backend_encoder.max_age_ticks;
    uint32_t changes=g_backend_encoder_changes,index=g_backend_encoder.cal_index;
    int32_t rpm=g_backend_encoder.rpm;
    uint16_t duty=g_backend_encoder.duty;
    uint8_t sector=g_backend_encoder.sector;
    bool active=g_backend_encoder.running,cal=g_backend_encoder.calibration,valid=g_backend_encoder.speed_valid;
    bool starting=g_backend_encoder.startup_active;
    uint32_t motion=g_backend_encoder.motion_ticks,end=g_backend_encoder.startup_end_ticks;
    const char *fault=g_backend_encoder.fault;
    board_motor_control_unlock(mask);
    board_motor_status_t power=board_motor_status();
    if (fault==NULL) { fault=power.fault; }
    const int32_t *p=settings_get()->parameter;
    snprintf(out,size,"OK MOTOR ENCODER STATE=%s CALIBRATION=%s TARGET=%ld RPM=%ld RPM_VALID=%u RAW=%lu SAMPLES=%lu TICKS=%lu SECTOR=%u SECTOR_CHANGES=%lu DUTY=%u.%u AGE_MAX_US=%lu DIR=%ld ZERO_DECIDEG=%ld CAL_POINT=%lu/7 CAPTURE_HZ=5000 COMM_BLANKS=%lu QUIET=%s STARTUP=%s MOTION_MS=%lu STARTUP_END_MS=%lu FAULT=%s",
        active?(cal?"CALIBRATING":"RUNNING"):"STOPPED",encoder_calibration_valid()?"VALID":"REQUIRED",
        (long)p[CONTROL_PARAM_TARGET],(long)rpm,valid?1U:0U,(unsigned long)raw,(unsigned long)sequence,
        (unsigned long)ticks,sector+1U,(unsigned long)changes,duty/10U,duty%10U,
        (unsigned long)(age/170U),(long)p[CONTROL_PARAM_ENCODER_DIR],(long)p[CONTROL_PARAM_ELEC_ZERO],
        (unsigned long)index+1U,(unsigned long)power.encoder_blank_count,g_backend_encoder_quiet?"ON":"OFF",
        cal?"CAL":starting?"BOOST":end!=0U?"DONE":"OFF",
        (unsigned long)(motion/(MOTOR_CONTROL_HZ/1000U)),(unsigned long)(end/(MOTOR_CONTROL_HZ/1000U)),fault?fault:"NONE");
#else
    snprintf(out,size,"ERR ENCODER UNAVAILABLE");
#endif
}

/** @brief 执行已获模型授权的请求，确认成功后才提交参数。 @param r 请求。 @param now 当前 ms。 @return NULL 表示成功。 */
/* 仅主循环处理串口请求；不参与采样ISR或1kHz控制，使用体积优化。 */
static const char *MOTOR_BACKGROUND_CODE
execute_request(const control_request_t *r, uint32_t now)
{
    switch (r->kind)
    {
    case CONTROL_REQUEST_START: return start_motor(NULL);
    case CONTROL_REQUEST_STEP_TEST: return start_motor(&r->step_test);
    case CONTROL_REQUEST_STOP: board_motor_stop(NULL); return NULL;
    case CONTROL_REQUEST_PARAMETER:
#if MOTOR_FOC_ENABLED
        if(board_foc_speed_active() && r->parameter==CONTROL_PARAM_TARGET)
        {
            if(control_model_mode()==CONTROL_POSITION)
            {
                if(!board_foc_position_target(r->value)) { return "FOC WAIT CLOSED"; }
                return settings_set(r->parameter,r->value,now)?NULL:"RANGE";
            }
            if(control_model_mode()==CONTROL_CURRENT)
            {
                if(r->value<-500 || r->value>500) { return "FOC IQ RANGE"; }
                if(!board_foc_iq_target(r->value)) { return "FOC WAIT CLOSED"; }
                return settings_set(r->parameter,r->value,now)?NULL:"RANGE";
            }
            if(r->value<1 || r->value>(int32_t)MOTOR_SPEED_MAX_RPM) { return "FOC SPEED RANGE"; }
            if(!board_foc_speed_target((uint32_t)r->value)) { return "FOC WAIT CLOSED"; }
            return settings_set(r->parameter,r->value,now)?NULL:"RANGE";
        }
#endif
        if (board_motor_status().running && r->parameter == CONTROL_PARAM_TARGET)
        {
            if (g_backend_test_dwell_ticks != 0U) { return "STEP TEST ACTIVE"; }
            const int32_t *p = settings_get()->parameter;
            if (r->value <= 0 || r->value > p[CONTROL_PARAM_SPEED_LIMIT]) { return "TARGET/SPEED LIMIT"; }
#if MOTOR_ENCODER_ENABLED
            if (g_backend_using_encoder)
            {
                if (g_backend_encoder.calibration) { return "ENC CAL ACTIVE"; }
                g_backend_target_rpm=(uint32_t)r->value;
                return settings_set(r->parameter,r->value,now)?NULL:"RANGE";
            }
#endif
            uint32_t frequency = electrical_frequency((uint32_t)r->value);
            /* 已启动后允许在线减速低于初频；新启动仍由start_motor检查牵引条件。 */
            if (frequency > MOTOR_CONTROL_HZ * 1000U / 12U) { return "ELECTRICAL SPEED"; }
            /* 32 位对齐单次写；ISR 在周期边界读取新目标。 */
            if (g_backend_using_sensorless)
            {
                if ((uint32_t)r->value < g_backend_sensorless.config.min_rpm) { return "BEMF SPEED RANGE"; }
                if (!run_snapshot().locked && (uint32_t)r->value < g_backend_sensorless.config.handoff_rpm)
                {
                    return "HANDOFF/TARGET";
                }
                if (frequency > MOTOR_CONTROL_HZ * 1000U / 60U) { return "BEMF SAMPLE RATE"; }
                if (MOTOR_SENSORLESS_CLOSED_LOOP && !sensorless_timing_valid(frequency,
                    g_backend_sensorless.config.comm_timing_decideg,
                    g_backend_sensorless.config.blank_us)) { return "BEMF TIMING RANGE"; }
                g_backend_target_rpm = (uint32_t)r->value;
                g_backend_target_mhz = frequency;
            }
            else
            {
                g_backend_step.config.target_mhz = frequency;
            }
        }
        if (!settings_set(r->parameter, r->value, now)) { return "RANGE"; }
        if (r->parameter == CONTROL_PARAM_SHUNT)
        {
            bool ok = board_motor_calibrate((uint32_t)r->value);
            board_motor_stop(NULL);
            (void)ok; /* 档位已提交；校准失败在故障页显示并保持功率禁用。 */
        }
        return NULL;
    case CONTROL_REQUEST_SOURCE: settings_source((control_source_t)r->value, now); return NULL;
    case CONTROL_REQUEST_CALIBRATE:
    case CONTROL_REQUEST_CLEAR_FAULT:
    {
        if (!board_motor_set_bemf(control_model_algorithm() == CONTROL_SENSORLESS_6STEP)) { return "ADC PROFILE"; }
        bool ok = board_motor_calibrate((uint32_t)settings_get()->parameter[CONTROL_PARAM_SHUNT]);
        board_motor_stop(NULL);
        return ok ? NULL : "DRV/ADC CAL";
    }
    default: return "NOT IMPLEMENTED";
    }
}

/** @brief 初始化开环及无感六步后端，真正闭环仍须事件资格及实际COM确认。 */
void MOTOR_BACKGROUND_CODE
motor_backend_init(void)
{
    /* 再初始化前先撤销输出，避免重置共享统计时仍有高速回调。 */
    g_backend_initialized = board_motor_init();
    memset(&g_backend_step, 0, sizeof(g_backend_step));
    memset(&g_backend_sensorless, 0, sizeof(g_backend_sensorless));
    g_backend_using_sensorless = false;
    g_backend_using_encoder = false;
#if MOTOR_ENCODER_ENABLED
    memset(&g_backend_encoder,0,sizeof(g_backend_encoder));
    g_backend_encoder_calibrated=g_backend_encoder_cal_pending=g_backend_encoder_cal_request=false;
    g_backend_encoder_quiet=true;
#endif
    g_backend_target_rpm = 0U;
    memset(&g_backend_test, 0, sizeof(g_backend_test));
    g_backend_test_dwell_ticks = 0U;
    memset((void *)&g_backend_run, 0, sizeof(g_backend_run));
    g_backend_run_sequence = 0U;
    g_backend_algorithm = control_model_algorithm();
    g_backend_mode = control_model_mode();
    settings_publish();
    if (!g_backend_initialized) { board_motor_stop("INIT FAILED"); }
    else
    {
        /* 冷启动电源可能尚在上升；只在此处有界重试，不能自动启动电机。 */
        for (unsigned attempt = 0; attempt < MOTOR_BOOT_CAL_ATTEMPTS; ++attempt)
        {
            bool ok = board_motor_calibrate((uint32_t)settings_get()->parameter[CONTROL_PARAM_SHUNT]);
            board_motor_stop(NULL);
            if (ok) { break; }
        }
    }
    g_backend_last_fault[0] = '\0';
    g_backend_publish_pending = true;
}

/** @brief 在主循环发布一致控制快照、处理请求及停机保存。 */
void MOTOR_BACKGROUND_CODE
motor_backend_service(uint32_t now)
{
#if MOTOR_ENCODER_ENABLED
    g_backend_encoder_now=now;
#endif
#if MOTOR_FOC_ENABLED
    board_foc_service();
#endif
    board_motor_service(now);
#if MOTOR_ENCODER_ENABLED
    if (g_backend_using_encoder && !board_motor_running())
    {
        board_encoder_control_end();
        g_backend_encoder.running=false;
        if (g_backend_encoder_cal_pending && board_motor_storage_safe())
        {
            g_backend_encoder_cal_pending=false;
            g_backend_encoder.running=false;
            bool ok=encoder_six_step_calibration_result(&g_backend_encoder);
            if (ok && strcmp(board_motor_status().fault,"NONE")==0)
            {
                ok=settings_set(CONTROL_PARAM_ENCODER_DIR,g_backend_encoder.config.direction,now) &&
                    settings_set(CONTROL_PARAM_ELEC_ZERO,g_backend_encoder.config.zero_decideg,now);
                if (ok)
                {
                    g_backend_encoder_saved_direction=g_backend_encoder.config.direction;
                    g_backend_encoder_saved_zero=g_backend_encoder.config.zero_decideg;
                    g_backend_encoder_saved_poles=g_backend_encoder.config.pole_pairs;
                }
                settings_publish();
            }
            else { ok=false; }
            g_backend_encoder_calibrated=ok;
            if (!ok && g_backend_encoder.fault!=NULL) { board_motor_stop(g_backend_encoder.fault); }
            g_backend_publish_pending=true;
        }
        g_backend_using_encoder=false;
    }
#endif
    if (g_backend_algorithm != control_model_algorithm() || g_backend_mode != control_model_mode())
    {
        g_backend_algorithm = control_model_algorithm();
        g_backend_mode = control_model_mode();
        settings_publish();
        g_backend_publish_pending = true;
    }
    control_request_t r;
    if (control_model_take_request(&r))
    {
        const char *reason = execute_request(&r, now);
        /* START后的采样、换相和PI独立由中断执行，完成请求不推进控制。 */
        (void)control_model_complete(r.sequence, reason == NULL, reason);
        /* 请求完成已更新参数值/来源，描述与静态范围保持原值。 */
        g_backend_publish_pending = true;
    }
    board_motor_status_t hw = board_motor_status();
#if MOTOR_FOC_ENABLED
    if (board_foc_active())
    {
        if(!g_backend_publish_pending && now-g_backend_published_ms<20U) { return; }
        g_backend_publish_pending=false;
        g_backend_published_ms=now;
        control_snapshot_t bench = {0};
        bench.algorithm = control_model_algorithm();
        bench.mode = control_model_mode();
        bench.run = CONTROL_RUNNING;
        bench.source = (control_source_t)settings_get()->source;
        bench.updated_ms = now;
        bench.capabilities = CONTROL_CAP_STOP;
        snprintf(bench.startup,sizeof(bench.startup),"FOC TIMING BENCH");
        snprintf(bench.fault,sizeof(bench.fault),"NONE");
        if(board_foc_speed_active())
        {
            board_foc_runtime_t live;
            board_foc_runtime(&live);
            bench.ready=true;
            bench.capabilities|=CONTROL_CAP_PARAMETERS;
            bench.calibrated=true;
            bench.values[CONTROL_TARGET]=(control_value_t){settings_value(CONTROL_PARAM_TARGET),bench.mode==CONTROL_CURRENT||bench.mode==CONTROL_POSITION?3U:0U,bench.mode!=CONTROL_HAPTIC};
            bench.values[CONTROL_BUS_VOLTAGE]=(control_value_t){live.bus_mv,3,true};
            bench.values[CONTROL_SPEED_VALUE]=(control_value_t){live.estimated_rpm,0,live.tracking};
            for(unsigned i=0;i<3;++i)
            { bench.values[CONTROL_IA+i]=(control_value_t){live.phase_ma[i],3,live.samples_valid}; }
            bench.values[CONTROL_ID]=(control_value_t){live.id_ma,3,live.samples_valid};
            bench.values[CONTROL_IQ]=(control_value_t){live.iq_ma,3,live.samples_valid};
            bench.values[CONTROL_UD]=(control_value_t){live.ud_mv,3,live.samples_valid};
            bench.values[CONTROL_UQ]=(control_value_t){live.uq_mv,3,live.samples_valid};
            /* 只报告相对电角；机械角始终无效。强拖指令只在启动阶段显示有效。 */
            bench.values[CONTROL_ELEC_ANGLE]=(control_value_t){live.relative_angle_decideg,1,live.tracking};
            bench.values[CONTROL_COMM_HZ]=(control_value_t){live.forced_millihz,3,!live.closed};
            snprintf(bench.startup,sizeof(bench.startup),"%s",foc_drive_stage_name(live.stage));
            snprintf(bench.commutation,sizeof(bench.commutation),"%s",live.encoder?"MT6835":live.closed?"SMO PLL":"STARTUP ANGLE");
            if(live.encoder)
            { bench.values[CONTROL_MECH_ANGLE]=(control_value_t){live.mechanical_angle_decideg,1,live.samples_valid}; }
        }
        (void)control_model_publish(&bench);
        return;
    }
#endif
    settings_service(board_motor_storage_safe(), now);
    const control_snapshot_t *previous = control_model_snapshot();
    if (!g_backend_publish_pending && now - g_backend_published_ms < 20U &&
        hw.running == (previous->run == CONTROL_RUNNING) && strcmp(hw.fault, previous->fault) == 0) { return; }
    g_backend_publish_pending = false;
    g_backend_published_ms = now;
    if (hw.running && g_backend_using_sensorless)
    {
        /* RUN冻结显示，发布控制模型必要字段即可，不格式化人读状态文本。 */
        control_snapshot_t live = *previous;
        motor_run_stats_t run = run_snapshot();
        live.run = CONTROL_RUNNING;
        live.power_disabled = false;
        live.calibrated = hw.calibrated;
        live.updated_ms = now;
        live.source = (control_source_t)settings_get()->source;
        live.values[CONTROL_TARGET] = (control_value_t){settings_get()->parameter[CONTROL_PARAM_TARGET], 0, true};
        live.values[CONTROL_BUS_VOLTAGE] = (control_value_t){hw.bus_mv, 3, hw.regular_count != 0U};
        live.values[CONTROL_SPEED_VALUE] = (control_value_t){(int32_t)run.estimated_rpm, 0, run.estimated_valid};
        live.values[CONTROL_COMM_HZ] = (control_value_t){(int32_t)run.frequency_mhz, 3, true};
        live.values[CONTROL_DUTY] = (control_value_t){(int32_t)run.duty, 1, true};
        /* 人读相序/阶段在停机诊断中保留；运行态、STOP能力和采样值继续更新。 */
        (void)control_model_publish(&live);
        return;
    }
    for (unsigned i = 0; i < 3; ++i)
    {
        control_parameter_t p = *control_model_parameter((control_parameter_id_t)(CONTROL_PARAM_OFFSET_A + i));
        if (hw.zero[i] != 0 && (!p.valid || p.value != hw.zero[i]))
        {
            p.value = hw.zero[i]; p.valid = true; p.editable = false;
            snprintf(p.unit, sizeof(p.unit), "ADC");
            (void)control_model_parameter_publish(&p);
        }
    }
    control_snapshot_t s = {0};
    s.algorithm = control_model_algorithm();
    s.mode = control_model_mode();
    s.ready = g_backend_initialized && board_storage_layout_valid() &&
        (s.algorithm == CONTROL_OPEN_6STEP || s.algorithm == CONTROL_SENSORLESS_6STEP);
#if MOTOR_ENCODER_ENABLED
    if (s.algorithm==CONTROL_ENCODER_6STEP)
    {
        encoder_snapshot_t feedback=board_encoder_snapshot(now);
        s.encoder_valid=true;
        s.encoder_online=feedback.angle_valid;
        s.ready=g_backend_initialized && board_storage_layout_valid() && feedback.angle_valid && s.mode==CONTROL_SPEED;
    }
#endif
#if MOTOR_FOC_ENABLED
    if(g_backend_initialized && board_storage_layout_valid() &&
        (s.algorithm==CONTROL_SENSORLESS_FOC || (MOTOR_ENCODER_ENABLED && s.algorithm==CONTROL_ENCODER_FOC)) &&
        (s.mode==CONTROL_SPEED || (s.algorithm==CONTROL_ENCODER_FOC && (s.mode==CONTROL_CURRENT || s.mode==CONTROL_POSITION || s.mode==CONTROL_HAPTIC))) && settings_get()->parameter[CONTROL_PARAM_SHUNT]==50 &&
        settings_get()->parameter[CONTROL_PARAM_POLES]==MOTOR_POLE_PAIRS) { s.ready=true; }
#endif
    s.source = (control_source_t)settings_get()->source;
    bool fault = strcmp(hw.fault, "NONE") != 0;
    s.run = hw.running ? CONTROL_RUNNING : fault ? CONTROL_FAULT : CONTROL_STOPPED;
    s.power_disabled = board_motor_storage_safe();
    s.calibrated = hw.calibrated;
#if MOTOR_ENCODER_ENABLED
    if (s.algorithm==CONTROL_ENCODER_6STEP || s.algorithm==CONTROL_ENCODER_FOC)
    {
        s.calibrated=encoder_calibration_valid();
        if(s.algorithm==CONTROL_ENCODER_FOC)
        {
            encoder_snapshot_t feedback=board_encoder_snapshot(now);
            s.encoder_valid=true;s.encoder_online=feedback.angle_valid;
            s.ready=s.ready&&feedback.angle_valid;
        }
    }
#endif
    s.updated_ms = now;
    s.capabilities = CONTROL_CAP_PARAMETERS | CONTROL_CAP_STOP | CONTROL_CAP_CLEAR_FAULT;
    if (s.ready) { s.capabilities |= CONTROL_CAP_START | CONTROL_CAP_SOURCE | CONTROL_CAP_CALIBRATE; }
    if (s.ready && s.algorithm == CONTROL_OPEN_6STEP) { s.capabilities |= CONTROL_CAP_STEP_TEST; }
    s.values[CONTROL_TARGET] = (control_value_t){settings_value(CONTROL_PARAM_TARGET), s.mode==CONTROL_CURRENT||s.mode==CONTROL_POSITION?3U:0U,
        s.mode==CONTROL_SPEED || (s.algorithm==CONTROL_ENCODER_FOC && (s.mode==CONTROL_CURRENT || s.mode==CONTROL_POSITION || s.mode==CONTROL_HAPTIC))};
    s.values[CONTROL_BUS_VOLTAGE] = (control_value_t){hw.bus_mv, 3, hw.regular_count != 0 && now - hw.slow_ms <= MOTOR_BUS_STALE_MS};
    /* 仅报告当前被实际采到的低侧相；异步三相样本不能用于相加校验。 */
    motor_run_stats_t run = run_snapshot();
    if (hw.running && run.sensorless)
    {
        s.values[CONTROL_SPEED_VALUE] = (control_value_t){(int32_t)run.estimated_rpm, 0, run.estimated_valid};
    }
#if MOTOR_ENCODER_ENABLED
    if (hw.running && s.algorithm==CONTROL_ENCODER_6STEP)
    {
        s.values[CONTROL_SPEED_VALUE]=(control_value_t){(int32_t)run.estimated_rpm,0,run.estimated_valid};
        s.values[CONTROL_MECH_ANGLE]=(control_value_t){(int32_t)((uint64_t)g_backend_encoder.raw*3600U/ENCODER_SIX_TURN),1,g_backend_encoder.feedback_seen};
    }
#endif
    if (hw.running && run.valid != 0)
    {
        int32_t current = (int32_t)((int64_t)run.last_delta * g_backend_vdda * 1000 / (4095U * g_backend_gain * g_backend_shunt));
        s.values[CONTROL_IA + run.last_low] = (control_value_t){current, 3, true};
    }
    if (hw.running)
    {
        /* 指令在尚无有效电流样本时也有意义，不能把定位渐增显示为保存的满幅。 */
        s.values[CONTROL_COMM_HZ] = (control_value_t){(int32_t)run.frequency_mhz, 3, true};
        s.values[CONTROL_DUTY] = (control_value_t){(int32_t)run.duty, 1, true};
    }
    const char *startup = !hw.running ? "STOPPED" : run.step_mode == 2U ? "STEP HOLD" :
        run.step_mode == 3U ? (run.reverse ? "STEP REV" : "STEP FWD") :
        run.step_mode==4U ? "ENCODER" :
        run.sensorless ? sensorless_six_step_stage_name(run.sensorless_stage) : startup_stage_name(run.stage);
    snprintf(s.startup, sizeof(s.startup), "%s", startup);
    uint8_t high, low;
    six_step_phases((uint8_t)run.sector, &high, &low);
    snprintf(s.commutation, sizeof(s.commutation), "S%lu %c+ %c- %cZ", (unsigned long)run.sector + 1U,
        (int)('A' + high), (int)('A' + low), (int)('A' + 3U - high - low));
    snprintf(s.fault, sizeof(s.fault), "%s", hw.fault);
    if (fault) { snprintf(g_backend_last_fault, sizeof(g_backend_last_fault), "%s", hw.fault); }
    snprintf(s.last_fault, sizeof(s.last_fault), "%s", g_backend_last_fault);
    (void)control_model_publish(&s);
}

/** @brief 见motor_backend.h；不执行启动、停机或参数写入。 */
void
motor_backend_foc_motion(char *out,size_t size,uint32_t first)
{
#if MOTOR_FOC_ENABLED
    board_foc_motion(out,size,first);
#else
    (void)first;
    snprintf(out,size,"ERR FOC DISABLED");
#endif
}

/** @brief 主循环报告本次运行的更新/峰顶次数和母线数据年龄。 @param out 输出。 @param size 容量。 */
void MOTOR_BACKGROUND_CODE
motor_backend_diagnostics(char *out, size_t size)
{
#if MOTOR_FOC_ENABLED
    if(board_foc_active()) { board_foc_diagnostics(out,size,false); return; }
#endif
    board_motor_status_t s = board_motor_status();
    snprintf(out, size, "OK MOTOR REV=%04X FAST=%lu SLOW=%lu SKIP=%lu ISR_CYCLES=%lu BUS_MV=%u VDDA_MV=%u ZERO=%u,%u,%u SHUNT_MOHM=%ld TRIP_MA=%u SAVE=%s DRV_LAST_OK=%u REG2_6=%03X,%03X,%03X,%03X,%03X UP=%lu PEAK=%lu BUS_AGE_MS=%lu PWM_HZ=%u CTRL_HZ=%u FAULT=%s",
        s.revision, (unsigned long)s.fast_count, (unsigned long)s.regular_count, (unsigned long)s.regular_skipped,
        (unsigned long)s.irq_max_cycles, s.bus_mv, s.vdda_mv, s.zero[0], s.zero[1], s.zero[2],
        (long)settings_get()->parameter[CONTROL_PARAM_SHUNT], MOTOR_TRIP_MA, settings_status(), s.driver_verified ? 1U : 0U,
        s.driver_registers[0], s.driver_registers[1], s.driver_registers[2], s.driver_registers[3], s.driver_registers[4],
        (unsigned long)s.update_count, (unsigned long)s.peak_count, (unsigned long)s.bus_age_ms,
        MOTOR_PWM_HZ, MOTOR_CONTROL_HZ, s.fault);
}

/** @brief 启动独立的功率时序测试，保持原六步保存参数。 */
bool
motor_backend_foc_bench(uint32_t duty, uint32_t duration_ms)
{
#if MOTOR_FOC_ENABLED
    if (settings_get()->parameter[CONTROL_PARAM_SHUNT] != 50 ||
        settings_get()->source != CONTROL_REMOTE) { return false; }
    return board_foc_bench_start(duty,duration_ms);
#else
    (void)duty;
    (void)duration_ms;
    return false;
#endif
}

/** @brief 独立静态矢量检查，不使用保存的速度、启动或电流环参数。 */
bool
motor_backend_foc_vector(uint32_t sector, uint32_t spread)
{
#if MOTOR_FOC_ENABLED
    if (settings_get()->parameter[CONTROL_PARAM_SHUNT] != 50 ||
        settings_get()->source != CONTROL_REMOTE) { return false; }
    return board_foc_vector_start(sector,spread);
#else
    (void)sector;
    (void)spread;
    return false;
#endif
}

/** @brief 独立高占空比采样扫描，保留当前设置。 */
bool
motor_backend_foc_sample(uint32_t trigger)
{
#if MOTOR_FOC_ENABLED
    if (settings_get()->parameter[CONTROL_PARAM_SHUNT] != 50 ||
        settings_get()->source != CONTROL_REMOTE) { return false; }
    return board_foc_sample_start(trigger);
#else
    (void)trigger;
    return false;
#endif
}

/** @brief 独立动态提交测试，保持原六步保存值；契约见头文件。 */
bool
motor_backend_foc_switch(uint32_t spread, uint32_t duration_ms)
{
#if MOTOR_FOC_ENABLED
    if (settings_get()->parameter[CONTROL_PARAM_SHUNT] != 50 ||
        settings_get()->source != CONTROL_REMOTE) { return false; }
    return board_foc_switch_start(spread,duration_ms);
#else
    (void)spread;
    (void)duration_ms;
    return false;
#endif
}

/** @brief 主循环转发状态及区间电压，不保存电流诊断历史。 */
bool
motor_backend_foc_current_start(bool q_axis, uint32_t target_ma, uint32_t duration_ms)
{
#if MOTOR_FOC_ENABLED
    if (settings_get()->parameter[CONTROL_PARAM_SHUNT] != 50 ||
        settings_get()->source != CONTROL_REMOTE) { return false; }
    return board_foc_current_start(q_axis,target_ma,duration_ms);
#else
    (void)q_axis;
    (void)target_ma;
    (void)duration_ms;
    return false;
#endif
}

/** @brief 有界I/f试验，保留独立六步参数。 */
bool
motor_backend_foc_if(uint32_t frequency_millihz, uint32_t duration_ms)
{
#if MOTOR_FOC_ENABLED
    if (settings_get()->parameter[CONTROL_PARAM_SHUNT] != 50 ||
        settings_get()->source != CONTROL_REMOTE) { return false; }
    return board_foc_if_start(frequency_millihz,duration_ms);
#else
    (void)frequency_millihz;
    (void)duration_ms;
    return false;
#endif
}

/** @brief 低电流模型范围内独立观察SMO，保留原有六步参数。 */
bool
motor_backend_foc_observe(uint32_t frequency_millihz,uint32_t duration_ms)
{
#if MOTOR_FOC_ENABLED
    if (settings_get()->parameter[CONTROL_PARAM_SHUNT] != 50 ||
        settings_get()->source != CONTROL_REMOTE) { return false; }
    return board_foc_observe_start(frequency_millihz,duration_ms);
#else
    (void)frequency_millihz;
    (void)duration_ms;
    return false;
#endif
}

/** @brief 独立有界量程试验，不改变保存配置。 */
bool
motor_backend_foc_sample_vector(uint32_t trigger,uint32_t sector)
{
#if MOTOR_FOC_ENABLED
    if(settings_get()->parameter[CONTROL_PARAM_SHUNT]!=50 || settings_get()->source!=CONTROL_REMOTE) { return false; }
    return board_foc_sample_vector_start(trigger,sector);
#else
    (void)trigger;
    (void)sector;
    return false;
#endif
}

/** @brief 独立有界量程试验，不改变保存配置。 */
bool
motor_backend_foc_range(uint32_t angle_slot,uint32_t target_ma)
{
#if MOTOR_FOC_ENABLED
    if(settings_get()->parameter[CONTROL_PARAM_SHUNT]!=50 || settings_get()->source!=CONTROL_REMOTE) { return false; }
    return board_foc_range_start(angle_slot,target_ma);
#else
    (void)angle_slot;
    (void)target_ma;
    return false;
#endif
}

/** @brief 有界完整接管试验，不改变保存配置。 */
bool
motor_backend_foc_closed(uint32_t target_rpm,uint32_t duration_ms)
{
#if MOTOR_FOC_ENABLED
    if(settings_get()->parameter[CONTROL_PARAM_SHUNT]!=50 || settings_get()->source!=CONTROL_REMOTE) { return false; }
    return board_foc_closed_start(target_rpm,duration_ms);
#else
    (void)target_rpm;
    (void)duration_ms;
    return false;
#endif
}

/** @brief 主循环转发真实阶段诊断，禁止ISR调用。 */
void
motor_backend_foc_diagnostics(char *out, size_t size, int page)
{
#if MOTOR_FOC_ENABLED
    if (page == -2 || page == -1) { board_foc_diagnostics(out,size,page == -1); }
    else if (page == -3) { board_foc_current_diagnostics(out,size); }
    else if (page == -4) { board_foc_observer_diagnostics(out,size); }
    else if (page == -5) { board_foc_drive_diagnostics(out,size); }
    else if (page == -7) { board_foc_confidence_diagnostics(out,size); }
    else if (page == -8) { board_foc_harmonic_diagnostics(out,size); }
    else if (page == -9) { board_foc_harmonic2_diagnostics(out,size); }
    else if(page==-6)
    {
        bool off=board_motor_storage_safe();
        bool still=off&&board_motor_standstill();
        board_motor_status_t s=board_motor_status();
        snprintf(out,size,"OK MOTOR FOC REST OUTPUT_DISABLED=%u STILL=%u PHASE_MV=%u,%u,%u SCANS=%lu LINE_PTP_MV=%lu RAW_PTP_MV=%lu BEMF_PROFILE=%u REGULAR=%lu MEASURED_MS=%lu FAULT=%s",
            off?1U:0U,still?1U:0U,s.phase_mv[0],s.phase_mv[1],s.phase_mv[2],
            (unsigned long)s.rest_scans,(unsigned long)s.rest_line_ptp_mv,(unsigned long)s.rest_line_raw_ptp_mv,s.bemf_enabled?1U:0U,
            (unsigned long)s.regular_count,(unsigned long)s.slow_ms,s.fault);
    }
    else { snprintf(out,size,"ERR FOC ARGUMENT"); }
#else
    (void)page;
    snprintf(out,size,"ERR FOC UNAVAILABLE");
#endif
}

/** @brief 见motor_backend.h；空闲主循环转发所选格式，未启用FOC时不产生输出。 */
bool
motor_backend_foc_current(char *out, size_t size, bool firewater)
{
#if MOTOR_FOC_ENABLED
    return board_foc_current(out,size,firewater);
#else
    (void)out;
    (void)size;
    (void)firewater;
    return false;
#endif
}

/** @brief 见motor_backend.h；基准日志仍只在主循环格式化。 */
bool
motor_backend_foc_trace(char *out,size_t size)
{
#if MOTOR_FOC_ENABLED
    return board_foc_trace(out,size);
#else
    (void)out;(void)size;
    return false;
#endif
}

/** @brief 见motor_backend.h；仅REMOTE正式FOC运行接受本轮固定Iq。 */
bool
motor_backend_foc_hold_iq(uint32_t current_ma)
{
#if MOTOR_FOC_ENABLED
    return settings_get()->source==CONTROL_REMOTE && board_foc_hold_iq(current_ma);
#else
    (void)current_ma;
    return false;
#endif
}

/** @brief 只允许REMOTE所选FOC算法的会话切换。
 * @param second 是否二次，否则五次。 @param enabled 启用请求。
 * @return 后端与板卡阶段允许并已设置。 */
static bool
backend_foc_harmonic(bool second,bool enabled)
{
#if MOTOR_FOC_ENABLED
    return settings_get()->source==CONTROL_REMOTE && control_model_algorithm()==CONTROL_SENSORLESS_FOC &&
        (second?board_foc_harmonic2(enabled):board_foc_harmonic(enabled));
#else
    (void)second;
    (void)enabled;
    return false;
#endif
}

/** @brief 见motor_backend.h；五次请求保持原接口。 */
bool
motor_backend_foc_harmonic(bool enabled)
{
    return backend_foc_harmonic(false,enabled);
}

/** @brief 见motor_backend.h；二次独立开关。 */
bool
motor_backend_foc_harmonic2(bool enabled)
{
    return backend_foc_harmonic(true,enabled);
}

/** @brief 见motor_backend.h；零点配置只在REMOTE且50mΩ时开始。 */
bool
motor_backend_foc_zero(uint32_t duration_ms)
{
#if MOTOR_FOC_ENABLED
    return settings_get()->source==CONTROL_REMOTE && settings_get()->parameter[CONTROL_PARAM_SHUNT]==50 &&
        board_foc_zero_start(duration_ms);
#else
    (void)duration_ms;
    return false;
#endif
}

/** @brief 见motor_backend.h；仅远程且安全停机允许改变接管试验速度。 */
bool
motor_backend_foc_handoff(uint32_t rpm)
{
#if MOTOR_FOC_ENABLED
    return settings_get()->source==CONTROL_REMOTE && board_foc_test_handoff(rpm);
#else
    (void)rpm;
    return false;
#endif
}

/** @brief 见motor_backend.h；查询不触发试验或修改保存参数。 */
uint32_t
motor_backend_foc_handoff_rpm(void)
{
#if MOTOR_FOC_ENABLED
    return board_foc_test_handoff_rpm();
#else
    return 0;
#endif
}

/** @brief 见motor_backend.h；采样位置不写Flash。 */
bool
motor_backend_foc_trigger(uint32_t trigger)
{
#if MOTOR_FOC_ENABLED
    return settings_get()->source==CONTROL_REMOTE && board_foc_test_trigger(trigger);
#else
    (void)trigger;
    return false;
#endif
}

/** @brief 见motor_backend.h；仅查询会话值。 */
uint32_t
motor_backend_foc_trigger_ticks(void)
{
#if MOTOR_FOC_ENABLED
    return board_foc_test_trigger_ticks();
#else
    return 0;
#endif
}

/** @brief 输出一次 RUN 的一致原始统计，参见 motor_backend_run_diagnostics；这里只在主循环换算电流。 */
void MOTOR_BACKGROUND_CODE
motor_backend_run_diagnostics(char *out, size_t size)
{
    motor_run_stats_t run = run_snapshot();
    char peaks[3][12], last[64];
    for (unsigned phase = 0; phase < 3; ++phase)
    {
        if (run.phase_samples[phase] == 0)
        {
            snprintf(peaks[phase], sizeof(peaks[phase]), "NA");
        }
        else
        {
            uint32_t ma = (uint32_t)((uint64_t)run.peak_delta[phase] * run.vdda_mv * 1000U /
                (4095U * run.gain * run.shunt_mohm));
            snprintf(peaks[phase], sizeof(peaks[phase]), "%lu", (unsigned long)ma);
        }
    }
    if (run.valid == 0)
    {
        snprintf(last, sizeof(last), "LOW=NA RAW=NA DELTA=NA");
    }
    else
    {
        snprintf(last, sizeof(last), "LOW=%c RAW=%lu DELTA=%ld", (int)('A' + run.last_low),
            (unsigned long)run.last_raw, (long)run.last_delta);
    }
    const char *stage = run.step_mode == 0U ? "NONE" : run.step_mode == 2U ? "HOLD" :
        run.step_mode == 3U ? "CYCLE" : run.sensorless ?
        sensorless_six_step_stage_name(run.sensorless_stage) : startup_stage_name(run.stage);
    snprintf(out, size, "OK MOTOR RUN TICKS=%lu VALID=%lu INVALID=%lu LIMIT_HITS=%lu DENSITY_SKIP=%lu PHASE_SAMPLES=%lu,%lu,%lu PEAK_MA=%s,%s,%s %s SECTOR=%lu HZ=%lu.%03lu DUTY=%lu.%lu STAGE=%s",
        (unsigned long)run.ticks, (unsigned long)run.valid, (unsigned long)run.invalid,
        (unsigned long)run.limit_hits, (unsigned long)run.density_skip,
        (unsigned long)run.phase_samples[0], (unsigned long)run.phase_samples[1], (unsigned long)run.phase_samples[2],
        peaks[0], peaks[1], peaks[2], last, (unsigned long)run.sector + 1U,
        (unsigned long)run.frequency_mhz / 1000U, (unsigned long)run.frequency_mhz % 1000U,
        (unsigned long)run.duty / 10U, (unsigned long)run.duty % 10U, stage);
}

/** @brief 格式化无感运行的一致观测快照，未获资格的过阈事件不冒充有效估速。 */
void
motor_backend_bemf_diagnostics(char *out, size_t size)
{
    motor_run_stats_t run = run_snapshot();
    board_motor_status_t hardware = board_motor_status();
    bool active = run.sensorless && hardware.running;
    const char *state = !run.sensorless ? "NONE" : !hardware.running ?
        strcmp(hardware.fault, "NONE") == 0 ? "STOPPED" : "FAULT" :
        sensorless_six_step_stage_name(run.sensorless_stage);
    char rpm[12];
    if (active && run.estimated_valid) { snprintf(rpm, sizeof(rpm), "%lu", (unsigned long)run.estimated_rpm); }
    else { snprintf(rpm, sizeof(rpm), "NA"); }
    snprintf(out, size, "OK MOTOR BEMF MODE=%s STATE=%s QUALIFIED=%u LOCKED=%u EST_RPM=%s EVENTS=%lu CONSEC=%lu DROP=%lu,%lu,%lu,%lu,%lu RAW=%u FLOAT=%c SECTOR=%u VALID=%u SAMPLE_US=%lu AGE_US=%lu T60_US=%lu PHASE_ERR_US=%ld NEXT_US=%lu",
        !run.sensorless ? "NONE" : MOTOR_SENSORLESS_CLOSED_LOOP ? "CLOSED_LOOP" : "OBSERVE_ONLY",
        state,
        active && run.qualified ? 1U : 0U, active && run.locked ? 1U : 0U, rpm,
        (unsigned long)run.events, (unsigned long)run.consecutive, (unsigned long)run.rejected,
        (unsigned long)run.invalid_bemf, (unsigned long)run.blanked, (unsigned long)run.duplicates,
        (unsigned long)run.late, run.bemf_raw, (int)('A' + run.floating), (unsigned)run.sampled_sector + 1U,
        active && run.bemf_valid ? 1U : 0U, (unsigned long)run.sample_us, (unsigned long)run.sample_age_us,
        (unsigned long)run.sector_period_us, (long)run.phase_error_us, (unsigned long)run.next_commutation_us);
}

/** @brief 分路径时序诊断，独立短响应避免MOTOR超出串口容量。 */
void MOTOR_BACKGROUND_CODE
motor_backend_timing_diagnostics(char *out, size_t size)
{
    board_motor_status_t s = board_motor_status();
    snprintf(out, size, "OK MOTOR TIMING ADC=%lu CURRENT=%lu BEMF=%lu TIM=%lu TIM_TOTAL=%lu SPEED=%lu SAMPLE=%lu EVENT=%lu PI=%lu AGE_MAX_US=%lu SPEED_HZ=%u PI_TICKS=%lu LOCK_SEEN=%u COM_CONFIRMED=%lu FIRST_LOCK_US=%lu HCLK_HZ=170000000",
        (unsigned long)s.irq_max_cycles, (unsigned long)s.current_max_cycles, (unsigned long)s.bemf_max_cycles,
        (unsigned long)s.peak_max_cycles, (unsigned long)s.tim_total_max_cycles, (unsigned long)s.speed_max_cycles,
        (unsigned long)g_backend_sample_max_cycles, (unsigned long)g_backend_event_max_cycles, (unsigned long)g_backend_pi_max_cycles,
        (unsigned long)s.sample_age_max_us, MOTOR_SPEED_LOOP_HZ, (unsigned long)g_backend_pi_updates,
        g_backend_lock_seen ? 1U : 0U, (unsigned long)g_backend_confirmed_com, (unsigned long)g_backend_first_lock_us);
}

/** @brief 停机导出原始快照，每页最多4项，不参与实时控制。
 * @param out 响应缓冲。 @param size 容量。 @param page 从0起的页。
 * @param events true导出事件记录，false导出原始样本。 */
void
motor_backend_bemf_trace_diagnostics(char *out, size_t size, unsigned page, bool events)
{
#if MOTOR_SENSORLESS_TRACE
    if (board_motor_running()) { snprintf(out, size, "ERR RUNNING STOP TO READ"); return; }
    uint32_t written = events ? g_backend_event_trace_written : g_backend_trace_written;
    uint32_t capacity = events ? 64U : 1024U;
    uint32_t count = written < capacity ? written : capacity;
    uint32_t begin = page * 4U;
    if (begin > count) { snprintf(out, size, "ERR MOTOR RANGE"); return; }
    size_t used = (size_t)snprintf(out, size, "OK MOTOR BEMF %s TOTAL=%lu PAGE=%u DATA=",
        events ? "EVENTS" : "TRACE", (unsigned long)count, page);
    for (uint32_t i=begin; i<count && i<begin+4U && used<size; ++i)
    {
        uint32_t index = ((written-count)+i) & (capacity-1U);
        int length;
        if (events)
        {
            const bemf_trace_event_t *r = &g_backend_event_trace[index];
            length = snprintf(out+used, size-used, "%s%lu:%lu:%lu:%lu:%ld:%u:%u:%u",
                i==begin ? "" : ",", (unsigned long)r->sample_us, (unsigned long)r->committed_us,
                (unsigned long)r->crossing_us, (unsigned long)r->period_us, (long)r->phase_error_us,
                r->raw, r->sector+1U, r->flags);
        }
        else
        {
            const bemf_trace_sample_t *r = &g_backend_trace[index];
            length = snprintf(out+used, size-used, "%s%lu:%lu:%u:%u:%u", i==begin ? "" : ",",
                (unsigned long)r->sample_us, (unsigned long)r->committed_us, r->raw, r->sector+1U, r->flags);
        }
        if (length < 0) { return; }
        used += (size_t)length;
    }
#else
    (void)page;
    (void)events;
    snprintf(out, size, "ERR BEMF CAPTURE DISABLED");
#endif
}

/** @brief 输出八个相位箱的均值/最小/最大/数量，保留未建立闭环的数据。 */
void
motor_backend_bemf_sector_diagnostics(char *out, size_t size, unsigned sector, unsigned page)
{
    (void)sector;
    (void)page;
    snprintf(out, size, "ERR BEMF CAPTURE DISABLED");
}

/** @brief 输出独立测试的一致指令快照，完整接口见 motor_backend_step_diagnostics。 */
void
motor_backend_step_diagnostics(char *out, size_t size)
{
    motor_run_stats_t run = run_snapshot();
    board_motor_status_t hw = board_motor_status();
    const char *modes[] = {"NONE", "NORMAL", "HOLD", "CYCLE"};
    uint8_t high, low;
    six_step_phases((uint8_t)run.sector, &high, &low);
    const char *state = hw.running ? "RUNNING" : strcmp(hw.fault, "NONE") == 0 ? "STOPPED" : "FAULT";
    snprintf(out, size, "OK STEP MODE=%s STATE=%s DIRECTION=%s SECTOR=%lu HIGH=%c LOW=%c FLOAT=%c DWELL_MS=%lu ELAPSED_MS=%lu COMPLETED=%lu/%u DUTY=%lu.%lu FAULT=%s",
        modes[run.step_mode], state, run.reverse ? "REV" : "FWD", (unsigned long)run.sector + 1U,
        (int)('A' + high), (int)('A' + low), (int)('A' + 3U - high - low),
        (unsigned long)run.dwell_ms, (unsigned long)run.ticks / (MOTOR_CONTROL_HZ / 1000U),
        (unsigned long)run.completed, run.step_mode == 3U ? 6U : run.step_mode == 2U ? 1U : 0U,
        (unsigned long)run.duty / 10U, (unsigned long)run.duty % 10U, hw.fault);
}

/** @brief 将存储状态格式化为独立短响应，避免扩展 MOTOR 超出发送容量。 @param out 输出。 @param size 容量。 */
void MOTOR_BACKGROUND_CODE
motor_backend_storage_diagnostics(char *out, size_t size)
{
    board_storage_status_t s = board_storage_status();
    snprintf(out, size, "OK STORAGE SAVE=%s RESULT=%s ACTUAL_DBANK=%u BUILD_DBANK=%u MATCH=%u FLASH_KB=%u OPTR=%08lX HAL_ERROR=%08lX ADDRESS=%08lX PAGE_ERROR=%08lX",
        settings_status(), s.result, s.actual_dual_bank ? 1U : 0U, s.build_dual_bank ? 1U : 0U,
        s.actual_dual_bank == s.build_dual_bank ? 1U : 0U, s.flash_kb,
        (unsigned long)s.optr, (unsigned long)s.hal_error, (unsigned long)s.address, (unsigned long)s.page_error);
}

/** @brief 见motor_backend.h。 */
void MOTOR_BACKGROUND_CODE
motor_backend_foc_position(char *out,size_t size)
{
#if MOTOR_ENCODER_ENABLED && MOTOR_FOC_ENABLED
    board_foc_position(out,size,control_model_mode()==CONTROL_POSITION?settings_value(CONTROL_PARAM_TARGET):0,
        settings_get()->position.tolerance_mdeg);
#else
    snprintf(out,size,"ERR FOC DISABLED");
#endif
}

/** @brief 见motor_backend.h。 */
const char *MOTOR_BACKGROUND_CODE
motor_backend_position_zero(void)
{
#if MOTOR_ENCODER_ENABLED && MOTOR_FOC_ENABLED
    if(control_model_algorithm()!=CONTROL_ENCODER_FOC || control_model_mode()!=CONTROL_POSITION ||
        settings_get()->source!=CONTROL_REMOTE) { return "POSITION MODE/REMOTE"; }
    if(board_motor_status().running || !board_motor_storage_safe()) { return "POWER ACTIVE"; }
    if(!encoder_calibration_valid()) { return "ENCODER CAL REQUIRED"; }
    encoder_snapshot_t feedback=board_encoder_snapshot(g_backend_encoder_now);
    if(!feedback.angle_valid || !feedback.speed_valid || feedback.rpm<-10 || feedback.rpm>10 || !board_motor_standstill())
    { return "ENCODER NOT STILL"; }
    if(!board_encoder_position_zero((int8_t)settings_get()->parameter[CONTROL_PARAM_ENCODER_DIR])) { return "POSITION DATA"; }
    settings_position_reset_target();
    settings_publish();
    return NULL;
#else
    return "DISABLED";
#endif
}

/** @brief 见motor_backend.h。 */
const char *MOTOR_BACKGROUND_CODE
motor_backend_position_tolerance(int32_t mdeg)
{
#if MOTOR_ENCODER_ENABLED && MOTOR_FOC_ENABLED
    if(board_motor_status().running || !board_motor_storage_safe()) { return "POWER ACTIVE"; }
    if(control_model_algorithm()!=CONTROL_ENCODER_FOC || control_model_mode()!=CONTROL_POSITION || settings_get()->source!=CONTROL_REMOTE)
    { return "POSITION MODE/REMOTE"; }
    return settings_position_tolerance(mdeg,g_backend_encoder_now)?NULL:"RANGE";
#else
    (void)mdeg; return "DISABLED";
#endif
}

/** @brief 见motor_backend.h；校验和状态互锁先于RAM及ISR发布。 */
const char *MOTOR_BACKGROUND_CODE
motor_backend_haptic_configure(const encoder_haptic_config_t *config)
{
#if MOTOR_ENCODER_ENABLED && MOTOR_FOC_ENABLED
    if(!encoder_haptic_valid(config)) { return "RANGE"; }
    if(control_model_algorithm()!=CONTROL_ENCODER_FOC || control_model_mode()!=CONTROL_HAPTIC ||
        settings_get()->source!=CONTROL_REMOTE) { return "HAPTIC MODE/REMOTE"; }
    board_motor_status_t hw=board_motor_status();
    if(strcmp(hw.fault,"NONE")!=0) { return hw.fault; }
    /* 六步状态副本不包含独立FOC运行标志；按实际FOC占用发布，避免只改RAM。 */
    if(board_foc_active())
    {
        if(!board_foc_haptic_configure(config)) { return "POWER ACTIVE"; }
    }
    else if(!board_motor_storage_safe()) { return "POWER ACTIVE"; }
    (void)settings_haptic_configure(config,g_backend_encoder_now);
    return NULL;
#else
    (void)config; return "DISABLED";
#endif
}

/** @brief 见motor_backend.h；配置来自已确认RAM，动态状态由板卡短快照读取。 */
void MOTOR_BACKGROUND_CODE
motor_backend_haptic(char *out,size_t size)
{
#if MOTOR_ENCODER_ENABLED && MOTOR_FOC_ENABLED
    const settings_haptic_t *h=&settings_get()->haptic;
    board_foc_haptic(out,size,&h->config,h->current_limit_ma,h->run_ms);
#else
    snprintf(out,size,"ERR FOC DISABLED");
#endif
}
