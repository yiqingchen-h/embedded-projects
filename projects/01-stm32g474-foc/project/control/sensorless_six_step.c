#include "sensorless_six_step.h"
#include "motor_config.h"
#include <string.h>

/* 精确分频的 1 kHz 速度环与控制周期；时间判断均使用有符号差值处理回绕。 */
#define SPEED_LOOP_DIVIDER (MOTOR_CONTROL_HZ / 1000U)
#define CONTROL_PERIOD_US (1000000U / MOTOR_CONTROL_HZ)
/* PI 累加精度为百万分之一千分比，避免低增益的每毫秒积分被舍弃。 */
#define DUTY_MICRO 1000000LL
/* 每毫秒最多改变一个占空比千分点，动态边界同时用于积分抗饱和。 */
#define SPEED_LOOP_MAX_STEP 1U
/* 12位ADC满量程；未经HAL或板卡依赖的样本上下文必须保持同一量程。 */
#define SAMPLE_MAX_RAW 4095U
_Static_assert(MOTOR_CONTROL_HZ % 1000U == 0U, "Sensorless speed loop divider");
_Static_assert(1000000U % MOTOR_CONTROL_HZ == 0U, "Sensorless control period");

/** @brief 对宽乘积优先使用硬件32位除法，超出32位时保留原64位精确结果。
 * @param numerator 非负分子，可由两个32位量相乘得到。
 * @param denominator 非零分母，已由配置或过零有效性保证。
 * @return 商的低32位，与原uint64除法后转换一致；不改变截断或取整规则。 */
static uint32_t
divide_wide(uint64_t numerator, uint64_t denominator)
{
    /* 20kHz事件路径中的us×ADC码、角度换算和整周估速通常均落在32位范围。
     * Cortex-M4没有64位除法指令，直接宽除法会调用耗时的软件循环。 */
    if ((numerator | denominator) <= UINT32_MAX)
    {
        return (uint32_t)numerator / (uint32_t)denominator;
    }
    return (uint32_t)(numerator / denominator);
}

/** @brief 比较同一 us 时钟中的时刻，允许自然回绕。
 * @param a 待比较时刻。 @param b 参考时刻。
 * @return a 在 b 或其之后为 true；调用者保证间隔小于 2^31。 */
static bool MOTOR_FAST_CODE
time_reached(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) >= 0;
}

/** @brief 获取缓存的开环命令或同步/锁定后估计的 60 度时间。
 * @param state 算法状态。
 * @return 周期 us；启动频率须已通过后台非零校验。 */
static uint32_t MOTOR_FAST_CODE
expected_period(const sensorless_six_step_t *state)
{
    if (state->locked || state->synchronizing)
    {
        return state->sector_period_us;
    }
    return state->open_period_us;
}

/** @brief 启动保留20%退磁消隐，闭环用10%窗口；保留静态下限和两周期确认空间。
 * @param state 当前状态，静态范围已由后台校验。 */
static void MOTOR_FAST_CODE
update_blanking(sensorless_six_step_t *state)
{
    uint32_t period = expected_period(state);
    /* 本板负载阶跃记录中，32码上升穿越提前至620us，旧755us消隐完全屏蔽事件。
     * 实际COM确认后缩短动态部分，仍保留150us静态退磁下限、回差及两样本确认。
     * 启动与首次同步维持原20%窗口，不能据此让未同步转子更容易获得资格。 */
    uint32_t blank = period / (state->locked ? 10U : 5U);
    if (state->locked || state->synchronizing)
    {
        /* 延后换相会使过零更靠近扇区开始，消隐不能掩盖该合法边沿。 */
        uint32_t crossing = period * (uint32_t)(300 + state->config.comm_timing_decideg) / 600U;
        uint32_t maximum = crossing > 2U * CONTROL_PERIOD_US ? crossing - 2U * CONTROL_PERIOD_US : 0U;
    if (blank > maximum) { blank = maximum; }
    }
    state->blanking_us = blank > state->config.blank_us ? blank : state->config.blank_us;
    state->gap_us = period / 2U;
    if (state->gap_us < 4U * CONTROL_PERIOD_US) { state->gap_us = 4U * CONTROL_PERIOD_US; }
}

/** @brief 清除过零检测器，不能跨无效样本或不同扇区拼接过阈。
 * @param state 算法状态。 */
static void MOTOR_FAST_CODE
reset_detector(sensorless_six_step_t *state)
{
    state->armed = false;
    state->crossing_valid = false;
    state->have_previous = false;
    state->after_samples = 0U;
}

/** @brief 失去连续资格时撤销有效位及整周历史，保留累计诊断计数。
 * @param state 算法状态。 */
static void MOTOR_FAST_CODE
reset_quality(sensorless_six_step_t *state)
{
    state->qualified = false;
    state->estimate_valid = false;
    state->consecutive_events = 0U;
    state->have_event = false;
    state->history_index = 0U;
    state->history_count = 0U;
    reset_detector(state);
}

/** @brief 结束算法并请求后端沿公共保护路径撤销输出。
 * @param state 算法状态。
 * @param reason 静态故障文本；NULL 表示正常总时限截止。 */
static void MOTOR_FAST_CODE
stop_algorithm(sensorless_six_step_t *state, const char *reason)
{
    state->running = false;
    state->locked = false;
    state->synchronizing = false;
    state->qualified = false;
    state->estimate_valid = false;
    state->deadline_valid = false;
    state->fault = reason;
    state->stage = NULL == reason ? SENSORLESS_SIX_STEP_STOPPED : SENSORLESS_SIX_STEP_FAULT;
}

/** @brief 记录拒绝事件；已经接管时不以开环代替丢失的转子反馈。
 * @param state 算法状态。
 * @param reason 锁定期间应上报的静态故障。 */
static void MOTOR_FAST_CODE
reject_event(sensorless_six_step_t *state, const char *reason)
{
    ++state->rejected_count;
    if (state->locked || state->synchronizing)
    {
        stop_algorithm(state, reason);
    }
    else
    {
        reset_quality(state);
    }
}

/** @brief 由补偿后的过零事件建立下一次 30 度换相截止时刻。
 * @param state 算法状态。
 * @param event_us 已扣除方向延迟的事件时刻。 */
static void MOTOR_FAST_CODE
set_deadline(sensorless_six_step_t *state, uint32_t event_us)
{
    uint32_t angle = (uint32_t)(300 - state->config.comm_timing_decideg);
    uint32_t delay = state->config.irq_control ? state->sector_period_us * angle / 600U :
        divide_wide((uint64_t)state->sector_period_us * angle, 600U);
    state->next_commutation_us = event_us + delay;
    if (state->locked)
    {
        /* 过零含交替方向偏置和PWM纹波；每次只校正1/4相位误差，避免COM逐次追随抖动。
         * 尚未实际接管的首次COM仍使用完整过零后的30度延时。 */
        /* 峰顶预约向上取整，扣半PWM使平均量化偏差为零，避免慢校相累积偏晚。 */
        state->next_commutation_us -= (uint32_t)(state->phase_error_us * 3 / 4) + CONTROL_PERIOD_US / 2U;
    }
    state->deadline_valid = true;
}

/** @brief 计算机械速度误差对应的比例项，保留定点精度。
 * @param state 算法状态。 @param target_rpm 机械目标 RPM。
 * @return 比例输出，百万分之一占空比千分比。 */
static int64_t
proportional(const sensorless_six_step_t *state, uint32_t target_rpm)
{
    int64_t error = (int64_t)target_rpm - state->estimated_rpm;
    return error * state->config.kp_milli * 1000LL;
}

/** @brief 检查连续事件的相序、退磁窗口与周期，再生成整电周期估速及同步预约。
 * @param state 算法状态。 @param input 当前样本上下文。
 * @param event_us 已扣除方向延迟的事件时刻。 */
static void MOTOR_FAST_CODE
accept_crossing(sensorless_six_step_t *state, const sensorless_six_step_input_t *input, uint32_t event_us)
{
    uint32_t period = expected_period(state);
    int32_t elapsed = (int32_t)(event_us - input->committed_us);
    /* 相位以实际COM和补偿角为参考；接管后用于缓慢校相，不重复检查中点窄窗。 */
    int32_t angle = state->locked ? 300 + state->config.comm_timing_decideg : 300;
    int32_t center = (int32_t)(state->config.irq_control ? period * (uint32_t)angle / 600U :
        divide_wide((uint64_t)period * (uint32_t)angle, 600U));
    state->phase_error_us = elapsed - center;
    /* 启动资格仍检查相邻周期±25%，另计一个PWM的离散采样误差。 */
    uint32_t timing_margin = period / 4U + CONTROL_PERIOD_US;
    /* 检测器已按采样时刻消隐；扣除方向滤波延迟后不重复用原消隐时刻拒绝事件。 */
    if (elapsed < 0 ||
        elapsed >= (int32_t)(period - 2U * CONTROL_PERIOD_US))
    {
        reject_event(state, "BEMF PHASE");
        return;
    }
    if (state->have_event)
    {
        uint32_t interval = event_us - state->last_event_us;
        if (input->sector != (uint8_t)((state->last_event_sector + 1U) % 6U) ||
            (!state->locked && (interval < period - timing_margin || interval > period + timing_margin)))
        {
            reject_event(state, "BEMF PERIOD");
            return;
        }
    }
    uint32_t revolution = 0U;
    if (state->history_count == 6U)
    {
        revolution = event_us - state->event_history[state->history_index];
        if (state->locked)
        {
            /* 同相同方向跨整周比较，交替门限延迟在这里抵消；周期失真仍须失锁。
             * 相邻事件只用于启动资格，不能再把局部PWM纹波当瞬时速度突变。 */
            uint32_t expected = period * 6U;
            uint32_t margin = expected / 4U + 2U * CONTROL_PERIOD_US;
            if (revolution < expected-margin || revolution > expected+margin)
            {
                reject_event(state, "BEMF PERIOD");
                return;
            }
        }
    }
    ++state->event_count;
    ++state->consecutive_events;
    state->sector_event = true;
    state->last_event_sector = input->sector;
    state->last_event_us = event_us;
    state->have_event = true;
    if (state->history_count == 6U)
    {
        uint64_t denominator = (uint64_t)revolution * state->config.pole_pairs;
        /* 取最近整数RPM，避免电频率mHz向下量化使1000RPM边界恒为999，仍按原门槛判断。 */
        state->estimated_rpm = state->config.irq_control ?
                (60000000U + (uint32_t)denominator / 2U) / (uint32_t)denominator :
                divide_wide(60000000ULL + denominator / 2U, denominator);
        state->sector_period_us = revolution / 6U;
    }
    else
    {
        ++state->history_count;
        state->sector_period_us = period;
    }
    state->event_history[state->history_index] = event_us;
    state->history_index = (uint8_t)((state->history_index + 1U) % 6U);
    if (state->locked || state->synchronizing) { update_blanking(state); }
    uint32_t minimum = state->config.handoff_rpm > state->config.min_rpm ?
        state->config.handoff_rpm : state->config.min_rpm;
    if (!state->locked && !state->synchronizing)
    {
        /* 未接管时逐事件重算，不能沿用速度已跌破 HANDOFF 之前的旧资格。 */
        state->qualified = state->consecutive_events >= state->config.lock_count && state->estimated_rpm >= minimum;
        state->estimate_valid = state->qualified;
    }
    if (state->locked || state->synchronizing)
    {
        set_deadline(state, event_us);
    }
    else if (state->qualified && !state->config.observe_only &&
        state->open_loop.sector == input->sector && state->open_loop.stage >= SIX_STEP_RAMP)
    {
        /* 冻结开环候选，按实测过零延迟30度；实际提交确认前不能报告闭环。 */
        state->synchronizing = true;
        state->desired_sector = input->sector;
        if (!state->config.irq_control)
        {
            state->integral_micro = (int64_t)state->duty * DUTY_MICRO - proportional(state, input->target_rpm);
        }
        state->pi_ticks = 0U;
        state->limit_latched = input->current_limited;
        set_deadline(state, event_us);
    }
}

/** @brief 对同一扇区的有效样本执行方向回差检测，过阈时间采用线性插值。
 * @param state 算法状态。 @param input 当前有效且消隐结束的样本。 */
static void MOTOR_FAST_CODE
detect_crossing(sensorless_six_step_t *state, const sensorless_six_step_input_t *input)
{
    /* AB/AC/BC/BA/CA/CB 的浮空相分别为 C/B/A/C/B/A，期望边沿交替下降/上升。 */
    bool rising = state->config.irq_control ? state->detector_rising : (input->sector & 1U) != 0U;
    uint16_t threshold = rising ? state->config.rising_threshold : state->config.falling_threshold;
    uint16_t lower = state->config.irq_control ? state->detector_lower : (uint16_t)(threshold - state->config.hysteresis);
    uint16_t upper = state->config.irq_control ? state->detector_upper : (uint16_t)(threshold + state->config.hysteresis);
    bool before = rising ? input->raw <= lower : input->raw >= upper;
    bool after = rising ? input->raw >= upper : input->raw <= lower;
    if (before)
    {
        state->armed = true;
        state->crossing_valid = false;
        state->after_samples = 0U;
    }
    if (state->armed && state->have_previous)
    {
        bool crossed = rising ? state->previous_raw < threshold && input->raw >= threshold :
            state->previous_raw > threshold && input->raw <= threshold;
        if (crossed)
        {
            uint32_t magnitude = rising ? input->raw - state->previous_raw : state->previous_raw - input->raw;
            uint32_t distance = rising ? threshold - state->previous_raw : state->previous_raw - threshold;
            uint32_t delta_us = input->sample_us - state->previous_sample_us;
            state->crossing_us = state->previous_sample_us + (state->config.irq_control ?
                delta_us * distance / magnitude : divide_wide((uint64_t)delta_us * distance, magnitude));
            state->crossing_valid = true;
        }
    }
    if (state->armed && state->crossing_valid && after)
    {
        /* 只确认连续越过回差的两个有效样本，单周期尖峰不能生成电角事件。 */
        if (++state->after_samples >= 2U)
        {
            uint32_t delay = rising ? state->config.rising_delay_us : state->config.falling_delay_us;
            state->armed = false;
            state->crossing_valid = false;
            state->after_samples = 0U;
            accept_crossing(state, input, state->crossing_us - delay);
        }
    }
    else
    {
        state->after_samples = 0U;
    }
    state->previous_raw = input->raw;
    state->previous_sample_us = input->sample_us;
    state->have_previous = true;
}

/** @brief 验证样本归属，拒绝旧序号、跨扇区拼接及未经消隐的波形。
 * @param state 算法状态。 @param input 当前输入。 */
#ifdef MOTOR_SENSORLESS_TEST_TICK
static void
observe_sample(sensorless_six_step_t *state, const sensorless_six_step_input_t *input)
{
    /* 合法跳脉冲只忽略电压；有效采样间隙和缺事件按真实时间处理。 */
    if (!input->valid)
    {
        ++state->invalid_count;
    }
    if (state->have_sequence && (int32_t)(input->sequence - state->last_sequence) <= 0)
    {
        if (!input->valid) { reset_detector(state); }
        ++state->duplicate_count;
        return;
    }
    state->have_sequence = true;
    state->last_sequence = input->sequence;
    uint8_t floating = (uint8_t)(2U - input->sector % 3U);
    if (input->sector >= 6U || input->floating != floating || !time_reached(input->sample_us, input->committed_us) ||
        !time_reached(input->now_us, input->sample_us) || input->now_us - input->sample_us > 2U * CONTROL_PERIOD_US ||
        (state->have_valid_sample && !time_reached(input->sample_us, state->last_valid_sample_us + 1U)) ||
        (input->valid && input->raw > SAMPLE_MAX_RAW))
    {
        reject_event(state, "BEMF SAMPLE");
        return;
    }
    if (state->have_sector && (input->sector != state->observed_sector || input->committed_us != state->observed_committed_us))
    {
        if (!time_reached(input->committed_us, state->observed_committed_us + 1U))
        {
            reject_event(state, "BEMF SAMPLE");
            return;
        }
        if (input->sector != (uint8_t)((state->observed_sector + 1U) % 6U) || !state->sector_event)
        {
            reject_event(state, "BEMF LOST");
            if (!state->running)
            {
                return;
            }
        }
        bool controlled = state->locked || state->synchronizing;
        if (controlled && (!state->commutation_pending || input->sector != state->desired_sector))
        {
            stop_algorithm(state, "BEMF SECTOR");
            return;
        }
        if (controlled && ((int32_t)(input->committed_us - state->submitted_us) < -1 ||
            time_reached(input->committed_us, state->submitted_us + CONTROL_PERIOD_US)))
        {
            /* 后端报告真实COM时刻；延迟到后续峰顶不能冒充本次预约已提交。 */
            stop_algorithm(state, "BEMF COMMIT");
            return;
        }
        state->have_sector = false;
        state->commutation_pending = false;
    if (state->synchronizing)
        {
            /* 只有收到新扇区及预约窗口内的真实COM时刻，才确认反馈换相已接管。 */
            state->synchronizing = false;
            state->locked = true;
            update_blanking(state);
        }
    }
    if (!state->have_sector)
    {
        state->have_sector = true;
        state->observed_sector = input->sector;
        state->observed_committed_us = input->committed_us;
        state->sector_event = false;
        reset_detector(state);
    }
    if (input->valid)
    {
        state->last_valid_sample_us = input->sample_us;
        state->have_valid_sample = true;
    }
    if (input->sample_us - input->committed_us < state->blanking_us)
    {
        ++state->blanked_count;
        reset_detector(state);
        return;
    }
    if (!input->valid)
    {
        return;
    }
    if (!state->sector_event)
    {
        detect_crossing(state, input);
    }
}
#endif

#ifdef MOTOR_SENSORLESS_TEST_TICK
/** @brief 旧同步模型的主机回归适配；正式固件使用独立板卡预约。
 * @param state 已接管的状态。 @param input 当前输入。 */
static void
schedule_commutation(sensorless_six_step_t *state, const sensorless_six_step_input_t *input)
{
    if (state->commutation_pending)
    {
        if (time_reached(input->now_us, state->submitted_us + 3U * CONTROL_PERIOD_US))
        {
            stop_algorithm(state, "BEMF COMMIT");
        }
        return;
    }
    if (!state->deadline_valid)
    {
        return;
    }
    if (!time_reached(input->next_commit_us, input->now_us + 1U) ||
        time_reached(input->next_commit_us, input->now_us + CONTROL_PERIOD_US + 1U) ||
        time_reached(input->next_commit_us - CONTROL_PERIOD_US, state->next_commutation_us))
    {
        ++state->late_count;
        stop_algorithm(state, "BEMF LATE");
        return;
    }
    if (time_reached(input->next_commit_us, state->next_commutation_us))
    {
        state->desired_sector = (uint8_t)((state->observed_sector + 1U) % 6U);
        state->deadline_valid = false;
        state->commutation_pending = true;
        state->submitted_us = input->next_commit_us;
    }
}
#endif

/** @brief 执行1kHz定点速度PI；限流主动退幅，静态范围与每毫秒变化率共同阻止积分饱和。
 * @param state 已接管的状态。 @param input 当前目标及限流标志。 */
#ifdef MOTOR_SENSORLESS_TEST_TICK
static void
speed_control(sensorless_six_step_t *state, const sensorless_six_step_input_t *input)
{
    sensorless_six_step_pi_t pi = {state->integral_micro, state->duty, true};
    bool limited = state->limit_latched || input->current_limited;
    state->duty = sensorless_six_step_pi_update(&pi, &state->config, input->target_rpm,
        state->estimated_rpm, state->duty, limited);
    state->integral_micro = pi.integral_micro;
    state->limit_latched = false;
}
#endif

/** @brief 独立PI计算，接口契约见头文件；仅做定点计算，不访问观测历史。 */
uint16_t MOTOR_FAST_CODE
sensorless_six_step_pi_update(sensorless_six_step_pi_t *pi,
    const sensorless_six_step_config_t *config, uint32_t target_rpm, uint32_t rpm,
    uint16_t initial_duty, bool limited)
{
    if (!pi->initialized)
    {
        pi->duty = initial_duty;
        pi->integral_micro = (int64_t)initial_duty * DUTY_MICRO -
            ((int64_t)target_rpm - rpm) * config->kp_milli * 1000LL;
        pi->initialized = true;
    }
    uint16_t duty = pi->duty;
    int64_t p = ((int64_t)target_rpm - rpm) * config->kp_milli * 1000LL;
    int64_t error = (int64_t)target_rpm - rpm;
    int64_t increment = error * config->ki_milli;
    uint16_t lower_duty = duty > config->min_duty + SPEED_LOOP_MAX_STEP ?
        (uint16_t)(duty - SPEED_LOOP_MAX_STEP) : config->min_duty;
    uint16_t upper_duty = duty + SPEED_LOOP_MAX_STEP < config->max_duty ?
        (uint16_t)(duty + SPEED_LOOP_MAX_STEP) : config->max_duty;
    int64_t lower = (int64_t)lower_duty * DUTY_MICRO;
    int64_t upper = (int64_t)upper_duty * DUTY_MICRO;
    int64_t previous_output = pi->integral_micro + p;
    if (limited)
    {
        /* 限流时只冻结积分会维持过高PWM幅度并继续跳脉冲，阻碍BEMF观测。
         * 每毫秒回退一个千分点，并跟踪实际幅度；解除限流后不会追赶旧积分。 */
        pi->duty = lower_duty;
        pi->integral_micro = lower - p;
        return pi->duty;
    }
    {
        /* 只保留本轮仍能实现的积分增量；完全冻结跨边界增量会让小P大I永远停在第一步。 */
        if (increment > 0 && previous_output < upper)
        {
            int64_t available = upper - previous_output;
            pi->integral_micro += increment < available ? increment : available;
        }
        else if (increment < 0 && previous_output > lower)
        {
            int64_t available = lower - previous_output;
            pi->integral_micro += increment > available ? increment : available;
        }
        /* 大比例项已在边界外且积分继续加深饱和时保持积分；反向增量可立即解除饱和。 */
    }
    int64_t output = pi->integral_micro + p;
    if (output > upper)
    {
        output = upper;
    }
    else if (output < lower)
    {
        output = lower;
    }
    /* 输出已限于0..1000千分比，百万精度乘积小于2^32，可直接硬件除法。 */
    pi->duty = (uint16_t)((uint32_t)output / (uint32_t)DUTY_MICRO);
    return pi->duty;
}

/** @brief 装入启动进度，不在主循环重复计算20kHz启动积分。 */
#ifdef MOTOR_SENSORLESS_TEST_TICK
void
sensorless_six_step_progress(sensorless_six_step_t *state, const six_step_t *startup, uint32_t now_us)
{
    if (!state->running) { return; }
    state->ticks = startup->ticks;
    if (!startup->running || state->ticks >= state->config.startup.run_ticks)
    {
        stop_algorithm(state, NULL);
        return;
    }
    if (state->locked || state->synchronizing) { return; }
    state->open_loop = *startup;
    state->desired_sector = startup->sector;
    state->duty = startup->duty;
    state->stage = (sensorless_six_step_stage_t)startup->stage;
    if (state->period_frequency_mhz != startup->frequency_mhz)
    {
        state->period_frequency_mhz = startup->frequency_mhz;
        state->open_period_us = 1000000000U / (startup->frequency_mhz * 6U);
        update_blanking(state);
    }
    if (!state->acquisition_started && startup->stage >= SIX_STEP_RAMP &&
        startup->frequency_mhz >= state->acquisition_frequency_mhz)
    {
        state->acquisition_started = true;
        state->acquisition_started_us = now_us;
    }
}
#endif

/** @brief 逐样本只做窗口、回差和事件计算；启动及PI均由独立入口推进。 */
#ifdef MOTOR_SENSORLESS_TEST_TICK
void
sensorless_six_step_sample(sensorless_six_step_t *state, const sensorless_six_step_input_t *input)
{
    if (!state->running || (!state->locked && !state->synchronizing && state->open_loop.stage < SIX_STEP_PULLIN)) { return; }
    if (state->have_valid_sample && input->sample_us - state->last_valid_sample_us > state->gap_us)
    {
        if (state->locked || state->synchronizing) { stop_algorithm(state, "BEMF GAP"); return; }
        reset_quality(state);
        state->have_valid_sample = false;
    }
    observe_sample(state, input);
    if (!state->running) { return; }
    state->stage = state->locked ? SENSORLESS_SIX_STEP_CLOSED : state->synchronizing ? SENSORLESS_SIX_STEP_SYNCHRONIZE :
        state->qualified ? SENSORLESS_SIX_STEP_QUALIFIED : state->open_loop.stage == SIX_STEP_RAMP ?
        SENSORLESS_SIX_STEP_RAMP : SENSORLESS_SIX_STEP_OBSERVE;
}
#endif

/** @brief 按时间而非服务次数运行失锁检查及1kHz PI；迟到不补算多次积分。 */
#ifdef MOTOR_SENSORLESS_TEST_TICK
void
sensorless_six_step_service(sensorless_six_step_t *state, uint32_t now_us, uint32_t target_rpm, bool limited)
{
    if (!state->running) { return; }
    state->limit_latched = state->limit_latched || limited;
    if (state->commutation_pending && time_reached(now_us, state->submitted_us + 3U * CONTROL_PERIOD_US))
    {
        stop_algorithm(state, "BEMF COMMIT"); return;
    }
    if (state->have_valid_sample && now_us - state->last_valid_sample_us > state->gap_us)
    {
        if (state->locked || state->synchronizing) { stop_algorithm(state, "BEMF GAP"); return; }
        reset_quality(state); state->have_valid_sample = false;
    }
    if (state->have_event && now_us - state->last_event_us > 2U * expected_period(state))
    {
        if (state->locked || state->synchronizing) { stop_algorithm(state, "BEMF LOST"); return; }
        reset_quality(state);
    }
    if (!state->locked && !state->synchronizing && !state->config.observe_only && state->acquisition_started &&
        now_us - state->acquisition_started_us >= state->config.timeout_us)
    {
        stop_algorithm(state, "BEMF STARTUP"); return;
    }
    if (now_us - state->pi_last_us >= 1000U)
    {
        state->pi_last_us = now_us - (now_us - state->pi_last_us) % 1000U;
        if (state->locked)
        {
            sensorless_six_step_input_t input = {.target_rpm=target_rpm, .current_limited=limited};
            speed_control(state, &input);
        }
    }
}
#endif

/** @brief 事件后立即交给板卡预约远处峰顶，不等到下一PWM才发布。 */
bool MOTOR_FAST_CODE
sensorless_six_step_schedule(const sensorless_six_step_t *state, uint8_t *sector, uint32_t *deadline_us)
{
    if (!state->running || !state->deadline_valid || state->commutation_pending) { return false; }
    *sector = (uint8_t)((state->observed_sector + 1U) % 6U);
    *deadline_us = state->next_commutation_us;
    return true;
}

/** @brief 保存板卡接受的首个峰顶；LOCKED仍等待实际COM反馈。 */
void MOTOR_FAST_CODE
sensorless_six_step_schedule_ack(sensorless_six_step_t *state, uint32_t commit_us)
{
    state->desired_sector = (uint8_t)((state->observed_sector + 1U) % 6U);
    state->deadline_valid = false;
    state->commutation_pending = true;
    state->submitted_us = commit_us;
}

/** @brief ADC导入已发布启动字段，完整契约见头文件。 */
void MOTOR_FAST_CODE
sensorless_six_step_progress_irq(sensorless_six_step_t *state, uint32_t ticks,
    uint32_t frequency_mhz, uint8_t sector, uint16_t duty, six_step_stage_t stage, uint32_t now_us)
{
    state->ticks = ticks;
    if (state->locked || state->synchronizing) { return; }
    state->open_loop.ticks = ticks;
    state->open_loop.frequency_mhz = frequency_mhz;
    state->open_loop.sector = sector;
    state->open_loop.duty = duty;
    state->open_loop.stage = stage;
    state->desired_sector = sector;
    state->duty = duty;
    state->stage = (sensorless_six_step_stage_t)stage;
    if (state->period_frequency_mhz != frequency_mhz)
    {
        state->period_frequency_mhz = frequency_mhz;
        state->open_period_us = 1000000000U / (frequency_mhz * 6U);
        update_blanking(state);
    }
    if (!state->acquisition_started && stage >= SIX_STEP_RAMP &&
        frequency_mhz >= state->acquisition_frequency_mhz)
    {
        state->acquisition_started = true;
        state->acquisition_started_us = now_us;
    }
}

/** @brief 从整周期牵引开始观测；保持原消隐/资格，避免丢弃已稳定的早期过零。
 * @note 定位阶段不观测；首次同步仍要求原RAMP阶段与连续12事件/最低速度。 */
void MOTOR_FAST_CODE
sensorless_six_step_sample_irq(sensorless_six_step_t *state, const sensorless_six_step_input_t *input)
{
    if (!state->running || (!state->locked && !state->synchronizing && state->open_loop.stage < SIX_STEP_PULLIN)) { return; }
    if (!state->have_sector || input->committed_us != state->observed_committed_us)
    {
        if (state->have_sector)
        {
            if (!state->sector_event || input->sector != (uint8_t)((state->observed_sector + 1U) % 6U))
            {
                reject_event(state, "BEMF LOST");
                if (!state->running) { return; }
            }
            if (state->locked || state->synchronizing)
            {
                if (!state->commutation_pending || input->sector != state->desired_sector ||
                    (int32_t)(input->committed_us - state->submitted_us) < -1 ||
                    (int32_t)(input->committed_us - state->submitted_us) >= (int32_t)CONTROL_PERIOD_US)
                {
                    stop_algorithm(state, "BEMF COMMIT"); return;
                }
                state->commutation_pending = false;
                state->synchronizing = false;
                state->locked = true;
                update_blanking(state);
            }
        }
        state->have_sector = true;
        state->observed_sector = input->sector;
        state->observed_committed_us = input->committed_us;
        state->sector_event = false;
        state->detector_rising = (input->sector & 1U) != 0U;
        uint16_t threshold = state->detector_rising ? state->config.rising_threshold : state->config.falling_threshold;
        state->detector_lower = (uint16_t)(threshold - state->config.hysteresis);
        state->detector_upper = (uint16_t)(threshold + state->config.hysteresis);
        reset_detector(state);
    }
    uint32_t elapsed = input->sample_us - input->committed_us;
    if (input->valid)
    {
        state->last_valid_sample_us = input->sample_us;
        state->have_valid_sample = true;
    }
    if (elapsed < state->blanking_us)
    {
        ++state->blanked_count;
        reset_detector(state);
    }
    else if (!input->valid)
    {
        ++state->invalid_count;
        /* 已知无导通窗口不破坏电角事件资格；真实长间隙由check_time保护。 */
    }
    else if (!state->sector_event) { detect_crossing(state, input); }
    if (state->running)
    {
        state->stage = state->locked ? SENSORLESS_SIX_STEP_CLOSED : state->synchronizing ? SENSORLESS_SIX_STEP_SYNCHRONIZE :
            state->qualified ? SENSORLESS_SIX_STEP_QUALIFIED : state->open_loop.stage == SIX_STEP_RAMP ?
            SENSORLESS_SIX_STEP_RAMP : SENSORLESS_SIX_STEP_OBSERVE;
    }
}

/** @brief 1ms反馈期限检查由ADC调用，完整契约见头文件。 */
void MOTOR_FAST_CODE
sensorless_six_step_check_time(sensorless_six_step_t *state, uint32_t now_us)
{
    if (!state->running) { return; }
    if (state->have_valid_sample && now_us - state->last_valid_sample_us > state->gap_us)
    {
        if (state->locked || state->synchronizing) { stop_algorithm(state, "BEMF GAP"); return; }
        reset_quality(state); state->have_valid_sample = false;
    }
    if (state->have_event && now_us - state->last_event_us > 2U * expected_period(state))
    {
        if (state->locked || state->synchronizing) { stop_algorithm(state, "BEMF LOST"); return; }
        reset_quality(state);
    }
    if (!state->locked && !state->synchronizing && !state->config.observe_only && state->acquisition_started &&
        now_us - state->acquisition_started_us >= state->config.timeout_us)
    {
        stop_algorithm(state, "BEMF STARTUP");
    }
}

/** @brief 装载尚未标定的仅观察默认值，参见 sensorless_six_step_defaults。 */
void
sensorless_six_step_defaults(sensorless_six_step_config_t *config)
{
    memset(config, 0, sizeof(*config));
    config->rising_threshold = 32U;
    config->falling_threshold = 32U;
    config->hysteresis = 4U;
    config->observe_only = true;
    config->blank_us = 150U;
    /* 新7极对电机维持约50Hz电频率资格起点，不能沿用旧机械1000RPM门槛。 */
    config->min_rpm = MOTOR_SIX_MIN_RPM;
    config->lock_count = 12U;
    config->timeout_us = 500000U;
    config->max_duty = MOTOR_DUTY_MAX;
}

/** @brief 初始化独立算法并调用已验证启动，参见 sensorless_six_step_start。 */
void
sensorless_six_step_start(sensorless_six_step_t *state, const sensorless_six_step_config_t *config)
{
    memset(state, 0, sizeof(*state));
    state->config = *config;
    uint32_t minimum = config->handoff_rpm > config->min_rpm ? config->handoff_rpm : config->min_rpm;
    /* 和既有开环目标保持相同的mHz向下量化，非3倍极对数时也能到达计时门槛。 */
    state->acquisition_frequency_mhz = divide_wide((uint64_t)minimum * config->pole_pairs * 1000U, 60U);
    six_step_start(&state->open_loop, &config->startup);
    state->period_frequency_mhz = state->open_loop.frequency_mhz;
    state->open_period_us = 1000000000U / (state->period_frequency_mhz * 6U);
    update_blanking(state);
    state->running = true;
    state->desired_sector = state->open_loop.sector;
    state->duty = state->open_loop.duty;
    state->stage = (sensorless_six_step_stage_t)state->open_loop.stage;
}

#ifdef MOTOR_SENSORLESS_TEST_TICK
/** @brief 保留旧模拟时基供历史主机回归，正式固件不编译此入口。 */
void
sensorless_six_step_tick(sensorless_six_step_t *state, const sensorless_six_step_input_t *input)
{
    if (!state->running)
    {
        return;
    }
    if (!state->locked && !state->synchronizing)
    {
        if (input->target_rpm != state->target_rpm)
        {
            state->target_rpm = input->target_rpm;
            state->open_loop.config.target_mhz =
                divide_wide((uint64_t)input->target_rpm * state->config.pole_pairs * 1000U, 60U);
        }
        six_step_tick(&state->open_loop);
        state->ticks = state->open_loop.ticks;
        state->desired_sector = state->open_loop.sector;
        state->duty = state->open_loop.duty;
        state->stage = (sensorless_six_step_stage_t)state->open_loop.stage;
        if (state->period_frequency_mhz != state->open_loop.frequency_mhz)
        {
            state->period_frequency_mhz = state->open_loop.frequency_mhz;
            state->open_period_us = 1000000000U / (state->period_frequency_mhz * 6U);
        }
    }
    else
    {
        ++state->ticks;
    }
    if (state->ticks >= state->config.startup.run_ticks)
    {
        stop_algorithm(state, NULL);
        return;
    }
    if (!state->locked && !state->synchronizing && state->open_loop.stage < SIX_STEP_PULLIN)
    {
        return;
    }
    if (!state->acquisition_started && state->open_loop.frequency_mhz >= state->acquisition_frequency_mhz)
    {
        state->acquisition_started = true;
        state->acquisition_started_us = input->now_us;
    }
    update_blanking(state);
    /* 不能跨长跳脉冲窗口拼接资格；在消费新样本前检测，防止新时间戳掩盖间隙。 */
    uint32_t gap = expected_period(state) / 2U;
    if (gap < 4U * CONTROL_PERIOD_US)
    {
        gap = 4U * CONTROL_PERIOD_US;
    }
    if (state->have_valid_sample && input->now_us - state->last_valid_sample_us > gap)
    {
        if (state->locked || state->synchronizing)
        {
            stop_algorithm(state, "BEMF GAP");
            return;
        }
        reset_quality(state);
        state->have_valid_sample = false;
    }
    observe_sample(state, input);
    if (!state->running)
    {
        return;
    }
    if (state->have_event && input->now_us - state->last_event_us > 2U * expected_period(state))
    {
        if (state->locked || state->synchronizing)
        {
            stop_algorithm(state, "BEMF LOST");
            return;
        }
        /* 即使 ADC 一直有新样本，没有新电角事件也不能长期保持可信估速。 */
        reset_quality(state);
    }
    if (state->locked || state->synchronizing)
    {
        schedule_commutation(state, input);
        if (!state->running)
        {
            return;
        }
        if (state->locked)
        {
            state->limit_latched = state->limit_latched || input->current_limited;
            if (++state->pi_ticks >= SPEED_LOOP_DIVIDER)
            {
                state->pi_ticks = 0U;
                speed_control(state, input);
            }
            state->stage = SENSORLESS_SIX_STEP_CLOSED;
        }
        else
        {
            state->limit_latched = state->limit_latched || input->current_limited;
            state->stage = SENSORLESS_SIX_STEP_SYNCHRONIZE;
        }
    }
    else
    {
        state->stage = state->qualified ? SENSORLESS_SIX_STEP_QUALIFIED :
            state->open_loop.stage == SIX_STEP_RAMP ? SENSORLESS_SIX_STEP_RAMP : SENSORLESS_SIX_STEP_OBSERVE;
        if (!state->config.observe_only && state->acquisition_started &&
            input->now_us - state->acquisition_started_us >= state->config.timeout_us)
        {
            stop_algorithm(state, "BEMF STARTUP");
        }
    }
}
#endif

/** @brief 返回固定阶段名称，参见 sensorless_six_step_stage_name。 */
const char *
sensorless_six_step_stage_name(sensorless_six_step_stage_t stage)
{
    static const char *const names[] = {
        "ALIGN_RAMP", "ALIGN_HOLD", "PULLIN", "RAMP", "OBSERVE", "QUALIFIED", "SYNCHRONIZE", "CLOSED", "STOPPED", "FAULT"
    };
    return (unsigned)stage < sizeof(names) / sizeof(names[0]) ? names[stage] : "NONE";
}
