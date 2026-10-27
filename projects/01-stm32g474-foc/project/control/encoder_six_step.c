#include "encoder_six_step.h"
#include "motor_config.h"
#include <stddef.h>
#include <string.h>

/** @brief 单圈有符号最短差。 @param value 新原码。 @param before 旧原码。 @return 半圈内差值。 */
static int32_t MOTOR_FAST_CODE
angle_delta(uint32_t value, uint32_t before)
{
    int32_t delta = (int32_t)value - (int32_t)before;
    if (delta > (int32_t)(ENCODER_SIX_TURN / 2U)) { delta -= (int32_t)ENCODER_SIX_TURN; }
    if (delta < -(int32_t)(ENCODER_SIX_TURN / 2U)) { delta += (int32_t)ENCODER_SIX_TURN; }
    return delta;
}

/** @brief 锁存故障并撤销控制幅度。 @param state 状态。 @param reason 静态错误码。 */
static void MOTOR_FAST_CODE
fail(encoder_six_step_t *state, const char *reason)
{
    state->running = false;
    state->duty = 0U;
    state->fault = reason;
}

/** @brief 从机械码计算有方向的转子电角；只使用已配置方向/零点。
 * @param state 状态。 @param raw 机械码。 @return 一圈电角原码。 */
static uint32_t MOTOR_FAST_CODE
electrical_angle(const encoder_six_step_t *state, uint32_t raw)
{
    uint32_t phase = raw * state->config.pole_pairs;
    if (state->config.direction < 0) { phase = 0U - phase; }
    return (phase - state->zero_raw) & (ENCODER_SIX_TURN - 1U);
}

/** @brief 接受新的CRC/磁场有效反馈，拒绝时间倒退、超速与跳变。
 * @param state 状态。 @param feedback 已完成反馈。 @param now_tick 当前周期。 @return 可用于控制。 */
static bool MOTOR_FAST_CODE
accept_feedback(encoder_six_step_t *state, const encoder_six_feedback_t *feedback, uint32_t now_tick)
{
    if (!feedback->valid || feedback->raw >= ENCODER_SIX_TURN)
    {
        if (feedback->sequence != 0U) { fail(state, "ENCODER DATA"); }
        return false;
    }
    if (now_tick - feedback->tick > (state->config.tick_hz / 1000000U) * ENCODER_SIX_MAX_AGE_US)
    {
        fail(state, "ENCODER STALE");
        return false;
    }
    if (now_tick - feedback->tick > state->max_age_ticks) { state->max_age_ticks = now_tick - feedback->tick; }
    if (state->feedback_seen && feedback->sequence == state->feedback_sequence) { return true; }
    if (state->feedback_seen)
    {
        uint32_t dt = feedback->tick - state->sample_tick;
        if (dt == 0U || dt > state->config.tick_hz / 1000U)
        {
            fail(state, "ENCODER TIME");
            return false;
        }
        int32_t delta = angle_delta(feedback->raw, state->raw);
        uint32_t magnitude = (uint32_t)(delta < 0 ? -delta : delta);
        if ((uint64_t)magnitude * 60U * state->config.tick_hz >=
            (uint64_t)state->config.hard_rpm * ENCODER_SIX_TURN * dt)
        {
            fail(state, "ENCODER OVERSPEED");
            return false;
        }
        state->speed_delta += delta;
        state->speed_ticks += dt;
        /* 这里只累加原码和时间；64位除法移到COM之后的1kHz分支。 */
    }
    state->raw = feedback->raw;
    state->sample_tick = feedback->tick;
    state->feedback_sequence = feedback->sequence;
    state->feedback_seen = true;
    return true;
}

/** @brief 见encoder_six_step.h。 */
bool
encoder_six_step_start(encoder_six_step_t *state, const encoder_six_config_t *config, bool calibration)
{
    if (NULL == state || NULL == config || config->tick_hz < 1000000U ||
        config->pwm_hz < 1000U || config->pwm_hz % 1000U != 0U ||
        config->pole_pairs == 0U || config->pole_pairs > 32U || config->run_ticks == 0U ||
        config->max_duty == 0U || config->max_duty > 950U ||
        config->calibration_duty == 0U || config->calibration_duty > config->max_duty ||
        (config->direction != 1 && config->direction != -1) || config->zero_decideg >= 3600U ||
        config->hard_rpm == 0U || config->target_rpm == 0U || config->target_rpm >= config->hard_rpm ||
        config->ramp_millirpm_s < 1000U || config->kp_milli > 100000U || config->ki_milli > 100000U)
    {
        return false;
    }
    memset(state, 0, sizeof(*state));
    state->config = *config;
    state->zero_raw = (uint32_t)((uint64_t)config->zero_decideg * ENCODER_SIX_TURN / 3600U);
    state->calibration = calibration;
    state->startup_active = config->startup_boost && !calibration &&
        config->target_rpm >= ENCODER_SIX_START_MIN_RPM;
    state->running = true;
    state->fault = NULL;
    return true;
}

/** @brief 完成一个固定扇区的静止样本窗口。 @param state 状态。 @return 静止且样本足够。 */
static bool MOTOR_FAST_CODE
calibration_point(encoder_six_step_t *state)
{
    if (state->cal_samples < 20U)
    {
        fail(state, "ENC CAL SAMPLES");
        return false;
    }
    if (state->cal_max - state->cal_min > (int32_t)(ENCODER_SIX_TURN / (72U * state->config.pole_pairs)))
    {
        fail(state, "ENC CAL MOTION");
        return false;
    }
    int32_t mean = state->cal_sum / (int32_t)state->cal_samples;
    state->cal_points[state->cal_index] = (state->cal_anchor + (uint32_t)mean) & (ENCODER_SIX_TURN - 1U);
    state->cal_samples = 0U;
    state->cal_sum = 0;
    return true;
}

/** @brief 见encoder_six_step.h。 */
void MOTOR_FAST_CODE
encoder_six_step_tick(encoder_six_step_t *state, const encoder_six_feedback_t *feedback, uint32_t now_tick)
{
    if (!state->running) { return; }
    ++state->ticks;
    bool fresh = feedback->sequence != state->feedback_sequence;
    if (!accept_feedback(state, feedback, now_tick))
    {
        state->duty = 0U;
        /* 首帧DMA完成之前绝不输出；最多等1ms，不沿用启动前旧角度。 */
        if (state->running && state->ticks >= state->config.pwm_hz / 1000U) { fail(state, "ENCODER STALE"); }
        return;
    }
    if (state->calibration)
    {
        uint32_t hold = ENCODER_SIX_CAL_HOLD_MS * (state->config.pwm_hz / 1000U);
        uint32_t index = (state->ticks - 1U) / hold;
        if (index != state->cal_index)
        {
            if (!calibration_point(state)) { return; }
            if (index >= ENCODER_SIX_CAL_POINTS)
            {
                state->calibration_complete = true;
                state->running = false;
                state->duty = 0U;
                return;
            }
            state->cal_index = index;
        }
        state->sector = (uint8_t)(index % 6U);
        uint32_t phase = (state->ticks - 1U) % hold;
        uint32_t ramp = 20U * (state->config.pwm_hz / 1000U);
        state->duty = phase < ramp ? (uint16_t)(state->config.calibration_duty * phase / ramp) : state->config.calibration_duty;
        if (fresh && phase >= hold - 20U * (state->config.pwm_hz / 1000U))
        {
            if (state->cal_samples == 0U)
            {
                state->cal_anchor = state->raw;
                state->cal_min = state->cal_max = 0;
            }
            int32_t delta = angle_delta(state->raw, state->cal_anchor);
            state->cal_sum += delta;
            if (delta < state->cal_min) { state->cal_min = delta; }
            if (delta > state->cal_max) { state->cal_max = delta; }
            ++state->cal_samples;
        }
        return;
    }
    if (state->ticks >= state->config.run_ticks)
    {
        state->running = false;
        state->duty = 0U;
        return;
    }
    state->electrical = electrical_angle(state, state->raw);
    /* 六个电压矢量中心为-30+60*s；正转选择领先转子60..120度的扇区。 */
    uint32_t phase = (state->electrical + ENCODER_SIX_TURN * 5U / 12U) & (ENCODER_SIX_TURN - 1U);
    uint8_t sector = (uint8_t)((phase * 6U) >> 21U);
    if (state->ticks == 1U || !state->speed_valid || sector == state->sector)
    {
        state->sector = sector;
    }
    else
    {
        /* 2电角度边界回差抑制编码器噪声；超出正常相邻扇区则按真实角重新定位。 */
        uint32_t within = (phase * 6U) & (ENCODER_SIX_TURN - 1U);
        if ((sector == (uint8_t)((state->sector + 1U) % 6U) && within >= ENCODER_SIX_TURN / 30U) ||
            (sector != (uint8_t)((state->sector + 1U) % 6U) && within <= ENCODER_SIX_TURN - ENCODER_SIX_TURN / 30U))
        {
            state->sector = sector;
        }
    }
}

/** @brief 见encoder_six_step.h。 */
void MOTOR_FAST_CODE
encoder_six_step_speed(encoder_six_step_t *state, uint32_t target_rpm, bool limited)
{
    if (!state->running || !state->feedback_seen) { return; }
    bool speed_updated = false;
    if (state->speed_ticks >= state->config.tick_hz / 50U)
    {
        /* 运行在硬件COM之后，避免首次20ms速度换算占用桥臂关闭窗口。 */
        int64_t numerator = (int64_t)state->speed_delta * 60 * state->config.tick_hz;
        state->rpm = (int32_t)(numerator / ((int64_t)ENCODER_SIX_TURN * state->speed_ticks));
        if (!state->calibration) { state->rpm *= state->config.direction; }
        state->speed_valid = true;
        speed_updated = true;
        state->speed_delta = 0;
        state->speed_ticks = 0U;
    }
    if (state->calibration) { return; }
    if (target_rpm == 0U || target_rpm >= state->config.hard_rpm) { fail(state, "ENCODER TARGET"); return; }
    uint32_t target = target_rpm * 1000U;
    uint32_t step = state->config.ramp_millirpm_s / 1000U;
    if (state->reference_millirpm < target)
    {
        uint32_t next = state->reference_millirpm + step;
        state->reference_millirpm = next < target ? next : target;
    }
    else
    {
        uint32_t difference = state->reference_millirpm - target;
        state->reference_millirpm -= difference < step ? difference : step;
    }
    int32_t rpm = state->speed_valid ? state->rpm : 0;
    if (rpm < -30) { ++state->reverse_ms; } else { state->reverse_ms = 0U; }
    if (state->reverse_ms >= 10U) { fail(state, "ENCODER REVERSE"); return; }
    if (state->reference_millirpm >= 100000U && rpm < 10) { ++state->stall_ms; } else { state->stall_ms = 0U; }
    if (state->stall_ms >= 500U) { fail(state, "ENCODER STALL"); return; }
    if (state->startup_active && speed_updated)
    {
        if (rpm >= ENCODER_SIX_START_MOTION_RPM)
        {
            if (state->startup_motion_windows == 0U) { state->motion_ticks = state->ticks; }
            ++state->startup_motion_windows;
        }
        else
        {
            state->startup_motion_windows = 0U;
            state->motion_ticks = 0U;
        }
    }
    if (state->startup_active && state->startup_motion_windows < 2U &&
        target_rpm >= ENCODER_SIX_START_MIN_RPM &&
        state->ticks >= ENCODER_SIX_START_TIMEOUT_MS * (state->config.pwm_hz / 1000U))
    {
        fail(state, "ENCODER START");
        return;
    }
    int32_t error = (int32_t)(state->reference_millirpm / 1000U) - rpm;
    int64_t proportional = (int64_t)state->config.kp_milli * error * 1000;
    int64_t maximum = (int64_t)state->config.max_duty * 1000000;
    if (state->startup_active &&
        (state->startup_motion_windows >= 2U || target_rpm < ENCODER_SIX_START_MIN_RPM))
    {
        /* 起转后将PI积分对齐已输出幅度，避免撤销补偿时占空比突然掉回接近0。 */
        if (target_rpm >= ENCODER_SIX_START_MIN_RPM && !limited)
        {
            int64_t tracking = (int64_t)state->duty * 1000000 - proportional;
            state->integral_microduty = tracking < 0 ? 0 : tracking > maximum ? maximum : tracking;
        }
        state->startup_active = false;
        state->startup_end_ticks = state->ticks;
    }
    int64_t integral = state->integral_microduty + (int64_t)state->config.ki_milli * error;
    int64_t requested = proportional + integral;
    if (!limited && ((requested >= 0 && requested <= maximum) ||
        (requested > maximum && error < 0) || (requested < 0 && error > 0)))
    {
        state->integral_microduty = integral < 0 ? 0 : integral > maximum ? maximum : integral;
    }
    requested = proportional + state->integral_microduty;
    if (state->startup_active && !limited)
    {
        /* 小于采样脉宽的阶段仅短暂通过；仍受DUTY上限、上升限速和原电流关断约束。 */
        int64_t floor = (int64_t)(state->config.max_duty < ENCODER_SIX_START_DUTY ?
            state->config.max_duty : ENCODER_SIX_START_DUTY) * 1000000;
        if (requested < floor) { requested = floor; }
    }
    uint16_t next = requested <= 0 ? 0U : requested >= maximum ? state->config.max_duty : (uint16_t)(requested / 1000000);
    /* 每毫秒最多增加0.2%占空比，限流或减速可立即降低。 */
    state->duty = next > state->duty + 2U ? (uint16_t)(state->duty + 2U) : next;
}

/** @brief 见encoder_six_step.h。 */
bool
encoder_six_step_calibration_result(encoder_six_step_t *state)
{
    if (state->running || !state->calibration_complete || NULL != state->fault) { return false; }
    int32_t total = 0;
    for (unsigned i = 1U; i < ENCODER_SIX_CAL_POINTS; ++i) { total += angle_delta(state->cal_points[i], state->cal_points[i - 1U]); }
    int32_t direction = total >= 0 ? 1 : -1;
    for (unsigned i = 1U; i < ENCODER_SIX_CAL_POINTS; ++i)
    {
        int32_t step = direction * angle_delta(state->cal_points[i], state->cal_points[i - 1U]) * (int32_t)state->config.pole_pairs;
        if (step < (int32_t)(ENCODER_SIX_TURN / 9U) || step > (int32_t)(ENCODER_SIX_TURN * 2U / 9U))
        {
            fail(state, "ENC CAL DIRECTION");
            return false;
        }
    }
    uint32_t base = 0U;
    int32_t sum = 0;
    for (unsigned i = 0U; i < ENCODER_SIX_CAL_POINTS; ++i)
    {
        uint32_t vector = (ENCODER_SIX_TURN * (i % 6U) / 6U + ENCODER_SIX_TURN - ENCODER_SIX_TURN / 12U) & (ENCODER_SIX_TURN - 1U);
        uint32_t directed = state->cal_points[i] * state->config.pole_pairs;
        if (direction < 0) { directed = 0U - directed; }
        uint32_t zero = (directed - vector) & (ENCODER_SIX_TURN - 1U);
        if (i == 0U) { base = zero; }
        int32_t delta = angle_delta(zero, base);
        if (delta < -(int32_t)(ENCODER_SIX_TURN / 36U) || delta > (int32_t)(ENCODER_SIX_TURN / 36U))
        {
            fail(state, "ENC CAL ZERO");
            return false;
        }
        sum += delta;
    }
    uint32_t mean = (base + (uint32_t)(sum / (int32_t)ENCODER_SIX_CAL_POINTS)) & (ENCODER_SIX_TURN - 1U);
    state->config.direction = (int8_t)direction;
    state->config.zero_decideg = (uint16_t)(((uint64_t)mean * 3600U + ENCODER_SIX_TURN / 2U) / ENCODER_SIX_TURN % 3600U);
    return true;
}
