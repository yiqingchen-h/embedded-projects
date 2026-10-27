#include "six_step.h"
#include "motor_config.h"
#include <string.h>

/** @brief 预计算定位增幅并装入已验证的配置，参见 six_step_start。 */
void
six_step_start(six_step_t *state, const six_step_config_t *config)
{
    memset(state, 0, sizeof(*state));
    state->config = *config;
    state->frequency_mhz = config->start_mhz;
    state->pullin_remaining = config->pullin_steps;
    state->align_ramp_ticks = config->align_ticks / 2U;
    if (state->align_ramp_ticks != 0U)
    {
        state->duty_step = (uint16_t)(config->duty / state->align_ramp_ticks);
        state->duty_remainder = config->duty % state->align_ramp_ticks;
        state->stage = SIX_STEP_ALIGN_RAMP;
    }
    else
    {
        /* 无定位或仅一个周期时，没有同时安排渐增与满幅保持的空间。 */
        state->duty = config->duty;
        state->stage = config->align_ticks != 0U ? SIX_STEP_ALIGN_HOLD :
            config->pullin_steps != 0U ? SIX_STEP_PULLIN :
            config->start_mhz == config->target_mhz ? SIX_STEP_RUN : SIX_STEP_RAMP;
    }
    state->running = true;
}

/** @brief 定位渐增、初频牵引及加速均按定时状态推进，参见 six_step_tick。 */
void MOTOR_FAST_CODE
six_step_tick(six_step_t *state)
{
    if (!state->running) { return; }
    if (++state->ticks >= state->config.run_ticks)
    {
        state->running = false;
        return;
    }
    if (state->ticks <= state->config.align_ticks)
    {
        if (state->stage == SIX_STEP_ALIGN_RAMP)
        {
            state->duty = (uint16_t)(state->duty + state->duty_step);
            state->duty_accumulator += state->duty_remainder;
            if (state->duty_accumulator >= state->align_ramp_ticks)
            {
                ++state->duty;
                state->duty_accumulator -= state->align_ramp_ticks;
            }
            if (state->ticks == state->align_ramp_ticks)
            {
                state->duty = state->config.duty;
                state->stage = SIX_STEP_ALIGN_HOLD;
            }
        }
        return;
    }
    if (state->pullin_remaining != 0U)
    {
        state->stage = SIX_STEP_PULLIN;
    }
    else
    {
        if (state->frequency_mhz > state->config.target_mhz)
        {
            state->ramp_remainder += state->config.ramp_mhz_s;
            uint32_t decrement = state->ramp_remainder / MOTOR_CONTROL_HZ;
            state->ramp_remainder %= MOTOR_CONTROL_HZ;
            if (decrement >= state->frequency_mhz - state->config.target_mhz)
            {
                state->frequency_mhz = state->config.target_mhz;
            }
            else
            {
                state->frequency_mhz -= decrement;
            }
        }
        else if (state->frequency_mhz < state->config.target_mhz)
        {
            state->ramp_remainder += state->config.ramp_mhz_s;
            state->frequency_mhz += state->ramp_remainder / MOTOR_CONTROL_HZ;
            state->ramp_remainder %= MOTOR_CONTROL_HZ;
            if (state->frequency_mhz > state->config.target_mhz)
            {
                state->frequency_mhz = state->config.target_mhz;
            }
        }
        state->stage = state->frequency_mhz == state->config.target_mhz ? SIX_STEP_RUN : SIX_STEP_RAMP;
    }
    state->phase += state->frequency_mhz * 6U;
    if (state->phase >= MOTOR_CONTROL_HZ * 1000U)
    {
        state->phase -= MOTOR_CONTROL_HZ * 1000U;
        state->sector = (uint8_t)((state->sector + 1U) % 6U);
        if (state->pullin_remaining != 0U) { --state->pullin_remaining; }
    }
}

/** @brief 正向六步顺序：AB、AC、BC、BA、CA、CB。 */
void MOTOR_FAST_CODE
six_step_phases(uint8_t sector, uint8_t *high, uint8_t *low)
{
    static const uint8_t high_phase[6] = {0, 0, 1, 1, 2, 2};
    static const uint8_t low_phase[6] = {1, 2, 2, 0, 0, 1};
    *high = high_phase[sector];
    *low = low_phase[sector];
}

/** @brief 留出实际启动开销，来不及的慢速采样直接丢弃本次机会。 */
bool MOTOR_FAST_CODE
six_step_regular_fits(uint32_t available, uint32_t scan, uint32_t launch)
{
    return available > scan && available - scan > launch;
}

/** @brief 参见six_step_pulse_density；随机门槛只延后整脉冲，绝不预支脉宽。 */
bool MOTOR_FAST_CODE
six_step_pulse_density(uint32_t *budget, uint32_t *noise, uint16_t requested,
    uint16_t minimum, bool limited, bool spread)
{
    if (spread && (limited || requested == 0U || requested >= minimum))
    {
        *budget = 0U;
        return !limited && requested != 0U;
    }
    if (requested >= minimum) { return !limited; }
    *budget += requested;
    if (limited)
    {
        *budget = 0U;
        return false;
    }
    uint32_t threshold = minimum;
    if (spread)
    {
        /* xorshift32与乘积高32位映射，不在峰顶引入宽除法或取模。 */
        uint32_t value = *noise;
        value ^= value << 13U;
        value ^= value >> 17U;
        value ^= value << 5U;
        *noise = value;
        threshold += (uint32_t)(((uint64_t)value * minimum) >> 32U);
    }
    if (*budget < threshold) { return false; }
    *budget -= minimum;
    return true;
}
