#include "switch_input.h"
#include <string.h>
/** @brief 归一化三路有效状态。 @param raw 原始电平。 @return 同位序有效位。 */
static uint8_t
normalize(uint8_t raw)
{
    return (uint8_t)((((raw & 1U) == SWITCH_PRESS_ACTIVE_LEVEL) ? 1U : 0U) |
        ((((raw >> 1U) & 1U) == SWITCH_DOWN_ACTIVE_LEVEL) ? 2U : 0U) |
        ((((raw >> 2U) & 1U) == SWITCH_UP_ACTIVE_LEVEL) ? 4U : 0U));
}
/** @brief 见switch_input.h。 */
void
switch_input_init(switch_input_t *state, uint8_t raw, uint32_t now)
{
    memset(state,0,sizeof(*state));
    state->raw = raw;
    state->candidate = normalize(raw);
    /* 上电即按住或拨住时，必须先释放，避免意外操作。 */
    state->stable = 7U;
    for (unsigned i=0U;i<3U;++i) { state->changed_at[i]=now; }

}
/** @brief 见switch_input.h。 */
uint8_t
switch_input_update(switch_input_t *state, uint8_t raw, uint32_t now, switch_event_t events[2])
{
    uint8_t value = normalize(raw);
    uint8_t count = 0U;
    state->raw = raw;
    for (unsigned i=0U;i<3U;++i)
    {
        uint8_t bit=(uint8_t)(1U<<i);
        if ((value & bit)!=(state->candidate & bit))
        {
            state->candidate=(uint8_t)((state->candidate & ~bit) | (value & bit));
            state->changed_at[i]=now;
        }
        if ((uint32_t)(now-state->changed_at[i])>=SWITCH_DEBOUNCE_MS)
        {
            state->stable=(uint8_t)((state->stable & ~bit) | (state->candidate & bit));
        }
    }
    uint8_t direction=state->stable & 6U;
    /* 原始冲突立即封锁，避免两路消抖时差造成伪方向事件。 */
    if ((value & 6U)==6U)
    {
        if (!state->conflict) { ++state->invalid_transitions; }
        state->conflict=true;
        state->direction_armed=false;
    }
    if (direction==0U && (value & 6U)==0U)
    {
        state->direction_armed=true;
        state->conflict=false;
    }
    else if (direction==6U) { state->direction_armed=false; }
    else if (state->direction_armed && direction==(value & 6U))
    {
        bool down=direction==2U;
#if SWITCH_SWAP_UP_DOWN
        down=!down;
#endif
        events[count++]=down ? SWITCH_DOWN : SWITCH_UP;
        state->direction_armed=false;
    }
    if ((state->stable & 1U) == 0U)
    {
        if (state->pressing && !state->long_sent) { events[count++] = SWITCH_CLICK; }
        state->pressing = false;
        state->press_armed = true;
    }
    else if (state->press_armed)
    {
        state->press_armed = false;
        state->pressing = true;
        state->long_sent = false;
        state->pressed_at = now;
    }
    if (state->pressing && !state->long_sent && (uint32_t)(now - state->pressed_at) >= SWITCH_LONG_PRESS_MS)
    {
        state->long_sent = true;
        events[count++] = SWITCH_LONG_PRESS;
    }
    return count;
}
