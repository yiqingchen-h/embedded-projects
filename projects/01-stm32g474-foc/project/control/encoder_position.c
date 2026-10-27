#include "encoder_position.h"
#include "motor_config.h"
#include <limits.h>

#define POSITION_TURN 2097152L
#define POSITION_MAX_GAP_TICKS 2550000U

/** @brief 见encoder_position.h；无SPI事务或浮点运算。 */
void MOTOR_FAST_CODE
encoder_position_sample(encoder_position_t *p, uint32_t raw, uint32_t tick, bool valid, bool still_bridge)
{
    if (!valid || raw >= POSITION_TURN)
    {
        p->valid = p->zero_valid = false;
        p->seen = false;
        return;
    }
    if (p->seen)
    {
        int32_t delta = (int32_t)raw - (int32_t)p->raw;
        if (delta > POSITION_TURN / 2) { delta -= POSITION_TURN; }
        if (delta < -POSITION_TURN / 2) { delta += POSITION_TURN; }
        if ((!still_bridge && tick - p->tick > POSITION_MAX_GAP_TICKS) ||
            delta == POSITION_TURN / 2 || delta == -POSITION_TURN / 2)
        {
            p->zero_valid = false;
        }
        p->count += delta;
    }
    else { p->count = raw; }
    p->raw = raw;
    p->tick = tick;
    p->seen = p->valid = true;
}

/** @brief 见encoder_position.h。 */
bool
encoder_position_zero(encoder_position_t *p, int8_t direction)
{
    if (!p->valid || (direction != 1 && direction != -1)) { return false; }
    p->origin = p->count;
    p->direction = direction;
    p->zero_valid = true;
    ++p->epoch;
    return true;
}

/** @brief 见encoder_position.h。 */
int64_t MOTOR_FAST_CODE
encoder_position_counts(const encoder_position_t *p)
{
    return (p->count - p->origin) * p->direction;
}

/** @brief 见encoder_position.h。 */
int64_t
encoder_position_target_counts(int32_t mdeg)
{
    return (int64_t)mdeg * POSITION_TURN / 360000L;
}

/** @brief 见encoder_position.h。 */
int32_t
encoder_position_mdeg(int64_t counts)
{
    int64_t value = counts * 360000L / POSITION_TURN;
    return value > INT32_MAX ? INT32_MAX : value < INT32_MIN ? INT32_MIN : (int32_t)value;
}
