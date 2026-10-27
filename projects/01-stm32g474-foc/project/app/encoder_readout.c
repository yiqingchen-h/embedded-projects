#include "encoder_readout.h"
#include <string.h>

/* 至少20ms角度增量用于速度，减小相邻样本量化和静止抖动的影响。 */
#define ENCODER_SPEED_WINDOW_HZ 50U
/* 在本机最高6000RPM范围，4ms仍小于半机械圈；长间隔禁止拼接。 */
#define ENCODER_MAX_GAP_MS 4U

/** @brief 清空速度历史，下一次有效读数仅建立基准。 @param readout 状态。 */
static void
reset_speed(encoder_readout_t *readout)
{
    readout->previous_valid = false;
    readout->window_delta = 0;
    readout->window_ticks = 0U;
    readout->value.speed_valid = false;
    readout->value.rpm = 0;
}

/** @brief 见encoder_readout.h。 */
void
encoder_readout_init(encoder_readout_t *readout, uint32_t tick_hz)
{
    memset(readout, 0, sizeof(*readout));
    readout->tick_hz = tick_hz;
    readout->value.state = ENCODER_STARTING;
    readout->value.age_ms = UINT32_MAX;
}

/** @brief 见encoder_readout.h。 */
void
encoder_readout_sample(encoder_readout_t *readout, uint32_t raw, uint8_t status, uint32_t now_ms, uint32_t tick)
{
    encoder_snapshot_t *value = &readout->value;
    if (raw >= ENCODER_COUNTS_PER_TURN || readout->tick_hz < 1000U)
    {
        encoder_readout_error(readout, ENCODER_IO_ERROR);
        return;
    }
    value->raw = raw;
    value->angle_mdeg = (uint32_t)((uint64_t)raw * 360000U / ENCODER_COUNTS_PER_TURN);
    value->status = status & 7U;
    ++value->samples;
    readout->sample_ms = now_ms;
    value->angle_valid = value->status == 0U;
    value->state = (value->status & 5U) != 0U ? ENCODER_SENSOR_FAULT :
        (value->status & 2U) != 0U ? ENCODER_WEAK_FIELD : ENCODER_OK;
    if (!value->angle_valid)
    {
        reset_speed(readout);
        return;
    }
    uint32_t dt = tick - readout->previous_tick;
    uint32_t gap_ms = now_ms - readout->previous_ms;
    if (readout->previous_valid && dt != 0U && gap_ms <= ENCODER_MAX_GAP_MS &&
        dt <= (readout->tick_hz / 1000U) * ENCODER_MAX_GAP_MS)
    {
        int32_t delta = (int32_t)raw - (int32_t)readout->previous_raw;
        int32_t half = (int32_t)(ENCODER_COUNTS_PER_TURN / 2U);
        if (delta > half) { delta -= (int32_t)ENCODER_COUNTS_PER_TURN; }
        if (delta < -half) { delta += (int32_t)ENCODER_COUNTS_PER_TURN; }
        if (delta == half || delta == -half)
        {
            reset_speed(readout);
        }
        else
        {
            readout->window_delta += delta;
            readout->window_ticks += dt;
            if (readout->window_ticks >= readout->tick_hz / ENCODER_SPEED_WINDOW_HZ)
            {
                int64_t numerator = (int64_t)readout->window_delta * 60 * readout->tick_hz;
                int64_t denominator = (int64_t)ENCODER_COUNTS_PER_TURN * readout->window_ticks;
                numerator += numerator >= 0 ? denominator / 2 : -denominator / 2;
                value->rpm = (int32_t)(numerator / denominator);
                value->speed_valid = true;
                readout->window_delta = 0;
                readout->window_ticks = 0U;
            }
        }
    }
    else
    {
        reset_speed(readout);
    }
    readout->previous_raw = raw;
    readout->previous_tick = tick;
    readout->previous_ms = now_ms;
    readout->previous_valid = true;
}

/** @brief 见encoder_readout.h。 */
void
encoder_readout_error(encoder_readout_t *readout, encoder_state_t error)
{
    readout->value.state = error;
    readout->value.angle_valid = false;
    if (error == ENCODER_CRC_ERROR) { ++readout->value.crc_errors; }
    else { ++readout->value.io_errors; }
    reset_speed(readout);
}

/** @brief 见encoder_readout.h。 */
encoder_snapshot_t
encoder_readout_snapshot(const encoder_readout_t *readout, uint32_t now_ms)
{
    encoder_snapshot_t value = readout->value;
    value.age_ms = value.samples != 0U ? now_ms - readout->sample_ms : UINT32_MAX;
    if (value.age_ms > ENCODER_STALE_MS && value.state == ENCODER_OK)
    {
        value.state = ENCODER_STALE;
        value.angle_valid = false;
        value.speed_valid = false;
    }
    return value;
}

/** @brief 见encoder_readout.h。 */
const char *
encoder_state_name(encoder_state_t state)
{
    static const char *const names[] = {
        "STARTING", "NO_DEVICE", "IO_ERROR", "CRC_ERROR", "WEAK_FIELD", "SENSOR_FAULT", "OK", "STALE"
    };
    return (unsigned)state < sizeof(names) / sizeof(names[0]) ? names[state] : "UNKNOWN";
}
