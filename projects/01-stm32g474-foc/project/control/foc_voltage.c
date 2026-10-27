#include "foc_voltage.h"
#include "motor_config.h"
#include <math.h>
#include <stddef.h>

/** @brief 校核并计算命令均值，完整契约见头文件。 */
bool MOTOR_FAST_CODE
foc_voltage_prepare(foc_voltage_command_t *command, uint16_t arr)
{
    if (NULL == command) { return false; }
    command->valid = false;
    if (arr == 0U || !isfinite(command->bus_v) || command->bus_v <= 0.0f ||
        command->ccr[0] > arr || command->ccr[1] > arr || command->ccr[2] > arr)
    {
        return false;
    }
    float scale = command->bus_v / (float)arr;
    command->alpha_v = scale * (2.0f * command->ccr[0] - command->ccr[1] - command->ccr[2]) / 3.0f;
    command->beta_v = scale * (command->ccr[1] - (float)command->ccr[2]) * 0.57735026919f;
    command->valid = true;
    return true;
}

/** @brief 校核实际生效序号与时刻，不接收预装载请求。 */
bool MOTOR_FAST_CODE
foc_voltage_commit(foc_voltage_history_t *history, const foc_voltage_command_t *applied)
{
    if (NULL == history || NULL == applied || !applied->valid || history->count > 2U)
    {
        return false;
    }
    if (history->count != 0U && (applied->sequence != history->current.sequence + 1U ||
        (int32_t)(applied->applied_ticks - history->current.applied_ticks) <= 0))
    {
        return false;
    }
    history->previous = history->current;
    history->current = *applied;
    if (history->count < 2U) { ++history->count; }
    return true;
}

/** @brief 至多两段周期平均值加权，不积分开关边沿。 */
bool MOTOR_FAST_CODE
foc_voltage_average(const foc_voltage_history_t *history, uint32_t previous_ticks,
    uint32_t sample_ticks, foc_voltage_interval_t *interval)
{
    if (NULL == history || NULL == interval || history->count == 0U || history->count > 2U ||
        !history->current.valid)
    {
        return false;
    }
    uint32_t duration = sample_ticks - previous_ticks;
    if (duration == 0U || duration >= 0x80000000U ||
        (int32_t)(sample_ticks - history->current.applied_ticks) < 0)
    {
        return false;
    }
    const foc_voltage_command_t *current = &history->current;
    *interval = (foc_voltage_interval_t){
        .alpha_v=current->alpha_v, .beta_v=current->beta_v,
        .duration_ticks=duration, .new_ticks=duration,
        .old_sequence=current->sequence, .new_sequence=current->sequence
    };
    if ((int32_t)(previous_ticks - current->applied_ticks) >= 0) { return true; }
    if (history->count != 2U || !history->previous.valid ||
        (int32_t)(previous_ticks - history->previous.applied_ticks) < 0)
    {
        return false;
    }
    uint32_t new_ticks = sample_ticks - current->applied_ticks;
    float fraction = (float)new_ticks / (float)duration;
    interval->alpha_v = history->previous.alpha_v + fraction * (current->alpha_v - history->previous.alpha_v);
    interval->beta_v = history->previous.beta_v + fraction * (current->beta_v - history->previous.beta_v);
    interval->new_ticks = new_ticks;
    interval->old_sequence = history->previous.sequence;
    return true;
}
