#ifndef FOC_CONTROL_FOC_VOLTAGE_H
#define FOC_CONTROL_FOC_VOLTAGE_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* 实际提交编号及采样同域的定时器tick，允许自然回绕。 */
    uint32_t sequence, applied_ticks;
    /* 限幅后实际生效的三个比较值。 */
    uint16_t ccr[3];
    /* 提交时使用的有效母线测量，单位V。 */
    float bus_v;
    /* 该命令的周期平均电压，单位V。 */
    float alpha_v, beta_v;
    /* false表示不能作为观察器输入。 */
    bool valid;
    /* 接管请求随最终CCR记录；只有真实生效的估计角命令才允许CLOSED。 */
    bool estimated_angle;
} foc_voltage_command_t;

typedef struct {
    /* 仅保留前后两条实际生效记录，不存未来请求。 */
    foc_voltage_command_t previous, current;
    /* 0/1/2条有效历史。 */
    unsigned count;
} foc_voltage_history_t;

typedef struct {
    /* 与两次电流采样配对的区间平均电压，单位V。 */
    float alpha_v, beta_v;
    /* 区间长度和新命令实际占用tick。 */
    uint32_t duration_ticks, new_ticks;
    /* 参与加权的实际命令编号。 */
    uint32_t old_sequence, new_sequence;
} foc_voltage_interval_t;

/** @brief 从中心对齐PWM1的最终CCR计算命令平均电压；不提交历史。
 * @param command 有效母线和最终CCR，输出平均电压及valid。
 * @param arr 定时器周期比较值。 @return 输入有限且CCR合法。 */
bool foc_voltage_prepare(foc_voltage_command_t *command, uint16_t arr);
/** @brief 只在硬件UEV确认生效后追加历史；重复或逆序记录拒绝。
 * @param history 两条历史。 @param applied 已确认生效记录。 @return 已追加。 */
bool foc_voltage_commit(foc_voltage_history_t *history, const foc_voltage_command_t *applied);
/** @brief 按实际占用时间加权，区间必须小于2^31 tick且最多跨一次提交。
 * @param history 仅包含已生效命令。 @param previous_ticks 前次采样保持时刻。
 * @param sample_ticks 本次采样保持时刻。 @param interval 输出，仅成功时有效。
 * @return 历史覆盖完整区间且电压有效。 */
bool foc_voltage_average(const foc_voltage_history_t *history, uint32_t previous_ticks,
    uint32_t sample_ticks, foc_voltage_interval_t *interval);
#endif
