#ifndef FOC_CONTROL_ENCODER_POSITION_H
#define FOC_CONTROL_ENCODER_POSITION_H
#include <stdbool.h>
#include <stdint.h>

/* 原始机械圈数与用户机械零点独立于电角标定，采集者为唯一写者。 */
typedef struct {
    int64_t count, origin;
    uint32_t raw, tick, epoch;
    int8_t direction;
    bool seen, valid, zero_valid;
} encoder_position_t;

/** @brief 累计未经滤波的机械原码，计数频率为170MHz。
 * @param position 坐标。 @param raw 21位原码。 @param tick 源时间戳。
 * @param valid CRC及磁场有效。 @param still_bridge 仅已确认静止的采集切换可跨初始化间隔。 */
void encoder_position_sample(encoder_position_t *position, uint32_t raw, uint32_t tick,
    bool valid, bool still_bridge);
/** @brief 在调用方确认输出关闭且轴静止后建立机械零点。
 * @param position 坐标。 @param direction 已电角标定的方向。 @return 有有效采样。 */
bool encoder_position_zero(encoder_position_t *position, int8_t direction);
/** @brief 计算相对机械零点的有符号累计原码。 @param position 坐标。 @return 原码。 */
int64_t encoder_position_counts(const encoder_position_t *position);
/** @brief 机械毫度目标转为原码，先做整数差再供浮点控制使用。 @param mdeg 目标毫度。 */
int64_t encoder_position_target_counts(int32_t mdeg);
/** @brief 机械原码转毫度；协议范围外只截取显示范围，不修改内部累计位置。
 * @param counts 累计原码。 @return 有符号毫度。 */
int32_t encoder_position_mdeg(int64_t counts);
#endif
