#ifndef FOC_CONTROL_ENCODER_HAPTIC_H
#define FOC_CONTROL_ENCODER_HAPTIC_H
#include <stdbool.h>
#include <stdint.h>

/* 手感类型只改变机械转矩请求，不修改真实编码器或Park角。 */
typedef enum { ENCODER_HAPTIC_DETENT, ENCODER_HAPTIC_DAMPED } encoder_haptic_kind_t;
typedef struct {
    /* 类型、每圈档数、档位峰值mA、粘性阻尼微安/RPM；配置一次完整发布。 */
    int32_t kind, detents, detent_ma, damping_ua_rpm;
} encoder_haptic_config_t;
typedef struct {
    /* 本次开启后的真实累计原码、档位网格起点、当前中心和累计档数。 */
    int64_t counts, anchor_counts, center_counts, index;
    /* ADC独占的已消费配置；只在其中1kHz外环拍更新。 */
    encoder_haptic_config_t applied;
    /* 增益平滑的起点、实际值及已执行时间s，避免在线调整引入电流阶跃。 */
    float from_strength_a, from_damping_a_rpm, strength_a, damping_a_rpm, transition_s;
    /* 5kHz真实反馈内累计1ms，只在外环拍更新配置、档中心、渐变和内部弹性阻尼增益。 */
    float outer_elapsed_s, outer_phase_s, spring_damping_a_rpm;
    /* 控制用可信位移累计、窗口时间、停转保持上限及时间加权速度；原始角和计数不变。 */
    int32_t velocity_delta;
    float velocity_elapsed_s, velocity_hold_s, velocity_rpm;
    /* 粘性转矩单独限制变化率，不能借用档位快速弹性通道放大高频反馈。 */
    float damping_iq_a;
    /* 最新参考Iq及同一拍真实编码器DWT时标。 */
    float iq_a, rpm;
    uint32_t tick, raw;
    bool anchored;
} encoder_haptic_t;

/** @brief 检查四项完整配置，不改动任何状态。
 * @param config 配置。 @return 类型/档数/电流和阻尼均在已定义范围内。 */
bool encoder_haptic_valid(const encoder_haptic_config_t *config);
/** @brief 在真实反馈处按原始位移/时间估计阻尼速度，使用64码门限及2ms时间加权。
 * @param state ADC独占状态。 @param delta 本帧方向归一机械增量。
 * @param dt_s 本帧真实时间s。 @return 控制用RPM；反向/停转撤销旧估计，原角度不滤波。 */
float encoder_haptic_velocity(encoder_haptic_t *state,int32_t delta,float dt_s);
/** @brief 在5kHz真实编码器反馈处校正Iq，档位/配置外环仍为1kHz，无外环积分。
 * @param state 每实例运行态。 @param config 原子发布的完整请求配置。
 * @param delta 同一采集链的方向归一机械原码增量。 @param tick 对应反馈DWT。
 * @param dt_s 实际窗口秒数。 @param rpm 同一实际窗口原始位移/时间得到的机械速度。
 * @param limit_a 本模式相电流限幅A。 @return 有界Iq请求A，Id由外部始终固定0；
 * 配置过渡2A/s，稳定弹性200A/s、两种手感的阻尼分量10A/s，原CURRENT不受影响。
 * 动态状态仅由ADC反馈调用者拥有，1kHz定时器不并发修改本结构。 */
float encoder_haptic_tick(encoder_haptic_t *state, const encoder_haptic_config_t *config,
    int32_t delta, uint32_t tick, float dt_s, float rpm, float limit_a);
#endif
