#ifndef FOC_CONTROL_ENCODER_FOC_H
#define FOC_CONTROL_ENCODER_FOC_H
#include "encoder_six_step.h"
#include "foc_drive.h"
#include "encoder_position.h"
#include "encoder_haptic.h"

typedef struct {
    /* ADC累计位移和DWT时间的原子交换窗口，接收方为1kHz速度中断。 */
    int32_t delta;
    uint32_t ticks;
} encoder_foc_speed_window_t;

typedef struct {
    /* 复用FOC独立参数，不使用其中的SMO接管或强拖参数。 */
    foc_drive_config_t drive;
    /* 与七点校准一致的机械方向及转子d轴零点，单位0.1电角度。 */
    int8_t direction;
    uint16_t zero_decideg;
    /* 独立电流模式绕过速度环；Iq为标定正方向的有符号A，Id始终为0。 */
    bool current_mode;
    float target_iq_a;
    /* 多圈位置目标不取模；位置参数与原速度模式独立。 */
    bool position_mode;
    int32_t target_position_mdeg, position_tolerance_mdeg;
    float position_kp, position_speed_limit_rpm;
    /* 手感模式使用独立无积分机械反馈，配置由主循环原子发布。 */
    bool haptic_mode;
    encoder_haptic_config_t haptic;
} encoder_foc_config_t;

typedef struct {
    /* dq电流环和初始化时验证的固定配置，仅控制中断拥有。 */
    foc_current_t current;
    encoder_foc_config_t config;
    /* 机械原码、最新完整反馈编号和170MHz时间戳，均允许自然回绕。 */
    uint32_t raw, feedback_sequence, feedback_tick, zero_raw, age_max_ticks;
    /* 20ms机械速度窗口；1kHz中计算，不在ADC中做宽除法。 */
    volatile int32_t speed_delta;
    /* 1kHz取走位移窗口时同步锁存的机械原码与源时间戳。 */
    uint32_t speed_raw, speed_tick;
    volatile uint32_t speed_elapsed;
    int32_t window_delta;
    uint32_t window_elapsed, speed_ticks;
    float rpm, omega_rad_s, rpm_per_raw_tick;
    /* 有界速度参考、PI积分和最终Iq指令，单位RPM/A。 */
    float reference_rpm, integral_a, reference_q_a;
    /* 当前采样及未来作用中心的转子电角rad，不使用强拖角。 */
    float sample_theta_rad, future_theta_rad;
    /* 真实控制样本、首次反馈角提交编号及启动/运动诊断时标。 */
    uint32_t samples, pending_sequence, motion_ms, startup_end_ms, motion_windows;
    /* 速度反转/位置逃离、软超速和失速时长ms，电压饱和积分防护计数。 */
    uint32_t reverse_ms, overspeed_ms, stall_ms, seen_voltage_limits;
    /* 首帧前保持零矢量；真实UEV后才确认编码器闭环已提交。 */
    bool running, feedback_seen, speed_valid, awaiting_commit, closed_seen;
    /* 仅最初80ms且首个运动窗尚未合格时允许有界Iq积分预充。 */
    bool startup_precharge;
    /* 上次Iq/电压受限时冻结角同步学习，状态归每个控制实例所有。 */
    bool ripple_frozen;
    /* 静态故障字符串；失败后仅请求关断，无六步或无感回退。 */
    const char *fault;
    /* 真实位移驱动的角度PLL速度，仅低速PI使用；保护继续取原20ms窗。 */
    float control_rpm;
    /* PLL机械位置预测残差原码、速度积分原码/s；每实例独立，初始化清零。 */
    float tracking_error, tracking_integral;
    /* G档实测14/12机械角次扰动的正交Iq系数A；每次启动清零，不写参数Flash。 */
    float ripple_c[2], ripple_s[2];
    /* Sguan位置模型LESO：机械位置预测残差RPM*s、速度RPM、总扰动RPM/s。 */
    float low_position_residual, low_velocity_rpm, low_disturbance_rpm_s;
    /* 有界速度轨迹与真实位移之差RPM*s，只作转矩反馈，不作Park角。 */
    float low_trajectory_error;
    /* 低速观测器首次进入时从当前控制状态接续；停机初始化清零。 */
    bool low_position_active;
    /* 低段28/36/84机械角次的负载预测系数A，每次进入低段清零，合成预算80mA。 */
    float low_load_c[3], low_load_s[3];
    /* 上个反馈时刻的预测负载A；LESO只估计扣除它之后的残余扰动。 */
    float low_load_previous_a;
    /* 由同一编码器采集链提供的机械零点相对原码，1kHz回调更新。 */
    int64_t position_counts, target_position_counts;
    /* 本次目标已达到的最小绝对误差原码，用于限制持续向外逃离。 */
    int64_t position_best_error_counts;
    bool position_valid;
    uint32_t position_settle_ms;
    /* 仅控制ISR持有的手感动态状态，停机初始化整块清零。 */
    encoder_haptic_t haptic;
} encoder_foc_t;

/** @brief 停机初始化编码器FOC，Id固定0、Iq由速度环或独立电流给定。
 * @param state 独立状态。 @param config 已校准方向/零点及FOC参数。
 * @return 静态参数及电流环初始化通过；不打开功率。 */
bool encoder_foc_init(encoder_foc_t *state, const encoder_foc_config_t *config);
/** @brief 接受完整反馈、补偿角度年龄，执行10kHz dq电流环。
 * @param state 状态。 @param feedback SPI1 DMA的最新CRC/磁场校验反馈。
 * @param now_tick 当前DWT时间戳。 @param sample_tick 双ADC第一保持点的DWT时间戳。
 * @param alpha_a 同时Clarke电流A。 @param beta_a 同时Clarke电流A。
 * @param bus_v 母线V。 @param sequence 下一PWM序号。
 * @param future_s 采样至未来PWM平均作用中心秒数。 @param ccr 8500归一化比较值。
 * @return 有效；无首帧时短暂保持零矢量，失效时调用者立即关断。 */
bool encoder_foc_sample(encoder_foc_t *state, const encoder_six_feedback_t *feedback,
    uint32_t now_tick, uint32_t sample_tick, float alpha_a, float beta_a, float bus_v,
    uint32_t sequence, float future_s, uint16_t ccr[3]);
/** @brief 仅在硬件UEV真正应用反馈角命令后确认闭环。
 * @param state 状态。 @param sequence 实际生效编号。 */
void encoder_foc_commit(encoder_foc_t *state, uint32_t sequence);
/** @brief 1kHz更新20ms机械速度、对应模式的Iq斜坡及运行保护。
 * @param state 状态。 @param target_rpm 速度模式已验证的正机械目标；电流模式不使用。
 * @param window 由短临界区取得的ADC机械位移增量。 */
void encoder_foc_speed_tick(encoder_foc_t *state, uint32_t target_rpm, encoder_foc_speed_window_t window);
/** @brief 交换ADC累计量，真实板卡调用者必须暂时屏蔽ADC抢占。
 * @param state 状态。 @return 原累计位移和时间；函数只复制并清零，不做浮点控制。 */
encoder_foc_speed_window_t encoder_foc_take_speed(encoder_foc_t *state);
/** @brief 撤销控制和待提交资格，保留本轮诊断。
 * @param state 状态。 @param reason NULL正常停止，其它为故障。 */
void encoder_foc_stop(encoder_foc_t *state, const char *reason);
/** @brief 在线更新多圈目标，机械零点保持；只允许在已验证的位置模式调用。
 * @param state 状态。 @param target_mdeg 机械毫度。 */
void encoder_foc_position_target(encoder_foc_t *state, int32_t target_mdeg);
#endif
