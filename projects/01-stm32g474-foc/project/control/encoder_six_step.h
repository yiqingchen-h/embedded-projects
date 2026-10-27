#ifndef FOC_CONTROL_ENCODER_SIX_STEP_H
#define FOC_CONTROL_ENCODER_SIX_STEP_H
#include <stdbool.h>
#include <stdint.h>

/* MT6835机械单圈码；电角也使用相同一圈尺度。 */
#define ENCODER_SIX_TURN 2097152U
/* 控制反馈允许的最长年龄，远小于观测显示的20ms。 */
#define ENCODER_SIX_MAX_AGE_US 300U
/* 校准按AB、AC、BC、BA、CA、CB、AB各保持300ms。 */
#define ENCODER_SIX_CAL_HOLD_MS 300U
#define ENCODER_SIX_CAL_POINTS 7U
/* G2208静止起步的电压补偿上限；这是幅度指令，不代表电流给定。 */
#define ENCODER_SIX_START_DUTY 120U
/* 补偿仅用于>=100RPM启动；两次20ms窗口>=30RPM后交还PI，300ms无运动即停。 */
#define ENCODER_SIX_START_MIN_RPM 100U
#define ENCODER_SIX_START_MOTION_RPM 30
#define ENCODER_SIX_START_TIMEOUT_MS 300U

typedef struct {
    /* CRC及磁场均有效的机械原码、保守事务开始时间和发布序号。 */
    uint32_t raw, tick, sequence;
    /* 通信/CRC/磁场错误立即清除此标记，不重复使用旧好样本。 */
    bool valid;
} encoder_six_feedback_t;

typedef struct {
    /* 自由运行时间戳频率、PWM频率、极对数和有界运行周期数。 */
    uint32_t tick_hz, pwm_hz, pole_pairs, run_ticks;
    /* 机械目标RPM及目标斜率，后者为毫RPM/s。 */
    uint32_t target_rpm, ramp_millirpm_s;
    /* 占空比千分比/RPM及千分比/(RPM*s)的千倍整数。 */
    uint32_t kp_milli, ki_milli;
    /* 运行幅度上限和校准幅度，均为千分比。 */
    uint16_t max_duty, calibration_duty;
    /* 编码器方向±1及电角度零点0..3599个0.1度。 */
    int8_t direction;
    /* 启动电压补偿，仅正常运行有效；占用原方向/零点之间的填充字节。 */
    bool startup_boost;
    uint16_t zero_decideg;
    /* 机械超速硬限值，任何方向达到即停机。 */
    uint32_t hard_rpm;
} encoder_six_config_t;

typedef struct {
    /* 仅TIM峰顶及其1kHz尾部持有；停止后主循环才提交标定结果。 */
    encoder_six_config_t config;
    bool running, calibration, calibration_complete, feedback_seen, speed_valid;
    /* 当前运行周期、最近反馈身份、机械码与电角码。 */
    uint32_t ticks, feedback_sequence, sample_tick, raw, electrical, zero_raw, max_age_ticks;
    /* 实测机械RPM、当前扇区0..5和幅度千分比。 */
    int32_t rpm;
    uint8_t sector;
    uint16_t duty;
    /* 首个反馈前功率保持关闭；运行故障为静态文本。 */
    const char *fault;
    /* 速度20ms窗累计；时间戳差允许DWT自然回绕。 */
    int32_t speed_delta;
    uint32_t speed_ticks;
    /* 1kHz目标斜坡、积分及反转/堵转累计次数。 */
    uint32_t reference_millirpm, reverse_ms, stall_ms;
    int64_t integral_microduty;
    /* 校准最后20ms相对锚点统计及7个机械位置。 */
    uint32_t cal_index, cal_anchor, cal_samples, cal_points[ENCODER_SIX_CAL_POINTS];
    int32_t cal_sum, cal_min, cal_max;
    /* 启动阶段只在1kHz尾部推进，电角换相始终由编码器决定。 */
    bool startup_active;
    /* 连续有效20ms运动窗口数；静止噪声不能提前撤销启动补偿。 */
    uint8_t startup_motion_windows;
    /* 首次合格运动窗口及完成无扰切换的PWM时标；0为尚未发生。 */
    uint32_t motion_ticks, startup_end_ticks;
} encoder_six_step_t;

/** @brief 停机初始化有感六步；未标定正常运行必须由调用方拒绝。
 * @param state 状态。 @param config 运行配置。 @param calibration true执行有界七点标定。
 * @return 静态参数合法且状态已初始化。 */
bool encoder_six_step_start(encoder_six_step_t *state, const encoder_six_config_t *config, bool calibration);
/** @brief PWM峰顶根据新鲜机械角决定换相，无反电动势、强拖升频或无感回退。
 * @param state 状态。 @param feedback 已完成DMA且CRC正确的反馈。 @param now_tick 当前自由运行计数器。 */
void encoder_six_step_tick(encoder_six_step_t *state, const encoder_six_feedback_t *feedback, uint32_t now_tick);
/** @brief 实际峰顶提交后的1kHz速度PI，软限流时冻结积分。
 * @param state 状态。 @param target_rpm 在线机械目标。 @param limited 过去1ms有软限流。 */
void encoder_six_step_speed(encoder_six_step_t *state, uint32_t target_rpm, bool limited);
/** @brief 停机主循环验证七点方向、零点一致性；不把不动当作标定成功。
 * @param state 已完成标定的状态。 @return 七点均通过，方向/零点写入config。 */
bool encoder_six_step_calibration_result(encoder_six_step_t *state);
#endif
