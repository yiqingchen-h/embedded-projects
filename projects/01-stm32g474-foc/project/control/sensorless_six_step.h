#ifndef FOC_CONTROL_SENSORLESS_SIX_STEP_H
#define FOC_CONTROL_SENSORLESS_SIX_STEP_H
#include "six_step.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    /* 开环启动阶段，含义与现有六步算法一致。 */
    SENSORLESS_SIX_STEP_ALIGN_RAMP = 0,
    SENSORLESS_SIX_STEP_ALIGN_HOLD,
    SENSORLESS_SIX_STEP_PULLIN,
    SENSORLESS_SIX_STEP_RAMP,
    /* 观察、事件资格、首次同步提交和真正闭环分别报告。 */
    SENSORLESS_SIX_STEP_OBSERVE,
    SENSORLESS_SIX_STEP_QUALIFIED,
    SENSORLESS_SIX_STEP_SYNCHRONIZE,
    SENSORLESS_SIX_STEP_CLOSED,
    SENSORLESS_SIX_STEP_STOPPED,
    SENSORLESS_SIX_STEP_FAULT
} sensorless_six_step_stage_t;

typedef struct {
    /* 开环启动配置；duty仅表示定位/牵引幅度，闭环上限由max_duty独立给出。 */
    six_step_config_t startup;
    /* 极对数、交接最低估速 RPM；handoff_rpm 为 0 时只使用 min_rpm。 */
    uint32_t pole_pairs, handoff_rpm;
    /* 比例/积分参数除以 1000，单位分别为千分比/RPM、千分比/(RPM*s)。 */
    uint32_t kp_milli, ki_milli;
    /* 电角度的十分之一，正数提前；0 为过零后延迟 30 度。 */
    int16_t comm_timing_decideg;
    /* 闭环可采样幅度范围，单位千分比；max_duty不得超过板卡已验证上限。 */
    uint16_t min_duty, max_duty;
    /* 同步导通窗口原始 ADC 码门限及回差；上下沿分别标定。 */
    uint16_t rising_threshold, falling_threshold, hysteresis;
    /* true 时永不接管，缺少过零也不产生观测器故障。 */
    bool observe_only;
    /* 正式中断路径使用可信硬件上下文，PI由独立低频中断状态持有。 */
    bool irq_control;
    /* 退磁消隐下限us（动态窗口取20%扇区）、最低资格RPM、连续事件数；超时从最低交接电频率起算。 */
    uint32_t blank_us, min_rpm, lock_count, timeout_us;
    /* 各方向从理想过零到观测过阈的标定延迟 us，事件时间须减去该值。 */
    uint32_t rising_delay_us, falling_delay_us;
} sensorless_six_step_config_t;

typedef struct {
    /* 最新同步导通样本及其浮空相、已提交扇区；不能填入候选扇区。 */
    uint16_t raw;
    uint8_t floating, sector;
    /* 无有效导通窗口、跳脉冲和未完成转换必须为 false；实际换相后消隐内的预期无效保留事件资格。 */
    bool valid;
    /* 样本序号、采样时刻、该扇区实际换相时刻，全部 us 时间属于同一时钟。 */
    uint32_t sequence, sample_us, committed_us;
    /* 本次候选可提交的下一峰顶和当前时刻，单位 us，下一峰顶必须尚未发生。 */
    uint32_t next_commit_us, now_us;
    /* 已通过后台校验的最新机械目标 RPM；改变时才更新开环电频率。 */
    uint32_t target_rpm;
    /* 软限流抑制增加输出，允许幅度及积分回退；保护关断仍由后端优先执行。 */
    bool current_limited;
} sensorless_six_step_input_t;

typedef struct {
    /* 停机装入静态配置；运行观测状态由ADC中断独占。 */
    sensorless_six_step_config_t config;
    /* 交接前使用原六步算法，同步阶段起冻结开环并保留启动历史。 */
    six_step_t open_loop;
    /* 下一峰顶的扇区候选、占空比千分比及可读阶段。 */
    uint8_t desired_sector;
    volatile uint16_t duty;
    sensorless_six_step_stage_t stage;
    /* 总运行、可信估速、连续资格及实际接管状态。 */
    volatile bool running, locked;
    bool estimate_valid, qualified;
    /* 连续资格已通过，冻结开环并等待首次过零预约的实际COM确认。 */
    bool synchronizing;
    /* 整电周期估算的机械RPM，四舍五入到整数；有效位为false时仅供诊断。 */
    volatile uint32_t estimated_rpm;
    /* 有效事件、拒绝、无效样本、重复样本、消隐及迟到排程计数。 */
    uint32_t event_count, rejected_count, invalid_count, duplicate_count, blanked_count, late_count;
    /* 当前连续事件数、估计 60 度周期、理想下一换相时刻和总控制周期数。 */
    uint32_t consecutive_events, sector_period_us, next_commutation_us, ticks;
    /* 最近事件相对预期扇区中点的偏差 us；正数表示偏晚。 */
    int32_t phase_error_us;
    /* 静态故障文本；无故障为 NULL，不在 ISR 格式化。 */
    const char *fault;
    /* 以下为观察器内部历史，正式固件仅由ADC中断修改。 */
    bool have_sequence, have_sector, armed, crossing_valid, sector_event;
    bool have_previous, have_event, have_valid_sample, acquisition_started;
    bool deadline_valid, commutation_pending, limit_latched;
    uint8_t observed_sector, last_event_sector, history_index, history_count;
    /* 扇区切换时装入门限与斜率，逐样本不重复换算。 */
    uint16_t detector_lower, detector_upper;
    bool detector_rising;
    /* 连续跨过回差的有效样本数；至少两个才确认首次插值过零时刻。 */
    uint8_t after_samples;
    uint16_t previous_raw;
    uint32_t last_sequence, observed_committed_us, previous_sample_us, crossing_us;
    uint32_t last_valid_sample_us, last_event_us, acquisition_started_us, submitted_us;
    /* 最新目标缓存及启动时预计算的最低交接电频率 mHz，避免在20 kHz重复64位换算。 */
    uint32_t target_rpm, acquisition_frequency_mhz;
    /* 开环周期只在频率改变时换算；退磁消隐随60度周期改变，单位us。 */
    uint32_t period_frequency_mhz, open_period_us, blanking_us;
    /* 六个同方向电周间隔端点的循环缓冲；第七个事件产生首个整周估速。 */
    uint32_t event_history[6];
    /* PI 积分为百万分之一占空比千分比，按绝对时间实现 1 kHz 更新。 */
    int64_t integral_micro;
    uint32_t pi_ticks;
    /* 历史主机模型的绝对PI时基及有效样本间隙预算。 */
    uint32_t pi_last_us, gap_us;
} sensorless_six_step_t;

/* 速度积分由TIM低频分支独占，ADC只发布转速和限流计数，不写该状态。 */
typedef struct {
    int64_t integral_micro;
    uint16_t duty;
    bool initialized;
} sensorless_six_step_pi_t;

/** @brief ADC装入TIM已完整发布的启动进度；不复制完整启动结构。
 * @param state ADC独占观测器。 @param ticks PWM总周期数。 @param frequency_mhz 启动电频率。
 * @param sector 实际启动进度扇区。 @param duty 千分比幅度。 @param stage 启动阶段。
 * @param now_us 本次运行微秒。 */
void sensorless_six_step_progress_irq(sensorless_six_step_t *state, uint32_t ticks,
    uint32_t frequency_mhz, uint8_t sector, uint16_t duty, six_step_stage_t stage, uint32_t now_us);
/** @brief PWM同步ADC中断执行轻量检测，事件时才更新资格/估速/换相期限。
 * @param state ADC独占状态。 @param input 来自板卡的可信同步上下文。
 * @note 从PULLIN开始观测，定位阶段忽略；首次同步及500ms资格等待仍从原RAMP阶段开始。
 * @note 不检查软件队列年龄/重复序号，原始码和扇区由硬件适配保证。 */
void sensorless_six_step_sample_irq(sensorless_six_step_t *state, const sensorless_six_step_input_t *input);
/** @brief ADC按1ms时基检查反馈及启动期限，速度PI由独立入口执行。
 * @param state ADC独占状态。 @param now_us 当前运行微秒。 */
void sensorless_six_step_check_time(sensorless_six_step_t *state, uint32_t now_us);
/** @brief 1kHz中断更新独立速度PI；首次闭环调用以当前幅度无跳变初始化。
 * @param pi TIM独占积分器。 @param config 不变的控制配置。 @param target_rpm 目标机械转速。
 * @param rpm 最新事件估速。 @param initial_duty 首次接管幅度。 @param limited 两次PI间是否发生限流；发生时主动退幅。
 * @return 下一PWM采用的千分比占空比。 */
uint16_t sensorless_six_step_pi_update(sensorless_six_step_pi_t *pi,
    const sensorless_six_step_config_t *config, uint32_t target_rpm, uint32_t rpm,
    uint16_t initial_duty, bool limited);

#ifdef MOTOR_SENSORLESS_TEST_TICK
/** @brief 历史主机异步模型导入启动进度；正式固件使用progress_irq。
 * @param state 主机模型独占观测器。 @param startup 启动器快照。 @param now_us 当前运行微秒。 */
void sensorless_six_step_progress(sensorless_six_step_t *state, const six_step_t *startup, uint32_t now_us);
/** @brief 消费一帧原始同步样本，仅检测及事件更新，不推进启动或PI。
 * @param state 主机模型独占状态。 @param input 原始样本与实际COM元数据。 */
void sensorless_six_step_sample(sensorless_six_step_t *state, const sensorless_six_step_input_t *input);
/** @brief 按绝对时间检查反馈期限，并以1ms时基运行速度PI。
 * @param state 主机模型独占状态。 @param now_us 当前运行微秒。 @param target_rpm 机械目标。
 * @param limited 自上次服务后是否出现软限流。 */
void sensorless_six_step_service(sensorless_six_step_t *state, uint32_t now_us, uint32_t target_rpm, bool limited);
#endif
/** @brief 查询事件生成的完整预约，成功发布后再ack；不使用样本中陈旧的未来峰顶。
 * @param state ADC独占状态。 @param sector 下一扇区输出。 @param deadline_us 最早允许换相时刻输出。
 * @return 有尚未发布的预约。 */
bool sensorless_six_step_schedule(const sensorless_six_step_t *state, uint8_t *sector, uint32_t *deadline_us);
/** @brief 记录板卡接受预约的首个峰顶，等待新样本中的真实COM确认。
 * @param state ADC独占状态。 @param commit_us 板卡返回的预约峰顶。 */
void sensorless_six_step_schedule_ack(sensorless_six_step_t *state, uint32_t commit_us);

/** @brief 填入仅供观察的保守门限；startup、极对数、PI 和 min_duty 仍须后端填写。
 * @param config 配置输出；默认门限及延迟尚未经过本板实测标定。 */
void sensorless_six_step_defaults(sensorless_six_step_config_t *config);
/** @brief 清空历史并复用现有开环启动，所有静态参数必须在调用前通过后台校验。
 * @param state 独占状态。
 * @param config 有效配置；start/target/ramp/pole_pairs非零，run_ticks覆盖启动，min_duty<=startup.duty<=max_duty<=1000。
 * @note 后台须验证 min_rpm>=1000、handoff_rpm<=机械目标、lock_count>=7、COMM_TIMING 在 +/-200 十分之一度；
 * 12位阈值加减回差须在0..4095，所有 us 间隔小于2^31；最高电频率下的过零位置须晚于 blank+2PWM，
 * 补偿后过零至换相的延迟须大于2PWM，方向延迟必须在上述采样/排程窗口内。 */
void sensorless_six_step_start(sensorless_six_step_t *state, const sensorless_six_step_config_t *config);
#ifdef MOTOR_SENSORLESS_TEST_TICK
/** @brief 仅历史同步主机模型使用；正式固件不编译，禁止放回ADC ISR。
 * @param state 主机模型独占状态；停机后调用无副作用。
 * @param input 本周期最新样本及未来峰顶；样本序号和时间戳允许 uint32 自然回绕，差值须小于2^31。
 * @note raw 为0..4095；只支持 AB/AC/BC/BA/CA/CB 正向相序，浮空相对应 C/B/A/C/B/A。
 * 样本必须归属已实际提交的扇区，sample_us>=committed_us 且 age<=2PWM；预约峰顶必须在当前时刻后一个PWM内。
 * 后端实际COM须在预约峰顶的[-1us,+1PWM)窗口内，首个换相周期可给出带新扇区元数据的消隐内无效样本。
 * @note 提交 desired_sector/duty 前，后端须优先处理 state->fault 或 !state->running。 */
void sensorless_six_step_tick(sensorless_six_step_t *state, const sensorless_six_step_input_t *input);
#endif
/** @brief 获取诊断阶段，不把观察资格当作闭环接管。
 * @param stage 阶段枚举。
 * @return 常量 ASCII 名称。 */
const char *sensorless_six_step_stage_name(sensorless_six_step_stage_t stage);
#endif
