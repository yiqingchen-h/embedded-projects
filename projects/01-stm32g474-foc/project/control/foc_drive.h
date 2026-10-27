#ifndef FOC_CONTROL_FOC_DRIVE_H
#define FOC_CONTROL_FOC_DRIVE_H
#include "foc_current.h"
#include "foc_smo.h"

typedef enum {
    FOC_STOPPED, FOC_CALIBRATE, FOC_ALIGN, FOC_I_F_RAMP, FOC_QUALIFY, FOC_BLEND, FOC_CLOSED
} foc_drive_stage_t;

typedef struct {
    /* 与六步分离的相模型、电流限值、噪声及电流环带宽。 */
    float resistance_ohm, inductance_h, current_limit_a, emf_noise_v, current_bandwidth_hz;
    /* 实测PWM平均模型误差上界V；与噪声门限分别处理，不允许误差被SMO当作转子EMF。 */
    float voltage_error_bound_v;
    /* 机械目标正转；G2208档上限1800RPM，原电机档保留既有范围。 */
    uint32_t target_rpm, pole_pairs;
    /* 独立启动接管机械速度RPM；0沿用本档约50Hz电频率对应值，不作为SMO输入。 */
    uint32_t startup_rpm;
    /* 本机额定速度内试验：扩展目标/观察带宽及速度越限范围，角度质量保护保持。 */
    bool extended_speed_range;
    /* 速度PI定点输入前转换为A/RPM及A/(RPM*s)，速度参考有斜坡。 */
    float speed_kp, speed_ki, speed_ramp_rpm_s;
} foc_drive_config_t;

typedef struct {
    foc_current_t current;
    foc_smo_t observer;
    foc_drive_config_t config;
    /* 固定噪声门限/R-L误差系数在初始化预计算；每个真实采样区间更新可辨识裕量。 */
    float emf_floor_v,resistance_error_ohm,inductance_error_v_per_a,model_bound_v,emf_margin_v;
    /* ADC拥有启动、资格、融合和电流状态；谷底只确认接管请求已生效。 */
    foc_drive_stage_t stage;
    uint32_t samples, qualify_cycles, qualify_begun, blend_begun, closed_begun, pending_sequence;
    /* 强拖角为定子电流方向rad，速度为电角rad/s；资格相位为估计转子角减强拖角。 */
    /* 已施加区间有功功率W的快/慢滤波状态，仅用于I/f频率阻尼，不是诊断缓存。 */
    float power_fast_w, power_mean_w;
    /* 初始化预计算的接管电角速度rad/s，避免在ADC重复换算。 */
    float startup_omega_rad_s;
    float force_theta_rad, force_omega_rad_s, qualify_phase_rad, correction_rad;
    /* 真正用于当前采样及未来PWM的角度/速度，不与强拖值混为一谈。 */
    float control_theta_rad, control_omega_rad_s, reference_d_a, reference_q_a, closed_initial_d_a;
    /* TIM的1kHz速度环状态，ADC只读取对齐的Iq结果。 */
    float speed_reference_rpm, speed_integral_a, speed_q_a;
    uint32_t speed_ticks, closed_samples;
    /* ADC独占的连续速度软越限计数，恢复正常范围即清零；角度质量不使用此延时。 */
    uint32_t speed_range_cycles;
    bool awaiting_commit, request_estimated_angle, closed_seen;
    /* 当前运行的固定Iq实验开关；停机和初始化清除，不改变保存的速度PI参数。 */
    bool hold_iq;
    /* 会话谐波抑制请求；实际启用还要求CLOSED和定位Id完全退出。 */
    volatile bool harmonic_requested;
    /* 主循环短临界区设置二次开关，ADC只在正式闭环后采用。 */
    volatile bool harmonic2_requested;
    /* 静态故障文本，调用者收到false时立即关闭硬件，不回退强拖。 */
    const char *fault;
} foc_drive_t;

/** @brief 停机初始化电流/观察器/速度配置，保持CALIBRATE直到显式开始。
 * @param drive 状态。 @param config 已验证设置。 @return 初始化成功。 */
bool foc_drive_init(foc_drive_t *drive,const foc_drive_config_t *config);
/** @brief 硬件校准及采样准备完成后开始ALIGN；调用时输出尚未打开。 */
void foc_drive_start(foc_drive_t *drive);
/** @brief ADC路径：已施加电压观察→资格/角融合→电流PI→未来PWM。
 * @param drive 状态。 @param applied 本次采样区间；首样本NULL。
 * @param alpha_a 同时电流。 @param beta_a 同时电流。 @param bus_v 有效母线。
 * @param sequence 下一命令序号。 @param future_s 采样至下一PWM平均作用中心的秒数。
 * @param ccr 输出未来请求。 @return 控制有效，否则立即关断。 */
bool foc_drive_sample(foc_drive_t *drive,const foc_voltage_interval_t *applied,float alpha_a,float beta_a,
    float bus_v,uint32_t sequence,float future_s,uint16_t ccr[3]);
/** @brief 硬件谷底提交确认后才允许CLOSED；预装载不能调用此接口。
 * @param drive 状态。 @param sequence 真实生效编号。 @param estimated_angle 实际记录的角度来源。 */
void foc_drive_commit(foc_drive_t *drive,uint32_t sequence,bool estimated_angle);
/** @brief 仅TIM的1kHz路径运行速度PI；未接管时不计算或积分。
 * @param drive 状态。 @param target_rpm 已验证机械目标。 */
void foc_drive_speed_tick(foc_drive_t *drive,uint32_t target_rpm);
/** @brief 完成接管及Id退出后临时固定Iq，保留估计器和全部运行保护。
 * @param drive 实际闭环状态。 @param current_a 0恢复速度PI，正值为固定Iq，单位A。
 * @return 状态和电流限幅满足要求且已应用；调用者须屏蔽控制中断。 */
bool foc_drive_hold_iq(foc_drive_t *drive,float current_a);
/** @brief 关闭输出时取消接管资格与待提交请求，保留必要验收统计。
 * @param drive 状态。 @param reason 故障或NULL正常停机。 */
void foc_drive_stop(foc_drive_t *drive,const char *reason);
/** @brief 主循环诊断的阶段名，不在中断格式化。 */
const char *foc_drive_stage_name(foc_drive_stage_t stage);
#endif
