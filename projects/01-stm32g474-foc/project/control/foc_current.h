#ifndef FOC_CONTROL_FOC_CURRENT_H
#define FOC_CONTROL_FOC_CURRENT_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* 独立谐波旋转坐标的学习电压和未来dq补偿，单位V。 */
    float x_v, y_v, d_v, q_v;
    /* 仅在实际CLOSED且Id退出后开启，停机关闭并清零。 */
    bool enabled;
    /* 此支路自身电压触限次数。 */
    uint32_t limited;
} foc_harmonic2_t;

typedef struct {
    /* 启动时根据模型确定的PI参数，ki_step已包含采样周期。 */
    float kp, ki_step, modulation;
    /* dq积分电压及最近真实电流，单位V/A；仅实时控制上下文修改。 */
    float integral_d, integral_q, id_a, iq_a;
    /* 最近指令与逆Park后的请求，仅供未来PWM，不代表已施加电压。 */
    float reference_d, reference_q, request_alpha_v, request_beta_v;
    /* 矢量限幅后、逆Park前的dq电压指令V；用于比较基准，不是已施加电压。 */
    float command_d_v, command_q_v;
    /* 本拍有符号电角速度乘Ld/Lq及永磁磁链，单位V/A、V；初始化为0表示关闭。 */
    float speed_ld_v_per_a, speed_lq_v_per_a, bemf_q_v;
    /* 负序五次旋转坐标中的积分电压V；步进增益V/A、矢量上限V。 */
    float harmonic_x_v, harmonic_y_v, harmonic_step, harmonic_limit_v;
    /* 未来作用时刻的dq谐波补偿电压V，仅供实时状态查询。 */
    float harmonic_d_v, harmonic_q_v;
    /* 初始化关闭，由启动状态机在实际CLOSED且Id退出后启用。 */
    bool harmonic_enabled;
    /* 谐波支路自身达到电压上限的次数，不代替总矢量饱和计数。 */
    uint32_t harmonic_limited;
    /* 与五次独立开关的负序二次支路，复用相同采样周期和学习增益。 */
    foc_harmonic2_t harmonic2;
    /* 正序七次支路与五次共用正式接管门控；未来dq为正六次，独立限幅0.1V。 */
    foc_harmonic2_t harmonic7;
    /* 不变的比较范围与饱和计数。 */
    uint16_t arr, minimum_ccr, maximum_ccr;
    uint32_t limited;
    /* 仅指定控制器启用的梯形积分；默认仍矩形，历史误差A不参与保护或遥测。 */
    bool trapezoidal;
    float previous_error_d, previous_error_q;
} foc_current_t;

/** @brief 以Kp=L·ωc、Ki=R·ωc初始化dq电流PI和采样窗口限幅。
 * @param loop 输出状态。 @param resistance_ohm 候选等效相电阻Ω。
 * @param inductance_h 候选相电感H。 @param bandwidth_hz 电流环带宽Hz。
 * @param sample_s 周期s。 @param arr 中心对齐ARR。
 * @param minimum_ccr 最小比较值。 @param maximum_ccr 最大比较值。
 * @return 参数有限、周期和调制范围有效；调用前输出必须关闭。 */
bool foc_current_init(foc_current_t *loop, float resistance_ohm, float inductance_h,
    float bandwidth_hz, float sample_s, uint16_t arr, uint16_t minimum_ccr, uint16_t maximum_ccr);

/** @brief 选择Sguan参考的梯形积分，保留积分电压与原矢量抗饱和。
 * @param loop 已初始化电流环。 @param enabled 是否使用相邻两拍误差平均积分。
 * @note 仅实时控制上下文调用；不改Kp/Ki、限流、采样或CCR范围。 */
void foc_current_trapezoidal(foc_current_t *loop, bool enabled);

/** @brief 更新本拍交叉解耦系数及反电动势前馈，实际Id/Iq在step内使用同拍Park结果。
 * @param loop 已初始化电流环。 @param omega_rad_s 实测有符号电角速度rad/s，不能传机械速度。
 * @param ld_h 已在启动前校验的d轴相电感H。 @param lq_h 已校验的q轴相电感H。
 * @param flux_wb 永磁基波相峰值磁链Wb，0可单独关闭反电动势项。
 * @note 仅控制上下文调用；动态非有限值由随后step的总电压检查拒绝。 */
void foc_current_feedforward(foc_current_t *loop, float omega_rad_s,
    float ld_h, float lq_h, float flux_wb);

/** @brief 在采样角度Park、dq PI/前馈共同矢量限幅及抗饱和后，用未来作用角逆Park/SVPWM。
 * @param loop 已初始化状态。 @param alpha_a 同步Clarke电流A。 @param beta_a 同步Clarke电流A。
 * @param sample_cos 采样时刻角度余弦。 @param sample_sin 采样时刻角度正弦。
 * @param future_cos 下一PWM平均作用时刻余弦。 @param future_sin 下一PWM平均作用时刻正弦。
 * @param bus_v 最新有效母线V。 @param reference_d Id指令A。 @param reference_q Iq指令A。
 * @param ccr 输出经限幅最终比较值；仅是未来请求，必须由硬件UEV另行确认生效。
 * @return 本次计算有限且调制成功，否则调用者关闭输出。 */
bool foc_current_step(foc_current_t *loop, float alpha_a, float beta_a,
    float sample_cos, float sample_sin, float future_cos, float future_sin,
    float bus_v, float reference_d, float reference_q, uint16_t ccr[3]);

/** @brief 切换负序五次及随附正序七次积分支路；切换时清零，不修改原PI增益。
 * @param loop 电流环状态。 @param enabled 是否启用。
 * @note 仅控制上下文或屏蔽控制中断后调用；七次编译期对照开关可单独禁用。 */
void foc_current_harmonic(foc_current_t *loop, bool enabled);

/** @brief 切换负序二次积分支路；切换时清零学习电压，五次支路保持。
 * @param loop 电流环状态。 @param enabled 是否启用。
 * @note 仅控制上下文或屏蔽控制中断后调用；增益20V/(A*s)，上限0.1V。 */
void foc_current_harmonic2(foc_current_t *loop, bool enabled);

/** @brief 接管时只旋转坐标变化引入的积分状态，不重复旋转正常电角速度增量。
 * @param loop 电流PI。 @param delta_cos 坐标修正角余弦。 @param delta_sin 坐标修正角正弦。 */
void foc_current_rotate(foc_current_t *loop, float delta_cos, float delta_sin);
#endif
