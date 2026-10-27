#ifndef FOC_CONTROL_FOC_SMO_H
#define FOC_CONTROL_FOC_SMO_H
#include "foc_voltage.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* 停机初始化的离散RL、边界层和低通参数；不在中断重复计算。 */
    float sample_s, plant_a, plant_b, gain_v, boundary_a, lpf_a, observer_pole;
    /* 隐式边界层直流增益逆值；二/五次系数仅在本次预测完成后学习，单位V。 */
    float emf_gain_inverse, harmonic_step, harmonic_x_v[2], harmonic_y_v[2];
    /* 最大20Hz/0.707 PLL；上次估计低于25Hz电频率时自然频率减半，50Hz以上保持原值。 */
    float pll_kp, pll_ki_step, omega_limit, bemf_min_v;
    /* 同区间电流预测和边界层估计，仅保留控制必要的上次状态。 */
    float estimated_alpha_a, estimated_beta_a, previous_alpha_a, previous_beta_a;
    /* 总滑模低通及分离受限二/五次后的基波区间反电动势，单位V。 */
    float filtered_alpha_v, filtered_beta_v, emf_alpha_v, emf_beta_v;
    /* PLL中点角度、积分速度、估计速度及本次采样时刻预测角度。 */
    float pll_theta_rad, omega_integral, omega_rad_s, sample_theta_rad;
    /* 振幅、归一化相位误差及独立单步电流模型残差，供接管资格。 */
    float emf_magnitude_v, phase_error, residual_a;
    bool sample_initialized, pll_initialized;
    uint32_t updates, sample_ticks;
} foc_smo_t;

/** @brief 停机初始化隐式边界层SMO、离散低通及独立PLL。
 * @param observer 输出状态。 @param resistance_ohm 已识别相电阻Ω。
 * @param inductance_h 已识别等效相电感H。 @param sample_s 周期s。
 * @param gain_v 滑模增益V。 @param boundary_a 电流边界层A。
 * @param filter_hz 反电动势低通截止Hz。 @param pll_hz PLL自然频率Hz。
 * @param maximum_electrical_hz 估计速度边界Hz。 @param bemf_min_v 启动PLL最低振幅V。
 * @param sample_ticks 配对区间必须具有的硬件tick长度。
 * @return 静态参数有效。 */
bool foc_smo_init(foc_smo_t *observer,float resistance_ohm,float inductance_h,float sample_s,
    float gain_v,float boundary_a,float filter_hz,float pll_hz,float maximum_electrical_hz,float bemf_min_v,uint32_t sample_ticks);

/** @brief 用因果估计速度逆补偿离散一阶极点的幅值和相位；仅使用上次独立估速。
 * @param pole 离散极点0..1。 @param omega_rad_s 上次估计电角速度。
 * @param sample_s 周期。 @param alpha_v 输入/输出矢量。 @param beta_v 输入/输出矢量。
 * @return 动态值有效；运行范围要求|omega*Ts|<=0.2。 */
bool foc_smo_compensate(float pole,float omega_rad_s,float sample_s,float *alpha_v,float *beta_v);

/** @brief 首样本只初始化；其后仅接收与两次电流采样配对的已施加区间电压。
 * @param observer 独立估计状态。 @param applied 已完成区间；首样本传NULL。
 * @param alpha_a 同时Clarke电流。 @param beta_a 同时Clarke电流。
 * @return 更新有效；缺样/区间错误/非有限值时返回false并撤销PLL资格。
 * @note 接口不接受PWM请求、目标转速或强拖频率，无法用未来请求解释过去电流。 */
bool foc_smo_step(foc_smo_t *observer,const foc_voltage_interval_t *applied,float alpha_a,float beta_a);
#endif
