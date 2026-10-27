#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "hal_data.h"
#include "hardware/Chassis/kinematics.h"

typedef enum e_chassis_control_mode
{
    /* 车体速度控制模式，通过 vx/vy/wz 逆解四轮目标速度。 */
    CHASSIS_CONTROL_MODE_BODY = 0,

    /* 单轮速度控制模式，直接使用四路轮速目标。 */
    CHASSIS_CONTROL_MODE_WHEEL,

    /* 开环 PWM 诊断模式，绕过编码器和 PI 直接输出四路 PWM。 */
    CHASSIS_CONTROL_MODE_PWM,
} chassis_control_mode_t;

typedef struct st_chassis_pid_params
{
    /* 速度环比例系数。 */
    float kp;

    /* 速度环积分系数。 */
    float ki;
} chassis_pid_params_t;

typedef struct st_chassis_api
{
    /**
     * @brief  初始化底盘闭环控制模块
     *
     * @return fsp_err_t  FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  设置底盘目标车体速度
     *
     * @param vx_mps    前进方向速度，单位 m/s
     * @param vy_mps    横移方向速度，单位 m/s
     * @param wz_radps  旋转角速度，单位 rad/s
     * @return void
     */
    void (*set_velocity)(float vx_mps, float vy_mps, float wz_radps);

    /**
     * @brief  设置四路轮目标速度
     *
     * @param wheel_speed_mps  四路轮速目标，单位 m/s
     * @return void
     */
    void (*set_wheel_speed)(const float wheel_speed_mps[CHASSIS_WHEEL_COUNT]);

    /**
     * @brief  设置四路开环 PWM 诊断输出
     *
     * @param pwm_counts 四路 PWM 计数，正负号表示方向
     * @return void
     */
    void (*set_pwm)(const int32_t pwm_counts[CHASSIS_WHEEL_COUNT]);

    /**
     * @brief  设置四路共用速度环 PI 参数
     *
     * @param kp  比例系数
     * @param ki  积分系数
     * @return void
     */
    void (*set_pid)(float kp, float ki);

    /**
     * @brief  获取四路共用速度环 PI 参数
     *
     * @param p_params  参数输出指针
     * @return void
     */
    void (*get_pid)(chassis_pid_params_t *p_params);

    /**
     * @brief  执行一次底盘闭环控制
     *
     * @param interval_ms  控制周期，单位 ms
     * @return fsp_err_t   FSP_SUCCESS 表示本次控制成功
     */
    fsp_err_t (*control_step)(uint32_t interval_ms);

    /**
     * @brief  停止底盘并清空速度目标
     *
     * @return void
     */
    void (*stop)(void);

    /**
     * @brief  获取当前底盘目标速度
     *
     * @return const chassis_velocity_t*  当前目标速度只读指针
     */
    const chassis_velocity_t *(*target_velocity)(void);

    /**
     * @brief  获取当前四路轮目标速度
     *
     * @return const float*  四路轮目标速度只读指针
     */
    const float *(*target_wheel_speed)(void);

    /**
     * @brief  获取当前底盘控制模式
     *
     * @return chassis_control_mode_t  当前控制模式
     */
    chassis_control_mode_t (*control_mode)(void);
} chassis_api_t;

extern const chassis_api_t chassis;

#endif /* CHASSIS_H_ */
