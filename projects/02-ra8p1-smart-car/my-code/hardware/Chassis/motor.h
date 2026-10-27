#ifndef CHASSIS_MOTOR_H_
#define CHASSIS_MOTOR_H_

#include "hal_data.h"
#include "hardware/Chassis/Chassis_config.h"
/**
 * @brief 
 * 
 */
typedef struct st_motor_api
{
    /**
     * @brief  初始化四路电机 PWM 定时器
     *
     * @return fsp_err_t  FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  设置单路电机 PWM
     *
     * @param wheel       电机编号
     * @param pwm_counts  有符号 PWM 计数
     * @return fsp_err_t  FSP_SUCCESS 表示设置成功
     */
    fsp_err_t (*set_pwm)(chassis_wheel_t wheel, int32_t pwm_counts);

    /**
     * @brief  设置四路电机 PWM
     *
     * @param pwm_counts  四路有符号 PWM 计数数组
     * @return fsp_err_t  FSP_SUCCESS 表示设置成功
     */
    fsp_err_t (*set_all_pwm)(const int32_t pwm_counts[CHASSIS_WHEEL_COUNT]);

    /**
     * @brief  停止所有电机输出
     *
     * @return void
     */
    void (*stop_all)(void);

    /**
     * @brief  获取电机 PWM 周期计数
     *
     * @return uint32_t  当前 PWM 周期计数
     */
    uint32_t (*period_counts)(void);
} motor_api_t;

extern const motor_api_t motor;

#endif /* CHASSIS_MOTOR_H_ */
