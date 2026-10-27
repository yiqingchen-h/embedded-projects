#ifndef SOFTWARE_DISPLAY_SPEED_CONTROL_H_
#define SOFTWARE_DISPLAY_SPEED_CONTROL_H_

#include "hal_data.h"
#include "hardware/Chassis/Chassis.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct st_display_speed_control
{
    /* 前进方向速度，单位 m/s。 */
    float vx_mps;

    /* 横移方向速度，单位 m/s。 */
    float vy_mps;

    /* 旋转角速度，单位 rad/s。 */
    float wz_radps;
} ui_speed_control;

typedef struct st_display_speed_control_api
{
    /**
     * @brief  设置底盘调参控制使能
     *
     * @param enabled true 表示允许底盘控制任务传入速度
     * @return void
     */
    void (*set_enable)(bool enabled);

    /**
     * @brief  设置整体速度控制值
     *
     * @param p_control 整体速度控制值
     * @return void
     */
    void (*set_body)(const ui_speed_control *p_control);

    /**
     * @brief  设置四轮独立速度控制值
     *
     * @param wheel_speed_mps 四轮速度目标，单位 m/s
     * @return void
     */
    void (*set_wheel)(const float wheel_speed_mps[CHASSIS_WHEEL_COUNT]);

    /**
     * @brief  设置四轮开环 PWM 诊断控制值
     *
     * @param pwm_counts 四轮 PWM 计数，正负号表示方向
     * @return void
     */
    void (*set_pwm)(const int32_t pwm_counts[CHASSIS_WHEEL_COUNT]);

    /**
     * @brief  清空底盘调参控制值并关闭使能
     *
     * @return void
     */
    void (*clear)(void);

    /**
     * @brief  读取底盘调参控制值
     *
     * @param p_control         整体速度控制值输出指针
     * @param wheel_speed_mps   四轮速度目标输出指针
     * @param pwm_counts        四轮开环 PWM 目标输出指针
     * @param p_mode            当前控制模式输出指针
     * @return bool true 表示当前允许底盘控制任务传入速度
     */
    bool (*read)(ui_speed_control *p_control,
                 float wheel_speed_mps[CHASSIS_WHEEL_COUNT],
                 int32_t pwm_counts[CHASSIS_WHEEL_COUNT],
                 chassis_control_mode_t *p_mode);
} display_speed_control_api_t;

extern const display_speed_control_api_t display_speed_control;

#endif /* SOFTWARE_DISPLAY_SPEED_CONTROL_H_ */
