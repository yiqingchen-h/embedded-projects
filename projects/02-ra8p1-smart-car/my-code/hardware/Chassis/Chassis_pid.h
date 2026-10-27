#ifndef CHASSIS_PID_H_
#define CHASSIS_PID_H_

#include "hardware/Chassis/Chassis_config.h"

typedef struct st_chassis_pid_controller
{
    float kp;
    float ki;
    float output_limit;
    float output;
    float last_error;
} chassis_pid_controller_t;

typedef struct st_chassis_pid_api
{
    /**
     * @brief  初始化单路底盘 PI 控制器
     *
     * @param p_ctrl        控制结构体
     * @param kp            比例系数
     * @param ki            积分系数
     * @param output_limit  输出绝对值上限
     * @return void
     */
    void (*init)(chassis_pid_controller_t *p_ctrl, float kp, float ki, float output_limit);

    /**
     * @brief  复位单路底盘 PI 控制器
     *
     * @param p_ctrl  控制结构体
     * @return void
     */
    void (*reset)(chassis_pid_controller_t *p_ctrl);

    /**
     * @brief  计算单路底盘 PI 输出
     *
     * @param p_ctrl    控制结构体
     * @param measured  测量值
     * @param target    目标值
     * @return float    PI 输出值
     */
    float (*calculate)(chassis_pid_controller_t *p_ctrl, float measured, float target);
} chassis_pid_api_t;

extern const chassis_pid_api_t chassis_pid;

#endif /* CHASSIS_PID_H_ */
