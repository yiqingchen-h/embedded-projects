#ifndef CHASSIS_KINEMATICS_H_
#define CHASSIS_KINEMATICS_H_

#include "hardware/Chassis/Chassis_config.h"

typedef struct st_chassis_velocity
{
    float vx_mps;
    float vy_mps;
    float wz_radps;
} chassis_velocity_t;

typedef struct st_kinematics_api
{
    /**
     * @brief  麦轮底盘逆运动学解算
     *
     * @param p_velocity         车体目标速度
     * @param wheel_targets_mps  输出四路轮目标速度，单位 m/s
     * @return void
     */
    void (*inverse)(const chassis_velocity_t *p_velocity, float wheel_targets_mps[CHASSIS_WHEEL_COUNT]);
} kinematics_api_t;

extern const kinematics_api_t kinematics;

#endif /* CHASSIS_KINEMATICS_H_ */
