#ifndef HARDWARE_CHASSIS_CHASSIS_AVOIDANCE_H_
#define HARDWARE_CHASSIS_CHASSIS_AVOIDANCE_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum e_chassis_avoidance_state
{
    /* 目标方向安全，直接透传上层速度。 */
    CHASSIS_AVOIDANCE_STATE_NORMAL = 0,

    /* 前向视场危险，使用 FOV-lite 生成横移避障速度。 */
    CHASSIS_AVOIDANCE_STATE_AVOID,

    /* 障碍解除后，使用编码器短时里程计尝试回到原轨道。 */
    CHASSIS_AVOIDANCE_STATE_RETURN_TRACK,

    /* 雷达异常、近距离危险或无安全方向时停车。 */
    CHASSIS_AVOIDANCE_STATE_FAIL_SAFE,
} chassis_avoidance_state_t;

typedef struct st_chassis_avoidance_status
{
    /**
     * @brief  当前避障状态
     */
    chassis_avoidance_state_t state;

    /**
     * @brief  当前使用的完整雷达容器序号
     */
    uint32_t radar_sequence;

    /**
     * @brief  雷达数据未更新累计时间，单位 ms
     */
    uint32_t radar_stale_ms;

    /**
     * @brief  上层期望运动方向，单位度
     */
    float target_angle_deg;

    /**
     * @brief  避障模块最终选择方向，FOV-lite 中表示横移避障方向，单位度
     */
    float selected_angle_deg;

    /**
     * @brief  前向视场内统计到的最近障碍距离，单位 mm
     */
    uint16_t nearest_distance_mm;

    /**
     * @brief  回轨横向误差估计，单位 m
     */
    float lateral_error_m;

    /**
     * @brief  前向视场是否安全
     */
    bool target_safe;
} chassis_avoidance_status_t;

typedef struct st_chassis_avoidance_api
{
    /**
     * @brief  初始化避障底盘入口模块
     *
     * @return fsp_err_t FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  带避障仲裁的底盘速度入口
     *
     * @param vx_mps       上层期望前进速度，单位 m/s
     * @param vy_mps       上层期望横移速度，单位 m/s
     * @param wz_radps     上层期望旋转速度，单位 rad/s
     * @param interval_ms  控制调用周期，单位 ms
     * @return fsp_err_t   FSP_SUCCESS 表示仲裁完成
     */
    fsp_err_t (*control)(float vx_mps, float vy_mps, float wz_radps, uint32_t interval_ms);

    /**
     * @brief  停止底盘并进入安全状态
     *
     * @return void
     */
    void (*stop)(void);

    /**
     * @brief  复位避障状态机和短时里程计
     *
     * @return void
     */
    void (*reset)(void);

    /**
     * @brief  读取避障模块运行状态
     *
     * @param p_status  状态输出指针
     * @return fsp_err_t FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*status_get)(chassis_avoidance_status_t *p_status);
} chassis_avoidance_api_t;

extern const chassis_avoidance_api_t chassis_avoidance;

#endif /* HARDWARE_CHASSIS_CHASSIS_AVOIDANCE_H_ */
