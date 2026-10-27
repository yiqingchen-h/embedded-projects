#ifndef CHASSIS_ENCODER_H_
#define CHASSIS_ENCODER_H_

#include "hal_data.h"
#include "hardware/Chassis/Chassis_config.h"

typedef struct st_encoder_api
{
    /**
     * @brief  初始化四路编码器 GPT 定时器
     *
     * @return fsp_err_t  FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  复位四路编码器计数和速度状态
     *
     * @return void
     */
    void (*reset)(void);

    /**
     * @brief  更新四路编码器轮速
     *
     * @param interval_ms      测速周期，单位 ms
     * @param wheel_speed_mps  输出四路轮速，单位 m/s
     * @return fsp_err_t       FSP_SUCCESS 表示更新成功
     */
    fsp_err_t (*update)(uint32_t interval_ms, float wheel_speed_mps[CHASSIS_WHEEL_COUNT]);

    /**
     * @brief  获取指定电机当前轮速
     *
     * @param wheel  电机编号
     * @return float  当前轮速，单位 m/s
     */
    float (*get_speed)(chassis_wheel_t wheel);

    /**
     * @brief  获取指定电机当前转速
     *
     * @param wheel  电机编号
     * @return float  当前转速，单位 rpm
     */
    float (*get_rpm)(chassis_wheel_t wheel);

    /**
     * @brief  获取指定编码器的 GPT 溢出累计值
     *
     * @param wheel  电机编号
     * @return int32_t  溢出累计值，向上计数溢出加 1，向下计数溢出减 1
     */
    int32_t (*get_overflow_count)(chassis_wheel_t wheel);
} encoder_api_t;

extern const encoder_api_t encoder;

#endif /* CHASSIS_ENCODER_H_ */
