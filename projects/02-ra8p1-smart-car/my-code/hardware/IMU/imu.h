#ifndef HARDWARE_IMU_IMU_H_
#define HARDWARE_IMU_IMU_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct st_imu_snapshot
{
    /**
     * @brief  true 表示欧拉角数据有效且未超时
     */
    bool angle_valid;

    /**
     * @brief  true 表示角速度数据有效且未超时
     */
    bool gyro_valid;

    /**
     * @brief  横滚角，单位度
     */
    float roll_deg;

    /**
     * @brief  俯仰角，单位度
     */
    float pitch_deg;

    /**
     * @brief  偏航角，单位度
     */
    float yaw_deg;

    /**
     * @brief  Z 轴角速度，单位度每秒
     */
    float gyro_z_dps;

    /**
     * @brief  最近一次欧拉角帧序号，从 1 开始递增
     */
    uint32_t angle_sequence;

    /**
     * @brief  最近一次角速度帧序号，从 1 开始递增
     */
    uint32_t gyro_sequence;

    /**
     * @brief  欧拉角数据距最近一次更新的时间，单位 ms
     */
    uint32_t angle_age_ms;

    /**
     * @brief  角速度数据距最近一次更新的时间，单位 ms
     */
    uint32_t gyro_age_ms;

    /**
     * @brief  页面显示用数据年龄，单位 ms
     */
    uint32_t age_ms;

    /**
     * @brief  UART 接收环形缓存溢出次数
     */
    uint32_t rx_overflow_count;

    /**
     * @brief  UART 错误中断次数
     */
    uint32_t uart_error_count;

    /**
     * @brief  帧校验和错误次数
     */
    uint32_t checksum_error_count;

    /**
     * @brief  未支持帧类型计数
     */
    uint32_t frame_error_count;

    /**
     * @brief  最近一次错误码
     */
    fsp_err_t last_error;
} imu_snapshot_t;

typedef struct st_imu_api
{
    /**
     * @brief  初始化 IMU UART0 接收
     *
     * @return fsp_err_t FSP_SUCCESS 表示 UART0 已打开
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  处理 UART0 接收缓存中的 IMU 连续字节流
     *
     * @param elapsed_ms 距离上次调用经过的时间，单位 ms
     * @return fsp_err_t FSP_SUCCESS 表示处理完成
     */
    fsp_err_t (*process)(uint32_t elapsed_ms);

    /**
     * @brief  读取最近一次 IMU 快照
     *
     * @param p_snapshot 快照输出指针
     * @return fsp_err_t FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*read)(imu_snapshot_t *p_snapshot);
} imu_api_t;

extern const imu_api_t imu;

#endif /* HARDWARE_IMU_IMU_H_ */
