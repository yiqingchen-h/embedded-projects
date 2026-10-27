#ifndef UART_MOTOR_H_
#define UART_MOTOR_H_

#include "hal_data.h"
#include "hardware/Chassis/Chassis_config.h"

typedef struct st_uart_motor_api
{
    /**
     * @brief  初始化 UART3 四路串口电机板，并默认停车
     *
     * @return fsp_err_t  FSP_SUCCESS 表示初始化和停车命令发送成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  设置底盘目标车体速度
     *
     * @param vx_mps    前进方向速度，单位 m/s
     * @param vy_mps    横移方向速度，单位 m/s
     * @param wz_radps  旋转角速度，单位 rad/s
     * @return fsp_err_t  FSP_SUCCESS 表示速度命令发送成功
     */
    fsp_err_t (*set_velocity)(float vx_mps, float vy_mps, float wz_radps);

    /**
     * @brief  直接设置四路轮目标速度
     *
     * @param wheel_speed_mps  四路轮速，数组下标沿用 CHASSIS_WHEEL_x
     * @return fsp_err_t       FSP_SUCCESS 表示速度命令发送成功
     */
    fsp_err_t (*set_wheel_speed)(const float wheel_speed_mps[CHASSIS_WHEEL_COUNT]);

    /**
     * @brief  停止四路串口电机输出
     *
     * @return fsp_err_t  FSP_SUCCESS 表示停车命令发送成功
     */
    fsp_err_t (*stop_all)(void);

    /**
     * @brief  获取最近一次发送的串口命令
     *
     * @return const char*  最近一次命令字符串
     */
    const char *(*get_last_command)(void);

    /**
     * @brief  获取最近一次错误码
     *
     * @return fsp_err_t  最近一次错误码
     */
    fsp_err_t (*get_last_error)(void);
} uart_motor_api_t;

extern const uart_motor_api_t uart_motor;

#endif /* UART_MOTOR_H_ */
