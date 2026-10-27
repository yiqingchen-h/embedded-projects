#ifndef HARDWARE_CHASSIS_CHASSIS_PARAM_H_
#define HARDWARE_CHASSIS_CHASSIS_PARAM_H_

#include "hal_data.h"
#include "hardware/Chassis/Chassis.h"
#include "hardware/Flash/flash.h"

/* 底盘 PID 参数 Flash 扇区地址，避开最后 64KB 诊断保留区。 */
#define CHASSIS_PARAM_FLASH_ADDRESS (0x01FE0000UL)

typedef struct st_chassis_param_api
{
    /**
     * @brief  从 Flash 加载底盘 PID 参数
     *
     * @param p_params  参数输出指针
     * @return fsp_err_t  FSP_SUCCESS 表示加载成功
     */
    fsp_err_t (*load)(chassis_pid_params_t *p_params);

    /**
     * @brief  保存底盘 PID 参数到 Flash
     *
     * @param p_params  待保存参数
     * @return fsp_err_t  FSP_SUCCESS 表示保存成功
     */
    fsp_err_t (*save)(const chassis_pid_params_t *p_params);
} chassis_param_api_t;

extern const chassis_param_api_t chassis_param;

#endif /* HARDWARE_CHASSIS_CHASSIS_PARAM_H_ */
