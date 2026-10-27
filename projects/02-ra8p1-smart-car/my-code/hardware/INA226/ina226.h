#ifndef HARDWARE_INA226_INA226_H_
#define HARDWARE_INA226_INA226_H_

#include "hal_data.h"

#include <stdint.h>

typedef struct st_ina226_sample
{
    /**
     * @brief  VBUS 引脚测得的母线电压，单位 mV
     */
    uint32_t bus_voltage_mv;

    /**
     * @brief  VIN+ 与 VIN- 之间的分流电阻压差，单位 uV，可为负值
     */
    int32_t shunt_voltage_uv;

    /**
     * @brief  按校准寄存器换算后的电流，单位 uA，可为负值
     */
    int32_t current_ua;
} ina226_sample_t;

typedef struct st_ina226_api
{
    /**
     * @brief  初始化 INA226 并写入采样配置和校准值
     *
     * @return fsp_err_t FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  从硬件读取并缓存采样值
     *
     * 访问 IIC 总线，读取母线电压、分流电压和电流寄存器，换算后写入内部缓存。
     *
     * @return fsp_err_t FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*update)(void);

    /**
     * @brief  获取最近一次 update 缓存的采样值
     *
     * 不访问 IIC 总线，直接从内存拷贝，可在任意上下文中调用。
     *
     * @param p_sample  采样数据输出
     * @return fsp_err_t FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*read)(ina226_sample_t *p_sample);
} ina226_api_t;

extern const ina226_api_t ina226;

#endif /* HARDWARE_INA226_INA226_H_ */
