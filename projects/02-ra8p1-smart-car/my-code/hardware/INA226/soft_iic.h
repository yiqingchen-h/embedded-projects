#ifndef HARDWARE_INA226_SOFT_IIC_H_
#define HARDWARE_INA226_SOFT_IIC_H_

#include "hal_data.h"

#include <stdint.h>

typedef struct st_soft_iic_api
{
    /**
     * @brief  初始化 INA226 专用软件 IIC 总线
     *
     * @return fsp_err_t FSP_SUCCESS 表示总线空闲可用
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  向 7 位 IIC 地址写入数据
     *
     * @param address  7 位从机地址
     * @param p_data   待写入数据
     * @param bytes    写入字节数
     * @return fsp_err_t FSP_SUCCESS 表示写入成功
     */
    fsp_err_t (*write)(uint8_t address, const uint8_t *p_data, uint32_t bytes);

    /**
     * @brief  先写入寄存器地址，再重复起始读取数据
     *
     * @param address      7 位从机地址
     * @param p_write      写入数据
     * @param write_bytes  写入字节数
     * @param p_read       读取缓冲区
     * @param read_bytes   读取字节数
     * @return fsp_err_t FSP_SUCCESS 表示读写成功
     */
    fsp_err_t (*write_read)(uint8_t address,
                            const uint8_t *p_write,
                            uint32_t write_bytes,
                            uint8_t *p_read,
                            uint32_t read_bytes);
} soft_iic_api_t;

extern const soft_iic_api_t soft_iic;

#endif /* HARDWARE_INA226_SOFT_IIC_H_ */
