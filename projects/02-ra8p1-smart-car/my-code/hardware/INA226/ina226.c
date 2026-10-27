#include "hardware/INA226/ina226.h"

#include "hardware/INA226/soft_iic.h"

#include <stdbool.h>
#include <stddef.h>

/* INA226 7 位 IIC 地址：原理图中 A0/A1 均下拉到 GND，因此地址为 0x40。 */
#define INA226_IIC_ADDRESS               (0x40U)

/* 配置寄存器地址：用于复位芯片、设置采样平均次数、转换时间和工作模式。 */
#define INA226_REG_CONFIG                (0x00U)

/* 分流电压寄存器地址：读取 VIN+ 与 VIN- 之间的压差，原始值为有符号数。 */
#define INA226_REG_SHUNT_VOLTAGE         (0x01U)

/* 母线电压寄存器地址：读取 VBUS 引脚电压，原始值为无符号数。 */
#define INA226_REG_BUS_VOLTAGE           (0x02U)

/* 电流寄存器地址：读取经过校准寄存器换算后的电流，原始值为有符号数。 */
#define INA226_REG_CURRENT               (0x04U)

/* 校准寄存器地址：写入分流电阻和电流 LSB 对应的校准值，决定电流寄存器比例。 */
#define INA226_REG_CALIBRATION           (0x05U)

/* 复位命令：写配置寄存器 bit15 复位 INA226，写入后该位会自动清零。 */
#define INA226_CONFIG_RESET              (0x8000U)

/* 连续采样配置：平均 1 次，VBUS/VSHUNT 转换时间均为 1.1ms，连续采样分流电压和母线电压。 */
#define INA226_CONFIG_CONTINUOUS         (0x4127U)

/* 校准值：CAL = 0.00512 / (Current_LSB * Rshunt)，Current_LSB=500uA，Rshunt=5mR，结果为 2048。 */
#define INA226_CALIBRATION_VALUE         (2048U)

/* 复位后等待寄存器恢复默认值。 */
#define INA226_RESET_DELAY_MS            (2U)

/* 当前配置下首次完整采样约为 1.1ms + 1.1ms，这里留 3ms 裕量。 */
#define INA226_FIRST_SAMPLE_DELAY_MS     (3U)

/* 母线电压寄存器 LSB 为 1.25mV，因此 raw * 5 / 4 得到 mV。 */
#define INA226_BUS_VOLTAGE_NUMERATOR     (5U)
#define INA226_BUS_VOLTAGE_DENOMINATOR   (4U)

/* 分流电压寄存器 LSB 为 2.5uV，因此 raw * 5 / 2 得到 uV。 */
#define INA226_SHUNT_VOLTAGE_NUMERATOR   (5)
#define INA226_SHUNT_VOLTAGE_DENOMINATOR (2)

/* 电流寄存器 LSB 由校准值决定，当前设置为 500uA/bit。 */
#define INA226_CURRENT_LSB_UA            (500)

static bool g_ina226_initialized = false;

static ina226_sample_t g_ina226_cache;

/**
 * @brief  写 INA226 16 位寄存器
 *
 * @param reg    寄存器地址
 * @param value  写入值
 * @return fsp_err_t FSP_SUCCESS 表示写入成功
 */
static fsp_err_t ina226_write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[3];

    data[0] = reg;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value & 0xFFU);

    return soft_iic.write(INA226_IIC_ADDRESS, data, sizeof(data));
}

/**
 * @brief  读取 INA226 16 位寄存器
 *
 * @param reg      寄存器地址
 * @param p_value  读取值输出
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t ina226_read_register(uint8_t reg, uint16_t *p_value)
{
    uint8_t data[2] = {0U, 0U};
    fsp_err_t err;

    if (NULL == p_value)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    err = soft_iic.write_read(INA226_IIC_ADDRESS, &reg, 1U, data, sizeof(data));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    *p_value = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    return FSP_SUCCESS;
}

/**
 * @brief  初始化 INA226
 *
 * @return fsp_err_t FSP_SUCCESS 表示 INA226 已可读取
 */
static fsp_err_t ina226_init_impl(void)
{
    fsp_err_t err;

    g_ina226_initialized = false;

    err = soft_iic.init();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = ina226_write_register(INA226_REG_CONFIG, INA226_CONFIG_RESET);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    R_BSP_SoftwareDelay(INA226_RESET_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);

    err = ina226_write_register(INA226_REG_CONFIG, INA226_CONFIG_CONTINUOUS);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = ina226_write_register(INA226_REG_CALIBRATION, INA226_CALIBRATION_VALUE);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    R_BSP_SoftwareDelay(INA226_FIRST_SAMPLE_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);

    g_ina226_initialized = true;
    return FSP_SUCCESS;
}

/**
 * @brief  从硬件读取并缓存 INA226 采样值
 *
 * 访问 IIC 总线读取三个寄存器，换算后写入内部缓存。
 *
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t ina226_update_impl(void)
{
    uint16_t bus_raw;
    uint16_t shunt_raw;
    uint16_t current_raw;
    fsp_err_t err;

    if (!g_ina226_initialized)
    {
        return FSP_ERR_NOT_OPEN;
    }

    err = ina226_read_register(INA226_REG_BUS_VOLTAGE, &bus_raw);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = ina226_read_register(INA226_REG_SHUNT_VOLTAGE, &shunt_raw);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = ina226_read_register(INA226_REG_CURRENT, &current_raw);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    g_ina226_cache.bus_voltage_mv = ((uint32_t)bus_raw * INA226_BUS_VOLTAGE_NUMERATOR) /
                                    INA226_BUS_VOLTAGE_DENOMINATOR;
    g_ina226_cache.shunt_voltage_uv = ((int32_t)((int16_t)shunt_raw) * INA226_SHUNT_VOLTAGE_NUMERATOR) /
                                      INA226_SHUNT_VOLTAGE_DENOMINATOR;
    g_ina226_cache.current_ua = (int32_t)((int16_t)current_raw) * INA226_CURRENT_LSB_UA;

    return FSP_SUCCESS;
}

/**
 * @brief  获取最近一次 update 缓存的采样值
 *
 * 不访问 IIC 总线，直接从内存缓存拷贝。
 *
 * @param p_sample  采样值输出
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t ina226_read_impl(ina226_sample_t *p_sample)
{
    if (NULL == p_sample)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    *p_sample = g_ina226_cache;
    return FSP_SUCCESS;
}

const ina226_api_t ina226 =
{
    .init = ina226_init_impl,
    .update = ina226_update_impl,
    .read = ina226_read_impl,
};
