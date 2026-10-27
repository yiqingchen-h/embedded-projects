#include "hardware/INA226/soft_iic.h"

#include <stdbool.h>
#include <stddef.h>

#define SOFT_IIC_SDA_PIN                  BSP_IO_PORT_05_PIN_01
#define SOFT_IIC_SCL_PIN                  BSP_IO_PORT_05_PIN_13
#define SOFT_IIC_DELAY_US                 (5U)
#define SOFT_IIC_WAIT_HIGH_TIMEOUT        (1000U)
#define SOFT_IIC_WRITE_BIT                (0U)
#define SOFT_IIC_READ_BIT                 (1U)

/**
 * @brief  软件 IIC 半周期延时，约 100kHz
 *
 * @return void
 */
static void soft_iic_delay(void)
{
    R_BSP_SoftwareDelay(SOFT_IIC_DELAY_US, BSP_DELAY_UNITS_MICROSECONDS);
}

/**
 * @brief  写 GPIO 电平
 *
 * 开漏输出下写高表示释放总线，写低表示主动拉低。
 *
 * @param pin    GPIO 引脚
 * @param level  目标电平
 * @return fsp_err_t FSP_SUCCESS 表示写入成功
 */
static fsp_err_t soft_iic_pin_write(bsp_io_port_pin_t pin, bsp_io_level_t level)
{
    return R_IOPORT_PinWrite(&g_ioport_ctrl, pin, level);
}

/**
 * @brief  读取 GPIO 是否为高电平
 *
 * @param pin     GPIO 引脚
 * @param p_high  高电平状态输出
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t soft_iic_pin_is_high(bsp_io_port_pin_t pin, bool *p_high)
{
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;
    fsp_err_t err;

    if (NULL == p_high)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    err = R_IOPORT_PinRead(&g_ioport_ctrl, pin, &level);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    *p_high = (BSP_IO_LEVEL_HIGH == level);
    return FSP_SUCCESS;
}

/**
 * @brief  释放 SDA
 *
 * @return fsp_err_t FSP_SUCCESS 表示释放成功
 */
static fsp_err_t soft_iic_sda_release(void)
{
    return soft_iic_pin_write(SOFT_IIC_SDA_PIN, BSP_IO_LEVEL_HIGH);
}

/**
 * @brief  拉低 SDA
 *
 * @return fsp_err_t FSP_SUCCESS 表示拉低成功
 */
static fsp_err_t soft_iic_sda_low(void)
{
    return soft_iic_pin_write(SOFT_IIC_SDA_PIN, BSP_IO_LEVEL_LOW);
}

/**
 * @brief  拉低 SCL
 *
 * @return fsp_err_t FSP_SUCCESS 表示拉低成功
 */
static fsp_err_t soft_iic_scl_low(void)
{
    return soft_iic_pin_write(SOFT_IIC_SCL_PIN, BSP_IO_LEVEL_LOW);
}

/**
 * @brief  释放 SCL 并等待其变为高电平
 *
 * @return fsp_err_t FSP_SUCCESS 表示 SCL 已变高，FSP_ERR_TIMEOUT 表示被拉低
 */
static fsp_err_t soft_iic_scl_release_wait(void)
{
    fsp_err_t err;
    bool is_high = false;

    err = soft_iic_pin_write(SOFT_IIC_SCL_PIN, BSP_IO_LEVEL_HIGH);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    for (uint32_t i = 0U; i < SOFT_IIC_WAIT_HIGH_TIMEOUT; i++)
    {
        err = soft_iic_pin_is_high(SOFT_IIC_SCL_PIN, &is_high);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        if (is_high)
        {
            return FSP_SUCCESS;
        }

        soft_iic_delay();
    }

    return FSP_ERR_TIMEOUT;
}

/**
 * @brief  检查 SDA 是否已释放为高电平
 *
 * @return fsp_err_t FSP_SUCCESS 表示 SDA 为高，FSP_ERR_TIMEOUT 表示总线被占用
 */
static fsp_err_t soft_iic_wait_sda_high(void)
{
    fsp_err_t err;
    bool is_high = false;

    for (uint32_t i = 0U; i < SOFT_IIC_WAIT_HIGH_TIMEOUT; i++)
    {
        err = soft_iic_pin_is_high(SOFT_IIC_SDA_PIN, &is_high);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        if (is_high)
        {
            return FSP_SUCCESS;
        }

        soft_iic_delay();
    }

    return FSP_ERR_TIMEOUT;
}

/**
 * @brief  发送 IIC 起始信号
 *
 * @return fsp_err_t FSP_SUCCESS 表示起始信号发送成功
 */
static fsp_err_t soft_iic_start(void)
{
    fsp_err_t err;

    err = soft_iic_sda_release();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = soft_iic_scl_release_wait();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = soft_iic_wait_sda_high();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();

    err = soft_iic_sda_low();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();
    return soft_iic_scl_low();
}

/**
 * @brief  发送 IIC 停止信号
 *
 * @return fsp_err_t FSP_SUCCESS 表示停止信号发送成功
 */
static fsp_err_t soft_iic_stop(void)
{
    fsp_err_t err;

    err = soft_iic_sda_low();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();

    err = soft_iic_scl_release_wait();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();

    err = soft_iic_sda_release();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();
    return soft_iic_wait_sda_high();
}

/**
 * @brief  写入 1 个字节并检查 ACK
 *
 * @param data  待写入字节
 * @return fsp_err_t FSP_SUCCESS 表示收到 ACK
 */
static fsp_err_t soft_iic_write_byte(uint8_t data)
{
    fsp_err_t err;
    bool ack_high = true;

    for (uint8_t mask = 0x80U; mask > 0U; mask = (uint8_t)(mask >> 1))
    {
        if (0U != (data & mask))
        {
            err = soft_iic_sda_release();
        }
        else
        {
            err = soft_iic_sda_low();
        }

        if (FSP_SUCCESS != err)
        {
            return err;
        }

        soft_iic_delay();

        err = soft_iic_scl_release_wait();
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        soft_iic_delay();

        err = soft_iic_scl_low();
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        soft_iic_delay();
    }

    err = soft_iic_sda_release();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();

    err = soft_iic_scl_release_wait();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();

    err = soft_iic_pin_is_high(SOFT_IIC_SDA_PIN, &ack_high);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = soft_iic_scl_low();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();
    return ack_high ? FSP_ERR_ABORTED : FSP_SUCCESS;
}

/**
 * @brief  发送 ACK 或 NACK
 *
 * @param ack  true 发送 ACK，false 发送 NACK
 * @return fsp_err_t FSP_SUCCESS 表示发送成功
 */
static fsp_err_t soft_iic_send_ack(bool ack)
{
    fsp_err_t err;

    err = ack ? soft_iic_sda_low() : soft_iic_sda_release();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();

    err = soft_iic_scl_release_wait();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();

    err = soft_iic_scl_low();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();
    return soft_iic_sda_release();
}

/**
 * @brief  读取 1 个字节
 *
 * @param p_data  读取数据输出
 * @param ack     true 表示读完后发送 ACK，false 表示发送 NACK
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t soft_iic_read_byte(uint8_t *p_data, bool ack)
{
    uint8_t data = 0U;
    fsp_err_t err;
    bool is_high = false;

    if (NULL == p_data)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    err = soft_iic_sda_release();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    for (uint8_t i = 0U; i < 8U; i++)
    {
        data = (uint8_t)(data << 1);

        err = soft_iic_scl_release_wait();
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        soft_iic_delay();

        err = soft_iic_pin_is_high(SOFT_IIC_SDA_PIN, &is_high);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        if (is_high)
        {
            data |= 0x01U;
        }

        err = soft_iic_scl_low();
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        soft_iic_delay();
    }

    *p_data = data;
    return soft_iic_send_ack(ack);
}

/**
 * @brief  初始化 INA226 专用软件 IIC
 *
 * @return fsp_err_t FSP_SUCCESS 表示总线空闲可用
 */
static fsp_err_t soft_iic_init_impl(void)
{
    fsp_err_t err;

    err = soft_iic_sda_release();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = soft_iic_scl_release_wait();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    soft_iic_delay();
    return soft_iic_wait_sda_high();
}

/**
 * @brief  写入 IIC 数据
 *
 * @param address  7 位从机地址
 * @param p_data   待写入数据
 * @param bytes    写入字节数
 * @return fsp_err_t FSP_SUCCESS 表示写入成功
 */
static fsp_err_t soft_iic_write_impl(uint8_t address, const uint8_t *p_data, uint32_t bytes)
{
    fsp_err_t err;
    fsp_err_t stop_err;

    if (address > 0x7FU)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    if ((NULL == p_data) && (bytes > 0U))
    {
        return FSP_ERR_INVALID_POINTER;
    }

    err = soft_iic_start();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = soft_iic_write_byte((uint8_t)((address << 1) | SOFT_IIC_WRITE_BIT));
    for (uint32_t i = 0U; (FSP_SUCCESS == err) && (i < bytes); i++)
    {
        err = soft_iic_write_byte(p_data[i]);
    }

    stop_err = soft_iic_stop();
    return (FSP_SUCCESS == err) ? stop_err : err;
}

/**
 * @brief  写寄存器后读取 IIC 数据
 *
 * @param address      7 位从机地址
 * @param p_write      写入数据
 * @param write_bytes  写入字节数
 * @param p_read       读取缓冲区
 * @param read_bytes   读取字节数
 * @return fsp_err_t FSP_SUCCESS 表示读写成功
 */
static fsp_err_t soft_iic_write_read_impl(uint8_t address,
                                          const uint8_t *p_write,
                                          uint32_t write_bytes,
                                          uint8_t *p_read,
                                          uint32_t read_bytes)
{
    fsp_err_t err;
    fsp_err_t stop_err;

    if ((address > 0x7FU) || (0U == read_bytes))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    if ((NULL == p_read) || ((NULL == p_write) && (write_bytes > 0U)))
    {
        return FSP_ERR_INVALID_POINTER;
    }

    err = soft_iic_start();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = soft_iic_write_byte((uint8_t)((address << 1) | SOFT_IIC_WRITE_BIT));
    for (uint32_t i = 0U; (FSP_SUCCESS == err) && (i < write_bytes); i++)
    {
        err = soft_iic_write_byte(p_write[i]);
    }

    if (FSP_SUCCESS == err)
    {
        err = soft_iic_start();
    }

    if (FSP_SUCCESS == err)
    {
        err = soft_iic_write_byte((uint8_t)((address << 1) | SOFT_IIC_READ_BIT));
    }

    for (uint32_t i = 0U; (FSP_SUCCESS == err) && (i < read_bytes); i++)
    {
        err = soft_iic_read_byte(&p_read[i], (i + 1U) < read_bytes);
    }

    stop_err = soft_iic_stop();
    return (FSP_SUCCESS == err) ? stop_err : err;
}

const soft_iic_api_t soft_iic =
{
    .init = soft_iic_init_impl,
    .write = soft_iic_write_impl,
    .write_read = soft_iic_write_read_impl,
};
