#include "hal_data.h"

/**
 * @brief  SCI-B SPI 回调占位
 *
 * 当前工程中该 SPI 实例暂不需要业务处理，保留符号供 FSP 链接。
 *
 * @param p_args  SPI 回调参数
 * @return void
 */
void sci_b_spi_callback(spi_callback_args_t *p_args)
{
    (void)p_args;
}

/**
 * @brief  SCI-B I2C Master 回调占位
 *
 * 当前工程中该 I2C 实例暂不需要业务处理，保留符号供 FSP 链接。
 *
 * @param p_args  I2C Master 回调参数
 * @return void
 */
void sci_b_i2c_master_callback(i2c_master_callback_args_t *p_args)
{
    (void)p_args;
}

/**
 * @brief  Ethos-U NPU 回调占位
 *
 * 当前 AI 推理使用同步调用，不需要在中断回调里处理业务事件。
 *
 * @param p_arg NPU 回调参数
 * @return void
 */
void ai_ethosu_callback(rm_ethosu_callback_args_t *p_arg)
{
    (void)p_arg;
}
