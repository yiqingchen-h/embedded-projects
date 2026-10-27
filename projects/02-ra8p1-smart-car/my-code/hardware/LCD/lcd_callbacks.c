#include "hardware/LCD/lcd_mipi_st7701p.h"
#include "hardware/LCD/lcd_st7789.h"

/**
 * @brief  MIPI DSI0 回调转发
 *
 * 该回调由 FSP MIPI DSI 驱动在命令序列完成或错误时调用。
 *
 * @param p_args  MIPI DSI 回调参数
 * @return void
 */
void mipi_dsi0_callback(mipi_dsi_callback_args_t *p_args)
{
    lcd_mipi_st7701p.notify_mipi_callback(p_args);
}

/**
 * @brief  SCI-B SPI2 回调转发
 *
 * 该回调保留给旧 ST7789 SPI 屏驱动，MIPI 默认路径不会使用。
 *
 * @param p_args  SPI 回调参数
 * @return void
 */
void sci_b_spi2_callback(spi_callback_args_t *p_args)
{
    lcd_st7789.notify_spi_callback(p_args);
}
