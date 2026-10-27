#include "SPI.h"
volatile bool g_lcd_spi_tx_complete = false;
void SPI_LCD_Init(void)
{

     fsp_err_t err;
    
    // 1. 打开 I/O 端口配置
    g_ioport.p_api->open(g_ioport.p_ctrl, g_ioport.p_cfg);

    // 2. 打开 SPI 模块 (它会自动打开关联的 TX 和 RX DMA)
    err = R_SPI_Open(&g_spi1_ctrl, &g_spi1_cfg);
    
    if (err == FSP_SUCCESS)
        printf("Success to open SPI1 with DMA\r\n");
    else
        printf("Failed to open SPI1, Error: %d\r\n", err);
}

void spi1_callback(spi_callback_args_t *arg)
{
    /* 判断是否是发送完成触发的中断 */
    /* 如果是的话就将发送完成标志位置1 */
    if (SPI_EVENT_TRANSFER_COMPLETE == arg->event)
        g_lcd_spi_tx_complete = true;
}

