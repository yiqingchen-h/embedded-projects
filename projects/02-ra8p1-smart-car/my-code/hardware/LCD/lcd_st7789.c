#include "hardware/LCD/lcd_st7789.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

/* ST7789 片选控制引脚。 */
#define LCD_PIN_CS                     BSP_IO_PORT_00_PIN_05

/* ST7789 命令/数据选择引脚。 */
#define LCD_PIN_DC                     BSP_IO_PORT_00_PIN_06

/* ST7789 复位控制引脚。 */
#define LCD_PIN_RES                    BSP_IO_PORT_04_PIN_02

/* ST7789 背光控制引脚。 */
#define LCD_PIN_BLK                    BSP_IO_PORT_04_PIN_03

/* ST7789 控制器比面板多出的偏移像素。 */
#define LCD_ST7789_PANEL_OFFSET        (20U)

#if ((90U == LCD_ST7789_ROTATION) || (270U == LCD_ST7789_ROTATION))
/* 横屏时有效区落在控制器列方向偏移后的位置。 */
#define LCD_X_OFFSET                   (LCD_ST7789_PANEL_OFFSET)

/* 横屏时行方向从 0 开始。 */
#define LCD_Y_OFFSET                   (0U)
#else
/* 竖屏时列方向从 0 开始。 */
#define LCD_X_OFFSET                   (0U)

/* 竖屏时有效区落在控制器行方向偏移后的位置。 */
#define LCD_Y_OFFSET                   (LCD_ST7789_PANEL_OFFSET)
#endif

#if (90U == LCD_ST7789_ROTATION)
/* ST7789 横屏 90 度 MADCTL 配置。 */
#define LCD_MADCTL_VALUE               (0xA0U)
#elif (270U == LCD_ST7789_ROTATION)
/* ST7789 横屏 270 度 MADCTL 配置。 */
#define LCD_MADCTL_VALUE               (0x60U)
#elif (180U == LCD_ST7789_ROTATION)
/* ST7789 竖屏 180 度 MADCTL 配置。 */
#define LCD_MADCTL_VALUE               (0x00U)
#else
/* ST7789 竖屏默认 MADCTL 配置，保持原工程显示方向。 */
#define LCD_MADCTL_VALUE               (0xC0U)
#endif

/* SPI 单次传输等待超时时间，单位 ms。 */
#define LCD_SPI_TIMEOUT_MS             (500U)

/* SCI SPI 单次传输最大字节数，避免超出底层长度限制。 */
#define LCD_SPI_MAX_TRANSFER_BYTES     (60000U)

/* SPI 异步传输完成标志。 */
static volatile bool g_lcd_spi_done = false;

/* SPI 异步传输结果。 */
static volatile fsp_err_t g_lcd_spi_result = FSP_SUCCESS;

/* ST7789 初始化完成标志。 */
static bool g_lcd_initialized = false;

static void lcd_pin_write(bsp_io_port_pin_t pin, bsp_io_level_t level)
{
    (void)R_IOPORT_PinWrite(&g_ioport_ctrl, pin, level);
}

static void lcd_cs_low(void)
{
    lcd_pin_write(LCD_PIN_CS, BSP_IO_LEVEL_LOW);
}

static void lcd_cs_high(void)
{
    lcd_pin_write(LCD_PIN_CS, BSP_IO_LEVEL_HIGH);
}

static void lcd_dc_command(void)
{
    lcd_pin_write(LCD_PIN_DC, BSP_IO_LEVEL_LOW);
}

static void lcd_dc_data(void)
{
    lcd_pin_write(LCD_PIN_DC, BSP_IO_LEVEL_HIGH);
}

static void lcd_delay_ms(uint32_t delay_ms)
{
    if (0U == delay_ms)
    {
        return;
    }

    if (taskSCHEDULER_RUNNING == xTaskGetSchedulerState())
    {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    else
    {
        R_BSP_SoftwareDelay(delay_ms, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

static fsp_err_t lcd_wait_transfer(uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0U;

    while (!g_lcd_spi_done)
    {
        if (elapsed_ms >= timeout_ms)
        {
            return FSP_ERR_TIMEOUT;
        }

        lcd_delay_ms(1U);
        elapsed_ms++;
    }

    return g_lcd_spi_result;
}

static fsp_err_t lcd_spi_write_block(const uint8_t *p_data, uint32_t bytes)
{
    fsp_err_t err;

    if ((NULL == p_data) || (0U == bytes))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    while (bytes > 0U)
    {
        uint32_t chunk = bytes;
        if (chunk > LCD_SPI_MAX_TRANSFER_BYTES)
        {
            chunk = LCD_SPI_MAX_TRANSFER_BYTES;
        }

        g_lcd_spi_done = false;
        g_lcd_spi_result = FSP_SUCCESS;

        err = g_sci_spi2.p_api->write(g_sci_spi2.p_ctrl, p_data, chunk, SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        err = lcd_wait_transfer(LCD_SPI_TIMEOUT_MS);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        p_data += chunk;
        bytes -= chunk;
    }

    return FSP_SUCCESS;
}

static fsp_err_t lcd_write_command(uint8_t command)
{
    fsp_err_t err;

    lcd_cs_low();
    lcd_dc_command();
    err = lcd_spi_write_block(&command, 1U);
    lcd_cs_high();

    return err;
}

static fsp_err_t lcd_write_data(const uint8_t *p_data, uint32_t bytes)
{
    fsp_err_t err;

    lcd_cs_low();
    lcd_dc_data();
    err = lcd_spi_write_block(p_data, bytes);
    lcd_cs_high();

    return err;
}

static fsp_err_t lcd_write_data1(uint8_t data)
{
    return lcd_write_data(&data, 1U);
}

static fsp_err_t lcd_write_command_data(uint8_t command, const uint8_t *p_data, uint32_t bytes)
{
    fsp_err_t err = lcd_write_command(command);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return lcd_write_data(p_data, bytes);
}

static fsp_err_t lcd_st7789_set_window_impl(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint8_t data[4];
    uint16_t column_start = 0U;
    uint16_t column_end = 0U;
    uint16_t row_start = 0U;
    uint16_t row_end = 0U;
    fsp_err_t err;

    if ((x1 >= LCD_ST7789_WIDTH) || (x2 >= LCD_ST7789_WIDTH) || (y1 >= LCD_ST7789_HEIGHT) ||
        (y2 >= LCD_ST7789_HEIGHT) || (x1 > x2) || (y1 > y2))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    column_start = (uint16_t)(x1 + LCD_X_OFFSET);
    column_end = (uint16_t)(x2 + LCD_X_OFFSET);
    row_start = (uint16_t)(y1 + LCD_Y_OFFSET);
    row_end = (uint16_t)(y2 + LCD_Y_OFFSET);

    data[0] = (uint8_t)(column_start >> 8);
    data[1] = (uint8_t)(column_start & 0xFFU);
    data[2] = (uint8_t)(column_end >> 8);
    data[3] = (uint8_t)(column_end & 0xFFU);
    err = lcd_write_command_data(0x2AU, data, sizeof(data));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    data[0] = (uint8_t)(row_start >> 8);
    data[1] = (uint8_t)(row_start & 0xFFU);
    data[2] = (uint8_t)(row_end >> 8);
    data[3] = (uint8_t)(row_end & 0xFFU);
    err = lcd_write_command_data(0x2BU, data, sizeof(data));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return lcd_write_command(0x2CU);
}

static fsp_err_t lcd_st7789_write_pixels_impl(const uint8_t *p_data, uint32_t bytes)
{
    fsp_err_t err;

    if (!g_lcd_initialized)
    {
        return FSP_ERR_NOT_OPEN;
    }

    lcd_cs_low();
    lcd_dc_data();
    err = lcd_spi_write_block(p_data, bytes);
    lcd_cs_high();

    return err;
}

static fsp_err_t lcd_st7789_fill_rgb565_impl(uint16_t color)
{
    static uint8_t line_buffer[LCD_ST7789_WIDTH * 2U];
    uint32_t i;
    uint32_t y;
    uint8_t color_high = (uint8_t)(color >> 8);
    uint8_t color_low = (uint8_t)(color & 0xFFU);
    fsp_err_t err;

    for (i = 0U; i < LCD_ST7789_WIDTH; i++)
    {
        line_buffer[(i * 2U)] = color_high;
        line_buffer[(i * 2U) + 1U] = color_low;
    }

    err = lcd_st7789_set_window_impl(0U, 0U, LCD_ST7789_WIDTH - 1U, LCD_ST7789_HEIGHT - 1U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    for (y = 0U; y < LCD_ST7789_HEIGHT; y++)
    {
        err = lcd_st7789_write_pixels_impl(line_buffer, sizeof(line_buffer));
        if (FSP_SUCCESS != err)
        {
            return err;
        }
    }

    return FSP_SUCCESS;
}

static fsp_err_t lcd_st7789_init_impl(void)
{
    fsp_err_t err;
    static const uint8_t b2_data[] = {0x0CU, 0x0CU, 0x00U, 0x33U, 0x33U};
    static const uint8_t d0_data[] = {0xA4U, 0xA1U};
    static const uint8_t e0_data[] =
    {
        0xD0U, 0x08U, 0x0EU, 0x09U, 0x09U, 0x05U, 0x31U,
        0x33U, 0x48U, 0x17U, 0x14U, 0x15U, 0x31U, 0x34U
    };
    static const uint8_t e1_data[] =
    {
        0xD0U, 0x08U, 0x0EU, 0x09U, 0x09U, 0x15U, 0x31U,
        0x33U, 0x48U, 0x17U, 0x14U, 0x15U, 0x31U, 0x34U
    };

    if (g_lcd_initialized)
    {
        return FSP_SUCCESS;
    }

    lcd_cs_high();
    lcd_dc_data();
    lcd_pin_write(LCD_PIN_BLK, BSP_IO_LEVEL_LOW);
    lcd_pin_write(LCD_PIN_RES, BSP_IO_LEVEL_LOW);

    err = g_sci_spi2.p_api->open(g_sci_spi2.p_ctrl, g_sci_spi2.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    lcd_delay_ms(100U);
    lcd_pin_write(LCD_PIN_RES, BSP_IO_LEVEL_HIGH);
    lcd_delay_ms(100U);

    err = lcd_write_command(0x11U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }
    lcd_delay_ms(120U);

    err = lcd_write_command(0x36U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }
    err = lcd_write_data1(LCD_MADCTL_VALUE);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command(0x3AU);
    if (FSP_SUCCESS != err)
    {
        return err;
    }
    err = lcd_write_data1(0x05U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command_data(0xB2U, b2_data, sizeof(b2_data));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command(0xB7U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }
    err = lcd_write_data1(0x35U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command(0xBBU);
    if (FSP_SUCCESS != err)
    {
        return err;
    }
    err = lcd_write_data1(0x32U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command(0xC2U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }
    err = lcd_write_data1(0x01U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command(0xC3U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }
    err = lcd_write_data1(0x15U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command(0xC4U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }
    err = lcd_write_data1(0x20U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command(0xC6U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }
    err = lcd_write_data1(0x0FU);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command_data(0xD0U, d0_data, sizeof(d0_data));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command_data(0xE0U, e0_data, sizeof(e0_data));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command_data(0xE1U, e1_data, sizeof(e1_data));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command(0x21U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_write_command(0x29U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    lcd_pin_write(LCD_PIN_BLK, BSP_IO_LEVEL_HIGH);

    g_lcd_initialized = true;

    return FSP_SUCCESS;
}

static bool lcd_st7789_is_ready_impl(void)
{
    return g_lcd_initialized;
}

static void lcd_st7789_notify_spi_callback_impl(spi_callback_args_t *p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        g_lcd_spi_result = FSP_SUCCESS;
    }
    else
    {
        g_lcd_spi_result = FSP_ERR_ABORTED;
    }

    g_lcd_spi_done = true;
}

const lcd_st7789_api_t lcd_st7789 =
{
    .init = lcd_st7789_init_impl,
    .set_window = lcd_st7789_set_window_impl,
    .write_pixels = lcd_st7789_write_pixels_impl,
    .fill_rgb565 = lcd_st7789_fill_rgb565_impl,
    .is_ready = lcd_st7789_is_ready_impl,
    .notify_spi_callback = lcd_st7789_notify_spi_callback_impl,
};
