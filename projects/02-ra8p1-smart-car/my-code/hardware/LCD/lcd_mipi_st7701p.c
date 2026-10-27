#include "hardware/LCD/lcd_mipi_st7701p.h"

#include "FreeRTOS.h"
#include "software/SDRAM/SDRAM.h"
#include "task.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define LCD_MIPI_PIN_RST                         LCD_RST
#define LCD_MIPI_PIN_BL                          LCD_BL

#define LCD_MIPI_LVGL_BYTES_PER_PIXEL            (2U)
#define LCD_MIPI_FRAMEBUFFER_BYTES_PER_PIXEL     (4U)
#define LCD_MIPI_DSI_TIMEOUT_MS                  (500U)
/* MIPI front framebuffer 初始索引。 */
#define LCD_MIPI_FRAMEBUFFER_FRONT_INITIAL       (0U)
/* MIPI back framebuffer 初始索引。 */
#define LCD_MIPI_FRAMEBUFFER_BACK_INITIAL        (1U)
/* MIPI 私有 framebuffer 数量，前后双缓冲。 */
#define LCD_MIPI_FRAMEBUFFER_COUNT               (2U)
#define LCD_MIPI_CACHE_LINE_BYTES                (32U)
#define LCD_MIPI_GLCDC_UPDATE_TIMEOUT_MS         (50U)

#if (DISPLAY_BITS_PER_PIXEL_INPUT0 != 32)
#error "MIPI LCD driver expects GLCDC input0 to be 32-bit RGB888."
#endif

static volatile bool g_lcd_mipi_cmd_done = false;
static volatile fsp_err_t g_lcd_mipi_cmd_result = FSP_SUCCESS;
static volatile mipi_dsi_sequence_status_t g_lcd_mipi_last_tx_status = MIPI_DSI_SEQUENCE_STATUS_NONE;
static bool g_lcd_mipi_initialized = false;
static uint8_t g_lcd_mipi_front_framebuffer = LCD_MIPI_FRAMEBUFFER_FRONT_INITIAL;
static uint8_t g_lcd_mipi_back_framebuffer = LCD_MIPI_FRAMEBUFFER_BACK_INITIAL;
static bool g_lcd_mipi_frame_started = false;

/* MIPI 私有 framebuffer，放入 non-cache SDRAM，避免 GLCDC 扫描 cacheable fb_background 产生残留条纹。 */
SDRAM_SECTION_NOINIT_NOCACHE static uint8_t g_lcd_mipi_framebuffer[LCD_MIPI_FRAMEBUFFER_COUNT]
                                                                    [DISPLAY_BUFFER_STRIDE_BYTES_INPUT0 *
                                                                     DISPLAY_VSIZE_INPUT0];

/*
 * P040B019-MIPI / ST7701P 原厂初始化表。
 * 格式：packet_id, delay_ms, payload_len, payload...
 * packet_id 0x39 表示 DCS long write，delay_ms 在本条命令发送完成后执行。
 */
static const uint8_t g_lcd_mipi_st7701p_init_sequence[] =
{
    0x39U, 0U,   6U,  0xFFU, 0x77U, 0x01U, 0x00U, 0x00U, 0x13U,
    0x39U, 0U,   2U,  0xEFU, 0x08U,
    0x39U, 0U,   6U,  0xFFU, 0x77U, 0x01U, 0x00U, 0x00U, 0x10U,
    0x39U, 0U,   3U,  0xC0U, 0x63U, 0x00U,
    0x39U, 0U,   3U,  0xC1U, 0x09U, 0x02U,
    0x39U, 0U,   3U,  0xC2U, 0x21U, 0x02U,
    0x39U, 0U,   2U,  0xCCU, 0x18U,
    0x39U, 0U,   17U, 0xB0U, 0x40U, 0x0EU, 0x51U, 0x0FU, 0x11U, 0x07U, 0x00U, 0x09U, 0x06U, 0x1EU, 0x04U, 0x12U, 0x11U, 0x64U, 0x29U, 0xDFU,
    0x39U, 0U,   17U, 0xB1U, 0x40U, 0x07U, 0x4CU, 0x0AU, 0x0EU, 0x04U, 0x00U, 0x08U, 0x09U, 0x1DU, 0x01U, 0x0EU, 0x0CU, 0x6AU, 0x34U, 0xDFU,
    0x39U, 0U,   6U,  0xFFU, 0x77U, 0x01U, 0x00U, 0x00U, 0x11U,
    0x39U, 0U,   2U,  0xB0U, 0x30U,
    0x39U, 0U,   2U,  0xB1U, 0x40U,
    0x39U, 0U,   2U,  0xB2U, 0x80U,
    0x39U, 0U,   2U,  0xB3U, 0x80U,
    0x39U, 0U,   2U,  0xB5U, 0x4FU,
    0x39U, 0U,   2U,  0xB7U, 0x85U,
    0x39U, 0U,   2U,  0xB8U, 0x23U,
    0x39U, 0U,   3U,  0xB9U, 0x22U, 0x13U,
    0x39U, 0U,   2U,  0xBBU, 0x03U,
    0x39U, 0U,   2U,  0xBCU, 0x10U,
    0x39U, 0U,   2U,  0xC0U, 0x89U,
    0x39U, 0U,   2U,  0xC1U, 0x78U,
    0x39U, 0U,   2U,  0xC2U, 0x78U,
    0x39U, 0U,   2U,  0xD0U, 0x88U,
    0x39U, 0U,   4U,  0xE0U, 0x00U, 0x00U, 0x02U,
    0x39U, 0U,   12U, 0xE1U, 0x04U, 0x00U, 0x00U, 0x00U, 0x05U, 0x00U, 0x00U, 0x00U, 0x00U, 0x10U, 0x10U,
    0x39U, 0U,   14U, 0xE2U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x39U, 0U,   5U,  0xE3U, 0x00U, 0x00U, 0x33U, 0x00U,
    0x39U, 0U,   3U,  0xE4U, 0x22U, 0x00U,
    0x39U, 0U,   17U, 0xE5U, 0x03U, 0x34U, 0xAFU, 0xB3U, 0x05U, 0x34U, 0xAFU, 0xB3U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x39U, 0U,   5U,  0xE6U, 0x00U, 0x00U, 0x33U, 0x00U,
    0x39U, 0U,   3U,  0xE7U, 0x22U, 0x00U,
    0x39U, 0U,   17U, 0xE8U, 0x04U, 0x34U, 0xAFU, 0xB3U, 0x06U, 0x34U, 0xAFU, 0xB3U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x39U, 0U,   8U,  0xEBU, 0x02U, 0x00U, 0x40U, 0x40U, 0x00U, 0x00U, 0x00U,
    0x39U, 0U,   3U,  0xECU, 0x00U, 0x00U,
    0x39U, 0U,   17U, 0xEDU, 0xFAU, 0x45U, 0x0BU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xB0U, 0x54U, 0xAFU,
    0x39U, 0U,   7U,  0xEFU, 0x08U, 0x08U, 0x08U, 0x45U, 0x3FU, 0x54U,
    0x39U, 0U,   6U,  0xFFU, 0x77U, 0x01U, 0x00U, 0x00U, 0x13U,
    0x39U, 0U,   3U,  0xE6U, 0x16U, 0x7CU,
    0x39U, 0U,   3U,  0xE8U, 0x00U, 0x0EU,
    0x39U, 0U,   6U,  0xFFU, 0x77U, 0x01U, 0x00U, 0x00U, 0x00U,
    0x39U, 120U, 2U,  0x11U, 0x00U,
    0x39U, 0U,   6U,  0xFFU, 0x77U, 0x01U, 0x00U, 0x00U, 0x13U,
    0x39U, 10U,  3U,  0xE8U, 0x00U, 0x0CU,
    0x39U, 0U,   3U,  0xE8U, 0x00U, 0x00U,
    0x39U, 0U,   6U,  0xFFU, 0x77U, 0x01U, 0x00U, 0x00U, 0x00U,
    0x39U, 0U,   2U,  0x35U, 0x00U,
    0x39U, 20U,  2U,  0x29U, 0x00U,
};

static void lcd_mipi_delay_ms(uint32_t delay_ms)
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

static void lcd_mipi_cache_clean(void *p_addr, uint32_t bytes)
{
#if defined(RENESAS_CORTEX_M85) && (BSP_CFG_DCACHE_ENABLED)
    uintptr_t start_addr = ((uintptr_t)p_addr) & ~((uintptr_t)LCD_MIPI_CACHE_LINE_BYTES - 1U);
    uintptr_t end_addr = ((uintptr_t)p_addr + (uintptr_t)bytes + ((uintptr_t)LCD_MIPI_CACHE_LINE_BYTES - 1U)) &
                         ~((uintptr_t)LCD_MIPI_CACHE_LINE_BYTES - 1U);

    SCB_CleanInvalidateDCache_by_Addr((void *)start_addr, (int32_t)(end_addr - start_addr));
#else
    FSP_PARAMETER_NOT_USED(p_addr);
    FSP_PARAMETER_NOT_USED(bytes);
#endif
}

/**
 * @brief  切换 GLCDC 图层到 MIPI 私有 framebuffer
 *
 * @param framebuffer_index 私有 framebuffer 索引
 * @return fsp_err_t FSP_SUCCESS 表示切换成功
 */
static fsp_err_t lcd_mipi_layer_change(uint8_t framebuffer_index)
{
    display_runtime_cfg_t runtime_cfg;

    if (framebuffer_index >= LCD_MIPI_FRAMEBUFFER_COUNT)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    runtime_cfg = g_display0_runtime_cfg_bg;
    runtime_cfg.input.p_base = (uint32_t *)g_lcd_mipi_framebuffer[framebuffer_index];

    return g_display0.p_api->layerChange(g_display0.p_ctrl, &runtime_cfg, DISPLAY_FRAME_LAYER_1);
}

static void lcd_mipi_pin_write(bsp_io_port_pin_t pin, bsp_io_level_t level)
{
    (void)g_ioport.p_api->pinWrite(g_ioport.p_ctrl, pin, level);
}

static fsp_err_t lcd_mipi_backlight_config(void)
{
    fsp_err_t err = g_ioport.p_api->pinCfg(g_ioport.p_ctrl,
                                           LCD_MIPI_PIN_BL,
                                           (uint32_t)IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                                           (uint32_t)IOPORT_CFG_PORT_OUTPUT_LOW);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    lcd_mipi_pin_write(LCD_MIPI_PIN_BL, BSP_IO_LEVEL_LOW);

    return FSP_SUCCESS;
}

static void lcd_mipi_reset_panel(void)
{
    lcd_mipi_pin_write(LCD_MIPI_PIN_RST, BSP_IO_LEVEL_LOW);
    lcd_mipi_delay_ms(20U);
    lcd_mipi_pin_write(LCD_MIPI_PIN_RST, BSP_IO_LEVEL_HIGH);
    lcd_mipi_delay_ms(120U);
}

static fsp_err_t lcd_mipi_wait_command(uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0U;

    while (!g_lcd_mipi_cmd_done)
    {
        if (elapsed_ms >= timeout_ms)
        {
            return FSP_ERR_TIMEOUT;
        }

        lcd_mipi_delay_ms(1U);
        elapsed_ms++;
    }

    return g_lcd_mipi_cmd_result;
}

static fsp_err_t lcd_mipi_wait_glcdc_update(uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0U;

    while (((bool)R_GLCDC->GR[DISPLAY_FRAME_LAYER_1].VEN_b.PVEN) ||
           ((bool)R_GLCDC->BG.EN_b.VEN))
    {
        if (elapsed_ms >= timeout_ms)
        {
            return FSP_ERR_TIMEOUT;
        }

        lcd_mipi_delay_ms(1U);
        elapsed_ms++;
    }

    return FSP_SUCCESS;
}

static fsp_err_t lcd_mipi_send_packet(mipi_cmd_id_t cmd_id, const uint8_t *p_payload, uint16_t bytes)
{
    mipi_dsi_cmd_t cmd;
    fsp_err_t err;

    if ((NULL == p_payload) || (0U == bytes))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    g_lcd_mipi_cmd_done = false;
    g_lcd_mipi_cmd_result = FSP_SUCCESS;
    g_lcd_mipi_last_tx_status = MIPI_DSI_SEQUENCE_STATUS_NONE;

    cmd.channel = 0U;
    cmd.cmd_id = cmd_id;
    cmd.flags = MIPI_DSI_CMD_FLAG_LOW_POWER;
    cmd.tx_len = bytes;
    cmd.p_tx_buffer = p_payload;
    cmd.p_rx_buffer = NULL;

    err = g_mipi_dsi0.p_api->command(g_mipi_dsi0.p_ctrl, &cmd);
    if (FSP_SUCCESS != err)
    {
        g_lcd_mipi_cmd_done = true;
        return err;
    }

    return lcd_mipi_wait_command(LCD_MIPI_DSI_TIMEOUT_MS);
}

static fsp_err_t lcd_mipi_send_table(const uint8_t *p_table, uint32_t bytes)
{
    uint32_t index = 0U;
    fsp_err_t err;

    while (index < bytes)
    {
        mipi_cmd_id_t cmd_id;
        uint8_t delay_ms;
        uint8_t payload_len;

        if ((bytes - index) < 3U)
        {
            return FSP_ERR_INVALID_ARGUMENT;
        }

        cmd_id = (mipi_cmd_id_t)p_table[index];
        index++;
        delay_ms = p_table[index];
        index++;
        payload_len = p_table[index];
        index++;

        if (((uint32_t)payload_len > (bytes - index)) || (0U == payload_len))
        {
            return FSP_ERR_INVALID_ARGUMENT;
        }

        err = lcd_mipi_send_packet(cmd_id, &p_table[index], payload_len);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        index += payload_len;
        lcd_mipi_delay_ms(delay_ms);
    }

    return FSP_SUCCESS;
}

static void lcd_mipi_rgb565_to_rgb888(uint16_t color, uint8_t *p_red, uint8_t *p_green, uint8_t *p_blue)
{
    uint8_t red5 = (uint8_t)((color >> 11) & 0x1FU);
    uint8_t green6 = (uint8_t)((color >> 5) & 0x3FU);
    uint8_t blue5 = (uint8_t)(color & 0x1FU);

    *p_red = (uint8_t)((red5 << 3) | (red5 >> 2));
    *p_green = (uint8_t)((green6 << 2) | (green6 >> 4));
    *p_blue = (uint8_t)((blue5 << 3) | (blue5 >> 2));
}

static void lcd_mipi_write_rgb888_pixel(uint8_t *p_dst, uint8_t red, uint8_t green, uint8_t blue)
{
    /*
     * GLCDC input0 当前是 32-bit RGB888，小端内存先按 B,G,R,0 写入。
     * 如果上板后红蓝反，只调整本函数的写入顺序。
     */
    p_dst[0] = blue;
    p_dst[1] = green;
    p_dst[2] = red;
    p_dst[3] = 0x00U;
}

static void lcd_mipi_clear_framebuffer(uint8_t framebuffer_index, uint16_t color)
{
    uint32_t row;
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    lcd_mipi_rgb565_to_rgb888(color, &red, &green, &blue);

    for (row = 0U; row < LCD_MIPI_ST7701P_HEIGHT; row++)
    {
        uint32_t col;
        uint8_t *p_dst = &g_lcd_mipi_framebuffer[framebuffer_index][row * DISPLAY_BUFFER_STRIDE_BYTES_INPUT0];

        for (col = 0U; col < LCD_MIPI_ST7701P_WIDTH; col++)
        {
            lcd_mipi_write_rgb888_pixel(&p_dst[col * LCD_MIPI_FRAMEBUFFER_BYTES_PER_PIXEL], red, green, blue);
        }
    }

    lcd_mipi_cache_clean(g_lcd_mipi_framebuffer[framebuffer_index],
                         (uint32_t)sizeof(g_lcd_mipi_framebuffer[framebuffer_index]));
}

static void lcd_mipi_prepare_back_framebuffer(void)
{
    if (g_lcd_mipi_frame_started)
    {
        return;
    }

    memcpy(g_lcd_mipi_framebuffer[g_lcd_mipi_back_framebuffer],
           g_lcd_mipi_framebuffer[g_lcd_mipi_front_framebuffer],
           sizeof(g_lcd_mipi_framebuffer[g_lcd_mipi_back_framebuffer]));
    lcd_mipi_cache_clean(g_lcd_mipi_framebuffer[g_lcd_mipi_back_framebuffer],
                         (uint32_t)sizeof(g_lcd_mipi_framebuffer[g_lcd_mipi_back_framebuffer]));
    g_lcd_mipi_frame_started = true;
}

static fsp_err_t lcd_mipi_panel_init_sequence(void)
{
    return lcd_mipi_send_table(g_lcd_mipi_st7701p_init_sequence, sizeof(g_lcd_mipi_st7701p_init_sequence));
}

static fsp_err_t lcd_mipi_st7701p_init_impl(void)
{
    fsp_err_t err;

    if (g_lcd_mipi_initialized)
    {
        return FSP_SUCCESS;
    }

    err = lcd_mipi_backlight_config();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    lcd_mipi_reset_panel();
    g_lcd_mipi_front_framebuffer = LCD_MIPI_FRAMEBUFFER_FRONT_INITIAL;
    g_lcd_mipi_back_framebuffer = LCD_MIPI_FRAMEBUFFER_BACK_INITIAL;
    g_lcd_mipi_frame_started = false;
    lcd_mipi_clear_framebuffer(g_lcd_mipi_front_framebuffer, LCD_MIPI_ST7701P_COLOR_BLACK);
    lcd_mipi_clear_framebuffer(g_lcd_mipi_back_framebuffer, LCD_MIPI_ST7701P_COLOR_BLACK);

    err = g_display0.p_api->open(g_display0.p_ctrl, g_display0.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    err = lcd_mipi_layer_change(g_lcd_mipi_front_framebuffer);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = lcd_mipi_panel_init_sequence();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = g_display0.p_api->start(g_display0.p_ctrl);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    lcd_mipi_pin_write(LCD_MIPI_PIN_BL, BSP_IO_LEVEL_HIGH);
    g_lcd_mipi_initialized = true;

    return FSP_SUCCESS;
}

static fsp_err_t lcd_mipi_st7701p_flush_area_rgb565_impl(uint16_t x1,
                                                         uint16_t y1,
                                                         uint16_t x2,
                                                         uint16_t y2,
                                                         const uint8_t *p_data)
{
    uint32_t row;
    uint32_t width;
    uint32_t height;
    uint32_t src_row_bytes;
    uint32_t dst_row_bytes;
    bool full_width;

    if (!g_lcd_mipi_initialized)
    {
        return FSP_ERR_NOT_OPEN;
    }

    if ((NULL == p_data) || (x1 > x2) || (y1 > y2) ||
        (x2 >= LCD_MIPI_ST7701P_WIDTH) || (y2 >= LCD_MIPI_ST7701P_HEIGHT))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    width = ((uint32_t)x2 - (uint32_t)x1) + 1U;
    height = ((uint32_t)y2 - (uint32_t)y1) + 1U;
    src_row_bytes = width * LCD_MIPI_LVGL_BYTES_PER_PIXEL;
    dst_row_bytes = width * LCD_MIPI_FRAMEBUFFER_BYTES_PER_PIXEL;
    full_width = ((0U == x1) && ((LCD_MIPI_ST7701P_WIDTH - 1U) == x2));

    lcd_mipi_prepare_back_framebuffer();

    for (row = 0U; row < height; row++)
    {
        uint32_t col;
        uint8_t *p_dst = &g_lcd_mipi_framebuffer[g_lcd_mipi_back_framebuffer]
                                               [(((uint32_t)y1 + row) * DISPLAY_BUFFER_STRIDE_BYTES_INPUT0) +
                                                ((uint32_t)x1 * LCD_MIPI_FRAMEBUFFER_BYTES_PER_PIXEL)];
        const uint8_t *p_src = &p_data[row * src_row_bytes];

        for (col = 0U; col < width; col++)
        {
            uint16_t color = (uint16_t)(((uint16_t)p_src[(col * LCD_MIPI_LVGL_BYTES_PER_PIXEL)]) |
                                         ((uint16_t)p_src[(col * LCD_MIPI_LVGL_BYTES_PER_PIXEL) + 1U] << 8));
            uint8_t red;
            uint8_t green;
            uint8_t blue;

            lcd_mipi_rgb565_to_rgb888(color, &red, &green, &blue);
            lcd_mipi_write_rgb888_pixel(&p_dst[col * LCD_MIPI_FRAMEBUFFER_BYTES_PER_PIXEL], red, green, blue);
        }

        if (!full_width)
        {
            lcd_mipi_cache_clean(p_dst, dst_row_bytes);
        }
    }

    if (full_width)
    {
        uint8_t *p_dst = &g_lcd_mipi_framebuffer[g_lcd_mipi_back_framebuffer]
                                               [(uint32_t)y1 * DISPLAY_BUFFER_STRIDE_BYTES_INPUT0];
        lcd_mipi_cache_clean(p_dst, height * DISPLAY_BUFFER_STRIDE_BYTES_INPUT0);
    }

    return FSP_SUCCESS;
}

static void lcd_mipi_rotation_point(uint16_t logical_x,
                                    uint16_t logical_y,
                                    lcd_mipi_st7701p_rotation_t rotation,
                                    uint16_t *p_physical_x,
                                    uint16_t *p_physical_y)
{
    if (LCD_MIPI_ST7701P_ROTATION_90 == rotation)
    {
        *p_physical_x = logical_y;
        *p_physical_y = (uint16_t)((LCD_MIPI_ST7701P_HEIGHT - 1U) - logical_x);
    }
    else
    {
        *p_physical_x = (uint16_t)((LCD_MIPI_ST7701P_WIDTH - 1U) - logical_y);
        *p_physical_y = logical_x;
    }
}

static void lcd_mipi_rotation_area(uint16_t x1,
                                   uint16_t y1,
                                   uint16_t x2,
                                   uint16_t y2,
                                   lcd_mipi_st7701p_rotation_t rotation,
                                   uint16_t *p_physical_x1,
                                   uint16_t *p_physical_y1,
                                   uint16_t *p_physical_x2,
                                   uint16_t *p_physical_y2)
{
    if (LCD_MIPI_ST7701P_ROTATION_90 == rotation)
    {
        *p_physical_x1 = y1;
        *p_physical_x2 = y2;
        *p_physical_y1 = (uint16_t)((LCD_MIPI_ST7701P_HEIGHT - 1U) - x2);
        *p_physical_y2 = (uint16_t)((LCD_MIPI_ST7701P_HEIGHT - 1U) - x1);
    }
    else
    {
        *p_physical_x1 = (uint16_t)((LCD_MIPI_ST7701P_WIDTH - 1U) - y2);
        *p_physical_x2 = (uint16_t)((LCD_MIPI_ST7701P_WIDTH - 1U) - y1);
        *p_physical_y1 = x1;
        *p_physical_y2 = x2;
    }
}

static fsp_err_t lcd_mipi_st7701p_flush_area_rgb565_rotated_impl(uint16_t x1,
                                                                 uint16_t y1,
                                                                 uint16_t x2,
                                                                 uint16_t y2,
                                                                 const uint8_t *p_data,
                                                                 lcd_mipi_st7701p_rotation_t rotation)
{
    uint32_t row;
    uint32_t width;
    uint32_t height;
    uint32_t src_row_bytes;
    uint16_t physical_x1;
    uint16_t physical_y1;
    uint16_t physical_x2;
    uint16_t physical_y2;

    if (LCD_MIPI_ST7701P_ROTATION_0 == rotation)
    {
        return lcd_mipi_st7701p_flush_area_rgb565_impl(x1, y1, x2, y2, p_data);
    }

    if (!g_lcd_mipi_initialized)
    {
        return FSP_ERR_NOT_OPEN;
    }

    if ((NULL == p_data) || (x1 > x2) || (y1 > y2) ||
        (x2 >= LCD_MIPI_ST7701P_HEIGHT) || (y2 >= LCD_MIPI_ST7701P_WIDTH))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    width = ((uint32_t)x2 - (uint32_t)x1) + 1U;
    height = ((uint32_t)y2 - (uint32_t)y1) + 1U;
    src_row_bytes = width * LCD_MIPI_LVGL_BYTES_PER_PIXEL;

    lcd_mipi_prepare_back_framebuffer();

    for (row = 0U; row < height; row++)
    {
        uint32_t col;
        const uint8_t *p_src = &p_data[row * src_row_bytes];

        for (col = 0U; col < width; col++)
        {
            uint16_t physical_x;
            uint16_t physical_y;
            uint16_t color = (uint16_t)(((uint16_t)p_src[(col * LCD_MIPI_LVGL_BYTES_PER_PIXEL)]) |
                                         ((uint16_t)p_src[(col * LCD_MIPI_LVGL_BYTES_PER_PIXEL) + 1U] << 8));
            uint8_t red;
            uint8_t green;
            uint8_t blue;
            uint8_t *p_dst;

            lcd_mipi_rotation_point((uint16_t)((uint32_t)x1 + col),
                                    (uint16_t)((uint32_t)y1 + row),
                                    rotation,
                                    &physical_x,
                                    &physical_y);
            p_dst = &g_lcd_mipi_framebuffer[g_lcd_mipi_back_framebuffer]
                                          [((uint32_t)physical_y * DISPLAY_BUFFER_STRIDE_BYTES_INPUT0) +
                                           ((uint32_t)physical_x * LCD_MIPI_FRAMEBUFFER_BYTES_PER_PIXEL)];

            lcd_mipi_rgb565_to_rgb888(color, &red, &green, &blue);
            lcd_mipi_write_rgb888_pixel(p_dst, red, green, blue);
        }
    }

    lcd_mipi_rotation_area(x1,
                           y1,
                           x2,
                           y2,
                           rotation,
                           &physical_x1,
                           &physical_y1,
                           &physical_x2,
                           &physical_y2);

    for (row = physical_y1; row <= physical_y2; row++)
    {
        uint8_t *p_dst = &g_lcd_mipi_framebuffer[g_lcd_mipi_back_framebuffer]
                                               [(row * DISPLAY_BUFFER_STRIDE_BYTES_INPUT0) +
                                                ((uint32_t)physical_x1 * LCD_MIPI_FRAMEBUFFER_BYTES_PER_PIXEL)];
        uint32_t clean_bytes = (((uint32_t)physical_x2 - (uint32_t)physical_x1) + 1U) *
                               LCD_MIPI_FRAMEBUFFER_BYTES_PER_PIXEL;

        lcd_mipi_cache_clean(p_dst, clean_bytes);
    }

    return FSP_SUCCESS;
}

static fsp_err_t lcd_mipi_st7701p_present_impl(void)
{
    fsp_err_t err;
    uint8_t old_front;

    if (!g_lcd_mipi_initialized)
    {
        return FSP_ERR_NOT_OPEN;
    }

    if (!g_lcd_mipi_frame_started)
    {
        return FSP_SUCCESS;
    }

    err = lcd_mipi_wait_glcdc_update(LCD_MIPI_GLCDC_UPDATE_TIMEOUT_MS);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = g_display0.p_api->bufferChange(g_display0.p_ctrl,
                                         g_lcd_mipi_framebuffer[g_lcd_mipi_back_framebuffer],
                                         DISPLAY_FRAME_LAYER_1);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /*
     * 等到影子寄存器在下一次 Vsync 生效，再把旧 front 当作新的 back。
     * 否则下一帧太快开始时，CPU 可能又写到 GLCDC 仍在扫描的 buffer。
     */
    err = lcd_mipi_wait_glcdc_update(LCD_MIPI_GLCDC_UPDATE_TIMEOUT_MS);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    old_front = g_lcd_mipi_front_framebuffer;
    g_lcd_mipi_front_framebuffer = g_lcd_mipi_back_framebuffer;
    g_lcd_mipi_back_framebuffer = old_front;
    g_lcd_mipi_frame_started = false;

    return FSP_SUCCESS;
}

static bool lcd_mipi_st7701p_is_ready_impl(void)
{
    return g_lcd_mipi_initialized;
}

static void lcd_mipi_st7701p_notify_mipi_callback_impl(mipi_dsi_callback_args_t *p_args)
{
    mipi_dsi_sequence_status_t error_status;

    if (NULL == p_args)
    {
        return;
    }

    if ((MIPI_DSI_EVENT_SEQUENCE_0 != p_args->event) && (MIPI_DSI_EVENT_SEQUENCE_1 != p_args->event))
    {
        if (MIPI_DSI_EVENT_FATAL == p_args->event)
        {
            g_lcd_mipi_cmd_result = FSP_ERR_ABORTED;
            g_lcd_mipi_cmd_done = true;
        }
        return;
    }

    g_lcd_mipi_last_tx_status = p_args->tx_status;
    error_status = (mipi_dsi_sequence_status_t)
        (p_args->tx_status &
         (MIPI_DSI_SEQUENCE_STATUS_DESCRIPTOR_ABORT |
          MIPI_DSI_SEQUENCE_STATUS_SIZE_ERROR |
          MIPI_DSI_SEQUENCE_STATUS_TX_INTERNAL_BUS_ERROR |
          MIPI_DSI_SEQUENCE_STATUS_RX_FATAL_ERROR |
          MIPI_DSI_SEQUENCE_STATUS_RX_FAIL |
          MIPI_DSI_SEQUENCE_STATUS_RX_PACKET_DATA_FAIL |
          MIPI_DSI_SEQUENCE_STATUS_RX_CORRECTABLE_ERROR |
          MIPI_DSI_SEQUENCE_STATUS_RX_ACK_AND_ERROR));

    if (MIPI_DSI_SEQUENCE_STATUS_NONE != error_status)
    {
        g_lcd_mipi_cmd_result = FSP_ERR_ABORTED;
        g_lcd_mipi_cmd_done = true;
        return;
    }

    if (0U != (p_args->tx_status &
               (MIPI_DSI_SEQUENCE_STATUS_ACTIONS_FINISHED | MIPI_DSI_SEQUENCE_STATUS_DESCRIPTORS_FINISHED)))
    {
        g_lcd_mipi_cmd_result = FSP_SUCCESS;
        g_lcd_mipi_cmd_done = true;
    }
}

const lcd_mipi_st7701p_api_t lcd_mipi_st7701p =
{
    .init = lcd_mipi_st7701p_init_impl,
    .flush_area_rgb565 = lcd_mipi_st7701p_flush_area_rgb565_impl,
    .flush_area_rgb565_rotated = lcd_mipi_st7701p_flush_area_rgb565_rotated_impl,
    .present = lcd_mipi_st7701p_present_impl,
    .is_ready = lcd_mipi_st7701p_is_ready_impl,
    .notify_mipi_callback = lcd_mipi_st7701p_notify_mipi_callback_impl,
};


