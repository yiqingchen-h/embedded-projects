#include "software/ui/ui_port.h"

#include "hardware/LCD/lcd_mipi_st7701p.h"
#include "hardware/LCD/lcd_st7789.h"
#include "software/SDRAM/SDRAM.h"
#include "third_party/lvgl/lvgl.h"

#include <stddef.h>

#ifndef UI_PORT_MIPI_ROTATION
#define UI_PORT_MIPI_ROTATION (0U)
#endif

#if UI_PORT_USE_MIPI_LCD
#if ((90U == UI_PORT_MIPI_ROTATION) || (270U == UI_PORT_MIPI_ROTATION))
#define UI_PORT_DISPLAY_WIDTH  LCD_MIPI_ST7701P_HEIGHT
#define UI_PORT_DISPLAY_HEIGHT LCD_MIPI_ST7701P_WIDTH
#else
#define UI_PORT_DISPLAY_WIDTH  LCD_MIPI_ST7701P_WIDTH
#define UI_PORT_DISPLAY_HEIGHT LCD_MIPI_ST7701P_HEIGHT
#endif
#define UI_PORT_BUF_BYTES      (UI_PORT_DISPLAY_WIDTH * LCD_MIPI_ST7701P_DRAW_BUF_LINES * 2U)
#else
#define UI_PORT_DISPLAY_WIDTH  LCD_ST7789_WIDTH
#define UI_PORT_DISPLAY_HEIGHT LCD_ST7789_HEIGHT
#define UI_PORT_BUF_BYTES      (LCD_ST7789_WIDTH * LCD_ST7789_DRAW_BUF_LINES * 2U)
#endif

SDRAM_SECTION_NOINIT_NOCACHE static uint8_t g_ui_draw_buf1[UI_PORT_BUF_BYTES];
SDRAM_SECTION_NOINIT_NOCACHE static uint8_t g_ui_draw_buf2[UI_PORT_BUF_BYTES];

static lv_display_t *g_ui_display = NULL;

#if !UI_PORT_USE_MIPI_LCD
static void ui_port_rgb565_swap(uint8_t *p_data, uint32_t pixel_count)
{
    uint32_t i;

    for (i = 0U; i < pixel_count; i++)
    {
        uint8_t tmp = p_data[(i * 2U)];
        p_data[(i * 2U)] = p_data[(i * 2U) + 1U];
        p_data[(i * 2U) + 1U] = tmp;
    }
}
#endif

#if UI_PORT_USE_MIPI_LCD
#if (90U == UI_PORT_MIPI_ROTATION)
static lcd_mipi_st7701p_rotation_t ui_port_mipi_rotation(void)
{
    return LCD_MIPI_ST7701P_ROTATION_90;
}
#elif (270U == UI_PORT_MIPI_ROTATION)
static lcd_mipi_st7701p_rotation_t ui_port_mipi_rotation(void)
{
    return LCD_MIPI_ST7701P_ROTATION_270;
}
#else
static lcd_mipi_st7701p_rotation_t ui_port_mipi_rotation(void)
{
    return LCD_MIPI_ST7701P_ROTATION_0;
}
#endif
#endif

static void ui_port_flush_cb(lv_display_t *p_display, const lv_area_t *p_area, uint8_t *p_px_map)
{
    int32_t width;
    int32_t height;
    fsp_err_t err;
#if !UI_PORT_USE_MIPI_LCD
    uint32_t pixel_count;
#else
    bool flush_is_last;
#endif

    if ((NULL == p_display) || (NULL == p_area) || (NULL == p_px_map))
    {
        return;
    }

    width = (p_area->x2 - p_area->x1) + 1;
    height = (p_area->y2 - p_area->y1) + 1;
    if ((width <= 0) || (height <= 0))
    {
        lv_display_flush_ready(p_display);
        return;
    }

#if UI_PORT_USE_MIPI_LCD
    flush_is_last = lv_display_flush_is_last(p_display);
    err = lcd_mipi_st7701p.flush_area_rgb565_rotated((uint16_t)p_area->x1,
                                                     (uint16_t)p_area->y1,
                                                     (uint16_t)p_area->x2,
                                                     (uint16_t)p_area->y2,
                                                     p_px_map,
                                                     ui_port_mipi_rotation());
    if ((FSP_SUCCESS == err) && flush_is_last)
    {
        err = lcd_mipi_st7701p.present();
    }
    (void)err;
#else
    pixel_count = (uint32_t)width * (uint32_t)height;
    ui_port_rgb565_swap(p_px_map, pixel_count);

    err = lcd_st7789.set_window((uint16_t)p_area->x1,
                                (uint16_t)p_area->y1,
                                (uint16_t)p_area->x2,
                                (uint16_t)p_area->y2);
    if (FSP_SUCCESS == err)
    {
        (void)lcd_st7789.write_pixels(p_px_map, pixel_count * 2U);
    }
#endif

    lv_display_flush_ready(p_display);
}

fsp_err_t ui_port_init(void)
{
    fsp_err_t err;

    if (NULL != g_ui_display)
    {
        return FSP_SUCCESS;
    }

    lv_init();

#if UI_PORT_USE_MIPI_LCD
    err = lcd_mipi_st7701p.init();
#else
    err = lcd_st7789.init();
#endif
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    g_ui_display = lv_display_create((int32_t)UI_PORT_DISPLAY_WIDTH, (int32_t)UI_PORT_DISPLAY_HEIGHT);
    if (NULL == g_ui_display)
    {
        return FSP_ERR_INTERNAL;
    }

    lv_display_set_color_format(g_ui_display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(g_ui_display, ui_port_flush_cb);
    lv_display_set_buffers(g_ui_display,
                           g_ui_draw_buf1,
                           g_ui_draw_buf2,
                           (uint32_t)sizeof(g_ui_draw_buf1),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    return FSP_SUCCESS;
}

void ui_port_tick_inc(uint32_t elapsed_ms)
{
    lv_tick_inc(elapsed_ms);
}

void ui_port_handler(void)
{
    (void)lv_timer_handler();
}

#include "software/ui/lvgl_keil_link_shims.c"
