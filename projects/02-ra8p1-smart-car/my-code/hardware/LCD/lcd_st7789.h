#ifndef HARDWARE_LCD_ST7789_H_
#define HARDWARE_LCD_ST7789_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

/* ST7789 原始可视区域宽度，单位像素。 */
#define LCD_ST7789_NATIVE_WIDTH        (240U)

/* ST7789 原始可视区域高度，单位像素。 */
#define LCD_ST7789_NATIVE_HEIGHT       (280U)

/* ST7789 默认显示方向，90U/270U 为横屏。 */
#ifndef LCD_ST7789_ROTATION
#define LCD_ST7789_ROTATION            (90U)
#endif

#if ((0U != LCD_ST7789_ROTATION) && (90U != LCD_ST7789_ROTATION) && \
     (180U != LCD_ST7789_ROTATION) && (270U != LCD_ST7789_ROTATION))
#error "LCD_ST7789_ROTATION must be 0U, 90U, 180U or 270U"
#endif

#if ((90U == LCD_ST7789_ROTATION) || (270U == LCD_ST7789_ROTATION))
/* ST7789 横屏逻辑宽度，单位像素。 */
#define LCD_ST7789_WIDTH               (LCD_ST7789_NATIVE_HEIGHT)

/* ST7789 横屏逻辑高度，单位像素。 */
#define LCD_ST7789_HEIGHT              (LCD_ST7789_NATIVE_WIDTH)
#else
/* ST7789 竖屏逻辑宽度，单位像素。 */
#define LCD_ST7789_WIDTH               (LCD_ST7789_NATIVE_WIDTH)

/* ST7789 竖屏逻辑高度，单位像素。 */
#define LCD_ST7789_HEIGHT              (LCD_ST7789_NATIVE_HEIGHT)
#endif

/* LVGL 局部绘制缓冲区行数。 */
#define LCD_ST7789_DRAW_BUF_LINES      (40U)

/* RGB565 黑色。 */
#define LCD_ST7789_COLOR_BLACK         (0x0000U)

/* RGB565 白色。 */
#define LCD_ST7789_COLOR_WHITE         (0xFFFFU)

/* RGB565 红色。 */
#define LCD_ST7789_COLOR_RED           (0xF800U)

/* RGB565 绿色。 */
#define LCD_ST7789_COLOR_GREEN         (0x07E0U)

/* RGB565 蓝色。 */
#define LCD_ST7789_COLOR_BLUE          (0x001FU)

typedef struct st_lcd_st7789_api
{
    /* 初始化 ST7789 屏幕。 */
    fsp_err_t (*init)(void);

    /* 设置后续像素写入窗口。 */
    fsp_err_t (*set_window)(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

    /* 写入 RGB565 像素数据。 */
    fsp_err_t (*write_pixels)(const uint8_t *p_data, uint32_t bytes);

    /* 全屏填充 RGB565 颜色。 */
    fsp_err_t (*fill_rgb565)(uint16_t color);

    /* 查询屏幕是否已经初始化。 */
    bool (*is_ready)(void);

    /* 转发 SCI-B SPI 传输完成回调。 */
    void (*notify_spi_callback)(spi_callback_args_t *p_args);
} lcd_st7789_api_t;

extern const lcd_st7789_api_t lcd_st7789;

#endif /* HARDWARE_LCD_ST7789_H_ */
