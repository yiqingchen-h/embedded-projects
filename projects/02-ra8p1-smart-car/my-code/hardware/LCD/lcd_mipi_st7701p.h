#ifndef HARDWARE_LCD_MIPI_ST7701P_H_
#define HARDWARE_LCD_MIPI_ST7701P_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

#define LCD_MIPI_ST7701P_WIDTH            (480U)
#define LCD_MIPI_ST7701P_HEIGHT           (800U)
#define LCD_MIPI_ST7701P_DRAW_BUF_LINES   (80U)

#define LCD_MIPI_ST7701P_COLOR_BLACK      (0x0000U)
#define LCD_MIPI_ST7701P_COLOR_WHITE      (0xFFFFU)

typedef enum e_lcd_mipi_st7701p_rotation
{
    /** @brief 不旋转，按物理 480x800 坐标写入 */
    LCD_MIPI_ST7701P_ROTATION_0 = 0,
    /** @brief 逻辑 800x480 顺时针映射到物理 480x800 */
    LCD_MIPI_ST7701P_ROTATION_90,
    /** @brief 逻辑 800x480 逆时针映射到物理 480x800 */
    LCD_MIPI_ST7701P_ROTATION_270,
} lcd_mipi_st7701p_rotation_t;

typedef struct st_lcd_mipi_st7701p_api
{
    fsp_err_t (*init)(void);
    fsp_err_t (*flush_area_rgb565)(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint8_t *p_data);
    fsp_err_t (*flush_area_rgb565_rotated)(uint16_t x1,
                                           uint16_t y1,
                                           uint16_t x2,
                                           uint16_t y2,
                                           const uint8_t *p_data,
                                           lcd_mipi_st7701p_rotation_t rotation);
    fsp_err_t (*present)(void);
    bool (*is_ready)(void);
    void (*notify_mipi_callback)(mipi_dsi_callback_args_t *p_args);
} lcd_mipi_st7701p_api_t;

extern const lcd_mipi_st7701p_api_t lcd_mipi_st7701p;

#endif /* HARDWARE_LCD_MIPI_ST7701P_H_ */
