#ifndef SOFTWARE_UI_LCD_SHOW_H_
#define SOFTWARE_UI_LCD_SHOW_H_

#include "hal_data.h"

#include <stdint.h>

typedef struct st_lcd_show_api
{
    /**
     * @brief  初始化 LCD、LVGL 和开机页
     *
     * @return fsp_err_t FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  处理显示周期时钟
     *
     * 由 UI 任务在 timer7 通知后调用，内部按 30Hz 刷新 LVGL。
     *
     * @param elapsed_ms 距离上次调用经过的时间，单位 ms
     * @return void
     */
    void (*process_1ms_tick)(uint32_t elapsed_ms);

    /**
     * @brief  设置开机页进度
     *
     * @param percent 进度百分比，范围 0~100
     * @return void
     */
    void (*set_boot_progress)(uint8_t percent);
} lcd_show_api_t;

extern const lcd_show_api_t lcd_show;

#endif /* SOFTWARE_UI_LCD_SHOW_H_ */
