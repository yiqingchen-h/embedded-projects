#ifndef SOFTWARE_UI_ST7789_UI_H_
#define SOFTWARE_UI_ST7789_UI_H_

#include "hal_data.h"
#include "hardware/Chassis/Chassis_config.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief ST7789 页面视图类型 */
typedef enum e_st7789_ui_view
{
    /* 开机动画视图。 */
    ST7789_UI_VIEW_BOOT = 0,

    /* 功能选择菜单视图。 */
    ST7789_UI_VIEW_MENU,

    /* 功能占位调试页视图。 */
    ST7789_UI_VIEW_PAGE,
} st7789_ui_view_t;

/** @brief ST7789 功能页面编号 */
typedef enum e_st7789_ui_page
{
    /* 麦克伦速度 PID 调试页。 */
    ST7789_UI_PAGE_MECANUM_PID = 0,

    /* 机械臂调试标定页。 */
    ST7789_UI_PAGE_ARM_CALIB,

    /* 雷达页。 */
    ST7789_UI_PAGE_RADAR,

    /* 避障 PID 调试页。 */
    ST7789_UI_PAGE_AVOID_PID,

    /* 数据监控页。 */
    ST7789_UI_PAGE_MONITOR,

    /* 功能页面数量。 */
    ST7789_UI_PAGE_COUNT,
} st7789_ui_page_t;

/** @brief ST7789 麦轮调试控制模式 */
typedef enum e_st7789_ui_mecanum_mode
{
    /* 底盘整体控制子界面。 */
    ST7789_UI_MECANUM_MODE_BODY = 0,

    /* 单轮速度控制子界面。 */
    ST7789_UI_MECANUM_MODE_WHEEL,

    /* PID 调参子界面。 */
    ST7789_UI_MECANUM_MODE_PID,

    /* 四路开环 PWM 诊断控制子界面。 */
    ST7789_UI_MECANUM_MODE_PWM,
} st7789_ui_mecanum_mode_t;

/** @brief ST7789 麦轮调试页面项目 */
typedef enum e_st7789_ui_mecanum_item
{
    /* 控制模式项目。 */
    ST7789_UI_MECANUM_ITEM_MODE = 0,

    /* 前进速度项目。 */
    ST7789_UI_MECANUM_ITEM_VX,

    /* 横移速度项目。 */
    ST7789_UI_MECANUM_ITEM_VY,

    /* 旋转角速度项目。 */
    ST7789_UI_MECANUM_ITEM_WZ,

    /* 1 号轮速度项目。 */
    ST7789_UI_MECANUM_ITEM_W1,

    /* 2 号轮速度项目。 */
    ST7789_UI_MECANUM_ITEM_W2,

    /* 3 号轮速度项目。 */
    ST7789_UI_MECANUM_ITEM_W3,

    /* 4 号轮速度项目。 */
    ST7789_UI_MECANUM_ITEM_W4,

    /* 比例系数项目。 */
    ST7789_UI_MECANUM_ITEM_KP,

    /* 积分系数项目。 */
    ST7789_UI_MECANUM_ITEM_KI,

    /* 停车动作项目。 */
    ST7789_UI_MECANUM_ITEM_STOP,

    /* 麦轮调试页面项目数量。 */
    ST7789_UI_MECANUM_ITEM_COUNT,
} st7789_ui_mecanum_item_t;

/** @brief ST7789 麦轮 PID 保存状态 */
typedef enum e_st7789_ui_mecanum_save_status
{
    /* 无保存提示。 */
    ST7789_UI_MECANUM_SAVE_NONE = 0,

    /* 正在保存。 */
    ST7789_UI_MECANUM_SAVE_SAVING,

    /* 保存成功。 */
    ST7789_UI_MECANUM_SAVE_SAVED,

    /* 保存失败。 */
    ST7789_UI_MECANUM_SAVE_ERR,
} st7789_ui_mecanum_save_status_t;

/** @brief ST7789 1.69 寸 UI 状态 */
typedef struct st_st7789_ui_state
{
    /* 开机页进度，范围 0~100。 */
    uint8_t boot_progress;

    /* 当前显示视图。 */
    st7789_ui_view_t view;

    /* 菜单页当前选中项。 */
    uint8_t selected_index;

    /* 当前进入的功能页。 */
    st7789_ui_page_t current_page;

    /* 当前功能页占位调试数值。 */
    int32_t edit_value;

    /* 当前功能页是否处于编辑状态。 */
    bool editing;

    /* 电压电流数据是否有效。 */
    bool power_valid;

    /* 母线电压，单位 mV。 */
    uint32_t bus_voltage_mv;

    /* 电流，单位 uA。 */
    int32_t current_ua;

    /* 麦轮调试当前控制模式。 */
    st7789_ui_mecanum_mode_t mecanum_mode;

    /* 麦轮调试当前选中项目。 */
    uint8_t mecanum_selected_item;

    /* 麦轮调试当前项目是否正在编辑。 */
    bool mecanum_editing;

    /* 麦轮调试编辑中的临时值。 */
    float mecanum_edit_value;

    /* 麦轮调试前进速度，单位 m/s。 */
    float mecanum_vx_mps;

    /* 麦轮调试横移速度，单位 m/s。 */
    float mecanum_vy_mps;

    /* 麦轮调试旋转角速度，单位 rad/s。 */
    float mecanum_wz_radps;

    /* 麦轮调试四路轮速，单位 m/s。 */
    float mecanum_wheel_speed_mps[CHASSIS_WHEEL_COUNT];

    /* 麦轮调试四路开环 PWM 计数值。 */
    int32_t mecanum_pwm_counts[CHASSIS_WHEEL_COUNT];

    /* 麦轮调试四路实测轮速，单位 m/s。 */
    float mecanum_measured_speed_mps[CHASSIS_WHEEL_COUNT];

    /* 麦轮调试比例系数。 */
    float mecanum_kp;

    /* 麦轮调试积分系数。 */
    float mecanum_ki;

    /* 麦轮调试 PID 保存状态。 */
    st7789_ui_mecanum_save_status_t mecanum_save_status;
} st7789_ui_state_t;

/**
 * @brief  初始化 ST7789 开机页 UI 对象
 *
 * @param p_state 初始 UI 状态
 * @return fsp_err_t FSP_SUCCESS 表示初始化成功
 */
fsp_err_t st7789_ui_init(const st7789_ui_state_t *p_state);

/**
 * @brief  按当前状态刷新 ST7789 开机页
 *
 * @param p_state 当前 UI 状态
 * @return void
 */
void st7789_ui_update(const st7789_ui_state_t *p_state);

#endif /* SOFTWARE_UI_ST7789_UI_H_ */
