#include "software/ui/st7789_ui.h"

#include "hardware/LCD/lcd_st7789.h"
#include "third_party/lvgl/lvgl.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

/* ST7789 UI 逻辑宽度，横屏默认 280。 */
#define ST7789_UI_WIDTH              ((int32_t)LCD_ST7789_WIDTH)

/* ST7789 UI 逻辑高度，横屏默认 240。 */
#define ST7789_UI_HEIGHT             ((int32_t)LCD_ST7789_HEIGHT)

/* 开机进度条左上角 X 坐标。 */
#define ST7789_UI_PROGRESS_X         (32)

/* 开机进度条左上角 Y 坐标。 */
#define ST7789_UI_PROGRESS_Y         (205)

/* 开机进度条宽度。 */
#define ST7789_UI_PROGRESS_W         (216)

/* 开机进度条高度。 */
#define ST7789_UI_PROGRESS_H         (18)

/* 开机进度条内边距。 */
#define ST7789_UI_PROGRESS_INSET     (3)

/* 彩虹色数量。 */
#define ST7789_UI_RAINBOW_COUNT      (6)

/* 顶部状态栏高度。 */
#define ST7789_UI_STATUS_H           (28)

/* 圆角屏顶部左右安全边距，避免状态文字被裁切。 */
#define ST7789_UI_STATUS_SAFE_X      (36)

/* 圆角屏底部左右安全边距，避免底部元素被裁切。 */
#define ST7789_UI_STATUS_SAFE_BOTTOM_X (36)

/* 顶部状态文字 Y 坐标。 */
#define ST7789_UI_STATUS_TEXT_Y      (6)

/* 顶部状态文字高度。 */
#define ST7789_UI_STATUS_TEXT_H      (18)

/* 菜单项左上角 X 坐标。 */
#define ST7789_UI_MENU_X             (16)

/* 菜单项起始 Y 坐标。 */
#define ST7789_UI_MENU_Y             (38)

/* 菜单项宽度。 */
#define ST7789_UI_MENU_W             (248)

/* 菜单项高度。 */
#define ST7789_UI_MENU_H             (31)

/* 菜单项垂直间距。 */
#define ST7789_UI_MENU_GAP           (7)

/* 功能页数值面板左上角 X 坐标。 */
#define ST7789_UI_PAGE_PANEL_X       (28)

/* 功能页数值面板左上角 Y 坐标。 */
#define ST7789_UI_PAGE_PANEL_Y       (104)

/* 功能页数值面板宽度。 */
#define ST7789_UI_PAGE_PANEL_W       (224)

/* 功能页数值面板高度。 */
#define ST7789_UI_PAGE_PANEL_H       (70)

/* 功能页编辑态边框宽度。 */
#define ST7789_UI_PAGE_EDIT_BORDER_W (2)

/* 电源状态文本缓冲区长度。 */
#define ST7789_UI_POWER_TEXT_LEN     (24U)

/* 功能页数值文本缓冲区长度。 */
#define ST7789_UI_VALUE_TEXT_LEN     (24U)

/* 麦轮调试页标题 Y 坐标。 */
#define ST7789_UI_MECANUM_TITLE_Y    (32)

/* 麦轮调试页列表左上角 X 坐标。 */
#define ST7789_UI_MECANUM_LIST_X     (10)

/* 麦轮调试页列表起始 Y 坐标。 */
#define ST7789_UI_MECANUM_LIST_Y     (56)

/* 麦轮调试页列表宽度。 */
#define ST7789_UI_MECANUM_LIST_W     (260)

/* 麦轮调试页列表行高度。 */
#define ST7789_UI_MECANUM_ROW_H      (22)

/* 麦轮调试页列表行间距。 */
#define ST7789_UI_MECANUM_ROW_GAP    (2)

/* 麦轮调试页可见行数。 */
#define ST7789_UI_MECANUM_ROW_COUNT  (6)

/* 麦轮调试页项目名称宽度。 */
#define ST7789_UI_MECANUM_NAME_W     (100)

/* 麦轮调试页数值文本缓冲区长度。 */
#define ST7789_UI_MECANUM_TEXT_LEN   (20U)

/* 麦轮调试页实测速度标签数量。 */
#define ST7789_UI_MECANUM_SPEED_COUNT (4U)

/* 麦轮调试页实测速度网格起始 Y 坐标。 */
#define ST7789_UI_MECANUM_SPEED_Y    (202)

/* 麦轮调试页实测速度标签宽度。 */
#define ST7789_UI_MECANUM_SPEED_W    (124)

/* 麦轮调试页实测速度标签高度。 */
#define ST7789_UI_MECANUM_SPEED_H    (16)

/* 开机页彩虹颜色表。 */
static const lv_color_t g_st7789_rainbow_colors[ST7789_UI_RAINBOW_COUNT] =
{
    LV_COLOR_MAKE(255, 64, 64),
    LV_COLOR_MAKE(255, 166, 48),
    LV_COLOR_MAKE(255, 232, 72),
    LV_COLOR_MAKE(80, 220, 92),
    LV_COLOR_MAKE(64, 172, 255),
    LV_COLOR_MAKE(182, 104, 255),
};

/* LVGL 当前活动屏幕对象。 */
static lv_obj_t *g_screen = NULL;

/* 进度条填充容器对象。 */
static lv_obj_t *g_boot_progress_fill = NULL;

/* 顶部电源状态标签对象。 */
static lv_obj_t *g_power_label = NULL;

/* 菜单项背景对象。 */
static lv_obj_t *g_menu_items[ST7789_UI_PAGE_COUNT] = {NULL};

/* 菜单项文字对象。 */
static lv_obj_t *g_menu_labels[ST7789_UI_PAGE_COUNT] = {NULL};

/* 功能页标题标签对象。 */
static lv_obj_t *g_page_title_label = NULL;

/* 功能页数值面板对象。 */
static lv_obj_t *g_page_panel = NULL;

/* 功能页数值标签对象。 */
static lv_obj_t *g_page_value_label = NULL;

/* 麦轮调试页保存状态标签对象。 */
static lv_obj_t *g_mecanum_save_label = NULL;

/* 麦轮调试页可见行背景对象。 */
static lv_obj_t *g_mecanum_rows[ST7789_UI_MECANUM_ROW_COUNT] = {NULL};

/* 麦轮调试页可见行名称标签对象。 */
static lv_obj_t *g_mecanum_name_labels[ST7789_UI_MECANUM_ROW_COUNT] = {NULL};

/* 麦轮调试页可见行数值标签对象。 */
static lv_obj_t *g_mecanum_value_labels[ST7789_UI_MECANUM_ROW_COUNT] = {NULL};

/* 麦轮调试页四路实测速度标签对象。 */
static lv_obj_t *g_mecanum_speed_labels[ST7789_UI_MECANUM_SPEED_COUNT] = {NULL};

/* 麦轮调试页可见行对应的项目编号。 */
static st7789_ui_mecanum_item_t g_mecanum_row_items[ST7789_UI_MECANUM_ROW_COUNT] =
{
    ST7789_UI_MECANUM_ITEM_MODE,
    ST7789_UI_MECANUM_ITEM_VX,
    ST7789_UI_MECANUM_ITEM_VY,
    ST7789_UI_MECANUM_ITEM_WZ,
    ST7789_UI_MECANUM_ITEM_W1,
    ST7789_UI_MECANUM_ITEM_W2,
};

/* 当前已绘制的开机进度。 */
static uint8_t g_current_progress = 0U;

/* 当前已绘制的视图。 */
static st7789_ui_view_t g_current_view = ST7789_UI_VIEW_BOOT;

/* 当前已绘制的菜单选中项。 */
static uint8_t g_current_selected_index = 0U;

/* 当前已绘制的功能页。 */
static st7789_ui_page_t g_current_page = ST7789_UI_PAGE_MECANUM_PID;

/* 当前已绘制的功能页占位数值。 */
static int32_t g_current_edit_value = 0;

/* 当前已绘制的功能页编辑状态。 */
static bool g_current_editing = false;

/* 当前已绘制的电源状态有效标志。 */
static bool g_current_power_valid = false;

/* 当前已绘制的母线电压，单位 mV。 */
static uint32_t g_current_bus_voltage_mv = 0U;

/* 当前已绘制的电流，单位 uA。 */
static int32_t g_current_current_ua = 0;

/* 开机页是否已经完成对象创建。 */
static bool g_ui_built = false;

/**
 * @brief  获取开机页背景顶部颜色
 *
 * @return lv_color_t LVGL 颜色
 */
static lv_color_t st7789_ui_bg_top(void)
{
    return lv_color_hex(0x070A18U);
}

/**
 * @brief  获取开机页背景底部颜色
 *
 * @return lv_color_t LVGL 颜色
 */
static lv_color_t st7789_ui_bg_bottom(void)
{
    return lv_color_hex(0x171038U);
}

/**
 * @brief  获取功能页英文缩写名称
 *
 * @param page 功能页编号
 * @return const char* 功能页名称字符串
 */
static const char *st7789_ui_page_name(st7789_ui_page_t page)
{
    switch (page)
    {
        case ST7789_UI_PAGE_MECANUM_PID:
            return "MECANUM PID";

        case ST7789_UI_PAGE_ARM_CALIB:
            return "ARM CALIB";

        case ST7789_UI_PAGE_RADAR:
            return "RADAR";

        case ST7789_UI_PAGE_AVOID_PID:
            return "AVOID PID";

        case ST7789_UI_PAGE_MONITOR:
            return "MONITOR";

        default:
            return "UNKNOWN";
    }
}

/**
 * @brief  获取麦轮调试页面项目名称
 *
 * @param item 项目编号
 * @return const char* 项目名称字符串
 */
static const char *st7789_ui_mecanum_item_name(st7789_ui_mecanum_item_t item)
{
    switch (item)
    {
        case ST7789_UI_MECANUM_ITEM_MODE:
            return "PAGE";

        case ST7789_UI_MECANUM_ITEM_VX:
            return "VX m/s";

        case ST7789_UI_MECANUM_ITEM_VY:
            return "VY m/s";

        case ST7789_UI_MECANUM_ITEM_WZ:
            return "WZ rad";

        case ST7789_UI_MECANUM_ITEM_W1:
            return "W1 LF";

        case ST7789_UI_MECANUM_ITEM_W2:
            return "W2 RF";

        case ST7789_UI_MECANUM_ITEM_W3:
            return "W3 LR";

        case ST7789_UI_MECANUM_ITEM_W4:
            return "W4 RR";

        case ST7789_UI_MECANUM_ITEM_KP:
            return "KP";

        case ST7789_UI_MECANUM_ITEM_KI:
            return "KI";

        case ST7789_UI_MECANUM_ITEM_STOP:
            return "STOP";

        default:
            return "UNKNOWN";
    }
}

/**
 * @brief  获取麦轮 PID 保存状态文本
 *
 * @param status 保存状态
 * @return const char* 状态字符串
 */
static const char *st7789_ui_mecanum_save_text(st7789_ui_mecanum_save_status_t status)
{
    switch (status)
    {
        case ST7789_UI_MECANUM_SAVE_SAVING:
            return "SAVING";

        case ST7789_UI_MECANUM_SAVE_SAVED:
            return "SAVED";

        case ST7789_UI_MECANUM_SAVE_ERR:
            return "ERR";

        case ST7789_UI_MECANUM_SAVE_NONE:
        default:
            return "";
    }
}

/**
 * @brief  将浮点数四舍五入为定点整数
 *
 * @param value 浮点数
 * @param scale 放大倍率
 * @return int32_t 定点整数
 */
static int32_t st7789_ui_float_to_scaled(float value, float scale)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * scale) + 0.5f);
    }

    return (int32_t)((value * scale) - 0.5f);
}

/**
 * @brief  格式化两位小数的有符号数值
 *
 * @param value    浮点数
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void st7789_ui_format_signed_2dp(float value, char *p_text, size_t text_len)
{
    int32_t scaled = st7789_ui_float_to_scaled(value, 100.0f);
    int32_t abs_scaled = scaled;
    char sign = '+';

    if ((NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (scaled < 0)
    {
        sign = '-';
        abs_scaled = -scaled;
    }

    (void)snprintf(p_text,
                   text_len,
                   "%c%ld.%02ld",
                   sign,
                   (long)(abs_scaled / 100),
                   (long)(abs_scaled % 100));
}

/**
 * @brief  格式化 PID 参数整数显示
 *
 * @param value    PID 参数
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void st7789_ui_format_pid_value(float value, char *p_text, size_t text_len)
{
    int32_t scaled = st7789_ui_float_to_scaled(value, 1.0f);

    if ((NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (scaled < 0)
    {
        scaled = 0;
    }

    (void)snprintf(p_text, text_len, "%ld", (long)scaled);
}

/**
 * @brief  获取麦轮当前子界面名称
 *
 * @param mode 子界面编号
 * @return const char* 子界面名称
 */
static const char *st7789_ui_mecanum_mode_name(st7789_ui_mecanum_mode_t mode)
{
    switch (mode)
    {
        case ST7789_UI_MECANUM_MODE_WHEEL:
            return "WHEEL";

        case ST7789_UI_MECANUM_MODE_PID:
            return "PID";

        case ST7789_UI_MECANUM_MODE_PWM:
            return "PWM";

        case ST7789_UI_MECANUM_MODE_BODY:
        default:
            return "BODY";
    }
}

/**
 * @brief  获取麦轮子界面可见项目数量
 *
 * @param mode 子界面编号
 * @return uint8_t 可见项目数量
 */
static uint8_t st7789_ui_mecanum_item_count(st7789_ui_mecanum_mode_t mode)
{
    if (ST7789_UI_MECANUM_MODE_WHEEL == mode)
    {
        return 6U;
    }

    if (ST7789_UI_MECANUM_MODE_PID == mode)
    {
        return 3U;
    }

    if (ST7789_UI_MECANUM_MODE_PWM == mode)
    {
        return 6U;
    }

    return 5U;
}

/**
 * @brief  获取麦轮子界面指定位置的项目
 *
 * @param mode  子界面编号
 * @param index 子界面内项目索引
 * @return st7789_ui_mecanum_item_t 项目编号
 */
static st7789_ui_mecanum_item_t st7789_ui_mecanum_item_at(st7789_ui_mecanum_mode_t mode, uint8_t index)
{
    const st7789_ui_mecanum_item_t body_items[] =
    {
        ST7789_UI_MECANUM_ITEM_MODE,
        ST7789_UI_MECANUM_ITEM_VX,
        ST7789_UI_MECANUM_ITEM_VY,
        ST7789_UI_MECANUM_ITEM_WZ,
        ST7789_UI_MECANUM_ITEM_STOP,
    };
    const st7789_ui_mecanum_item_t wheel_items[] =
    {
        ST7789_UI_MECANUM_ITEM_MODE,
        ST7789_UI_MECANUM_ITEM_W1,
        ST7789_UI_MECANUM_ITEM_W2,
        ST7789_UI_MECANUM_ITEM_W3,
        ST7789_UI_MECANUM_ITEM_W4,
        ST7789_UI_MECANUM_ITEM_STOP,
    };
    const st7789_ui_mecanum_item_t pid_items[] =
    {
        ST7789_UI_MECANUM_ITEM_MODE,
        ST7789_UI_MECANUM_ITEM_KP,
        ST7789_UI_MECANUM_ITEM_KI,
    };
    const st7789_ui_mecanum_item_t pwm_items[] =
    {
        ST7789_UI_MECANUM_ITEM_MODE,
        ST7789_UI_MECANUM_ITEM_W1,
        ST7789_UI_MECANUM_ITEM_W2,
        ST7789_UI_MECANUM_ITEM_W3,
        ST7789_UI_MECANUM_ITEM_W4,
        ST7789_UI_MECANUM_ITEM_STOP,
    };

    if (ST7789_UI_MECANUM_MODE_WHEEL == mode)
    {
        return wheel_items[index];
    }

    if (ST7789_UI_MECANUM_MODE_PID == mode)
    {
        return pid_items[index];
    }

    if (ST7789_UI_MECANUM_MODE_PWM == mode)
    {
        return pwm_items[index];
    }

    return body_items[index];
}

/**
 * @brief  获取麦轮当前项目显示值
 *
 * @param p_state  UI 状态
 * @param item     项目编号
 * @return float   项目数值
 */
static float st7789_ui_mecanum_item_value(const st7789_ui_state_t *p_state, st7789_ui_mecanum_item_t item)
{
    if (NULL == p_state)
    {
        return 0.0f;
    }

    if ((p_state->mecanum_editing) &&
        ((uint8_t)item == p_state->mecanum_selected_item))
    {
        return p_state->mecanum_edit_value;
    }

    switch (item)
    {
        case ST7789_UI_MECANUM_ITEM_VX:
            return p_state->mecanum_vx_mps;

        case ST7789_UI_MECANUM_ITEM_VY:
            return p_state->mecanum_vy_mps;

        case ST7789_UI_MECANUM_ITEM_WZ:
            return p_state->mecanum_wz_radps;

        case ST7789_UI_MECANUM_ITEM_W1:
            if (ST7789_UI_MECANUM_MODE_PWM == p_state->mecanum_mode)
            {
                return (float)p_state->mecanum_pwm_counts[0];
            }
            return p_state->mecanum_wheel_speed_mps[0];

        case ST7789_UI_MECANUM_ITEM_W2:
            if (ST7789_UI_MECANUM_MODE_PWM == p_state->mecanum_mode)
            {
                return (float)p_state->mecanum_pwm_counts[1];
            }
            return p_state->mecanum_wheel_speed_mps[1];

        case ST7789_UI_MECANUM_ITEM_W3:
            if (ST7789_UI_MECANUM_MODE_PWM == p_state->mecanum_mode)
            {
                return (float)p_state->mecanum_pwm_counts[2];
            }
            return p_state->mecanum_wheel_speed_mps[2];

        case ST7789_UI_MECANUM_ITEM_W4:
            if (ST7789_UI_MECANUM_MODE_PWM == p_state->mecanum_mode)
            {
                return (float)p_state->mecanum_pwm_counts[3];
            }
            return p_state->mecanum_wheel_speed_mps[3];

        case ST7789_UI_MECANUM_ITEM_KP:
            return p_state->mecanum_kp;

        case ST7789_UI_MECANUM_ITEM_KI:
            return p_state->mecanum_ki;

        case ST7789_UI_MECANUM_ITEM_MODE:
        case ST7789_UI_MECANUM_ITEM_STOP:
        default:
            return 0.0f;
    }
}

/**
 * @brief  格式化麦轮项目显示值
 *
 * @param p_state  UI 状态
 * @param item     项目编号
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void st7789_ui_format_mecanum_value(const st7789_ui_state_t *p_state,
                                           st7789_ui_mecanum_item_t item,
                                           char *p_text,
                                           size_t text_len)
{
    float value = 0.0f;

    if ((NULL == p_state) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (ST7789_UI_MECANUM_ITEM_MODE == item)
    {
        (void)snprintf(p_text, text_len, "%s", st7789_ui_mecanum_mode_name(p_state->mecanum_mode));
        return;
    }

    if (ST7789_UI_MECANUM_ITEM_STOP == item)
    {
        (void)snprintf(p_text, text_len, "PRESS");
        return;
    }

    /* PWM 模式下四路 W1~W4 显示为原始有符号整数值。 */
    if ((ST7789_UI_MECANUM_ITEM_W1 <= item) && (item <= ST7789_UI_MECANUM_ITEM_W4) &&
        (ST7789_UI_MECANUM_MODE_PWM == p_state->mecanum_mode))
    {
        int32_t pwm_val = (int32_t)st7789_ui_mecanum_item_value(p_state, item);
        (void)snprintf(p_text, text_len, "%ld", (long)pwm_val);
        return;
    }

    value = st7789_ui_mecanum_item_value(p_state, item);
    if ((ST7789_UI_MECANUM_ITEM_KP == item) || (ST7789_UI_MECANUM_ITEM_KI == item))
    {
        st7789_ui_format_pid_value(value, p_text, text_len);
    }
    else
    {
        st7789_ui_format_signed_2dp(value, p_text, text_len);
    }
}

/**
 * @brief  检查麦轮项目是否属于当前子界面
 *
 * @param mode 子界面编号
 * @param item 项目编号
 * @return bool true 表示属于当前子界面
 */
static bool st7789_ui_mecanum_item_visible(st7789_ui_mecanum_mode_t mode, st7789_ui_mecanum_item_t item)
{
    uint8_t count = st7789_ui_mecanum_item_count(mode);
    uint8_t i = 0U;

    for (i = 0U; i < count; i++)
    {
        if (item == st7789_ui_mecanum_item_at(mode, i))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief  创建固定区域英文标签
 *
 * @param parent 父对象
 * @param x      左上角 X 坐标
 * @param y      左上角 Y 坐标
 * @param w      宽度
 * @param h      高度
 * @param text   初始文本
 * @param color  文本颜色
 * @return lv_obj_t* 新建标签对象指针
 */
static lv_obj_t *st7789_ui_label(lv_obj_t *parent,
                                 int32_t x,
                                 int32_t y,
                                 int32_t w,
                                 int32_t h,
                                 const char *text,
                                 lv_color_t color)
{
    lv_obj_t *label = lv_label_create(parent);

    lv_obj_set_pos(label, x, y);
    lv_obj_set_size(label, w, h);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(label, text);

    return label;
}

/**
 * @brief  创建一个无滚动矩形对象
 *
 * @param parent 父对象
 * @param x      左上角 X 坐标
 * @param y      左上角 Y 坐标
 * @param w      宽度
 * @param h      高度
 * @param color  填充颜色
 * @param radius 圆角半径
 * @return lv_obj_t* 新建对象指针
 */
static lv_obj_t *st7789_ui_rect(lv_obj_t *parent,
                                int32_t x,
                                int32_t y,
                                int32_t w,
                                int32_t h,
                                lv_color_t color,
                                int32_t radius)
{
    lv_obj_t *obj = lv_obj_create(parent);

    lv_obj_remove_style_all(obj);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_bg_color(obj, color, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj, radius, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    return obj;
}

/**
 * @brief  应用开机页深色渐变背景
 *
 * @return void
 */
static void st7789_ui_apply_background(void)
{
    lv_obj_remove_style_all(g_screen);
    lv_obj_set_size(g_screen, ST7789_UI_WIDTH, ST7789_UI_HEIGHT);
    lv_obj_set_style_bg_color(g_screen, st7789_ui_bg_top(), 0);
    lv_obj_set_style_bg_grad_color(g_screen, st7789_ui_bg_bottom(), 0);
    lv_obj_set_style_bg_grad_dir(g_screen, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(g_screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(g_screen, LV_OBJ_FLAG_SCROLLABLE);
}

/**
 * @brief  清空屏幕并重新应用背景
 *
 * @return void
 */
static void st7789_ui_clear_screen(void)
{
    int32_t i = 0;

    g_boot_progress_fill = NULL;
    g_power_label = NULL;
    g_page_title_label = NULL;
    g_page_panel = NULL;
    g_page_value_label = NULL;
    g_mecanum_save_label = NULL;
    for (i = 0; i < (int32_t)ST7789_UI_PAGE_COUNT; i++)
    {
        g_menu_items[i] = NULL;
        g_menu_labels[i] = NULL;
    }
    for (i = 0; i < ST7789_UI_MECANUM_ROW_COUNT; i++)
    {
        g_mecanum_rows[i] = NULL;
        g_mecanum_name_labels[i] = NULL;
        g_mecanum_value_labels[i] = NULL;
        g_mecanum_row_items[i] = ST7789_UI_MECANUM_ITEM_MODE;
    }
    for (i = 0; i < (int32_t)ST7789_UI_MECANUM_SPEED_COUNT; i++)
    {
        g_mecanum_speed_labels[i] = NULL;
    }

    lv_obj_clean(g_screen);
    st7789_ui_apply_background();
}

/**
 * @brief  刷新开机进度条填充宽度
 *
 * @param progress 进度百分比，范围 0~100
 * @return void
 */
static void st7789_ui_update_boot_progress(uint8_t progress)
{
    int32_t inner_w = ST7789_UI_PROGRESS_W - (ST7789_UI_PROGRESS_INSET * 2);
    int32_t fill_w = 0;

    if (progress > 100U)
    {
        progress = 100U;
    }

    fill_w = (inner_w * (int32_t)progress) / 100;
    if (NULL != g_boot_progress_fill)
    {
        lv_obj_set_width(g_boot_progress_fill, fill_w);
    }
}

/**
 * @brief  格式化电压电流状态文本
 *
 * @param p_state  UI 状态
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void st7789_ui_format_power_text(const st7789_ui_state_t *p_state, char *p_text, size_t text_len)
{
    uint32_t voltage_whole = 0U;
    uint32_t voltage_frac = 0U;
    uint32_t current_abs_ua = 0U;
    uint32_t current_whole = 0U;
    uint32_t current_frac = 0U;
    char current_sign = '+';

    if ((NULL == p_state) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (!p_state->power_valid)
    {
        (void)snprintf(p_text, text_len, "--.--V --.--A");
        return;
    }

    voltage_whole = p_state->bus_voltage_mv / 1000U;
    voltage_frac = (p_state->bus_voltage_mv % 1000U) / 10U;

    if (p_state->current_ua < 0)
    {
        current_sign = '-';
        current_abs_ua = (uint32_t)(-p_state->current_ua);
    }
    else
    {
        current_abs_ua = (uint32_t)p_state->current_ua;
    }

    current_whole = current_abs_ua / 1000000U;
    current_frac = (current_abs_ua % 1000000U) / 10000U;

    (void)snprintf(p_text,
                   text_len,
                   "%lu.%02luV %c%lu.%02luA",
                   (unsigned long)voltage_whole,
                   (unsigned long)voltage_frac,
                   current_sign,
                   (unsigned long)current_whole,
                   (unsigned long)current_frac);
}

/**
 * @brief  判断电源状态是否与当前绘制内容不同
 *
 * @param p_state UI 状态
 * @return bool true 表示需要刷新电源状态标签
 */
static bool st7789_ui_power_changed(const st7789_ui_state_t *p_state)
{
    if (g_current_power_valid != p_state->power_valid)
    {
        return true;
    }

    if (g_current_bus_voltage_mv != p_state->bus_voltage_mv)
    {
        return true;
    }

    if (g_current_current_ua != p_state->current_ua)
    {
        return true;
    }

    return false;
}

/**
 * @brief  刷新顶部电源状态标签
 *
 * @param p_state UI 状态
 * @return void
 */
static void st7789_ui_update_power_label(const st7789_ui_state_t *p_state)
{
    char power_text[ST7789_UI_POWER_TEXT_LEN] = {0};

    if ((NULL == p_state) || (NULL == g_power_label))
    {
        return;
    }

    st7789_ui_format_power_text(p_state, power_text, sizeof(power_text));
    lv_label_set_text(g_power_label, power_text);

    g_current_power_valid = p_state->power_valid;
    g_current_bus_voltage_mv = p_state->bus_voltage_mv;
    g_current_current_ua = p_state->current_ua;
}

/**
 * @brief  绘制顶部状态栏
 *
 * @param p_state UI 状态
 * @return void
 */
static void st7789_ui_create_status_bar(const st7789_ui_state_t *p_state)
{
    lv_obj_t *bar = st7789_ui_rect(g_screen,
                                   0,
                                   0,
                                   ST7789_UI_WIDTH,
                                   ST7789_UI_STATUS_H,
                                   lv_color_hex(0x0D1226U),
                                   0);

    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_side(bar, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(bar, lv_color_hex(0x263250U), 0);

    g_power_label = st7789_ui_label(g_screen,
                                    ST7789_UI_STATUS_SAFE_X,
                                    ST7789_UI_STATUS_TEXT_Y,
                                    ST7789_UI_WIDTH - (ST7789_UI_STATUS_SAFE_X * 2),
                                    ST7789_UI_STATUS_TEXT_H,
                                    "",
                                    lv_color_hex(0xF8FAFCU));
    lv_obj_set_style_text_align(g_power_label, LV_TEXT_ALIGN_CENTER, 0);
    st7789_ui_update_power_label(p_state);
}

/**
 * @brief  刷新菜单选中项样式
 *
 * @param selected_index 当前选中项索引
 * @return void
 */
static void st7789_ui_update_menu_selection(uint8_t selected_index)
{
    int32_t i = 0;
    bool selected = false;

    if (selected_index >= (uint8_t)ST7789_UI_PAGE_COUNT)
    {
        selected_index = 0U;
    }

    for (i = 0; i < (int32_t)ST7789_UI_PAGE_COUNT; i++)
    {
        selected = ((uint8_t)i == selected_index);

        if (NULL != g_menu_items[i])
        {
            lv_obj_set_style_bg_color(g_menu_items[i],
                                      selected ? lv_color_hex(0xFDE68AU) : lv_color_hex(0x151C33U),
                                      0);
            lv_obj_set_style_border_color(g_menu_items[i],
                                          selected ? lv_color_hex(0xF59E0BU) : lv_color_hex(0x2B3656U),
                                          0);
        }

        if (NULL != g_menu_labels[i])
        {
            lv_obj_set_style_text_color(g_menu_labels[i],
                                        selected ? lv_color_hex(0x111827U) : lv_color_hex(0xE5E7EBU),
                                        0);
        }
    }

    g_current_selected_index = selected_index;
}

/**
 * @brief  刷新功能页数值标签
 *
 * @param value 当前占位调试数值
 * @return void
 */
static void st7789_ui_update_page_value(int32_t value)
{
    char value_text[ST7789_UI_VALUE_TEXT_LEN] = {0};

    if (NULL == g_page_value_label)
    {
        return;
    }

    (void)snprintf(value_text, sizeof(value_text), "VALUE: %ld", (long)value);
    lv_label_set_text(g_page_value_label, value_text);
    g_current_edit_value = value;
}

/**
 * @brief  刷新功能页编辑状态样式
 *
 * @param editing true 表示当前参数正在编辑
 * @return void
 */
static void st7789_ui_update_page_editing(bool editing)
{
    if (NULL != g_page_panel)
    {
        lv_obj_set_style_border_width(g_page_panel, editing ? ST7789_UI_PAGE_EDIT_BORDER_W : 1, 0);
        lv_obj_set_style_border_color(g_page_panel,
                                      editing ? lv_color_hex(0x22C55EU) : lv_color_hex(0x3B82F6U),
                                      0);
    }

    if (NULL != g_page_value_label)
    {
        lv_obj_set_style_text_color(g_page_value_label,
                                    editing ? lv_color_hex(0xBBF7D0U) : lv_color_hex(0xFDE68AU),
                                    0);
    }

    g_current_editing = editing;
}

/**
 * @brief  刷新麦轮 PID 保存状态标签
 *
 * @param p_state UI 状态
 * @return void
 */
static void st7789_ui_update_mecanum_save_label(const st7789_ui_state_t *p_state)
{
    lv_color_t color = lv_color_hex(0x94A3B8U);

    if ((NULL == p_state) || (NULL == g_mecanum_save_label))
    {
        return;
    }

    if (ST7789_UI_MECANUM_SAVE_SAVED == p_state->mecanum_save_status)
    {
        color = lv_color_hex(0x86EFACU);
    }
    else if (ST7789_UI_MECANUM_SAVE_ERR == p_state->mecanum_save_status)
    {
        color = lv_color_hex(0xFCA5A5U);
    }
    else if (ST7789_UI_MECANUM_SAVE_SAVING == p_state->mecanum_save_status)
    {
        color = lv_color_hex(0xFDE68AU);
    }

    lv_label_set_text(g_mecanum_save_label, st7789_ui_mecanum_save_text(p_state->mecanum_save_status));
    lv_obj_set_style_text_color(g_mecanum_save_label, color, 0);
}

/**
 * @brief  刷新麦轮调试页行内容和样式
 *
 * @param p_state UI 状态
 * @return void
 */
static void st7789_ui_update_mecanum_rows(const st7789_ui_state_t *p_state)
{
    uint8_t item_count = 0U;
    uint8_t selected_item = 0U;
    uint8_t row = 0U;
    st7789_ui_mecanum_item_t item = ST7789_UI_MECANUM_ITEM_MODE;
    bool selected = false;
    bool editing = false;
    char value_text[ST7789_UI_MECANUM_TEXT_LEN] = {0};

    if (NULL == p_state)
    {
        return;
    }

    selected_item = p_state->mecanum_selected_item;
    if (!st7789_ui_mecanum_item_visible(p_state->mecanum_mode, (st7789_ui_mecanum_item_t)selected_item))
    {
        selected_item = (uint8_t)ST7789_UI_MECANUM_ITEM_MODE;
    }

    item_count = st7789_ui_mecanum_item_count(p_state->mecanum_mode);

    for (row = 0U; row < ST7789_UI_MECANUM_ROW_COUNT; row++)
    {
        if (row >= item_count)
        {
            if (NULL != g_mecanum_rows[row])
            {
                lv_obj_add_flag(g_mecanum_rows[row], LV_OBJ_FLAG_HIDDEN);
            }
            continue;
        }

        item = st7789_ui_mecanum_item_at(p_state->mecanum_mode, row);
        selected = ((uint8_t)item == selected_item);
        editing = selected && p_state->mecanum_editing;
        g_mecanum_row_items[row] = item;

        if (NULL != g_mecanum_rows[row])
        {
            lv_obj_clear_flag(g_mecanum_rows[row], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(g_mecanum_rows[row],
                                      selected ? lv_color_hex(0xFDE68AU) : lv_color_hex(0x151C33U),
                                      0);
            lv_obj_set_style_border_width(g_mecanum_rows[row], editing ? 2 : 1, 0);
            lv_obj_set_style_border_color(g_mecanum_rows[row],
                                          editing ? lv_color_hex(0x22C55EU) :
                                          (selected ? lv_color_hex(0xF59E0BU) : lv_color_hex(0x2B3656U)),
                                          0);
        }

        if (NULL != g_mecanum_name_labels[row])
        {
            lv_label_set_text(g_mecanum_name_labels[row], st7789_ui_mecanum_item_name(item));
            lv_obj_set_style_text_color(g_mecanum_name_labels[row],
                                        selected ? lv_color_hex(0x111827U) : lv_color_hex(0xE5E7EBU),
                                        0);
        }

        if (NULL != g_mecanum_value_labels[row])
        {
            st7789_ui_format_mecanum_value(p_state, item, value_text, sizeof(value_text));
            lv_label_set_text(g_mecanum_value_labels[row], value_text);
            lv_obj_set_style_text_color(g_mecanum_value_labels[row],
                                        editing ? lv_color_hex(0x166534U) :
                                        (selected ? lv_color_hex(0x111827U) : lv_color_hex(0xBAE6FDU)),
                                        0);
        }
    }
}

/**
 * @brief  刷新麦轮调试页四路实测速度
 *
 * @param p_state UI 状态
 * @return void
 */
static void st7789_ui_update_mecanum_speed_labels(const st7789_ui_state_t *p_state)
{
    const char *wheel_names[ST7789_UI_MECANUM_SPEED_COUNT] =
    {
        "W1",
        "W2",
        "W3",
        "W4",
    };
    char value_text[ST7789_UI_MECANUM_TEXT_LEN] = {0};
    char speed_text[ST7789_UI_MECANUM_TEXT_LEN] = {0};
    uint8_t i = 0U;
    bool show_speed = false;

    if (NULL == p_state)
    {
        return;
    }

    show_speed = ((ST7789_UI_MECANUM_MODE_BODY == p_state->mecanum_mode) ||
                  (ST7789_UI_MECANUM_MODE_WHEEL == p_state->mecanum_mode) ||
                  (ST7789_UI_MECANUM_MODE_PWM == p_state->mecanum_mode));

    for (i = 0U; i < ST7789_UI_MECANUM_SPEED_COUNT; i++)
    {
        if (NULL == g_mecanum_speed_labels[i])
        {
            continue;
        }

        if (!show_speed)
        {
            lv_obj_add_flag(g_mecanum_speed_labels[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        lv_obj_clear_flag(g_mecanum_speed_labels[i], LV_OBJ_FLAG_HIDDEN);
        st7789_ui_format_signed_2dp(p_state->mecanum_measured_speed_mps[i],
                                    value_text,
                                    sizeof(value_text));
        (void)snprintf(speed_text, sizeof(speed_text), "%s %s", wheel_names[i], value_text);
        lv_label_set_text(g_mecanum_speed_labels[i], speed_text);
    }
}

/**
 * @brief  刷新麦轮调试页
 *
 * @param p_state UI 状态
 * @return void
 */
static void st7789_ui_update_mecanum_page(const st7789_ui_state_t *p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    st7789_ui_update_power_label(p_state);
    st7789_ui_update_mecanum_save_label(p_state);
    st7789_ui_update_mecanum_rows(p_state);
    st7789_ui_update_mecanum_speed_labels(p_state);
}

/**
 * @brief  绘制 ST7789 小屏独立布局的像素风彩虹图案
 *
 * SPI 屏不从 MIPI 坐标非等比缩放，避免像素图被压扁。
 *
 * @return void
 */
static void st7789_ui_create_pixel_cat(void)
{
    lv_obj_t *cat = lv_obj_create(g_screen);
    int32_t i = 0;

    lv_obj_remove_style_all(cat);
    lv_obj_set_pos(cat, 22, 20);
    lv_obj_set_size(cat, 236, 160);
    lv_obj_clear_flag(cat, LV_OBJ_FLAG_SCROLLABLE);

    for (i = 0; i < (int32_t)ST7789_UI_RAINBOW_COUNT; i++)
    {
        st7789_ui_rect(cat, 0, 70 + (i * 10), 112, 8, g_st7789_rainbow_colors[i], 2);
        st7789_ui_rect(cat, 10, 75 + (i * 10), 100, 4, g_st7789_rainbow_colors[i], 1);
    }

    st7789_ui_rect(cat, 104, 44, 78, 56, lv_color_hex(0xF8C9E1U), 5);
    st7789_ui_rect(cat, 111, 51, 64, 42, lv_color_hex(0xFFEAF4U), 4);
    st7789_ui_rect(cat, 120, 60, 10, 10, lv_color_hex(0xFF78B4U), 1);
    st7789_ui_rect(cat, 148, 60, 10, 10, lv_color_hex(0xFF78B4U), 1);

    st7789_ui_rect(cat, 176, 42, 52, 52, lv_color_hex(0xF8C9E1U), 5);
    st7789_ui_rect(cat, 181, 28, 14, 18, lv_color_hex(0xF8C9E1U), 1);
    st7789_ui_rect(cat, 208, 28, 14, 18, lv_color_hex(0xF8C9E1U), 1);
    st7789_ui_rect(cat, 185, 32, 7, 11, lv_color_hex(0xE968A5U), 1);
    st7789_ui_rect(cat, 211, 32, 7, 11, lv_color_hex(0xE968A5U), 1);
    st7789_ui_rect(cat, 190, 58, 6, 6, lv_color_hex(0x111827U), 1);
    st7789_ui_rect(cat, 208, 58, 6, 6, lv_color_hex(0x111827U), 1);
    st7789_ui_rect(cat, 200, 69, 7, 4, lv_color_hex(0xC94A8DU), 1);
    st7789_ui_rect(cat, 188, 80, 10, 3, lv_color_hex(0xC9D8FFU), 1);
    st7789_ui_rect(cat, 207, 80, 10, 3, lv_color_hex(0xC9D8FFU), 1);

    st7789_ui_rect(cat, 116, 100, 13, 24, lv_color_hex(0xF8C9E1U), 2);
    st7789_ui_rect(cat, 154, 100, 13, 24, lv_color_hex(0xF8C9E1U), 2);
    st7789_ui_rect(cat, 91, 58, 18, 14, lv_color_hex(0xF8C9E1U), 2);
    st7789_ui_rect(cat, 80, 52, 13, 12, lv_color_hex(0xF8C9E1U), 2);
    st7789_ui_rect(cat, 91, 46, 12, 10, lv_color_hex(0xF8C9E1U), 1);
    st7789_ui_rect(cat, 113, 45, 34, 5, lv_color_hex(0xFFFFFFU), 2);
    st7789_ui_rect(cat, 112, 90, 58, 6, lv_color_hex(0xFFFFFFU), 2);
}

/**
 * @brief  绘制彩虹开机进度条
 *
 * @param progress 初始进度百分比
 * @return void
 */
static void st7789_ui_create_progress_bar(uint8_t progress)
{
    int32_t inner_w = ST7789_UI_PROGRESS_W - (ST7789_UI_PROGRESS_INSET * 2);
    int32_t inner_h = ST7789_UI_PROGRESS_H - (ST7789_UI_PROGRESS_INSET * 2);
    int32_t seg_w = (inner_w / (int32_t)ST7789_UI_RAINBOW_COUNT) + 2;
    int32_t i = 0;
    lv_obj_t *bar = st7789_ui_rect(g_screen,
                                   ST7789_UI_PROGRESS_X,
                                   ST7789_UI_PROGRESS_Y,
                                   ST7789_UI_PROGRESS_W,
                                   ST7789_UI_PROGRESS_H,
                                   lv_color_hex(0x121A33U),
                                   9);

    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_color(bar, lv_color_hex(0x6FA8FFU), 0);
    lv_obj_set_style_border_opa(bar, LV_OPA_80, 0);

    g_boot_progress_fill = st7789_ui_rect(bar,
                                          ST7789_UI_PROGRESS_INSET,
                                          ST7789_UI_PROGRESS_INSET,
                                          0,
                                          inner_h,
                                          g_st7789_rainbow_colors[0],
                                          6);

    for (i = 0; i < (int32_t)ST7789_UI_RAINBOW_COUNT; i++)
    {
        st7789_ui_rect(g_boot_progress_fill,
                       i * (inner_w / (int32_t)ST7789_UI_RAINBOW_COUNT),
                       0,
                       seg_w,
                       inner_h,
                       g_st7789_rainbow_colors[i],
                       0);
    }

    st7789_ui_update_boot_progress(progress);
}

/**
 * @brief  构建完整 ST7789 开机页
 *
 * @param p_state 当前 UI 状态
 * @return void
 */
static void st7789_ui_build_boot(const st7789_ui_state_t *p_state)
{
    st7789_ui_clear_screen();
    st7789_ui_create_pixel_cat();
    st7789_ui_create_progress_bar(p_state->boot_progress);

    g_current_progress = p_state->boot_progress;
    g_current_view = ST7789_UI_VIEW_BOOT;
    g_ui_built = true;
}

/**
 * @brief  构建 ST7789 功能选择菜单页
 *
 * @param p_state 当前 UI 状态
 * @return void
 */
static void st7789_ui_build_menu(const st7789_ui_state_t *p_state)
{
    int32_t i = 0;
    int32_t item_y = 0;

    st7789_ui_clear_screen();
    st7789_ui_create_status_bar(p_state);

    for (i = 0; i < (int32_t)ST7789_UI_PAGE_COUNT; i++)
    {
        item_y = ST7789_UI_MENU_Y + (i * (ST7789_UI_MENU_H + ST7789_UI_MENU_GAP));
        g_menu_items[i] = st7789_ui_rect(g_screen,
                                         ST7789_UI_MENU_X,
                                         item_y,
                                         ST7789_UI_MENU_W,
                                         ST7789_UI_MENU_H,
                                         lv_color_hex(0x151C33U),
                                         6);
        lv_obj_set_style_border_width(g_menu_items[i], 1, 0);
        lv_obj_set_style_border_color(g_menu_items[i], lv_color_hex(0x2B3656U), 0);

        g_menu_labels[i] = st7789_ui_label(g_menu_items[i],
                                           12,
                                           7,
                                           ST7789_UI_MENU_W - 24,
                                           18,
                                           st7789_ui_page_name((st7789_ui_page_t)i),
                                           lv_color_hex(0xE5E7EBU));
    }

    st7789_ui_update_menu_selection(p_state->selected_index);

    g_current_view = ST7789_UI_VIEW_MENU;
    g_ui_built = true;
}

/**
 * @brief  构建 ST7789 麦轮速度 PID 调试页
 *
 * @param p_state 当前 UI 状态
 * @return void
 */
static void st7789_ui_build_mecanum_page(const st7789_ui_state_t *p_state)
{
    uint8_t row = 0U;
    uint8_t speed_index = 0U;
    int32_t row_y = 0;
    int32_t speed_x = 0;
    int32_t speed_y = 0;

    st7789_ui_clear_screen();
    st7789_ui_create_status_bar(p_state);

    g_page_title_label = st7789_ui_label(g_screen,
                                         12,
                                         ST7789_UI_MECANUM_TITLE_Y,
                                         150,
                                         18,
                                         "MECANUM PID",
                                         lv_color_hex(0xF8FAFCU));

    g_mecanum_save_label = st7789_ui_label(g_screen,
                                           168,
                                           ST7789_UI_MECANUM_TITLE_Y,
                                           96,
                                           18,
                                           "",
                                           lv_color_hex(0x94A3B8U));
    lv_obj_set_style_text_align(g_mecanum_save_label, LV_TEXT_ALIGN_RIGHT, 0);

    for (row = 0U; row < ST7789_UI_MECANUM_ROW_COUNT; row++)
    {
        row_y = ST7789_UI_MECANUM_LIST_Y +
                ((int32_t)row * (ST7789_UI_MECANUM_ROW_H + ST7789_UI_MECANUM_ROW_GAP));

        g_mecanum_rows[row] = st7789_ui_rect(g_screen,
                                             ST7789_UI_MECANUM_LIST_X,
                                             row_y,
                                             ST7789_UI_MECANUM_LIST_W,
                                             ST7789_UI_MECANUM_ROW_H,
                                             lv_color_hex(0x151C33U),
                                             5);
        lv_obj_set_style_border_width(g_mecanum_rows[row], 1, 0);
        lv_obj_set_style_border_color(g_mecanum_rows[row], lv_color_hex(0x2B3656U), 0);

        g_mecanum_name_labels[row] = st7789_ui_label(g_mecanum_rows[row],
                                                     9,
                                                     3,
                                                     ST7789_UI_MECANUM_NAME_W,
                                                     16,
                                                     "",
                                                     lv_color_hex(0xE5E7EBU));

        g_mecanum_value_labels[row] = st7789_ui_label(g_mecanum_rows[row],
                                                      ST7789_UI_MECANUM_NAME_W,
                                                      3,
                                                      ST7789_UI_MECANUM_LIST_W -
                                                      ST7789_UI_MECANUM_NAME_W - 18,
                                                      16,
                                                      "",
                                                      lv_color_hex(0xBAE6FDU));
        lv_obj_set_style_text_align(g_mecanum_value_labels[row], LV_TEXT_ALIGN_RIGHT, 0);
    }

    for (speed_index = 0U; speed_index < ST7789_UI_MECANUM_SPEED_COUNT; speed_index++)
    {
        speed_x = ST7789_UI_STATUS_SAFE_BOTTOM_X +
                  ((int32_t)(speed_index % 2U) * (ST7789_UI_MECANUM_SPEED_W + 12));
        speed_y = ST7789_UI_MECANUM_SPEED_Y +
                  ((int32_t)(speed_index / 2U) * (ST7789_UI_MECANUM_SPEED_H + 2));
        g_mecanum_speed_labels[speed_index] = st7789_ui_label(g_screen,
                                                              speed_x,
                                                              speed_y,
                                                              ST7789_UI_MECANUM_SPEED_W,
                                                              ST7789_UI_MECANUM_SPEED_H,
                                                              "",
                                                              lv_color_hex(0x93C5FDU));
    }

    st7789_ui_update_mecanum_page(p_state);

    g_current_page = ST7789_UI_PAGE_MECANUM_PID;
    g_current_view = ST7789_UI_VIEW_PAGE;
    g_ui_built = true;
}

/**
 * @brief  构建 ST7789 功能占位调试页
 *
 * @param p_state 当前 UI 状态
 * @return void
 */
static void st7789_ui_build_page(const st7789_ui_state_t *p_state)
{
    if (ST7789_UI_PAGE_MECANUM_PID == p_state->current_page)
    {
        st7789_ui_build_mecanum_page(p_state);
        return;
    }

    st7789_ui_clear_screen();
    st7789_ui_create_status_bar(p_state);

    g_page_title_label = st7789_ui_label(g_screen,
                                         18,
                                         54,
                                         ST7789_UI_WIDTH - 36,
                                         24,
                                         st7789_ui_page_name(p_state->current_page),
                                         lv_color_hex(0xF8FAFCU));
    lv_obj_set_style_text_align(g_page_title_label, LV_TEXT_ALIGN_CENTER, 0);

    g_page_panel = st7789_ui_rect(g_screen,
                                  ST7789_UI_PAGE_PANEL_X,
                                  ST7789_UI_PAGE_PANEL_Y,
                                  ST7789_UI_PAGE_PANEL_W,
                                  ST7789_UI_PAGE_PANEL_H,
                                  lv_color_hex(0x151C33U),
                                  8);
    lv_obj_set_style_border_width(g_page_panel, 1, 0);
    lv_obj_set_style_border_color(g_page_panel, lv_color_hex(0x3B82F6U), 0);

    g_page_value_label = st7789_ui_label(g_page_panel,
                                         0,
                                         23,
                                         ST7789_UI_PAGE_PANEL_W,
                                         24,
                                         "",
                                         lv_color_hex(0xFDE68AU));
    lv_obj_set_style_text_align(g_page_value_label, LV_TEXT_ALIGN_CENTER, 0);
    st7789_ui_update_page_value(p_state->edit_value);
    st7789_ui_update_page_editing(p_state->editing);

    g_current_page = p_state->current_page;
    g_current_view = ST7789_UI_VIEW_PAGE;
    g_ui_built = true;
}

/**
 * @brief  根据状态构建当前视图
 *
 * @param p_state 当前 UI 状态
 * @return void
 */
static void st7789_ui_build_view(const st7789_ui_state_t *p_state)
{
    switch (p_state->view)
    {
        case ST7789_UI_VIEW_MENU:
            st7789_ui_build_menu(p_state);
            break;

        case ST7789_UI_VIEW_PAGE:
            st7789_ui_build_page(p_state);
            break;

        case ST7789_UI_VIEW_BOOT:
        default:
            st7789_ui_build_boot(p_state);
            break;
    }
}

/**
 * @brief  初始化 ST7789 开机页 UI 对象
 *
 * @param p_state 初始 UI 状态
 * @return fsp_err_t FSP_SUCCESS 表示初始化成功
 */
fsp_err_t st7789_ui_init(const st7789_ui_state_t *p_state)
{
    if (NULL == p_state)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    g_screen = lv_display_get_screen_active(NULL);
    if (NULL == g_screen)
    {
        return FSP_ERR_INTERNAL;
    }

    st7789_ui_build_view(p_state);

    return FSP_SUCCESS;
}

/**
 * @brief  按当前状态刷新 ST7789 开机页
 *
 * @param p_state 当前 UI 状态
 * @return void
 */
void st7789_ui_update(const st7789_ui_state_t *p_state)
{
    if ((NULL == p_state) || (NULL == g_screen))
    {
        return;
    }

    if ((!g_ui_built) || (g_current_view != p_state->view))
    {
        st7789_ui_build_view(p_state);
        return;
    }

    if (ST7789_UI_VIEW_BOOT == p_state->view)
    {
        if (g_current_progress != p_state->boot_progress)
        {
            g_current_progress = p_state->boot_progress;
            st7789_ui_update_boot_progress(p_state->boot_progress);
        }
        return;
    }

    if (st7789_ui_power_changed(p_state))
    {
        st7789_ui_update_power_label(p_state);
    }

    if (ST7789_UI_VIEW_MENU == p_state->view)
    {
        if (g_current_selected_index != p_state->selected_index)
        {
            st7789_ui_update_menu_selection(p_state->selected_index);
        }
        return;
    }

    if (ST7789_UI_VIEW_PAGE == p_state->view)
    {
        if (g_current_page != p_state->current_page)
        {
            st7789_ui_build_page(p_state);
            return;
        }

        if (ST7789_UI_PAGE_MECANUM_PID == p_state->current_page)
        {
            st7789_ui_update_mecanum_page(p_state);
            return;
        }

        if (g_current_edit_value != p_state->edit_value)
        {
            st7789_ui_update_page_value(p_state->edit_value);
        }

        if (g_current_editing != p_state->editing)
        {
            st7789_ui_update_page_editing(p_state->editing);
        }
    }
}
