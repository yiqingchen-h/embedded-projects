#include "software/ui/mipi_ui.h"

#include "hardware/Chassis/Chassis_config.h"
/* LVGL 头文件在 ARMClang 下有已知窄化转换告警，只在本文件 include 时局部屏蔽。 */
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wimplicit-int-conversion"
#pragma clang diagnostic ignored "-Wsign-conversion"
#endif
#include "third_party/lvgl/lvgl.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* MIPI 纯色诊断开关，默认关闭。置 1 时只显示纯色页，用于排查条纹是否来自驱动层。 */
#define MIPI_UI_DIAG_SOLID_TEST        (0)

/* MIPI 纯色诊断颜色编号：0 黑，1 灰，2 蓝，3 红，4 绿。 */
#define MIPI_UI_DIAG_SOLID_COLOR_INDEX (2U)

/* MIPI 竖屏 UI 逻辑宽度。 */
#define MIPI_UI_WIDTH                  (480)

/* MIPI 竖屏 UI 逻辑高度。 */
#define MIPI_UI_HEIGHT                 (800)

/* 顶部状态栏高度。 */
#define MIPI_TOP_BAR_H                 (52)

/* 首页底部提示栏高度。 */
#define MIPI_BOTTOM_HINT_H             (44)

/* 开机页标题 Y 坐标。 */
#define MIPI_BOOT_TITLE_Y              (44)

/* 开机页企鹅主体 X 坐标。 */
#define MIPI_BOOT_PENGUIN_X            (120)

/* 开机页企鹅主体 Y 坐标。 */
#define MIPI_BOOT_PENGUIN_Y            (178)

/* 开机页进度条 X 坐标。 */
#define MIPI_BOOT_PROGRESS_X           (60)

/* 开机页进度条 Y 坐标。 */
#define MIPI_BOOT_PROGRESS_Y           (570)

/* 开机页进度条宽度。 */
#define MIPI_BOOT_PROGRESS_W           (360)

/* 开机页进度条高度。 */
#define MIPI_BOOT_PROGRESS_H           (26)

/* 首页中心卡片 X 坐标。 */
#define MIPI_HOME_CENTER_X             (40)

/* 首页中心卡片 Y 坐标。 */
#define MIPI_HOME_CENTER_Y             (252)

/* 首页中心卡片宽度。 */
#define MIPI_HOME_CENTER_W             (400)

/* 首页中心卡片高度。 */
#define MIPI_HOME_CENTER_H             (232)

/* 首页弱化卡片 X 坐标。 */
#define MIPI_HOME_SIDE_X               (80)

/* 首页上一项卡片 Y 坐标。 */
#define MIPI_HOME_PREV_Y               (118)

/* 首页下一项卡片 Y 坐标。 */
#define MIPI_HOME_NEXT_Y               (534)

/* 首页弱化卡片宽度。 */
#define MIPI_HOME_SIDE_W               (320)

/* 首页弱化卡片高度。 */
#define MIPI_HOME_SIDE_H               (92)

/* 底盘模式栏 Y 坐标。 */
#define MIPI_CHASSIS_MODE_Y            (52)

/* 底盘模式栏高度。 */
#define MIPI_CHASSIS_MODE_H            (52)

/* 底盘主参数区 Y 坐标。 */
#define MIPI_CHASSIS_MAIN_Y            (112)

/* 底盘主参数区高度。 */
#define MIPI_CHASSIS_MAIN_H            (190)

/* 底盘 PID 参数区 Y 坐标。 */
#define MIPI_CHASSIS_PID_Y             (312)

/* 底盘 PID 参数区高度。 */
#define MIPI_CHASSIS_PID_H             (150)

/* 底盘反馈区 Y 坐标。 */
#define MIPI_CHASSIS_FEEDBACK_Y        (472)

/* 底盘反馈区高度。 */
#define MIPI_CHASSIS_FEEDBACK_H        (190)

/* 底盘状态区 Y 坐标。 */
#define MIPI_CHASSIS_STATUS_Y          (668)

/* 底盘状态区高度。 */
#define MIPI_CHASSIS_STATUS_H          (50)

/* 底盘操作区 Y 坐标。 */
#define MIPI_CHASSIS_ACTION_Y          (724)

/* 底盘操作区高度。 */
#define MIPI_CHASSIS_ACTION_H          (64)

/* 通用文本缓冲区长度。 */
#define MIPI_UI_TEXT_LEN               (72U)

/* 底盘主参数行数量。 */
#define MIPI_CHASSIS_MAIN_ROW_COUNT    (4U)

/* 底盘 PID 可编辑行数量。 */
#define MIPI_CHASSIS_PID_EDIT_COUNT    (2U)

/* 底盘反馈行数量。 */
#define MIPI_CHASSIS_FEEDBACK_COUNT    (4U)

/* 底盘动作按钮数量。 */
#define MIPI_CHASSIS_ACTION_COUNT      (4U)

/* 雷达点云区域 X 坐标。 */
#define MIPI_RADAR_CLOUD_X             (24)

/* 雷达点云区域 Y 坐标。 */
#define MIPI_RADAR_CLOUD_Y             (146)

/* 雷达点云区域宽度。 */
#define MIPI_RADAR_CLOUD_W             (432)

/* 雷达点云区域高度。 */
#define MIPI_RADAR_CLOUD_H             (432)

/* 雷达点云中心 X 坐标，基于点云区域。 */
#define MIPI_RADAR_CENTER_X            (216)

/* 雷达点云中心 Y 坐标，基于点云区域。 */
#define MIPI_RADAR_CENTER_Y            (216)

/* 雷达点云起始圆环半径，单位 px。 */
#define MIPI_RADAR_INNER_RADIUS_PX     (36)

/* 雷达点云最大显示半径，单位 px。 */
#define MIPI_RADAR_MAX_RADIUS_PX       (150)

/* 雷达点云最大显示距离，单位 mm。 */
#define MIPI_RADAR_MAX_DISTANCE_MM     (2000U)

/* 雷达点云圆点尺寸，单位 px。 */
#define MIPI_RADAR_POINT_SIZE          (7)

/* 雷达方向标记尺寸，单位 px。 */
#define MIPI_RADAR_MARKER_SIZE         (10)

/* 机械臂参数行数量。 */
#define MIPI_ARM_PARAM_COUNT           (6U)

/* 机械臂动作按钮数量。 */
#define MIPI_ARM_ACTION_COUNT          (4U)

/* 机械臂页面动作项起始索引。 */
#define MIPI_ARM_ACTION_BASE_INDEX     (6U)

/* 机械臂页面最近命令显示长度。 */
#define MIPI_ARM_COMMAND_TEXT_LEN      (32U)

/* 机械臂标定页参数数量。 */
#define MIPI_ARM_CALIB_PARAM_COUNT     (2U)

/* 机械臂标定页动作按钮数量。 */
#define MIPI_ARM_CALIB_ACTION_COUNT    (6U)

/* 机械臂标定页动作项起始索引。 */
#define MIPI_ARM_CALIB_ACTION_BASE_INDEX (2U)

/* Mission 页面状态行数量。 */
#define MIPI_MISSION_ROW_COUNT         (7U)

/* Mission 页面状态行 X 坐标。 */
#define MIPI_MISSION_ROW_X             (18)

/* Mission 页面首个状态行 Y 坐标。 */
#define MIPI_MISSION_ROW_Y             (64)

/* Mission 页面状态行宽度。 */
#define MIPI_MISSION_ROW_W             (444)

/* Mission 页面状态行高度。 */
#define MIPI_MISSION_ROW_H             (78)

/* Mission 页面状态行间距。 */
#define MIPI_MISSION_ROW_GAP           (12)

/* Mission 页面退出键提示 X 坐标。 */
#define MIPI_MISSION_EXIT_X            (18)

/* Mission 页面退出键提示 Y 坐标。 */
#define MIPI_MISSION_EXIT_Y            (620)

/* Mission 页面退出键提示宽度。 */
#define MIPI_MISSION_EXIT_W            (444)

/* Mission 页面退出键提示高度。 */
#define MIPI_MISSION_EXIT_H            (108)

/* 语音数据集流程行数量。 */
#define MIPI_VOICE_FLOW_ROW_COUNT      (3U)

/* 语音数据集流程行 X 坐标。 */
#define MIPI_VOICE_FLOW_ROW_X          (18)

/* 语音数据集首个流程行 Y 坐标。 */
#define MIPI_VOICE_FLOW_ROW_Y          (76)

/* 语音数据集流程行宽度。 */
#define MIPI_VOICE_FLOW_ROW_W          (444)

/* 语音数据集流程行高度。 */
#define MIPI_VOICE_FLOW_ROW_H          (82)

/* 语音数据集流程行间距。 */
#define MIPI_VOICE_FLOW_ROW_GAP        (14)

/* 语音数据集进度条 X 坐标。 */
#define MIPI_VOICE_PROGRESS_X          (32)

/* 语音数据集进度条 Y 坐标。 */
#define MIPI_VOICE_PROGRESS_Y          (500)

/* 语音数据集进度条宽度。 */
#define MIPI_VOICE_PROGRESS_W          (416)

/* 语音数据集进度条高度。 */
#define MIPI_VOICE_PROGRESS_H          (32)

/* 语音数据集操作按钮 X 坐标。 */
#define MIPI_VOICE_ACTION_X            (48)

/* 语音数据集操作按钮 Y 坐标。 */
#define MIPI_VOICE_ACTION_Y            (620)

/* 语音数据集操作按钮宽度。 */
#define MIPI_VOICE_ACTION_W            (384)

/* 语音数据集操作按钮高度。 */
#define MIPI_VOICE_ACTION_H            (86)

typedef struct st_mipi_boot_objs
{
    /* 开机进度条填充对象。 */
    lv_obj_t *progress_fill;

    /* 开机进度百分比标签。 */
    lv_obj_t *percent_label;
} mipi_boot_objs_t;

typedef struct st_mipi_home_objs
{
    /* 首页电源状态标签。 */
    lv_obj_t *power_label;

    /* 首页上一项卡片。 */
    lv_obj_t *prev_card;

    /* 首页中心卡片。 */
    lv_obj_t *center_card;

    /* 首页下一项卡片。 */
    lv_obj_t *next_card;

    /* 首页上一项标题。 */
    lv_obj_t *prev_title;

    /* 首页上一项副标题。 */
    lv_obj_t *prev_subtitle;

    /* 首页中心项标题。 */
    lv_obj_t *center_title;

    /* 首页中心项副标题。 */
    lv_obj_t *center_subtitle;

    /* 首页下一项标题。 */
    lv_obj_t *next_title;

    /* 首页下一项副标题。 */
    lv_obj_t *next_subtitle;

    /* 首页页码标签。 */
    lv_obj_t *page_label;
} mipi_home_objs_t;

typedef struct st_mipi_chassis_objs
{
    /* 底盘页电源状态标签。 */
    lv_obj_t *power_label;

    /* BODY 模式标签背景。 */
    lv_obj_t *tab_body;

    /* WHEEL 模式标签背景。 */
    lv_obj_t *tab_wheel;

    /* PWM 模式标签背景。 */
    lv_obj_t *tab_pwm;

    /* 底盘主参数区标题。 */
    lv_obj_t *main_title;

    /* 底盘主参数行背景。 */
    lv_obj_t *main_row[MIPI_CHASSIS_MAIN_ROW_COUNT];

    /* 底盘主参数名称。 */
    lv_obj_t *main_name[MIPI_CHASSIS_MAIN_ROW_COUNT];

    /* 底盘主参数数值。 */
    lv_obj_t *main_value[MIPI_CHASSIS_MAIN_ROW_COUNT];

    /* PID Kp/Ki 行背景。 */
    lv_obj_t *pid_row[MIPI_CHASSIS_PID_EDIT_COUNT];

    /* PID Kp 数值。 */
    lv_obj_t *pid_kp_value;

    /* PID Ki 数值。 */
    lv_obj_t *pid_ki_value;

    /* PID 输出限幅只读值。 */
    lv_obj_t *pid_limit_value;

    /* PID 死区补偿只读值。 */
    lv_obj_t *pid_deadband_value;

    /* PID 前馈系数只读值。 */
    lv_obj_t *pid_ff_value;

    /* PID 保存状态值。 */
    lv_obj_t *pid_status_value;

    /* 轮组反馈行背景。 */
    lv_obj_t *feedback_row[MIPI_CHASSIS_FEEDBACK_COUNT];

    /* 轮组反馈名称。 */
    lv_obj_t *feedback_line[MIPI_CHASSIS_FEEDBACK_COUNT];

    /* 轮组反馈数值。 */
    lv_obj_t *feedback_value[MIPI_CHASSIS_FEEDBACK_COUNT];

    /* 当前控制模式状态。 */
    lv_obj_t *status_mode;

    /* 当前使能状态。 */
    lv_obj_t *status_enable;

    /* 编码器状态。 */
    lv_obj_t *status_encoder;

    /* 应用按钮背景。 */
    lv_obj_t *action_apply;

    /* 保存 PID 按钮背景。 */
    lv_obj_t *action_save_pid;

    /* 清零按钮背景。 */
    lv_obj_t *action_zero;

    /* 停止按钮背景。 */
    lv_obj_t *action_stop;
} mipi_chassis_objs_t;

typedef struct st_mipi_placeholder_objs
{
    /* 占位页电源状态标签。 */
    lv_obj_t *power_label;

    /* 占位页标题标签。 */
    lv_obj_t *title_label;

    /* 占位页副标题标签。 */
    lv_obj_t *subtitle_label;
} mipi_placeholder_objs_t;

typedef struct st_mipi_radar_cloud_objs
{
    /* 雷达页电源状态标签。 */
    lv_obj_t *power_label;

    /* 雷达页点云状态标签。 */
    lv_obj_t *status_label;

    /* 雷达页避障状态标签。 */
    lv_obj_t *state_label;

    /* 雷达点云显示区域。 */
    lv_obj_t *cloud_area;

    /* 雷达页底部提示标签。 */
    lv_obj_t *hint_label;

    /* 雷达 72 扇区点对象。 */
    lv_obj_t *point_objs[N10_RADAR_CLOUD_SECTOR_COUNT];

    /* 点云起始空心圆环。 */
    lv_obj_t *inner_ring;

    /* 上层目标方向标记。 */
    lv_obj_t *target_marker;

    /* 避障选中方向标记。 */
    lv_obj_t *selected_marker;

    /* 小车中心标记。 */
    lv_obj_t *car_center;

    /* 车头方向标记。 */
    lv_obj_t *front_arrow;
} mipi_radar_cloud_objs_t;

typedef struct st_mipi_arm_debug_objs
{
    /* 机械臂页电源状态标签。 */
    lv_obj_t *power_label;

    /* 机械臂参数行背景。 */
    lv_obj_t *param_row[MIPI_ARM_PARAM_COUNT];

    /* 机械臂参数名。 */
    lv_obj_t *param_name[MIPI_ARM_PARAM_COUNT];

    /* 机械臂参数值。 */
    lv_obj_t *param_value[MIPI_ARM_PARAM_COUNT];

    /* 机械臂舵机反馈行背景。 */
    lv_obj_t *servo_row[MECH_HAND_SERVO_COUNT];

    /* 机械臂舵机目标 PWM。 */
    lv_obj_t *servo_target[MECH_HAND_SERVO_COUNT];

    /* 机械臂舵机真实反馈。 */
    lv_obj_t *servo_feedback[MECH_HAND_SERVO_COUNT];

    /* 机械臂状态标签。 */
    lv_obj_t *status_label;

    /* 机械臂最近命令标签。 */
    lv_obj_t *command_label;

    /* HOME 动作按钮背景。 */
    lv_obj_t *action_home;

    /* GRIP 动作按钮背景。 */
    lv_obj_t *action_grip;

    /* STOP 动作按钮背景。 */
    lv_obj_t *action_stop;

    /* CALIB 入口按钮背景。 */
    lv_obj_t *action_calib;
} mipi_arm_debug_objs_t;

typedef struct st_mipi_arm_calib_objs
{
    /* 机械臂标定页电源状态标签。 */
    lv_obj_t *power_label;

    /* 机械臂标定参数行背景。 */
    lv_obj_t *param_row[MIPI_ARM_CALIB_PARAM_COUNT];

    /* 机械臂标定参数名。 */
    lv_obj_t *param_name[MIPI_ARM_CALIB_PARAM_COUNT];

    /* 机械臂标定参数值。 */
    lv_obj_t *param_value[MIPI_ARM_CALIB_PARAM_COUNT];

    /* 当前点位坐标标签。 */
    lv_obj_t *target_label;

    /* 当前目标上方坐标标签。 */
    lv_obj_t *above_label;

    /* 自动流程状态标签。 */
    lv_obj_t *flow_label;

    /* 机械臂运行状态标签。 */
    lv_obj_t *status_label;

    /* 机械臂最近命令标签。 */
    lv_obj_t *command_label;

    /* 机械臂反馈摘要标签。 */
    lv_obj_t *feedback_label;

    /* 机械臂标定动作按钮背景。 */
    lv_obj_t *action[MIPI_ARM_CALIB_ACTION_COUNT];
} mipi_arm_calib_objs_t;

typedef struct st_mipi_mission_objs
{
    /* Mission 页电源状态标签。 */
    lv_obj_t *power_label;

    /* Mission 状态行背景。 */
    lv_obj_t *row[MIPI_MISSION_ROW_COUNT];

    /* Mission 状态行名称。 */
    lv_obj_t *name[MIPI_MISSION_ROW_COUNT];

    /* Mission 状态行数值。 */
    lv_obj_t *value[MIPI_MISSION_ROW_COUNT];

    /* Mission 退出提示背景。 */
    lv_obj_t *exit_panel;
} mipi_mission_objs_t;

typedef struct st_mipi_voice_dataset_objs
{
    /* 语音采集页电源状态标签。 */
    lv_obj_t *power_label;

    /* 流程行背景。 */
    lv_obj_t *flow_row[MIPI_VOICE_FLOW_ROW_COUNT];

    /* 流程行标题。 */
    lv_obj_t *flow_title[MIPI_VOICE_FLOW_ROW_COUNT];

    /* 流程行状态。 */
    lv_obj_t *flow_status[MIPI_VOICE_FLOW_ROW_COUNT];

    /* 当前状态标签。 */
    lv_obj_t *status_label;

    /* 进度条填充对象。 */
    lv_obj_t *progress_fill;

    /* 进度百分比标签。 */
    lv_obj_t *progress_label;

    /* 操作按钮背景。 */
    lv_obj_t *action_button;

    /* 操作按钮标题。 */
    lv_obj_t *action_title;

    /* 操作按钮副标题。 */
    lv_obj_t *action_subtitle;
} mipi_voice_dataset_objs_t;

/* LVGL 当前活动屏幕对象。 */
static lv_obj_t *g_screen = NULL;

/* 当前已构建视图。 */
static mipi_ui_view_t g_current_view = MIPI_UI_VIEW_BOOT;

/* MIPI UI 对象是否已构建。 */
static bool g_ui_built = false;

/* 开机页对象缓存。 */
static mipi_boot_objs_t g_boot_objs = {0};

/* 首页对象缓存。 */
static mipi_home_objs_t g_home_objs = {0};

/* 底盘页对象缓存。 */
static mipi_chassis_objs_t g_chassis_objs = {0};

/* 占位页对象缓存。 */
static mipi_placeholder_objs_t g_placeholder_objs = {0};

/* 雷达点云页对象缓存。 */
static mipi_radar_cloud_objs_t g_radar_cloud_objs = {0};

/* 机械臂调试页对象缓存。 */
static mipi_arm_debug_objs_t g_arm_debug_objs = {0};

/* 机械臂标定页对象缓存。 */
static mipi_arm_calib_objs_t g_arm_calib_objs = {0};

/* Mission 页对象缓存。 */
static mipi_mission_objs_t g_mission_objs = {0};

/* 语音数据集页对象缓存。 */
static mipi_voice_dataset_objs_t g_voice_dataset_objs = {0};

/* 当前顶栏语音 AI 识别结果标签。 */
static lv_obj_t *g_top_voice_ai_label = NULL;

/**
 * @brief  更新底盘调试页局部对象
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_chassis(const mipi_ui_state_t *p_state);

/**
 * @brief  获取背景顶部颜色
 *
 * @return lv_color_t LVGL 颜色
 */
static lv_color_t mipi_ui_bg_top(void)
{
    return lv_color_hex(0x07101FU);
}

/**
 * @brief  获取背景底部颜色
 *
 * @return lv_color_t LVGL 颜色
 */
static lv_color_t mipi_ui_bg_bottom(void)
{
    return lv_color_hex(0x101729U);
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
static lv_obj_t *mipi_ui_rect(lv_obj_t *parent,
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
 * @brief  创建空心圆环对象
 *
 * @param parent 父对象
 * @param x      左上角 X 坐标
 * @param y      左上角 Y 坐标
 * @param size   圆环外接正方形尺寸
 * @param color  圆环颜色
 * @return lv_obj_t* 新建对象指针
 */
static lv_obj_t *mipi_ui_hollow_circle(lv_obj_t *parent,
                                       int32_t x,
                                       int32_t y,
                                       int32_t size,
                                       lv_color_t color)
{
    lv_obj_t *obj = lv_obj_create(parent);

    lv_obj_remove_style_all(obj);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, size, size);
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(obj, size / 2, 0);
    lv_obj_set_style_border_width(obj, 2, 0);
    lv_obj_set_style_border_color(obj, color, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    return obj;
}

/**
 * @brief  创建固定区域文本标签
 *
 * @param parent 父对象
 * @param x      左上角 X 坐标
 * @param y      左上角 Y 坐标
 * @param w      宽度
 * @param h      高度
 * @param text   初始文本
 * @param font   字体
 * @param color  文本颜色
 * @param align  文本对齐方式
 * @return lv_obj_t* 新建标签对象指针
 */
static lv_obj_t *mipi_ui_label(lv_obj_t *parent,
                               int32_t x,
                               int32_t y,
                               int32_t w,
                               int32_t h,
                               const char *text,
                               const lv_font_t *font,
                               lv_color_t color,
                               lv_text_align_t align)
{
    lv_obj_t *label = lv_label_create(parent);

    lv_obj_remove_style_all(label);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_size(label, w, h);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_align(label, align, 0);
    lv_label_set_text(label, text);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE);

    return label;
}

/**
 * @brief  仅在文本变化时更新标签
 *
 * @param label 标签对象
 * @param text  新文本
 * @return void
 */
static void mipi_ui_label_set_text_if_changed(lv_obj_t *label, const char *text)
{
    const char *p_new_text = text;
    const char *p_old_text = NULL;

    if (NULL == label)
    {
        return;
    }

    if (NULL == p_new_text)
    {
        p_new_text = "";
    }

    p_old_text = lv_label_get_text(label);
    if ((NULL == p_old_text) || (0 != strcmp(p_old_text, p_new_text)))
    {
        lv_label_set_text(label, p_new_text);
    }
}

/**
 * @brief  设置对象边框样式
 *
 * @param obj       对象指针
 * @param selected  true 表示选中
 * @param editing   true 表示编辑态
 * @return void
 */
static void mipi_ui_apply_select_style(lv_obj_t *obj, bool selected, bool editing)
{
    if (NULL == obj)
    {
        return;
    }

    lv_obj_set_style_border_width(obj, selected ? 2 : 1, 0);
    lv_obj_set_style_border_color(obj,
                                  editing ? lv_color_hex(0xFDE68AU) :
                                  (selected ? lv_color_hex(0x38BDF8U) : lv_color_hex(0x334155U)),
                                  0);
    lv_obj_set_style_bg_color(obj,
                              selected ? lv_color_hex(0x153453U) : lv_color_hex(0x0F172AU),
                              0);
}

/**
 * @brief  应用竖屏深色背景
 *
 * @return void
 */
static void mipi_ui_apply_background(void)
{
    lv_obj_remove_style_all(g_screen);
    lv_obj_set_size(g_screen, MIPI_UI_WIDTH, MIPI_UI_HEIGHT);
    lv_obj_set_style_bg_color(g_screen, mipi_ui_bg_top(), 0);
    lv_obj_set_style_bg_grad_color(g_screen, mipi_ui_bg_bottom(), 0);
    lv_obj_set_style_bg_grad_dir(g_screen, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(g_screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(g_screen, LV_OBJ_FLAG_SCROLLABLE);
}

/**
 * @brief  清空屏幕并重置页面对象缓存
 *
 * 仅 view 切换时调用，避免同页数据刷新造成整页闪烁。
 *
 * @return void
 */
static void mipi_ui_clear_screen(void)
{
    (void)memset(&g_boot_objs, 0, sizeof(g_boot_objs));
    (void)memset(&g_home_objs, 0, sizeof(g_home_objs));
    (void)memset(&g_chassis_objs, 0, sizeof(g_chassis_objs));
    (void)memset(&g_placeholder_objs, 0, sizeof(g_placeholder_objs));
    (void)memset(&g_radar_cloud_objs, 0, sizeof(g_radar_cloud_objs));
    (void)memset(&g_arm_debug_objs, 0, sizeof(g_arm_debug_objs));
    (void)memset(&g_arm_calib_objs, 0, sizeof(g_arm_calib_objs));
    (void)memset(&g_mission_objs, 0, sizeof(g_mission_objs));
    (void)memset(&g_voice_dataset_objs, 0, sizeof(g_voice_dataset_objs));
    g_top_voice_ai_label = NULL;
    lv_obj_clean(g_screen);
    mipi_ui_apply_background();
}

/**
 * @brief  浮点数四舍五入为定点整数
 *
 * @param value 浮点数
 * @param scale 放大倍率
 * @return int32_t 定点整数
 */
static int32_t mipi_ui_float_to_scaled(float value, float scale)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * scale) + 0.5f);
    }

    return (int32_t)((value * scale) - 0.5f);
}

/**
 * @brief  格式化两位小数有符号数值
 *
 * @param value    浮点数
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void mipi_ui_format_signed_2dp(float value, char *p_text, size_t text_len)
{
    int32_t scaled = mipi_ui_float_to_scaled(value, 100.0f);
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
 * @brief  格式化两位小数数值并追加单位
 *
 * @param value    浮点数值
 * @param p_unit   单位字符串
 * @param p_text   输出缓冲区
 * @param text_len 输出缓冲区长度
 * @return void
 */
static void mipi_ui_format_signed_2dp_unit(float value, const char *p_unit, char *p_text, size_t text_len)
{
    char number_text[24] = {0};

    if ((NULL == p_unit) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    mipi_ui_format_signed_2dp(value, number_text, sizeof(number_text));
    (void)snprintf(p_text, text_len, "%s %s", number_text, p_unit);
}

/**
 * @brief  格式化整数 PID 参数
 *
 * @param value    参数值
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void mipi_ui_format_pid_value(float value, char *p_text, size_t text_len)
{
    int32_t scaled = mipi_ui_float_to_scaled(value, 1.0f);

    if ((NULL == p_text) || (0U == text_len))
    {
        return;
    }

    (void)snprintf(p_text, text_len, "%ld", (long)scaled);
}

/**
 * @brief  格式化电流文本
 *
 * @param current_ua 电流，单位 uA
 * @param p_text     文本输出缓冲区
 * @param text_len   文本缓冲区长度
 * @return void
 */
static void mipi_ui_format_current(int32_t current_ua, char *p_text, size_t text_len)
{
    int32_t scaled = current_ua / 10000;
    int32_t abs_scaled = scaled;
    char sign = ' ';

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
                   "%c%ld.%02ldA",
                   sign,
                   (long)(abs_scaled / 100),
                   (long)(abs_scaled % 100));
}

/**
 * @brief  格式化电压电流状态文本
 *
 * @param p_state  UI 状态
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void mipi_ui_format_power_text(const mipi_ui_state_t *p_state, char *p_text, size_t text_len)
{
    uint32_t voltage_01v = 0U;
    char current_text[16] = {0};

    if ((NULL == p_state) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (!p_state->power_valid)
    {
        (void)snprintf(p_text, text_len, "WAIT");
        return;
    }

    voltage_01v = (p_state->bus_voltage_mv + 50U) / 100U;
    mipi_ui_format_current(p_state->current_ua, current_text, sizeof(current_text));
    (void)snprintf(p_text,
                   text_len,
                   "%lu.%luV | %s",
                   (unsigned long)(voltage_01v / 10U),
                   (unsigned long)(voltage_01v % 10U),
                   current_text);
}

/**
 * @brief  格式化语音 AI 识别结果文本
 *
 * @param p_state  UI 状态
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void mipi_ui_format_voice_ai_text(const mipi_ui_state_t *p_state, char *p_text, size_t text_len)
{
    const char *p_label = NULL;

    if ((NULL == p_state) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (!p_state->voice_ai_valid)
    {
        (void)snprintf(p_text, text_len, "AI WAIT");
        return;
    }

    p_label = p_state->voice_ai_label;
    if ('\0' == p_state->voice_ai_label[0])
    {
        p_label = "unknown";
    }

    (void)snprintf(p_text,
                   text_len,
                   "AI %s %u%%",
                   p_label,
                   (unsigned int)p_state->voice_ai_score_percent);
}

/**
 * @brief  获取首页功能标题
 *
 * @param page 首页功能项
 * @return const char* 功能标题
 */
static const char *mipi_ui_home_title(mipi_home_page_t page)
{
    switch (page)
    {
        case MIPI_HOME_PAGE_CHASSIS:
            return "CHASSIS DEBUG";

        case MIPI_HOME_PAGE_RADAR:
            return "RADAR CLOUD";

        case MIPI_HOME_PAGE_ARM:
            return "ARM DEBUG";

        case MIPI_HOME_PAGE_MISSION:
            return "MISSION";

        case MIPI_HOME_PAGE_VOICE:
            return "VOICE DATASET";

        default:
            return "UNKNOWN";
    }
}

/**
 * @brief  获取首页功能副标题
 *
 * @param page 首页功能项
 * @return const char* 功能副标题
 */
static const char *mipi_ui_home_subtitle(mipi_home_page_t page)
{
    switch (page)
    {
        case MIPI_HOME_PAGE_CHASSIS:
            return "Avoid Body / Wheel / PWM";

        case MIPI_HOME_PAGE_RADAR:
            return "360 deg sector cloud";

        case MIPI_HOME_PAGE_ARM:
            return "XYZ / Servo / Grip";

        case MIPI_HOME_PAGE_MISSION:
            return "Vision pick and place";

        case MIPI_HOME_PAGE_VOICE:
            return "Record and send PCM";

        default:
            return "Pending";
    }
}

/**
 * @brief  获取底盘模式文本
 *
 * @param mode 底盘模式
 * @return const char* 模式文本
 */
static const char *mipi_ui_chassis_mode_text(mipi_chassis_mode_t mode)
{
    switch (mode)
    {
        case MIPI_CHASSIS_MODE_WHEEL:
            return "WHEEL";

        case MIPI_CHASSIS_MODE_PWM:
            return "PWM";

        case MIPI_CHASSIS_MODE_BODY:
        default:
            return "BODY";
    }
}

/**
 * @brief  获取 PID 保存状态文本
 *
 * @param status PID 保存状态
 * @return const char* 状态文本
 */
static const char *mipi_ui_pid_save_text(mipi_pid_save_status_t status)
{
    switch (status)
    {
        case MIPI_PID_SAVE_SAVING:
            return "SAVING";

        case MIPI_PID_SAVE_SAVED:
            return "SAVED";

        case MIPI_PID_SAVE_ERR:
            return "ERR";

        case MIPI_PID_SAVE_NONE:
        default:
            return "--";
    }
}

/**
 * @brief  获取避障状态文本
 *
 * @param state 避障状态
 * @return const char* 状态文本
 */
static const char *mipi_ui_avoidance_state_text(chassis_avoidance_state_t state)
{
    switch (state)
    {
        case CHASSIS_AVOIDANCE_STATE_AVOID:
            return "AVOID";

        case CHASSIS_AVOIDANCE_STATE_RETURN_TRACK:
            return "RETURN";

        case CHASSIS_AVOIDANCE_STATE_FAIL_SAFE:
            return "FAILSAFE";

        case CHASSIS_AVOIDANCE_STATE_NORMAL:
        default:
            return "NORMAL";
    }
}

/**
 * @brief  获取 Mission 状态文本
 *
 * @param state Mission 状态
 * @return const char* 状态文本
 */
static const char *mipi_ui_mission_state_text(mission_state_t state)
{
    switch (state)
    {
        case MISSION_STATE_TRACE_NO_AVOID:
            return "TRACE";

        case MISSION_STATE_TRACE_AVOID:
            return "TRACE+AVOID";

        case MISSION_STATE_WAIT_PICK_POS:
            return "WAIT PICK";

        case MISSION_STATE_WAIT_PLACE_POS:
            return "WAIT PLACE";

        case MISSION_STATE_WAIT_ACTION_DONE:
            return "WAIT ARM";

        case MISSION_STATE_PLACE_AFTER_HOLD:
            return "PLACE HOLD";

        case MISSION_STATE_ERROR:
            return "ERROR";

        case MISSION_STATE_IDLE:
        default:
            return "IDLE";
    }
}

/**
 * @brief  获取 Mission 循迹模式文本
 *
 * @param mode Mission 循迹模式
 * @return const char* 模式文本
 */
static const char *mipi_ui_mission_trace_text(mission_trace_mode_t mode)
{
    if (MISSION_TRACE_MODE_AVOID == mode)
    {
        return "AVOID";
    }

    return "NO AVOID";
}

/**
 * @brief  获取 Mission 抓取目标显示名
 *
 * @param p_label 原始语音标签名
 * @return const char* 去掉 select_ 前缀后的目标名
 */
static const char *mipi_ui_mission_pick_label(const char *p_label)
{
    if (NULL == p_label)
    {
        return "unknown";
    }

    if (0 == strncmp(p_label, "select_", 7U))
    {
        return &p_label[7];
    }

    return p_label;
}

/**
 * @brief  获取轮组名称
 *
 * @param index 轮组索引
 * @return const char* 轮组名称
 */
static const char *mipi_ui_wheel_name(uint8_t index)
{
    static const char *const names[CHASSIS_WHEEL_COUNT] =
    {
        "W1 LF",
        "W2 RF",
        "W3 LR",
        "W4 RR",
    };

    if (index >= (uint8_t)CHASSIS_WHEEL_COUNT)
    {
        return "W?";
    }

    return names[index];
}

/**
 * @brief  获取带循环的首页功能项
 *
 * @param page   当前页面
 * @param offset 偏移量
 * @return mipi_home_page_t 循环后的功能项
 */
static mipi_home_page_t mipi_ui_home_offset(mipi_home_page_t page, int32_t offset)
{
    int32_t index = (int32_t)page + offset;

    while (index < 0)
    {
        index += (int32_t)MIPI_HOME_PAGE_COUNT;
    }

    while (index >= (int32_t)MIPI_HOME_PAGE_COUNT)
    {
        index -= (int32_t)MIPI_HOME_PAGE_COUNT;
    }

    return (mipi_home_page_t)index;
}

/**
 * @brief  更新电源状态标签
 *
 * @param label   标签对象
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_power_label(lv_obj_t *label, const mipi_ui_state_t *p_state)
{
    char power_text[MIPI_UI_TEXT_LEN] = {0};

    if (NULL == p_state)
    {
        return;
    }

    if (NULL != label)
    {
        mipi_ui_format_power_text(p_state, power_text, sizeof(power_text));
        mipi_ui_label_set_text_if_changed(label, power_text);
    }

    if (NULL != g_top_voice_ai_label)
    {
        mipi_ui_format_voice_ai_text(p_state, power_text, sizeof(power_text));
        mipi_ui_label_set_text_if_changed(g_top_voice_ai_label, power_text);
    }
}

/**
 * @brief  创建顶部状态栏
 *
 * @param title   左侧标题
 * @param p_state UI 状态
 * @return lv_obj_t* 电源状态标签
 */
static lv_obj_t *mipi_ui_create_top_bar(const char *title, const mipi_ui_state_t *p_state)
{
    lv_obj_t *power_label = NULL;

    mipi_ui_rect(g_screen, 0, 0, MIPI_UI_WIDTH, MIPI_TOP_BAR_H, lv_color_hex(0x08111FU), 0);
    mipi_ui_label(g_screen,
                  16,
                  16,
                  128,
                  24,
                  title,
                  &lv_font_montserrat_20,
                  lv_color_hex(0xF8FAFCU),
                  LV_TEXT_ALIGN_LEFT);
    g_top_voice_ai_label = mipi_ui_label(g_screen,
                                         152,
                                         13,
                                         132,
                                         28,
                                         "",
                                         &lv_font_montserrat_20,
                                         lv_color_hex(0xF8FAFCU),
                                         LV_TEXT_ALIGN_CENTER);
    power_label = mipi_ui_label(g_screen,
                                292,
                                13,
                                170,
                                28,
                                "",
                                &lv_font_montserrat_20,
                                lv_color_hex(0xF8FAFCU),
                                LV_TEXT_ALIGN_RIGHT);
    mipi_ui_update_power_label(power_label, p_state);

    return power_label;
}

/**
 * @brief  创建底部提示栏
 *
 * @param text 提示文本
 * @return void
 */
static void mipi_ui_create_bottom_hint(const char *text)
{
    mipi_ui_rect(g_screen,
                 0,
                 MIPI_UI_HEIGHT - MIPI_BOTTOM_HINT_H,
                 MIPI_UI_WIDTH,
                 MIPI_BOTTOM_HINT_H,
                 lv_color_hex(0x08111FU),
                 0);
    mipi_ui_label(g_screen,
                  14,
                  MIPI_UI_HEIGHT - 34,
                  MIPI_UI_WIDTH - 28,
                  24,
                  text,
                  &lv_font_montserrat_14,
                  lv_color_hex(0xF8FAFCU),
                  LV_TEXT_ALIGN_CENTER);
}

#if MIPI_UI_DIAG_SOLID_TEST
/**
 * @brief  获取纯色诊断颜色
 *
 * @return lv_color_t 诊断颜色
 */
static lv_color_t mipi_ui_diag_color(void)
{
    switch (MIPI_UI_DIAG_SOLID_COLOR_INDEX)
    {
        case 1U:
            return lv_color_hex(0x808080U);

        case 2U:
            return lv_color_hex(0x0000FFU);

        case 3U:
            return lv_color_hex(0xFF0000U);

        case 4U:
            return lv_color_hex(0x00FF00U);

        default:
            return lv_color_hex(0x000000U);
    }
}

/**
 * @brief  构建纯色诊断页
 *
 * @return void
 */
static void mipi_ui_build_diag_solid(void)
{
    lv_obj_remove_style_all(g_screen);
    lv_obj_set_size(g_screen, MIPI_UI_WIDTH, MIPI_UI_HEIGHT);
    lv_obj_set_style_bg_color(g_screen, mipi_ui_diag_color(), 0);
    lv_obj_set_style_bg_opa(g_screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(g_screen, LV_OBJ_FLAG_SCROLLABLE);
}
#endif

/**
 * @brief  绘制更圆润的企鹅助手
 *
 * @return void
 */
static void mipi_ui_create_penguin(void)
{
    lv_obj_t *penguin = lv_obj_create(g_screen);

    lv_obj_remove_style_all(penguin);
    lv_obj_set_pos(penguin, MIPI_BOOT_PENGUIN_X, MIPI_BOOT_PENGUIN_Y);
    lv_obj_set_size(penguin, 240, 320);
    lv_obj_clear_flag(penguin, LV_OBJ_FLAG_SCROLLABLE);

    mipi_ui_rect(penguin, 0, 72, 56, 134, lv_color_hex(0x111827U), 30);
    mipi_ui_rect(penguin, 184, 72, 56, 134, lv_color_hex(0x111827U), 30);
    mipi_ui_rect(penguin, 20, 6, 200, 260, lv_color_hex(0x0B1220U), 100);
    mipi_ui_rect(penguin, 54, 120, 132, 122, lv_color_hex(0xF8FAFCU), 68);
    mipi_ui_rect(penguin, 60, 56, 42, 42, lv_color_hex(0xF8FAFCU), 21);
    mipi_ui_rect(penguin, 138, 56, 42, 42, lv_color_hex(0xF8FAFCU), 21);
    mipi_ui_rect(penguin, 72, 68, 18, 18, lv_color_hex(0x020617U), 9);
    mipi_ui_rect(penguin, 150, 68, 18, 18, lv_color_hex(0x020617U), 9);
    mipi_ui_rect(penguin, 78, 72, 6, 6, lv_color_hex(0xFFFFFFU), 3);
    mipi_ui_rect(penguin, 156, 72, 6, 6, lv_color_hex(0xFFFFFFU), 3);
    mipi_ui_rect(penguin, 104, 98, 32, 18, lv_color_hex(0xF97316U), 9);
    mipi_ui_rect(penguin, 50, 244, 72, 32, lv_color_hex(0xF97316U), 16);
    mipi_ui_rect(penguin, 118, 244, 72, 32, lv_color_hex(0xF97316U), 16);
}

/**
 * @brief  更新开机页
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_boot(const mipi_ui_state_t *p_state)
{
    int32_t fill_w = 0;
    char text[12] = {0};

    if ((NULL == p_state) || (NULL == g_boot_objs.progress_fill))
    {
        return;
    }

    fill_w = ((MIPI_BOOT_PROGRESS_W - 8) * (int32_t)p_state->boot_progress) / 100;
    lv_obj_set_width(g_boot_objs.progress_fill, fill_w);

    if (NULL != g_boot_objs.percent_label)
    {
        (void)snprintf(text, sizeof(text), "%u%%", (unsigned int)p_state->boot_progress);
        mipi_ui_label_set_text_if_changed(g_boot_objs.percent_label, text);
    }
}

/**
 * @brief  构建开机页对象
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_boot(const mipi_ui_state_t *p_state)
{
    lv_obj_t *bar = NULL;

    mipi_ui_label(g_screen, 0, MIPI_BOOT_TITLE_Y, MIPI_UI_WIDTH, 30, "RA8P1 SMART CAR", &lv_font_montserrat_24, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_CENTER);
    mipi_ui_label(g_screen, 0, MIPI_BOOT_TITLE_Y + 36, MIPI_UI_WIDTH, 20, "MIPI DISPLAY BOOT", &lv_font_montserrat_14, lv_color_hex(0x93C5FDU), LV_TEXT_ALIGN_CENTER);
    mipi_ui_create_penguin();
    mipi_ui_label(g_screen, 0, 520, MIPI_UI_WIDTH, 22, "System initializing...", &lv_font_montserrat_14, lv_color_hex(0xCBD5E1U), LV_TEXT_ALIGN_CENTER);

    bar = mipi_ui_rect(g_screen,
                       MIPI_BOOT_PROGRESS_X,
                       MIPI_BOOT_PROGRESS_Y,
                       MIPI_BOOT_PROGRESS_W,
                       MIPI_BOOT_PROGRESS_H,
                       lv_color_hex(0x172033U),
                       13);
    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_color(bar, lv_color_hex(0x3B82F6U), 0);
    g_boot_objs.progress_fill = mipi_ui_rect(bar, 4, 4, 0, MIPI_BOOT_PROGRESS_H - 8, lv_color_hex(0x38BDF8U), 8);
    g_boot_objs.percent_label = mipi_ui_label(g_screen, 0, 606, MIPI_UI_WIDTH, 24, "0%", &lv_font_montserrat_20, lv_color_hex(0xE0F2FEU), LV_TEXT_ALIGN_CENTER);

    mipi_ui_label(g_screen, 98, 652, 90, 18, "LCD", &lv_font_montserrat_14, lv_color_hex(0x94A3B8U), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 248, 652, 120, 18, "OK", &lv_font_montserrat_14, lv_color_hex(0x86EFACU), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 98, 680, 90, 18, "LVGL", &lv_font_montserrat_14, lv_color_hex(0x94A3B8U), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 248, 680, 120, 18, "OK", &lv_font_montserrat_14, lv_color_hex(0x86EFACU), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 98, 708, 90, 18, "CHASSIS", &lv_font_montserrat_14, lv_color_hex(0x94A3B8U), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 248, 708, 120, 18, "INIT", &lv_font_montserrat_14, lv_color_hex(0xFDE68AU), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 98, 736, 90, 18, "INA226", &lv_font_montserrat_14, lv_color_hex(0x94A3B8U), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 248, 736, 120, 18, "WAIT", &lv_font_montserrat_14, lv_color_hex(0xFDE68AU), LV_TEXT_ALIGN_LEFT);

    mipi_ui_update_boot(p_state);
}

/**
 * @brief  创建首页卡片对象
 *
 * @param x        X 坐标
 * @param y        Y 坐标
 * @param w        宽度
 * @param h        高度
 * @param selected true 表示中心卡片
 * @param pp_title 标题标签输出
 * @param pp_sub   副标题标签输出
 * @return lv_obj_t* 卡片背景对象
 */
static lv_obj_t *mipi_ui_create_home_card(int32_t x,
                                          int32_t y,
                                          int32_t w,
                                          int32_t h,
                                          bool selected,
                                          lv_obj_t **pp_title,
                                          lv_obj_t **pp_sub)
{
    lv_obj_t *card = mipi_ui_rect(g_screen, x, y, w, h, selected ? lv_color_hex(0x102A43U) : lv_color_hex(0x101827U), 12);

    lv_obj_set_style_border_width(card, selected ? 2 : 1, 0);
    lv_obj_set_style_border_color(card, selected ? lv_color_hex(0x38BDF8U) : lv_color_hex(0x334155U), 0);
    mipi_ui_rect(card, selected ? 30 : 20, selected ? 38 : 22, selected ? 56 : 32, selected ? 56 : 32, lv_color_hex(0x1D4ED8U), selected ? 14 : 8);
    mipi_ui_rect(card, selected ? 46 : 30, selected ? 54 : 32, selected ? 24 : 12, selected ? 24 : 12, lv_color_hex(0xE0F2FEU), selected ? 6 : 3);

    if (NULL != pp_title)
    {
        *pp_title = mipi_ui_label(card,
                                  selected ? 120 : 78,
                                  selected ? 54 : 22,
                                  selected ? 240 : 210,
                                  selected ? 34 : 24,
                                  "",
                                  selected ? &lv_font_montserrat_24 : &lv_font_montserrat_14,
                                  selected ? lv_color_hex(0xF8FAFCU) : lv_color_hex(0xCBD5E1U),
                                  LV_TEXT_ALIGN_LEFT);
    }

    if (NULL != pp_sub)
    {
        *pp_sub = mipi_ui_label(card,
                                selected ? 120 : 78,
                                selected ? 100 : 50,
                                selected ? 240 : 210,
                                selected ? 56 : 24,
                                "",
                                selected ? &lv_font_montserrat_20 : &lv_font_montserrat_14,
                                selected ? lv_color_hex(0xF8FAFCU) : lv_color_hex(0xCBD5E1U),
                                LV_TEXT_ALIGN_LEFT);
    }

    return card;
}

/**
 * @brief  更新首页卡片文本
 *
 * @param title_label    标题标签
 * @param subtitle_label 副标题标签
 * @param page           功能项
 * @return void
 */
static void mipi_ui_update_home_card(lv_obj_t *title_label, lv_obj_t *subtitle_label, mipi_home_page_t page)
{
    if (NULL != title_label)
    {
        mipi_ui_label_set_text_if_changed(title_label, mipi_ui_home_title(page));
    }

    if (NULL != subtitle_label)
    {
        mipi_ui_label_set_text_if_changed(subtitle_label, mipi_ui_home_subtitle(page));
    }
}

/**
 * @brief  更新首页
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_home(const mipi_ui_state_t *p_state)
{
    char page_text[16] = {0};
    mipi_home_page_t prev = MIPI_HOME_PAGE_CHASSIS;
    mipi_home_page_t next = MIPI_HOME_PAGE_CHASSIS;

    if (NULL == p_state)
    {
        return;
    }

    prev = mipi_ui_home_offset(p_state->home_selected, -1);
    next = mipi_ui_home_offset(p_state->home_selected, 1);
    mipi_ui_update_power_label(g_home_objs.power_label, p_state);
    mipi_ui_update_home_card(g_home_objs.prev_title, g_home_objs.prev_subtitle, prev);
    mipi_ui_update_home_card(g_home_objs.center_title, g_home_objs.center_subtitle, p_state->home_selected);
    mipi_ui_update_home_card(g_home_objs.next_title, g_home_objs.next_subtitle, next);

    (void)snprintf(page_text,
                   sizeof(page_text),
                   "%u / %u",
                   (unsigned int)((uint8_t)p_state->home_selected + 1U),
                   (unsigned int)MIPI_HOME_PAGE_COUNT);
    if (NULL != g_home_objs.page_label)
    {
        mipi_ui_label_set_text_if_changed(g_home_objs.page_label, page_text);
    }
}

/**
 * @brief  构建首页对象
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_home(const mipi_ui_state_t *p_state)
{
    g_home_objs.power_label = mipi_ui_create_top_bar("HOME", p_state);
    g_home_objs.prev_card = mipi_ui_create_home_card(MIPI_HOME_SIDE_X,
                                                     MIPI_HOME_PREV_Y,
                                                     MIPI_HOME_SIDE_W,
                                                     MIPI_HOME_SIDE_H,
                                                     false,
                                                     &g_home_objs.prev_title,
                                                     &g_home_objs.prev_subtitle);
    g_home_objs.center_card = mipi_ui_create_home_card(MIPI_HOME_CENTER_X,
                                                       MIPI_HOME_CENTER_Y,
                                                       MIPI_HOME_CENTER_W,
                                                       MIPI_HOME_CENTER_H,
                                                       true,
                                                       &g_home_objs.center_title,
                                                       &g_home_objs.center_subtitle);
    g_home_objs.next_card = mipi_ui_create_home_card(MIPI_HOME_SIDE_X,
                                                     MIPI_HOME_NEXT_Y,
                                                     MIPI_HOME_SIDE_W,
                                                     MIPI_HOME_SIDE_H,
                                                     false,
                                                     &g_home_objs.next_title,
                                                     &g_home_objs.next_subtitle);
    g_home_objs.page_label = mipi_ui_label(g_screen,
                                           0,
                                           676,
                                           MIPI_UI_WIDTH,
                                           24,
                                           "",
                                           &lv_font_montserrat_20,
                                           lv_color_hex(0xE0F2FEU),
                                           LV_TEXT_ALIGN_CENTER);
    mipi_ui_create_bottom_hint("KNOB Select | PRESS Enter | LONG Back");
    mipi_ui_update_home(p_state);
}

/**
 * @brief  获取底盘主参数动作起始索引
 *
 * @param mode 底盘模式
 * @return uint8_t 动作起始索引
 */
static uint8_t mipi_ui_chassis_action_base(mipi_chassis_mode_t mode)
{
    if (MIPI_CHASSIS_MODE_BODY == mode)
    {
        return 8U;
    }

    return 9U;
}

/**
 * @brief  判断指定索引是否选中
 *
 * @param p_state UI 状态
 * @param index   选中项索引
 * @return bool true 表示选中
 */
static bool mipi_ui_is_selected(const mipi_ui_state_t *p_state, uint8_t index)
{
    return (NULL != p_state) && (p_state->selected_index == index);
}

/**
 * @brief  创建底盘主参数行
 *
 * @param row   行号
 * @param y     Y 坐标
 * @return void
 */
static void mipi_ui_create_chassis_main_row(uint8_t row, int32_t y)
{
    if (row >= MIPI_CHASSIS_MAIN_ROW_COUNT)
    {
        return;
    }

    g_chassis_objs.main_row[row] = mipi_ui_rect(g_screen, 34, y, 412, 42, lv_color_hex(0x0F172AU), 9);
    g_chassis_objs.main_name[row] = mipi_ui_label(g_chassis_objs.main_row[row],
                                                  12,
                                                  10,
                                                  118,
                                                  24,
                                                  "",
                                                  &lv_font_montserrat_20,
                                                  lv_color_hex(0xCBD5E1U),
                                                  LV_TEXT_ALIGN_LEFT);
    g_chassis_objs.main_value[row] = mipi_ui_label(g_chassis_objs.main_row[row],
                                                   140,
                                                   7,
                                                   250,
                                                   30,
                                                   "",
                                                   &lv_font_montserrat_24,
                                                   lv_color_hex(0xE0F2FEU),
                                                   LV_TEXT_ALIGN_RIGHT);
}

/**
 * @brief  创建底盘页静态对象
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_chassis(const mipi_ui_state_t *p_state)
{
    static const char *const action_text[MIPI_CHASSIS_ACTION_COUNT] =
    {
        "APPLY",
        "SAVE PID",
        "ZERO",
        "STOP",
    };
    lv_obj_t *tab = NULL;
    uint8_t i = 0U;

    g_chassis_objs.power_label = mipi_ui_create_top_bar("< CHASSIS", p_state);

    g_chassis_objs.tab_body = mipi_ui_rect(g_screen, 18, MIPI_CHASSIS_MODE_Y + 7, 136, 38, lv_color_hex(0x111827U), 8);
    g_chassis_objs.tab_wheel = mipi_ui_rect(g_screen, 172, MIPI_CHASSIS_MODE_Y + 7, 136, 38, lv_color_hex(0x111827U), 8);
    g_chassis_objs.tab_pwm = mipi_ui_rect(g_screen, 326, MIPI_CHASSIS_MODE_Y + 7, 136, 38, lv_color_hex(0x111827U), 8);
    tab = g_chassis_objs.tab_body;
    mipi_ui_label(tab, 0, 8, 136, 24, "BODY", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_CENTER);
    tab = g_chassis_objs.tab_wheel;
    mipi_ui_label(tab, 0, 8, 136, 24, "WHEEL", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_CENTER);
    tab = g_chassis_objs.tab_pwm;
    mipi_ui_label(tab, 0, 8, 136, 24, "PWM", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_CENTER);

    mipi_ui_rect(g_screen, 18, MIPI_CHASSIS_MAIN_Y, 444, MIPI_CHASSIS_MAIN_H, lv_color_hex(0x0B1220U), 10);
    g_chassis_objs.main_title = mipi_ui_label(g_screen, 34, MIPI_CHASSIS_MAIN_Y + 12, 220, 22, "", &lv_font_montserrat_20, lv_color_hex(0x93C5FDU), LV_TEXT_ALIGN_LEFT);
    for (i = 0U; i < MIPI_CHASSIS_MAIN_ROW_COUNT; i++)
    {
        mipi_ui_create_chassis_main_row(i, MIPI_CHASSIS_MAIN_Y + 42 + ((int32_t)i * 36));
    }

    mipi_ui_rect(g_screen, 18, MIPI_CHASSIS_PID_Y, 444, MIPI_CHASSIS_PID_H, lv_color_hex(0x0B1220U), 10);
    mipi_ui_label(g_screen, 34, MIPI_CHASSIS_PID_Y + 10, 180, 22, "PID PARAM", &lv_font_montserrat_20, lv_color_hex(0x93C5FDU), LV_TEXT_ALIGN_LEFT);
    for (i = 0U; i < MIPI_CHASSIS_PID_EDIT_COUNT; i++)
    {
        int32_t x = (0U == i) ? 34 : 258;
        g_chassis_objs.pid_row[i] = mipi_ui_rect(g_screen, x, MIPI_CHASSIS_PID_Y + 42, 188, 42, lv_color_hex(0x0F172AU), 9);
        mipi_ui_label(g_chassis_objs.pid_row[i], 12, 10, 52, 24, (0U == i) ? "Kp" : "Ki", &lv_font_montserrat_20, lv_color_hex(0xCBD5E1U), LV_TEXT_ALIGN_LEFT);
    }
    g_chassis_objs.pid_kp_value = mipi_ui_label(g_chassis_objs.pid_row[0], 64, 7, 104, 30, "", &lv_font_montserrat_24, lv_color_hex(0xE0F2FEU), LV_TEXT_ALIGN_RIGHT);
    g_chassis_objs.pid_ki_value = mipi_ui_label(g_chassis_objs.pid_row[1], 64, 7, 104, 30, "", &lv_font_montserrat_24, lv_color_hex(0xE0F2FEU), LV_TEXT_ALIGN_RIGHT);
    mipi_ui_label(g_screen, 34, MIPI_CHASSIS_PID_Y + 94, 70, 24, "LIMIT", &lv_font_montserrat_20, lv_color_hex(0xE2E8F0U), LV_TEXT_ALIGN_LEFT);
    g_chassis_objs.pid_limit_value = mipi_ui_label(g_screen, 108, MIPI_CHASSIS_PID_Y + 94, 86, 24, "5000", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 214, MIPI_CHASSIS_PID_Y + 94, 58, 24, "DEAD", &lv_font_montserrat_20, lv_color_hex(0xE2E8F0U), LV_TEXT_ALIGN_LEFT);
    g_chassis_objs.pid_deadband_value = mipi_ui_label(g_screen, 276, MIPI_CHASSIS_PID_Y + 94, 92, 24, "13500", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 34, MIPI_CHASSIS_PID_Y + 122, 36, 24, "FF", &lv_font_montserrat_20, lv_color_hex(0xE2E8F0U), LV_TEXT_ALIGN_LEFT);
    g_chassis_objs.pid_ff_value = mipi_ui_label(g_screen, 82, MIPI_CHASSIS_PID_Y + 122, 82, 24, "8200", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_LEFT);
    mipi_ui_label(g_screen, 214, MIPI_CHASSIS_PID_Y + 122, 64, 24, "SAVE", &lv_font_montserrat_20, lv_color_hex(0xE2E8F0U), LV_TEXT_ALIGN_LEFT);
    g_chassis_objs.pid_status_value = mipi_ui_label(g_screen, 286, MIPI_CHASSIS_PID_Y + 122, 120, 24, "", &lv_font_montserrat_20, lv_color_hex(0xFDE68AU), LV_TEXT_ALIGN_LEFT);

    mipi_ui_rect(g_screen, 18, MIPI_CHASSIS_FEEDBACK_Y, 444, MIPI_CHASSIS_FEEDBACK_H, lv_color_hex(0x0B1220U), 10);
    mipi_ui_label(g_screen, 34, MIPI_CHASSIS_FEEDBACK_Y + 10, 220, 22, "WHEEL FEEDBACK", &lv_font_montserrat_20, lv_color_hex(0x93C5FDU), LV_TEXT_ALIGN_LEFT);
    for (i = 0U; i < MIPI_CHASSIS_FEEDBACK_COUNT; i++)
    {
        int32_t y = MIPI_CHASSIS_FEEDBACK_Y + 42 + ((int32_t)i * 35);
        g_chassis_objs.feedback_row[i] = mipi_ui_rect(g_screen, 34, y, 412, 32, lv_color_hex(0x0F172AU), 8);
        g_chassis_objs.feedback_line[i] = mipi_ui_label(g_chassis_objs.feedback_row[i], 12, 6, 104, 22, "", &lv_font_montserrat_20, lv_color_hex(0xCBD5E1U), LV_TEXT_ALIGN_LEFT);
        g_chassis_objs.feedback_value[i] = mipi_ui_label(g_chassis_objs.feedback_row[i], 116, 6, 276, 22, "", &lv_font_montserrat_20, lv_color_hex(0xE0F2FEU), LV_TEXT_ALIGN_RIGHT);
    }

    mipi_ui_rect(g_screen, 18, MIPI_CHASSIS_STATUS_Y, 444, MIPI_CHASSIS_STATUS_H, lv_color_hex(0x0B1220U), 10);
    g_chassis_objs.status_mode = mipi_ui_label(g_screen, 34, MIPI_CHASSIS_STATUS_Y + 12, 146, 26, "", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_LEFT);
    g_chassis_objs.status_enable = mipi_ui_label(g_screen, 192, MIPI_CHASSIS_STATUS_Y + 12, 116, 26, "", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_LEFT);
    g_chassis_objs.status_encoder = mipi_ui_label(g_screen, 326, MIPI_CHASSIS_STATUS_Y + 12, 104, 26, "ENC OK", &lv_font_montserrat_20, lv_color_hex(0x86EFACU), LV_TEXT_ALIGN_LEFT);

    for (i = 0U; i < MIPI_CHASSIS_ACTION_COUNT; i++)
    {
        lv_obj_t *button = mipi_ui_rect(g_screen, 18 + ((int32_t)i * 112), MIPI_CHASSIS_ACTION_Y, 106, MIPI_CHASSIS_ACTION_H, lv_color_hex(0x0F172AU), 9);
        mipi_ui_label(button, 0, 18, 106, 26, action_text[i], &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_CENTER);
        if (0U == i)
        {
            g_chassis_objs.action_apply = button;
        }
        else if (1U == i)
        {
            g_chassis_objs.action_save_pid = button;
        }
        else if (2U == i)
        {
            g_chassis_objs.action_zero = button;
        }
        else
        {
            g_chassis_objs.action_stop = button;
        }
    }

    mipi_ui_update_chassis(p_state);
}

/**
 * @brief  更新模式标签样式
 *
 * @param obj      标签背景对象
 * @param active   true 表示当前模式
 * @param selected true 表示当前选中
 * @return void
 */
static void mipi_ui_update_mode_tab(lv_obj_t *obj, bool active, bool selected)
{
    if (NULL == obj)
    {
        return;
    }

    lv_obj_set_style_bg_color(obj, active ? lv_color_hex(0x0E7490U) : lv_color_hex(0x111827U), 0);
    lv_obj_set_style_border_width(obj, selected ? 2 : 1, 0);
    lv_obj_set_style_border_color(obj, selected ? lv_color_hex(0xFDE68AU) : lv_color_hex(0x334155U), 0);
}

/**
 * @brief  设置底盘主参数行内容
 *
 * @param p_state UI 状态
 * @param row     行号
 * @param index   选中项索引
 * @param name    名称
 * @param value   数值文本
 * @return void
 */
static void mipi_ui_set_chassis_main_row(const mipi_ui_state_t *p_state,
                                         uint8_t row,
                                         uint8_t index,
                                         const char *name,
                                         const char *value)
{
    bool selected = false;
    bool editing = false;

    if ((NULL == p_state) || (row >= MIPI_CHASSIS_MAIN_ROW_COUNT))
    {
        return;
    }

    selected = mipi_ui_is_selected(p_state, index);
    editing = selected && p_state->editing;

    if (NULL != g_chassis_objs.main_row[row])
    {
        lv_obj_clear_flag(g_chassis_objs.main_row[row], LV_OBJ_FLAG_HIDDEN);
    }

    mipi_ui_apply_select_style(g_chassis_objs.main_row[row], selected, editing);

    if (NULL != g_chassis_objs.main_name[row])
    {
        mipi_ui_label_set_text_if_changed(g_chassis_objs.main_name[row], name);
        lv_obj_set_style_text_color(g_chassis_objs.main_name[row], selected ? lv_color_hex(0xF8FAFCU) : lv_color_hex(0xCBD5E1U), 0);
    }

    if (NULL != g_chassis_objs.main_value[row])
    {
        mipi_ui_label_set_text_if_changed(g_chassis_objs.main_value[row], value);
        lv_obj_set_style_text_color(g_chassis_objs.main_value[row], editing ? lv_color_hex(0xFDE68AU) : lv_color_hex(0xE0F2FEU), 0);
    }
}

/**
 * @brief  更新底盘主参数区
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_chassis_main(const mipi_ui_state_t *p_state)
{
    char value_text[MIPI_CHASSIS_MAIN_ROW_COUNT][MIPI_UI_TEXT_LEN] = {{0}};
    uint8_t i = 0U;
    float value = 0.0f;

    if (NULL == p_state)
    {
        return;
    }

    if (NULL != g_chassis_objs.main_title)
    {
        mipi_ui_label_set_text_if_changed(g_chassis_objs.main_title,
                                          (MIPI_CHASSIS_MODE_BODY == p_state->chassis_mode) ? "BODY SPEED" :
                                          ((MIPI_CHASSIS_MODE_WHEEL == p_state->chassis_mode) ? "WHEEL TARGET" : "OPEN PWM"));
    }

    if (MIPI_CHASSIS_MODE_BODY == p_state->chassis_mode)
    {
        value = (p_state->editing && (3U == p_state->selected_index)) ? p_state->edit_value : p_state->vx_mps;
        mipi_ui_format_signed_2dp_unit(value, "m/s", value_text[0], sizeof(value_text[0]));

        value = (p_state->editing && (4U == p_state->selected_index)) ? p_state->edit_value : p_state->vy_mps;
        mipi_ui_format_signed_2dp_unit(value, "m/s", value_text[1], sizeof(value_text[1]));

        value = (p_state->editing && (5U == p_state->selected_index)) ? p_state->edit_value : p_state->wz_radps;
        mipi_ui_format_signed_2dp_unit(value, "rad/s", value_text[2], sizeof(value_text[2]));

        mipi_ui_set_chassis_main_row(p_state, 0U, 3U, "VX", value_text[0]);
        mipi_ui_set_chassis_main_row(p_state, 1U, 4U, "VY", value_text[1]);
        mipi_ui_set_chassis_main_row(p_state, 2U, 5U, "WZ", value_text[2]);
        lv_obj_add_flag(g_chassis_objs.main_row[3], LV_OBJ_FLAG_HIDDEN);
        return;
    }

    for (i = 0U; i < (uint8_t)CHASSIS_WHEEL_COUNT; i++)
    {
        uint8_t index = (uint8_t)(3U + i);

        lv_obj_clear_flag(g_chassis_objs.main_row[i], LV_OBJ_FLAG_HIDDEN);
        if (MIPI_CHASSIS_MODE_PWM == p_state->chassis_mode)
        {
            int32_t pwm_value = (p_state->editing && (index == p_state->selected_index)) ?
                                (int32_t)p_state->edit_value :
                                p_state->pwm_counts[i];
            (void)snprintf(value_text[i], sizeof(value_text[i]), "%+ld", (long)pwm_value);
        }
        else
        {
            value = (p_state->editing && (index == p_state->selected_index)) ?
                    p_state->edit_value :
                    p_state->wheel_target_mps[i];
            mipi_ui_format_signed_2dp_unit(value, "m/s", value_text[i], sizeof(value_text[i]));
        }

        mipi_ui_set_chassis_main_row(p_state, i, index, mipi_ui_wheel_name(i), value_text[i]);
    }
}

/**
 * @brief  更新 PID 参数区
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_chassis_pid(const mipi_ui_state_t *p_state)
{
    char kp_text[16] = {0};
    char ki_text[16] = {0};
    uint8_t kp_index = 0U;
    uint8_t ki_index = 0U;
    bool selected = false;
    bool editing = false;
    float kp = 0.0f;
    float ki = 0.0f;

    if (NULL == p_state)
    {
        return;
    }

    kp_index = (MIPI_CHASSIS_MODE_BODY == p_state->chassis_mode) ? 6U : 7U;
    ki_index = (MIPI_CHASSIS_MODE_BODY == p_state->chassis_mode) ? 7U : 8U;
    kp = (p_state->editing && (kp_index == p_state->selected_index)) ? p_state->edit_value : p_state->kp;
    ki = (p_state->editing && (ki_index == p_state->selected_index)) ? p_state->edit_value : p_state->ki;
    mipi_ui_format_pid_value(kp, kp_text, sizeof(kp_text));
    mipi_ui_format_pid_value(ki, ki_text, sizeof(ki_text));

    selected = mipi_ui_is_selected(p_state, kp_index);
    editing = selected && p_state->editing;
    mipi_ui_apply_select_style(g_chassis_objs.pid_row[0], selected, editing);
    selected = mipi_ui_is_selected(p_state, ki_index);
    editing = selected && p_state->editing;
    mipi_ui_apply_select_style(g_chassis_objs.pid_row[1], selected, editing);

    if (NULL != g_chassis_objs.pid_kp_value)
    {
        mipi_ui_label_set_text_if_changed(g_chassis_objs.pid_kp_value, kp_text);
    }
    if (NULL != g_chassis_objs.pid_ki_value)
    {
        mipi_ui_label_set_text_if_changed(g_chassis_objs.pid_ki_value, ki_text);
    }
    if (NULL != g_chassis_objs.pid_status_value)
    {
        mipi_ui_label_set_text_if_changed(g_chassis_objs.pid_status_value, mipi_ui_pid_save_text(p_state->pid_save_status));
    }
}

/**
 * @brief  更新轮组反馈区
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_chassis_feedback(const mipi_ui_state_t *p_state)
{
    uint8_t i = 0U;

    if (NULL == p_state)
    {
        return;
    }

    for (i = 0U; i < (uint8_t)CHASSIS_WHEEL_COUNT; i++)
    {
        char target_text[20] = {0};
        char measured_text[20] = {0};
        char value_text[MIPI_UI_TEXT_LEN] = {0};

        if (MIPI_CHASSIS_MODE_PWM == p_state->chassis_mode)
        {
            (void)snprintf(target_text, sizeof(target_text), "%+ld", (long)p_state->pwm_counts[i]);
        }
        else
        {
            mipi_ui_format_signed_2dp(p_state->wheel_target_mps[i], target_text, sizeof(target_text));
        }

        mipi_ui_format_signed_2dp(p_state->wheel_measured_mps[i], measured_text, sizeof(measured_text));
        (void)snprintf(value_text, sizeof(value_text), "T %s   A %s", target_text, measured_text);

        if (NULL != g_chassis_objs.feedback_line[i])
        {
            mipi_ui_label_set_text_if_changed(g_chassis_objs.feedback_line[i], mipi_ui_wheel_name(i));
        }
        if (NULL != g_chassis_objs.feedback_value[i])
        {
            mipi_ui_label_set_text_if_changed(g_chassis_objs.feedback_value[i], value_text);
        }
    }
}

/**
 * @brief  更新底盘动作按钮
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_chassis_actions(const mipi_ui_state_t *p_state)
{
    lv_obj_t *buttons[MIPI_CHASSIS_ACTION_COUNT] =
    {
        g_chassis_objs.action_apply,
        g_chassis_objs.action_save_pid,
        g_chassis_objs.action_zero,
        g_chassis_objs.action_stop,
    };
    uint8_t base_index = 0U;
    uint8_t i = 0U;

    if (NULL == p_state)
    {
        return;
    }

    base_index = mipi_ui_chassis_action_base(p_state->chassis_mode);
    for (i = 0U; i < MIPI_CHASSIS_ACTION_COUNT; i++)
    {
        mipi_ui_apply_select_style(buttons[i], mipi_ui_is_selected(p_state, (uint8_t)(base_index + i)), false);
    }
}

/**
 * @brief  更新底盘调试页
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_chassis(const mipi_ui_state_t *p_state)
{
    char status_text[24] = {0};

    if (NULL == p_state)
    {
        return;
    }

    mipi_ui_update_power_label(g_chassis_objs.power_label, p_state);
    mipi_ui_update_mode_tab(g_chassis_objs.tab_body,
                            MIPI_CHASSIS_MODE_BODY == p_state->chassis_mode,
                            mipi_ui_is_selected(p_state, 0U));
    mipi_ui_update_mode_tab(g_chassis_objs.tab_wheel,
                            MIPI_CHASSIS_MODE_WHEEL == p_state->chassis_mode,
                            mipi_ui_is_selected(p_state, 1U));
    mipi_ui_update_mode_tab(g_chassis_objs.tab_pwm,
                            MIPI_CHASSIS_MODE_PWM == p_state->chassis_mode,
                            mipi_ui_is_selected(p_state, 2U));
    mipi_ui_update_chassis_main(p_state);
    mipi_ui_update_chassis_pid(p_state);
    mipi_ui_update_chassis_feedback(p_state);
    mipi_ui_update_chassis_actions(p_state);

    if (NULL != g_chassis_objs.status_mode)
    {
        (void)snprintf(status_text, sizeof(status_text), "Mode %s", mipi_ui_chassis_mode_text(p_state->chassis_mode));
        mipi_ui_label_set_text_if_changed(g_chassis_objs.status_mode, status_text);
    }
    if (NULL != g_chassis_objs.status_enable)
    {
        mipi_ui_label_set_text_if_changed(g_chassis_objs.status_enable, p_state->chassis_control_enabled ? "Enabled" : "Disabled");
    }
}

/**
 * @brief  更新占位页
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_placeholder(const mipi_ui_state_t *p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    mipi_ui_update_power_label(g_placeholder_objs.power_label, p_state);
    if (NULL != g_placeholder_objs.title_label)
    {
        mipi_ui_label_set_text_if_changed(g_placeholder_objs.title_label, mipi_ui_home_title(p_state->placeholder_page));
    }
    if (NULL != g_placeholder_objs.subtitle_label)
    {
        mipi_ui_label_set_text_if_changed(g_placeholder_objs.subtitle_label, mipi_ui_home_subtitle(p_state->placeholder_page));
    }
}

/**
 * @brief  构建占位页对象
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_placeholder(const mipi_ui_state_t *p_state)
{
    g_placeholder_objs.power_label = mipi_ui_create_top_bar("PLACEHOLDER", p_state);
    mipi_ui_rect(g_screen, 48, 250, 384, 260, lv_color_hex(0x0F172AU), 14);
    g_placeholder_objs.title_label = mipi_ui_label(g_screen, 70, 310, 340, 34, "", &lv_font_montserrat_24, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_CENTER);
    g_placeholder_objs.subtitle_label = mipi_ui_label(g_screen, 70, 360, 340, 24, "", &lv_font_montserrat_14, lv_color_hex(0x93C5FDU), LV_TEXT_ALIGN_CENTER);
    mipi_ui_label(g_screen, 70, 420, 340, 24, "Function pending", &lv_font_montserrat_14, lv_color_hex(0xCBD5E1U), LV_TEXT_ALIGN_CENTER);
    mipi_ui_create_bottom_hint("PRESS or LONG Back Home");
    mipi_ui_update_placeholder(p_state);
}

/**
 * @brief  归一化雷达角度
 *
 * @param angle_deg 原始角度，单位度
 * @return int16_t 0~359 度整数角度
 */
static int16_t mipi_ui_radar_normalize_angle(float angle_deg)
{
    int32_t angle = mipi_ui_float_to_scaled(angle_deg, 1.0f);

    while (angle < 0)
    {
        angle += 360;
    }

    while (angle >= 360)
    {
        angle -= 360;
    }

    return (int16_t)angle;
}

/**
 * @brief  计算雷达点云显示半径
 *
 * @param distance_mm 距离，单位 mm
 * @return int32_t 显示半径，单位 px
 */
static int32_t mipi_ui_radar_radius(uint16_t distance_mm)
{
    uint32_t distance = distance_mm;
    uint32_t radius = 0U;

    if (distance > MIPI_RADAR_MAX_DISTANCE_MM)
    {
        distance = MIPI_RADAR_MAX_DISTANCE_MM;
    }

    radius = ((uint32_t)(MIPI_RADAR_MAX_RADIUS_PX - MIPI_RADAR_INNER_RADIUS_PX) * distance) /
             MIPI_RADAR_MAX_DISTANCE_MM;

    return (int32_t)(MIPI_RADAR_INNER_RADIUS_PX + radius);
}

/**
 * @brief  雷达极坐标转点云区域坐标
 *
 * @param angle_deg 角度，0 度为车头
 * @param radius    半径，单位 px
 * @param p_x       X 坐标输出
 * @param p_y       Y 坐标输出
 * @return void
 */
static void mipi_ui_radar_to_xy(int16_t angle_deg, int32_t radius, int32_t *p_x, int32_t *p_y)
{
    int32_t sin_value = 0;
    int32_t cos_value = 0;

    if ((NULL == p_x) || (NULL == p_y))
    {
        return;
    }

    sin_value = lv_trigo_sin(angle_deg);
    cos_value = lv_trigo_sin((int16_t)(angle_deg + 90));
    *p_x = MIPI_RADAR_CENTER_X + ((radius * sin_value) >> LV_TRIGO_SHIFT);
    *p_y = MIPI_RADAR_CENTER_Y - ((radius * cos_value) >> LV_TRIGO_SHIFT);
}

/**
 * @brief  获取雷达点颜色
 *
 * @param distance_mm 距离，单位 mm
 * @return lv_color_t 点颜色
 */
static lv_color_t mipi_ui_radar_point_color(uint16_t distance_mm)
{
    if (distance_mm < 350U)
    {
        return lv_color_hex(0xF97316U);
    }

    if (distance_mm < 900U)
    {
        return lv_color_hex(0xFACC15U);
    }

    return lv_color_hex(0x38BDF8U);
}

/**
 * @brief  格式化雷达点云状态
 *
 * @param p_state  UI 状态
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void mipi_ui_format_radar_status(const mipi_ui_state_t *p_state, char *p_text, size_t text_len)
{
    int32_t rpm_10 = 0;

    if ((NULL == p_state) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (!p_state->radar_cloud_valid)
    {
        (void)snprintf(p_text, text_len, "SEQ --  RPM --  NEAR INVALID");
        return;
    }

    rpm_10 = mipi_ui_float_to_scaled(p_state->radar_rpm, 10.0f);
    if (0U == p_state->radar_nearest_distance_mm)
    {
        (void)snprintf(p_text,
                       text_len,
                       "SEQ %lu  RPM %ld.%ld  NEAR INVALID",
                       (unsigned long)p_state->radar_cloud_sequence,
                       (long)(rpm_10 / 10),
                       (long)(rpm_10 % 10));
    }
    else
    {
        (void)snprintf(p_text,
                       text_len,
                       "SEQ %lu  RPM %ld.%ld  NEAR %umm",
                       (unsigned long)p_state->radar_cloud_sequence,
                       (long)(rpm_10 / 10),
                       (long)(rpm_10 % 10),
                       (unsigned int)p_state->radar_nearest_distance_mm);
    }
}

/**
 * @brief  格式化避障只读状态
 *
 * @param p_state  UI 状态
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void mipi_ui_format_avoidance_status(const mipi_ui_state_t *p_state, char *p_text, size_t text_len)
{
    int32_t target_angle = 0;
    int32_t selected_angle = 0;

    if ((NULL == p_state) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (!p_state->radar_cloud_valid)
    {
        (void)snprintf(p_text, text_len, "AVD WAIT  STALE --  T --  S --");
        return;
    }

    target_angle = mipi_ui_radar_normalize_angle(p_state->avoidance_target_angle_deg);
    selected_angle = mipi_ui_radar_normalize_angle(p_state->avoidance_selected_angle_deg);
    (void)snprintf(p_text,
                   text_len,
                   "AVD %s  STALE %lums  T %ld  S %ld  %s",
                   mipi_ui_avoidance_state_text(p_state->avoidance_state),
                   (unsigned long)p_state->avoidance_radar_stale_ms,
                   (long)target_angle,
                   (long)selected_angle,
                   p_state->avoidance_target_safe ? "SAFE" : "BLOCK");
}

/**
 * @brief  更新雷达方向标记位置
 *
 * @param marker    标记对象
 * @param angle_deg 角度，单位度
 * @param visible   true 表示显示
 * @return void
 */
static void mipi_ui_update_radar_marker(lv_obj_t *marker, float angle_deg, bool visible)
{
    int32_t x = 0;
    int32_t y = 0;

    if (NULL == marker)
    {
        return;
    }

    if (!visible)
    {
        lv_obj_add_flag(marker, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    mipi_ui_radar_to_xy(mipi_ui_radar_normalize_angle(angle_deg), MIPI_RADAR_MAX_RADIUS_PX, &x, &y);
    lv_obj_set_pos(marker, x - (MIPI_RADAR_MARKER_SIZE / 2), y - (MIPI_RADAR_MARKER_SIZE / 2));
    lv_obj_clear_flag(marker, LV_OBJ_FLAG_HIDDEN);
}

/**
 * @brief  绘制雷达点云对象位置
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_draw_radar_cloud(const mipi_ui_state_t *p_state)
{
    lv_obj_t *point = NULL;
    int32_t x = 0;
    int32_t y = 0;
    int32_t radius = 0;
    int16_t angle = 0;

    if (NULL == p_state)
    {
        return;
    }

    for (uint32_t i = 0U; i < N10_RADAR_CLOUD_SECTOR_COUNT; i++)
    {
        point = g_radar_cloud_objs.point_objs[i];
        if (NULL == point)
        {
            continue;
        }

        if ((!p_state->radar_cloud_valid) || (0U == p_state->radar_sector_valid[i]))
        {
            lv_obj_add_flag(point, LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        radius = mipi_ui_radar_radius(p_state->radar_sector_min_mm[i]);
        angle = (int16_t)(i * N10_RADAR_CLOUD_SECTOR_DEG);
        mipi_ui_radar_to_xy(angle, radius, &x, &y);
        lv_obj_set_pos(point, x - (MIPI_RADAR_POINT_SIZE / 2), y - (MIPI_RADAR_POINT_SIZE / 2));
        lv_obj_set_style_bg_color(point, mipi_ui_radar_point_color(p_state->radar_sector_min_mm[i]), 0);
        lv_obj_clear_flag(point, LV_OBJ_FLAG_HIDDEN);
    }

    mipi_ui_update_radar_marker(g_radar_cloud_objs.target_marker,
                                p_state->avoidance_target_angle_deg,
                                p_state->radar_cloud_valid);
    mipi_ui_update_radar_marker(g_radar_cloud_objs.selected_marker,
                                p_state->avoidance_selected_angle_deg,
                                p_state->radar_cloud_valid);
}

/**
 * @brief  更新雷达点云页
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_radar_cloud(const mipi_ui_state_t *p_state)
{
    char status_text[MIPI_UI_TEXT_LEN] = {0};
    char state_text[MIPI_UI_TEXT_LEN] = {0};

    if (NULL == p_state)
    {
        return;
    }

    mipi_ui_update_power_label(g_radar_cloud_objs.power_label, p_state);
    mipi_ui_format_radar_status(p_state, status_text, sizeof(status_text));
    mipi_ui_format_avoidance_status(p_state, state_text, sizeof(state_text));
    mipi_ui_label_set_text_if_changed(g_radar_cloud_objs.status_label, status_text);
    mipi_ui_label_set_text_if_changed(g_radar_cloud_objs.state_label, state_text);
    mipi_ui_draw_radar_cloud(p_state);
}

/**
 * @brief  构建雷达点云页对象
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_radar_cloud(const mipi_ui_state_t *p_state)
{
    lv_obj_t *outer_ring = NULL;

    g_radar_cloud_objs.power_label = mipi_ui_create_top_bar("RADAR CLOUD", p_state);
    g_radar_cloud_objs.status_label = mipi_ui_label(g_screen,
                                                    24,
                                                    66,
                                                    432,
                                                    22,
                                                    "",
                                                    &lv_font_montserrat_14,
                                                    lv_color_hex(0xE0F2FEU),
                                                    LV_TEXT_ALIGN_LEFT);
    g_radar_cloud_objs.state_label = mipi_ui_label(g_screen,
                                                   24,
                                                   94,
                                                   432,
                                                   22,
                                                   "",
                                                   &lv_font_montserrat_14,
                                                   lv_color_hex(0xCBD5E1U),
                                                   LV_TEXT_ALIGN_LEFT);

    g_radar_cloud_objs.cloud_area = mipi_ui_rect(g_screen,
                                                 MIPI_RADAR_CLOUD_X,
                                                 MIPI_RADAR_CLOUD_Y,
                                                 MIPI_RADAR_CLOUD_W,
                                                 MIPI_RADAR_CLOUD_H,
                                                 lv_color_hex(0x0B1220U),
                                                 16);
    lv_obj_set_style_border_width(g_radar_cloud_objs.cloud_area, 1, 0);
    lv_obj_set_style_border_color(g_radar_cloud_objs.cloud_area, lv_color_hex(0x1E3A5FU), 0);

    outer_ring = mipi_ui_hollow_circle(g_radar_cloud_objs.cloud_area,
                                       MIPI_RADAR_CENTER_X - MIPI_RADAR_MAX_RADIUS_PX,
                                       MIPI_RADAR_CENTER_Y - MIPI_RADAR_MAX_RADIUS_PX,
                                       MIPI_RADAR_MAX_RADIUS_PX * 2,
                                       lv_color_hex(0x1E3A5FU));
    lv_obj_set_style_border_width(outer_ring, 1, 0);

    g_radar_cloud_objs.inner_ring = mipi_ui_hollow_circle(g_radar_cloud_objs.cloud_area,
                                                         MIPI_RADAR_CENTER_X - MIPI_RADAR_INNER_RADIUS_PX,
                                                         MIPI_RADAR_CENTER_Y - MIPI_RADAR_INNER_RADIUS_PX,
                                                         MIPI_RADAR_INNER_RADIUS_PX * 2,
                                                         lv_color_hex(0x60A5FAU));

    for (uint32_t i = 0U; i < N10_RADAR_CLOUD_SECTOR_COUNT; i++)
    {
        g_radar_cloud_objs.point_objs[i] = mipi_ui_rect(g_radar_cloud_objs.cloud_area,
                                                        0,
                                                        0,
                                                        MIPI_RADAR_POINT_SIZE,
                                                        MIPI_RADAR_POINT_SIZE,
                                                        lv_color_hex(0x38BDF8U),
                                                        MIPI_RADAR_POINT_SIZE / 2);
        lv_obj_add_flag(g_radar_cloud_objs.point_objs[i], LV_OBJ_FLAG_HIDDEN);
    }

    g_radar_cloud_objs.target_marker = mipi_ui_rect(g_radar_cloud_objs.cloud_area,
                                                    0,
                                                    0,
                                                    MIPI_RADAR_MARKER_SIZE,
                                                    MIPI_RADAR_MARKER_SIZE,
                                                    lv_color_hex(0xFDE047U),
                                                    MIPI_RADAR_MARKER_SIZE / 2);
    lv_obj_add_flag(g_radar_cloud_objs.target_marker, LV_OBJ_FLAG_HIDDEN);

    g_radar_cloud_objs.selected_marker = mipi_ui_rect(g_radar_cloud_objs.cloud_area,
                                                      0,
                                                      0,
                                                      MIPI_RADAR_MARKER_SIZE,
                                                      MIPI_RADAR_MARKER_SIZE,
                                                      lv_color_hex(0x22C55EU),
                                                      MIPI_RADAR_MARKER_SIZE / 2);
    lv_obj_add_flag(g_radar_cloud_objs.selected_marker, LV_OBJ_FLAG_HIDDEN);

    g_radar_cloud_objs.car_center = mipi_ui_rect(g_radar_cloud_objs.cloud_area,
                                                 MIPI_RADAR_CENTER_X - 12,
                                                 MIPI_RADAR_CENTER_Y - 14,
                                                 24,
                                                 30,
                                                 lv_color_hex(0xE0F2FEU),
                                                 10);
    g_radar_cloud_objs.front_arrow = mipi_ui_rect(g_radar_cloud_objs.cloud_area,
                                                  MIPI_RADAR_CENTER_X - 4,
                                                  MIPI_RADAR_CENTER_Y - 36,
                                                  8,
                                                  20,
                                                  lv_color_hex(0x38BDF8U),
                                                  4);

    g_radar_cloud_objs.hint_label = mipi_ui_label(g_screen,
                                                  24,
                                                  604,
                                                  432,
                                                  24,
                                                  "HARD 145-245  WEAK 245-310  USE 310-145",
                                                  &lv_font_montserrat_14,
                                                  lv_color_hex(0xF8FAFCU),
                                                  LV_TEXT_ALIGN_CENTER);
    mipi_ui_label(g_screen,
                  24,
                  632,
                  432,
                  22,
                  "Yellow: target  Green: selected",
                  &lv_font_montserrat_14,
                  lv_color_hex(0x93C5FDU),
                  LV_TEXT_ALIGN_CENTER);
    mipi_ui_create_bottom_hint("KEY4 Back | LONG Home");
    mipi_ui_update_radar_cloud(p_state);
}

/**
 * @brief  获取机械臂动作按钮对象
 *
 * @param action 动作序号
 * @return lv_obj_t* 动作按钮对象
 */
static lv_obj_t *mipi_ui_arm_action_obj(uint8_t action)
{
    switch (action)
    {
        case 0U:
            return g_arm_debug_objs.action_home;

        case 1U:
            return g_arm_debug_objs.action_grip;

        case 2U:
            return g_arm_debug_objs.action_stop;

        case 3U:
            return g_arm_debug_objs.action_calib;

        default:
            return NULL;
    }
}

/**
 * @brief  创建机械臂动作按钮
 *
 * @param x    X 坐标
 * @param y    Y 坐标
 * @param w    宽度
 * @param text 按钮文本
 * @return lv_obj_t* 按钮背景对象
 */
static lv_obj_t *mipi_ui_create_arm_action(int32_t x, int32_t y, int32_t w, const char *text)
{
    lv_obj_t *button = mipi_ui_rect(g_screen, x, y, w, 46, lv_color_hex(0x0F172AU), 10);

    lv_obj_set_style_border_width(button, 1, 0);
    lv_obj_set_style_border_color(button, lv_color_hex(0x334155U), 0);
    mipi_ui_label(button,
                  0,
                  12,
                  w,
                  24,
                  text,
                  &lv_font_montserrat_20,
                  lv_color_hex(0xF8FAFCU),
                  LV_TEXT_ALIGN_CENTER);

    return button;
}

/**
 * @brief  格式化机械臂目标参数
 *
 * @param p_state  UI 状态
 * @param index    参数索引
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void mipi_ui_format_arm_param(const mipi_ui_state_t *p_state, uint8_t index, char *p_text, size_t text_len)
{
    int32_t value = 0;

    if ((NULL == p_state) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (p_state->editing && (p_state->selected_index == index))
    {
        value = mipi_ui_float_to_scaled(p_state->edit_value, 1.0f);
    }

    switch (index)
    {
        case 0U:
            value = p_state->editing && (p_state->selected_index == index) ? value :
                    mipi_ui_float_to_scaled(p_state->arm_x_mm, 1.0f);
            (void)snprintf(p_text, text_len, "%ld mm", (long)value);
            break;

        case 1U:
            value = p_state->editing && (p_state->selected_index == index) ? value :
                    mipi_ui_float_to_scaled(p_state->arm_y_mm, 1.0f);
            (void)snprintf(p_text, text_len, "%ld mm", (long)value);
            break;

        case 2U:
            value = p_state->editing && (p_state->selected_index == index) ? value :
                    mipi_ui_float_to_scaled(p_state->arm_z_mm, 1.0f);
            (void)snprintf(p_text, text_len, "%ld mm", (long)value);
            break;

        case 3U:
            value = p_state->editing && (p_state->selected_index == index) ? value : (int32_t)p_state->arm_time_ms;
            (void)snprintf(p_text, text_len, "%ld ms", (long)value);
            break;

        case 4U:
            value = p_state->editing && (p_state->selected_index == index) ? value : (int32_t)p_state->arm_servo_index;
            (void)snprintf(p_text, text_len, "S%ld", (long)value);
            break;

        case 5U:
            value = p_state->editing && (p_state->selected_index == index) ? value : (int32_t)p_state->arm_servo_pwm;
            (void)snprintf(p_text, text_len, "%ld", (long)value);
            break;

        default:
            (void)snprintf(p_text, text_len, "--");
            break;
    }
}

/**
 * @brief  更新机械臂目标参数区
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_arm_params(const mipi_ui_state_t *p_state)
{
    static const char *const names[MIPI_ARM_PARAM_COUNT] =
    {
        "X",
        "Y",
        "Z",
        "TIME",
        "IDX",
        "PWM",
    };
    char text[MIPI_UI_TEXT_LEN] = {0};

    if (NULL == p_state)
    {
        return;
    }

    for (uint32_t i = 0U; i < MIPI_ARM_PARAM_COUNT; i++)
    {
        bool selected = mipi_ui_is_selected(p_state, (uint8_t)i);
        bool editing = p_state->editing && selected;

        mipi_ui_label_set_text_if_changed(g_arm_debug_objs.param_name[i], names[i]);
        mipi_ui_format_arm_param(p_state, (uint8_t)i, text, sizeof(text));
        mipi_ui_label_set_text_if_changed(g_arm_debug_objs.param_value[i], text);
        mipi_ui_apply_select_style(g_arm_debug_objs.param_row[i], selected, editing);
        if (NULL != g_arm_debug_objs.param_name[i])
        {
            lv_obj_set_style_text_color(g_arm_debug_objs.param_name[i], selected ? lv_color_hex(0xF8FAFCU) : lv_color_hex(0xCBD5E1U), 0);
        }
        if (NULL != g_arm_debug_objs.param_value[i])
        {
            lv_obj_set_style_text_color(g_arm_debug_objs.param_value[i], editing ? lv_color_hex(0xFDE68AU) : lv_color_hex(0xE0F2FEU), 0);
        }
    }
}

/**
 * @brief  更新机械臂舵机反馈区
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_arm_servos(const mipi_ui_state_t *p_state)
{
    char target_text[MIPI_UI_TEXT_LEN] = {0};
    char feedback_text[MIPI_UI_TEXT_LEN] = {0};
    const mech_hand_servo_feedback_t *p_feedback = NULL;

    if (NULL == p_state)
    {
        return;
    }

    for (uint32_t i = 0U; i < MECH_HAND_SERVO_COUNT; i++)
    {
        p_feedback = &p_state->arm_feedback.servo[i];
        (void)snprintf(target_text,
                       sizeof(target_text),
                       "S%lu T%u",
                       (unsigned long)i,
                       (unsigned int)p_state->arm_target_positions.servo_pwm[i]);
        if (p_feedback->valid)
        {
            (void)snprintf(feedback_text,
                           sizeof(feedback_text),
                           "P%u  %dC  V%u",
                           (unsigned int)p_feedback->position_pwm,
                           (int)p_feedback->temperature_c,
                           (unsigned int)p_feedback->voltage_raw);
        }
        else
        {
            (void)snprintf(feedback_text,
                           sizeof(feedback_text),
                           "P--  --C  V--  E%d",
                           p_feedback->last_error);
        }

        mipi_ui_label_set_text_if_changed(g_arm_debug_objs.servo_target[i], target_text);
        mipi_ui_label_set_text_if_changed(g_arm_debug_objs.servo_feedback[i], feedback_text);
    }
}

/**
 * @brief  更新机械臂状态和动作区
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_arm_status_actions(const mipi_ui_state_t *p_state)
{
    char status_text[MIPI_UI_TEXT_LEN] = {0};
    char command_text[MIPI_UI_TEXT_LEN] = {0};
    const char *ready_text = NULL;
    const char *safe_text = NULL;
    const char *grip_text = NULL;
    const char *cmd_text = NULL;

    if (NULL == p_state)
    {
        return;
    }

    ready_text = p_state->arm_initialized ? "READY" : "WAIT";
    safe_text = p_state->arm_last_clamped ? "CLAMP" : "OK";
    grip_text = p_state->arm_gripper_opened ? "OPEN" : "CLOSE";
    cmd_text = ('\0' == p_state->arm_last_command[0]) ? "--" : p_state->arm_last_command;
    (void)snprintf(status_text,
                   sizeof(status_text),
                   "%s  RET %d  SAFE %s  GRIP %s",
                   ready_text,
                   p_state->arm_last_result,
                   safe_text,
                   grip_text);
    (void)snprintf(command_text, sizeof(command_text), "CMD %s", cmd_text);

    mipi_ui_label_set_text_if_changed(g_arm_debug_objs.status_label, status_text);
    mipi_ui_label_set_text_if_changed(g_arm_debug_objs.command_label, command_text);

    for (uint32_t i = 0U; i < MIPI_ARM_ACTION_COUNT; i++)
    {
        uint8_t selected_index = (uint8_t)(MIPI_ARM_ACTION_BASE_INDEX + i);

        mipi_ui_apply_select_style(mipi_ui_arm_action_obj((uint8_t)i),
                                   mipi_ui_is_selected(p_state, selected_index),
                                   false);
    }
}

/**
 * @brief  更新机械臂调试页
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_arm_debug(const mipi_ui_state_t *p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    mipi_ui_update_power_label(g_arm_debug_objs.power_label, p_state);
    mipi_ui_update_arm_params(p_state);
    mipi_ui_update_arm_servos(p_state);
    mipi_ui_update_arm_status_actions(p_state);
}

/**
 * @brief  构建机械臂调试页
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_arm_debug(const mipi_ui_state_t *p_state)
{
    int32_t x = 0;
    int32_t y = 0;

    g_arm_debug_objs.power_label = mipi_ui_create_top_bar("ARM DEBUG", p_state);
    mipi_ui_label(g_screen, 24, 62, 180, 24, "TARGET", &lv_font_montserrat_20, lv_color_hex(0xE2E8F0U), LV_TEXT_ALIGN_LEFT);

    for (uint32_t i = 0U; i < MIPI_ARM_PARAM_COUNT; i++)
    {
        x = (i < 3U) ? 24 : 246;
        y = 92 + (int32_t)(i % 3U) * 50;
        g_arm_debug_objs.param_row[i] = mipi_ui_rect(g_screen, x, y, 210, 44, lv_color_hex(0x0F172AU), 9);
        g_arm_debug_objs.param_name[i] = mipi_ui_label(g_arm_debug_objs.param_row[i],
                                                       10,
                                                       11,
                                                       58,
                                                       24,
                                                       "",
                                                       &lv_font_montserrat_20,
                                                       lv_color_hex(0xCBD5E1U),
                                                       LV_TEXT_ALIGN_LEFT);
        g_arm_debug_objs.param_value[i] = mipi_ui_label(g_arm_debug_objs.param_row[i],
                                                        70,
                                                        7,
                                                        128,
                                                        30,
                                                        "",
                                                        &lv_font_montserrat_24,
                                                        lv_color_hex(0xE0F2FEU),
                                                        LV_TEXT_ALIGN_RIGHT);
    }

    mipi_ui_label(g_screen, 24, 250, 260, 24, "SERVO TARGET / FEEDBACK", &lv_font_montserrat_20, lv_color_hex(0xE2E8F0U), LV_TEXT_ALIGN_LEFT);
    for (uint32_t i = 0U; i < MECH_HAND_SERVO_COUNT; i++)
    {
        y = 282 + (int32_t)i * 38;
        g_arm_debug_objs.servo_row[i] = mipi_ui_rect(g_screen, 24, y, 432, 34, lv_color_hex(0x0F172AU), 7);
        g_arm_debug_objs.servo_target[i] = mipi_ui_label(g_arm_debug_objs.servo_row[i],
                                                         12,
                                                         7,
                                                         122,
                                                         24,
                                                         "",
                                                         &lv_font_montserrat_20,
                                                         lv_color_hex(0xF8FAFCU),
                                                         LV_TEXT_ALIGN_LEFT);
        g_arm_debug_objs.servo_feedback[i] = mipi_ui_label(g_arm_debug_objs.servo_row[i],
                                                           134,
                                                           7,
                                                           286,
                                                           24,
                                                           "",
                                                           &lv_font_montserrat_20,
                                                           lv_color_hex(0xE0F2FEU),
                                                           LV_TEXT_ALIGN_RIGHT);
    }

    g_arm_debug_objs.status_label = mipi_ui_label(g_screen, 24, 522, 432, 26, "", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_LEFT);
    g_arm_debug_objs.command_label = mipi_ui_label(g_screen, 24, 556, 432, 30, "", &lv_font_montserrat_20, lv_color_hex(0xCBD5E1U), LV_TEXT_ALIGN_LEFT);
    g_arm_debug_objs.action_home = mipi_ui_create_arm_action(24, 642, 96, "HOME");
    g_arm_debug_objs.action_grip = mipi_ui_create_arm_action(136, 642, 96, "GRIP");
    g_arm_debug_objs.action_stop = mipi_ui_create_arm_action(248, 642, 96, "STOP");
    g_arm_debug_objs.action_calib = mipi_ui_create_arm_action(360, 642, 96, "CALIB");
    mipi_ui_create_bottom_hint("PRESS Edit/Run | KEY4 Back | LONG Stop");
    mipi_ui_update_arm_debug(p_state);
}

/**
 * @brief  获取机械臂标定自动流程文本
 *
 * @param state 自动流程状态
 * @return const char* 状态文本
 */
static const char *mipi_ui_arm_calib_flow_text(mipi_arm_calib_flow_state_t state)
{
    switch (state)
    {
        case MIPI_ARM_CALIB_FLOW_IDLE:
            return "IDLE";

        case MIPI_ARM_CALIB_FLOW_OPEN:
            return "OPEN";

        case MIPI_ARM_CALIB_FLOW_ABOVE:
            return "ABOVE";

        case MIPI_ARM_CALIB_FLOW_CLEAR:
            return "CLEAR";

        case MIPI_ARM_CALIB_FLOW_HOME:
            return "HOME";

        case MIPI_ARM_CALIB_FLOW_DONE:
            return "DONE";

        case MIPI_ARM_CALIB_FLOW_STOP:
            return "STOP";

        case MIPI_ARM_CALIB_FLOW_ERR:
            return "ERR";

        default:
            return "--";
    }
}

/**
 * @brief  格式化机械臂标定参数
 *
 * @param p_state  UI 状态
 * @param index    参数索引
 * @param p_text   文本输出缓冲区
 * @param text_len 文本缓冲区长度
 * @return void
 */
static void mipi_ui_format_arm_calib_param(const mipi_ui_state_t *p_state,
                                           uint8_t index,
                                           char *p_text,
                                           size_t text_len)
{
    int32_t value = 0;

    if ((NULL == p_state) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (p_state->editing && (p_state->selected_index == index))
    {
        value = mipi_ui_float_to_scaled(p_state->edit_value, 1.0f);
    }

    switch (index)
    {
        case 0U:
            value = p_state->editing && (p_state->selected_index == index) ? value :
                    ((int32_t)p_state->arm_calib_point_index + 1);
            (void)snprintf(p_text,
                           text_len,
                           "%ld/%u",
                           (long)value,
                           (unsigned int)p_state->arm_calib_point_count);
            break;

        case 1U:
            value = p_state->editing && (p_state->selected_index == index) ? value :
                    mipi_ui_float_to_scaled(p_state->arm_calib_z_mm, 1.0f);
            (void)snprintf(p_text, text_len, "%ld mm", (long)value);
            break;

        default:
            (void)snprintf(p_text, text_len, "--");
            break;
    }
}

/**
 * @brief  更新机械臂标定参数区
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_arm_calib_params(const mipi_ui_state_t *p_state)
{
    static const char *const names[MIPI_ARM_CALIB_PARAM_COUNT] =
    {
        "POINT",
        "Z",
    };
    char text[MIPI_UI_TEXT_LEN] = {0};

    if (NULL == p_state)
    {
        return;
    }

    for (uint32_t i = 0U; i < MIPI_ARM_CALIB_PARAM_COUNT; i++)
    {
        bool selected = mipi_ui_is_selected(p_state, (uint8_t)i);
        bool editing = p_state->editing && selected;

        mipi_ui_label_set_text_if_changed(g_arm_calib_objs.param_name[i], names[i]);
        mipi_ui_format_arm_calib_param(p_state, (uint8_t)i, text, sizeof(text));
        mipi_ui_label_set_text_if_changed(g_arm_calib_objs.param_value[i], text);
        mipi_ui_apply_select_style(g_arm_calib_objs.param_row[i], selected, editing);
        if (NULL != g_arm_calib_objs.param_name[i])
        {
            lv_obj_set_style_text_color(g_arm_calib_objs.param_name[i], selected ? lv_color_hex(0xF8FAFCU) : lv_color_hex(0xCBD5E1U), 0);
        }
        if (NULL != g_arm_calib_objs.param_value[i])
        {
            lv_obj_set_style_text_color(g_arm_calib_objs.param_value[i], editing ? lv_color_hex(0xFDE68AU) : lv_color_hex(0xE0F2FEU), 0);
        }
    }
}

/**
 * @brief  更新机械臂标定状态和动作区
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_arm_calib_status_actions(const mipi_ui_state_t *p_state)
{
    char target_text[MIPI_UI_TEXT_LEN] = {0};
    char above_text[MIPI_UI_TEXT_LEN] = {0};
    char flow_text[MIPI_UI_TEXT_LEN] = {0};
    char status_text[MIPI_UI_TEXT_LEN] = {0};
    char command_text[MIPI_UI_TEXT_LEN] = {0};
    char feedback_text[MIPI_UI_TEXT_LEN] = {0};
    const char *ready_text = NULL;
    const char *safe_text = NULL;
    const char *grip_text = NULL;
    const char *cmd_text = NULL;

    if (NULL == p_state)
    {
        return;
    }

    ready_text = p_state->arm_initialized ? "READY" : "WAIT";
    safe_text = p_state->arm_last_clamped ? "CLAMP" : "OK";
    grip_text = p_state->arm_gripper_opened ? "OPEN" : "CLOSE";
    cmd_text = ('\0' == p_state->arm_last_command[0]) ? "--" : p_state->arm_last_command;

    (void)snprintf(target_text,
                   sizeof(target_text),
                   "P%u/%u  X %ld  Y %ld  Z %ld",
                   (unsigned int)p_state->arm_calib_point_index + 1U,
                   (unsigned int)p_state->arm_calib_point_count,
                   (long)mipi_ui_float_to_scaled(p_state->arm_calib_x_mm, 1.0f),
                   (long)mipi_ui_float_to_scaled(p_state->arm_calib_y_mm, 1.0f),
                   (long)mipi_ui_float_to_scaled(p_state->arm_calib_z_mm, 1.0f));
    (void)snprintf(above_text,
                   sizeof(above_text),
                   "ABOVE X %ld  Y %ld  Z %ld",
                   (long)mipi_ui_float_to_scaled(p_state->arm_calib_x_mm, 1.0f),
                   (long)mipi_ui_float_to_scaled(p_state->arm_calib_y_mm, 1.0f),
                   (long)mipi_ui_float_to_scaled(p_state->arm_calib_above_z_mm, 1.0f));
    (void)snprintf(flow_text,
                   sizeof(flow_text),
                   "FLOW %s  %s",
                   mipi_ui_arm_calib_flow_text(p_state->arm_calib_flow_state),
                   p_state->arm_calib_auto_running ? "RUN" : "IDLE");
    (void)snprintf(status_text,
                   sizeof(status_text),
                   "%s  RET %d  SAFE %s  GRIP %s",
                   ready_text,
                   p_state->arm_last_result,
                   safe_text,
                   grip_text);
    (void)snprintf(command_text, sizeof(command_text), "CMD %s", cmd_text);
    (void)snprintf(feedback_text,
                   sizeof(feedback_text),
                   "FB %s  SEQ %lu  ERR %d",
                   p_state->arm_feedback.initialized ? "READY" : "WAIT",
                   (unsigned long)p_state->arm_feedback.sequence,
                   p_state->arm_feedback.last_error);

    mipi_ui_label_set_text_if_changed(g_arm_calib_objs.target_label, target_text);
    mipi_ui_label_set_text_if_changed(g_arm_calib_objs.above_label, above_text);
    mipi_ui_label_set_text_if_changed(g_arm_calib_objs.flow_label, flow_text);
    mipi_ui_label_set_text_if_changed(g_arm_calib_objs.status_label, status_text);
    mipi_ui_label_set_text_if_changed(g_arm_calib_objs.command_label, command_text);
    mipi_ui_label_set_text_if_changed(g_arm_calib_objs.feedback_label, feedback_text);

    for (uint32_t i = 0U; i < MIPI_ARM_CALIB_ACTION_COUNT; i++)
    {
        uint8_t selected_index = (uint8_t)(MIPI_ARM_CALIB_ACTION_BASE_INDEX + i);

        mipi_ui_apply_select_style(g_arm_calib_objs.action[i],
                                   mipi_ui_is_selected(p_state, selected_index),
                                   false);
    }
}

/**
 * @brief  更新机械臂标定页
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_arm_calib(const mipi_ui_state_t *p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    mipi_ui_update_power_label(g_arm_calib_objs.power_label, p_state);
    mipi_ui_update_arm_calib_params(p_state);
    mipi_ui_update_arm_calib_status_actions(p_state);
}

/**
 * @brief  构建机械臂标定页
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_arm_calib(const mipi_ui_state_t *p_state)
{
    static const char *const action_text[MIPI_ARM_CALIB_ACTION_COUNT] =
    {
        "MOVE",
        "PICK",
        "GRIP",
        "ABMOV",
        "HOME",
        "STOP",
    };
    int32_t x = 0;
    int32_t y = 0;

    g_arm_calib_objs.power_label = mipi_ui_create_top_bar("ARM CALIB", p_state);
    mipi_ui_label(g_screen, 24, 62, 220, 24, "CALIB TARGET", &lv_font_montserrat_20, lv_color_hex(0xE2E8F0U), LV_TEXT_ALIGN_LEFT);

    for (uint32_t i = 0U; i < MIPI_ARM_CALIB_PARAM_COUNT; i++)
    {
        x = (0U == i) ? 24 : 246;
        g_arm_calib_objs.param_row[i] = mipi_ui_rect(g_screen, x, 92, 210, 48, lv_color_hex(0x0F172AU), 9);
        g_arm_calib_objs.param_name[i] = mipi_ui_label(g_arm_calib_objs.param_row[i],
                                                       10,
                                                       12,
                                                       72,
                                                       24,
                                                       "",
                                                       &lv_font_montserrat_20,
                                                       lv_color_hex(0xCBD5E1U),
                                                       LV_TEXT_ALIGN_LEFT);
        g_arm_calib_objs.param_value[i] = mipi_ui_label(g_arm_calib_objs.param_row[i],
                                                        86,
                                                        8,
                                                        112,
                                                        30,
                                                        "",
                                                        &lv_font_montserrat_24,
                                                        lv_color_hex(0xE0F2FEU),
                                                        LV_TEXT_ALIGN_RIGHT);
    }

    mipi_ui_rect(g_screen, 24, 162, 432, 104, lv_color_hex(0x0B1220U), 10);
    g_arm_calib_objs.target_label = mipi_ui_label(g_screen, 40, 184, 400, 26, "", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_LEFT);
    g_arm_calib_objs.above_label = mipi_ui_label(g_screen, 40, 224, 400, 26, "", &lv_font_montserrat_20, lv_color_hex(0xBAE6FDU), LV_TEXT_ALIGN_LEFT);

    mipi_ui_rect(g_screen, 24, 286, 432, 156, lv_color_hex(0x0B1220U), 10);
    g_arm_calib_objs.flow_label = mipi_ui_label(g_screen, 40, 306, 400, 26, "", &lv_font_montserrat_20, lv_color_hex(0xFDE68AU), LV_TEXT_ALIGN_LEFT);
    g_arm_calib_objs.status_label = mipi_ui_label(g_screen, 40, 340, 400, 26, "", &lv_font_montserrat_20, lv_color_hex(0xF8FAFCU), LV_TEXT_ALIGN_LEFT);
    g_arm_calib_objs.command_label = mipi_ui_label(g_screen, 40, 374, 400, 26, "", &lv_font_montserrat_20, lv_color_hex(0xCBD5E1U), LV_TEXT_ALIGN_LEFT);
    g_arm_calib_objs.feedback_label = mipi_ui_label(g_screen, 40, 408, 400, 24, "", &lv_font_montserrat_20, lv_color_hex(0xE0F2FEU), LV_TEXT_ALIGN_LEFT);

    mipi_ui_label(g_screen, 24, 470, 220, 24, "ACTIONS", &lv_font_montserrat_20, lv_color_hex(0xE2E8F0U), LV_TEXT_ALIGN_LEFT);
    for (uint32_t i = 0U; i < MIPI_ARM_CALIB_ACTION_COUNT; i++)
    {
        x = 24 + (int32_t)(i % 3U) * 148;
        y = 506 + (int32_t)(i / 3U) * 62;
        g_arm_calib_objs.action[i] = mipi_ui_create_arm_action(x, y, 136, action_text[i]);
    }

    mipi_ui_create_bottom_hint("PRESS Edit/Run | KEY4 Debug | LONG Stop");
    mipi_ui_update_arm_calib(p_state);
}

/**
 * @brief  格式化 Mission 速度行
 *
 * @param p_snapshot Mission 快照
 * @param p_text     输出文本
 * @param text_len   输出缓冲区长度
 * @return void
 */
static void mipi_ui_format_mission_speed(const mission_snapshot_t *p_snapshot, char *p_text, size_t text_len)
{
    char vx_text[16] = {0};
    char vy_text[16] = {0};
    char wz_text[16] = {0};

    if ((NULL == p_snapshot) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    mipi_ui_format_signed_2dp(p_snapshot->vx_mps, vx_text, sizeof(vx_text));
    mipi_ui_format_signed_2dp(p_snapshot->vy_mps, vy_text, sizeof(vy_text));
    mipi_ui_format_signed_2dp(p_snapshot->wz_radps, wz_text, sizeof(wz_text));
    (void)snprintf(p_text,
                   text_len,
                   "VX %s  VY %s  WZ %s",
                   vx_text,
                   vy_text,
                   wz_text);
}

/**
 * @brief  格式化 Mission 行内容
 *
 * @param p_state  UI 状态
 * @param row      行号
 * @param p_text   输出文本
 * @param text_len 输出缓冲区长度
 * @return void
 */
static void mipi_ui_format_mission_value(const mipi_ui_state_t *p_state, uint8_t row, char *p_text, size_t text_len)
{
    const mission_snapshot_t *p_snapshot = NULL;
    uint16_t frame_code = 0U;
    int32_t value_a = 0;
    int32_t value_b = 0;
    int32_t value_c = 0;

    if ((NULL == p_state) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    p_snapshot = &p_state->mission_snapshot;
    frame_code = (uint16_t)(p_snapshot->latest_frame.data4 + 0.5f);

    switch (row)
    {
        case 0U:
            (void)snprintf(p_text,
                           text_len,
                           "%s  %s  %s",
                           p_snapshot->enabled ? "ON" : "OFF",
                           mipi_ui_mission_state_text(p_snapshot->state),
                           mipi_ui_mission_trace_text(p_snapshot->trace_mode));
            break;

        case 1U:
            mipi_ui_format_mission_speed(p_snapshot, p_text, text_len);
            break;

        case 2U:
            if (p_snapshot->pick_send_valid)
            {
                (void)snprintf(p_text,
                               text_len,
                               "%s  CODE %u  %s  %u%%",
                               mipi_ui_mission_pick_label(p_snapshot->pick_label),
                               (unsigned int)p_snapshot->pick_target_code,
                               p_snapshot->pick_send_ok ? "OK" : "ERR",
                               (unsigned int)p_snapshot->pick_score_percent);
            }
            else
            {
                (void)snprintf(p_text, text_len, "--");
            }
            break;

        case 3U:
            if (p_snapshot->target_valid)
            {
                value_a = mipi_ui_float_to_scaled(p_snapshot->target_u, 1.0f);
                value_b = mipi_ui_float_to_scaled(p_snapshot->target_v, 1.0f);
                value_c = mipi_ui_float_to_scaled(p_snapshot->target_depth, 1.0f);
                (void)snprintf(p_text, text_len, "U %ld  V %ld  D %ld", (long)value_a, (long)value_b, (long)value_c);
            }
            else
            {
                (void)snprintf(p_text, text_len, "--");
            }
            break;

        case 4U:
            if (p_snapshot->target_valid)
            {
                value_a = mipi_ui_float_to_scaled(p_snapshot->target_x_mm, 1.0f);
                value_b = mipi_ui_float_to_scaled(p_snapshot->target_y_mm, 1.0f);
                value_c = mipi_ui_float_to_scaled(p_snapshot->target_z_mm, 1.0f);
                (void)snprintf(p_text, text_len, "X %ld  Y %ld  Z %ld", (long)value_a, (long)value_b, (long)value_c);
            }
            else
            {
                (void)snprintf(p_text, text_len, "--");
            }
            break;

        case 5U:
            (void)snprintf(p_text,
                           text_len,
                           "RX %lu  CODE %u  ACK %s",
                           (unsigned long)p_snapshot->latest_frame.rx_ok_count,
                           (unsigned int)frame_code,
                           p_snapshot->pending_done_ack ? "WAIT" : "OK");
            break;

        case 6U:
            (void)snprintf(p_text,
                           text_len,
                           "ARM %d  GRIP %d  CLAMP %u",
                           p_snapshot->last_arm_result,
                           p_snapshot->last_gripper_result,
                           p_snapshot->was_last_command_clamped ? 1U : 0U);
            break;

        default:
            (void)snprintf(p_text, text_len, "--");
            break;
    }
}

/**
 * @brief  更新 Mission 页面
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_mission(const mipi_ui_state_t *p_state)
{
    static const char *const row_names[MIPI_MISSION_ROW_COUNT] =
    {
        "STATE",
        "SPEED",
        "PICK",
        "IMAGE",
        "BASE",
        "VISION",
        "ARM",
    };
    char value_text[MIPI_UI_TEXT_LEN] = {0};

    if (NULL == p_state)
    {
        return;
    }

    mipi_ui_update_power_label(g_mission_objs.power_label, p_state);
    for (uint32_t i = 0U; i < MIPI_MISSION_ROW_COUNT; i++)
    {
        if (NULL != g_mission_objs.name[i])
        {
            mipi_ui_label_set_text_if_changed(g_mission_objs.name[i], row_names[i]);
        }

        mipi_ui_format_mission_value(p_state, (uint8_t)i, value_text, sizeof(value_text));
        if (NULL != g_mission_objs.value[i])
        {
            mipi_ui_label_set_text_if_changed(g_mission_objs.value[i], value_text);
        }
    }
}

/**
 * @brief  构建 Mission 页面
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_mission(const mipi_ui_state_t *p_state)
{
    int32_t y = 0;
    lv_color_t row_color = lv_color_hex(0x0F172AU);

    g_mission_objs.power_label = mipi_ui_create_top_bar("< MISSION", p_state);

    for (uint32_t i = 0U; i < MIPI_MISSION_ROW_COUNT; i++)
    {
        y = MIPI_MISSION_ROW_Y + ((int32_t)i * (MIPI_MISSION_ROW_H + MIPI_MISSION_ROW_GAP));
        row_color = (0U == i) ? lv_color_hex(0x0B2A3FU) : lv_color_hex(0x0F172AU);
        g_mission_objs.row[i] = mipi_ui_rect(g_screen,
                                             MIPI_MISSION_ROW_X,
                                             y,
                                             MIPI_MISSION_ROW_W,
                                             MIPI_MISSION_ROW_H,
                                             row_color,
                                             8);
        lv_obj_set_style_border_width(g_mission_objs.row[i], 1, 0);
        lv_obj_set_style_border_color(g_mission_objs.row[i],
                                      (0U == i) ? lv_color_hex(0x38BDF8U) : lv_color_hex(0x334155U),
                                      0);
        g_mission_objs.name[i] = mipi_ui_label(g_mission_objs.row[i],
                                               16,
                                               8,
                                               180,
                                               26,
                                               "",
                                               &lv_font_montserrat_20,
                                               lv_color_hex(0x7DD3FCU),
                                               LV_TEXT_ALIGN_LEFT);
        g_mission_objs.value[i] = mipi_ui_label(g_mission_objs.row[i],
                                                16,
                                                38,
                                                412,
                                                32,
                                                "",
                                                &lv_font_montserrat_24,
                                                lv_color_hex(0xF8FAFCU),
                                                LV_TEXT_ALIGN_LEFT);
    }

    mipi_ui_update_mission(p_state);
}

/**
 * @brief  获取语音数据集状态文本
 *
 * @param state 语音采集状态
 * @return const char* 状态文本
 */
static const char *mipi_ui_voice_state_text(voice_dataset_state_t state)
{
    switch (state)
    {
        case VOICE_DATASET_STATE_RECORDING:
            return "RECORDING";

        case VOICE_DATASET_STATE_SENDING:
            return "SENDING";

        case VOICE_DATASET_STATE_DONE:
            return "DONE";

        case VOICE_DATASET_STATE_ERROR:
            return "ERROR";

        case VOICE_DATASET_STATE_IDLE:
        default:
            return "READY";
    }
}

/**
 * @brief  格式化语音数据集流程行状态
 *
 * @param p_status 当前状态
 * @param row       行号
 * @param p_text    输出文本
 * @param text_len  输出缓冲长度
 * @return void
 */
static void mipi_ui_format_voice_flow_status(const voice_dataset_status_t *p_status,
                                             uint8_t row,
                                             char *p_text,
                                             size_t text_len)
{
    if ((NULL == p_status) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    switch (row)
    {
        case 0U:
            (void)snprintf(p_text, text_len, "%s", p_status->label);
            break;

        case 1U:
            if (VOICE_DATASET_STATE_RECORDING == p_status->state)
            {
                (void)snprintf(p_text, text_len, "%u%%", (unsigned int)p_status->progress);
            }
            else if ((VOICE_DATASET_STATE_SENDING == p_status->state) ||
                     (VOICE_DATASET_STATE_DONE == p_status->state))
            {
                (void)snprintf(p_text, text_len, "OK");
            }
            else if (VOICE_DATASET_STATE_ERROR == p_status->state)
            {
                (void)snprintf(p_text, text_len, "ERR");
            }
            else
            {
                (void)snprintf(p_text, text_len, "WAIT");
            }
            break;

        case 2U:
            if (VOICE_DATASET_STATE_SENDING == p_status->state)
            {
                (void)snprintf(p_text, text_len, "TX");
            }
            else if (VOICE_DATASET_STATE_DONE == p_status->state)
            {
                (void)snprintf(p_text, text_len, "OK");
            }
            else if (VOICE_DATASET_STATE_ERROR == p_status->state)
            {
                (void)snprintf(p_text, text_len, "ERR");
            }
            else
            {
                (void)snprintf(p_text, text_len, "WAIT");
            }
            break;

        case 3U:
            if (VOICE_DATASET_STATE_DONE == p_status->state)
            {
                (void)snprintf(p_text, text_len, "SAVED");
            }
            else if (VOICE_DATASET_STATE_ERROR == p_status->state)
            {
                (void)snprintf(p_text, text_len, "CHECK");
            }
            else
            {
                (void)snprintf(p_text, text_len, "PC RX");
            }
            break;

        default:
            (void)snprintf(p_text, text_len, "--");
            break;
    }
}

/**
 * @brief  格式化语音数据集状态行
 *
 * @param p_status 当前状态
 * @param p_text   输出文本
 * @param text_len 输出缓冲长度
 * @return void
 */
static void mipi_ui_format_voice_status(const voice_dataset_status_t *p_status, char *p_text, size_t text_len)
{
    if ((NULL == p_status) || (NULL == p_text) || (0U == text_len))
    {
        return;
    }

    if (VOICE_DATASET_STATE_ERROR == p_status->state)
    {
        (void)snprintf(p_text,
                       text_len,
                       "%s",
                       mipi_ui_voice_state_text(p_status->state));
        return;
    }

    (void)snprintf(p_text,
                   text_len,
                   "%s",
                   mipi_ui_voice_state_text(p_status->state));
}

/**
 * @brief  更新语音数据集操作按钮文本
 *
 * @param p_status 当前状态
 * @return void
 */
static void mipi_ui_update_voice_action(const voice_dataset_status_t *p_status)
{
    const char *title = "START REC";
    const char *subtitle = "PRESS KEY3";
    lv_color_t border_color = lv_color_hex(0x38BDF8U);

    if (NULL == p_status)
    {
        return;
    }

    if (VOICE_DATASET_STATE_RECORDING == p_status->state)
    {
        title = "RECORDING";
        subtitle = "MIC PCM";
        border_color = lv_color_hex(0xFDE68AU);
    }
    else if (VOICE_DATASET_STATE_SENDING == p_status->state)
    {
        title = "SENDING";
        subtitle = "MIC_DATA";
        border_color = lv_color_hex(0xA78BFAU);
    }
    else if (VOICE_DATASET_STATE_DONE == p_status->state)
    {
        title = "SEND OK";
        subtitle = "PRESS KEY3 AGAIN";
        border_color = lv_color_hex(0x86EFACU);
    }
    else if (VOICE_DATASET_STATE_ERROR == p_status->state)
    {
        title = "RETRY";
        subtitle = "CHECK UART";
        border_color = lv_color_hex(0xFB7185U);
    }
    else
    {
        /* IDLE 状态保持默认开始录音文本。 */
    }

    if (NULL != g_voice_dataset_objs.action_button)
    {
        lv_obj_set_style_border_color(g_voice_dataset_objs.action_button, border_color, 0);
    }

    mipi_ui_label_set_text_if_changed(g_voice_dataset_objs.action_title, title);
    mipi_ui_label_set_text_if_changed(g_voice_dataset_objs.action_subtitle, subtitle);
}

/**
 * @brief  更新语音数据集页面
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_voice_dataset(const mipi_ui_state_t *p_state)
{
    static const char *const flow_titles[MIPI_VOICE_FLOW_ROW_COUNT] =
    {
        "1 LABEL",
        "2 RECORD PCM",
        "3 SEND PACKET",
    };
    char text[MIPI_UI_TEXT_LEN] = {0};
    int32_t fill_w = 0;

    if (NULL == p_state)
    {
        return;
    }

    mipi_ui_update_power_label(g_voice_dataset_objs.power_label, p_state);
    for (uint32_t i = 0U; i < MIPI_VOICE_FLOW_ROW_COUNT; i++)
    {
        mipi_ui_label_set_text_if_changed(g_voice_dataset_objs.flow_title[i], flow_titles[i]);
        mipi_ui_format_voice_flow_status(&p_state->voice_dataset_status, (uint8_t)i, text, sizeof(text));
        mipi_ui_label_set_text_if_changed(g_voice_dataset_objs.flow_status[i], text);
    }

    mipi_ui_format_voice_status(&p_state->voice_dataset_status, text, sizeof(text));
    mipi_ui_label_set_text_if_changed(g_voice_dataset_objs.status_label, text);

    fill_w = ((MIPI_VOICE_PROGRESS_W - 8) * (int32_t)p_state->voice_dataset_status.progress) / 100;
    if (NULL != g_voice_dataset_objs.progress_fill)
    {
        lv_obj_set_width(g_voice_dataset_objs.progress_fill, fill_w);
    }

    (void)snprintf(text, sizeof(text), "%u%%", (unsigned int)p_state->voice_dataset_status.progress);
    mipi_ui_label_set_text_if_changed(g_voice_dataset_objs.progress_label, text);
    mipi_ui_update_voice_action(&p_state->voice_dataset_status);
}

/**
 * @brief  构建语音数据集页面
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_voice_dataset(const mipi_ui_state_t *p_state)
{
    lv_obj_t *bar = NULL;
    int32_t y = 0;

    g_voice_dataset_objs.power_label = mipi_ui_create_top_bar("< VOICE", p_state);

    for (uint32_t i = 0U; i < MIPI_VOICE_FLOW_ROW_COUNT; i++)
    {
        y = MIPI_VOICE_FLOW_ROW_Y + ((int32_t)i * (MIPI_VOICE_FLOW_ROW_H + MIPI_VOICE_FLOW_ROW_GAP));
        g_voice_dataset_objs.flow_row[i] = mipi_ui_rect(g_screen,
                                                        MIPI_VOICE_FLOW_ROW_X,
                                                        y,
                                                        MIPI_VOICE_FLOW_ROW_W,
                                                        MIPI_VOICE_FLOW_ROW_H,
                                                        (0U == i) ? lv_color_hex(0x0B2A3FU) : lv_color_hex(0x0F172AU),
                                                        8);
        lv_obj_set_style_border_width(g_voice_dataset_objs.flow_row[i], 1, 0);
        lv_obj_set_style_border_color(g_voice_dataset_objs.flow_row[i],
                                      (0U == i) ? lv_color_hex(0x38BDF8U) : lv_color_hex(0x334155U),
                                      0);
        g_voice_dataset_objs.flow_title[i] = mipi_ui_label(g_voice_dataset_objs.flow_row[i],
                                                           18,
                                                           13,
                                                           150,
                                                           30,
                                                           "",
                                                           &lv_font_montserrat_20,
                                                           lv_color_hex(0x7DD3FCU),
                                                           LV_TEXT_ALIGN_LEFT);
        g_voice_dataset_objs.flow_status[i] = mipi_ui_label(g_voice_dataset_objs.flow_row[i],
                                                            174,
                                                            22,
                                                            248,
                                                            32,
                                                            "",
                                                            &lv_font_montserrat_24,
                                                            lv_color_hex(0xF8FAFCU),
                                                            LV_TEXT_ALIGN_RIGHT);
    }

    g_voice_dataset_objs.status_label = mipi_ui_label(g_screen,
                                                      24,
                                                      376,
                                                      432,
                                                      56,
                                                      "",
                                                      &lv_font_montserrat_24,
                                                      lv_color_hex(0xF8FAFCU),
                                                      LV_TEXT_ALIGN_CENTER);

    bar = mipi_ui_rect(g_screen,
                       MIPI_VOICE_PROGRESS_X,
                       MIPI_VOICE_PROGRESS_Y,
                       MIPI_VOICE_PROGRESS_W,
                       MIPI_VOICE_PROGRESS_H,
                       lv_color_hex(0x172033U),
                       12);
    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_color(bar, lv_color_hex(0x334155U), 0);
    g_voice_dataset_objs.progress_fill = mipi_ui_rect(bar,
                                                      4,
                                                      4,
                                                      0,
                                                      MIPI_VOICE_PROGRESS_H - 8,
                                                      lv_color_hex(0x38BDF8U),
                                                      8);
    g_voice_dataset_objs.progress_label = mipi_ui_label(g_screen,
                                                        0,
                                                        542,
                                                        MIPI_UI_WIDTH,
                                                        28,
                                                        "",
                                                        &lv_font_montserrat_24,
                                                        lv_color_hex(0xE0F2FEU),
                                                        LV_TEXT_ALIGN_CENTER);

    g_voice_dataset_objs.action_button = mipi_ui_rect(g_screen,
                                                      MIPI_VOICE_ACTION_X,
                                                      MIPI_VOICE_ACTION_Y,
                                                      MIPI_VOICE_ACTION_W,
                                                      MIPI_VOICE_ACTION_H,
                                                      lv_color_hex(0x102A43U),
                                                      12);
    lv_obj_set_style_border_width(g_voice_dataset_objs.action_button, 2, 0);
    lv_obj_set_style_border_color(g_voice_dataset_objs.action_button, lv_color_hex(0x38BDF8U), 0);
    g_voice_dataset_objs.action_title = mipi_ui_label(g_voice_dataset_objs.action_button,
                                                      0,
                                                      16,
                                                      MIPI_VOICE_ACTION_W,
                                                      32,
                                                      "",
                                                      &lv_font_montserrat_24,
                                                      lv_color_hex(0xF8FAFCU),
                                                      LV_TEXT_ALIGN_CENTER);
    g_voice_dataset_objs.action_subtitle = mipi_ui_label(g_voice_dataset_objs.action_button,
                                                         0,
                                                         52,
                                                         MIPI_VOICE_ACTION_W,
                                                         22,
                                                         "",
                                                         &lv_font_montserrat_20,
                                                         lv_color_hex(0xBAE6FDU),
                                                         LV_TEXT_ALIGN_CENTER);

    mipi_ui_create_bottom_hint("KNOB LABEL | PRESS REC | KEY4 BACK");
    mipi_ui_update_voice_dataset(p_state);
}

/**
 * @brief  构建当前视图
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_build_view(const mipi_ui_state_t *p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    switch (p_state->view)
    {
        case MIPI_UI_VIEW_BOOT:
            mipi_ui_build_boot(p_state);
            break;

        case MIPI_UI_VIEW_HOME:
            mipi_ui_build_home(p_state);
            break;

        case MIPI_UI_VIEW_CHASSIS:
            mipi_ui_build_chassis(p_state);
            break;

        case MIPI_UI_VIEW_RADAR_CLOUD:
            mipi_ui_build_radar_cloud(p_state);
            break;

        case MIPI_UI_VIEW_ARM_DEBUG:
            mipi_ui_build_arm_debug(p_state);
            break;

        case MIPI_UI_VIEW_ARM_CALIB:
            mipi_ui_build_arm_calib(p_state);
            break;

        case MIPI_UI_VIEW_MISSION:
            mipi_ui_build_mission(p_state);
            break;

        case MIPI_UI_VIEW_VOICE_DATASET:
            mipi_ui_build_voice_dataset(p_state);
            break;

        case MIPI_UI_VIEW_PLACEHOLDER:
            mipi_ui_build_placeholder(p_state);
            break;

        default:
            mipi_ui_build_home(p_state);
            break;
    }
}

/**
 * @brief  更新当前视图
 *
 * @param p_state UI 状态
 * @return void
 */
static void mipi_ui_update_view(const mipi_ui_state_t *p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    switch (p_state->view)
    {
        case MIPI_UI_VIEW_BOOT:
            mipi_ui_update_boot(p_state);
            break;

        case MIPI_UI_VIEW_HOME:
            mipi_ui_update_home(p_state);
            break;

        case MIPI_UI_VIEW_CHASSIS:
            mipi_ui_update_chassis(p_state);
            break;

        case MIPI_UI_VIEW_RADAR_CLOUD:
            mipi_ui_update_radar_cloud(p_state);
            break;

        case MIPI_UI_VIEW_ARM_DEBUG:
            mipi_ui_update_arm_debug(p_state);
            break;

        case MIPI_UI_VIEW_ARM_CALIB:
            mipi_ui_update_arm_calib(p_state);
            break;

        case MIPI_UI_VIEW_MISSION:
            mipi_ui_update_mission(p_state);
            break;

        case MIPI_UI_VIEW_VOICE_DATASET:
            mipi_ui_update_voice_dataset(p_state);
            break;

        case MIPI_UI_VIEW_PLACEHOLDER:
            mipi_ui_update_placeholder(p_state);
            break;

        default:
            break;
    }
}

/**
 * @brief  初始化 MIPI 竖屏 UI 对象
 *
 * @param p_state 初始 UI 状态
 * @return fsp_err_t FSP_SUCCESS 表示初始化成功
 */
fsp_err_t mipi_ui_init(const mipi_ui_state_t *p_state)
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

#if MIPI_UI_DIAG_SOLID_TEST
    mipi_ui_build_diag_solid();
    return FSP_SUCCESS;
#else
    mipi_ui_clear_screen();
    mipi_ui_build_view(p_state);
    g_current_view = p_state->view;
    g_ui_built = true;

    return FSP_SUCCESS;
#endif
}

/**
 * @brief  按当前状态刷新 MIPI 竖屏 UI
 *
 * @param p_state 当前 UI 状态
 * @return void
 */
void mipi_ui_update(const mipi_ui_state_t *p_state)
{
#if MIPI_UI_DIAG_SOLID_TEST
    FSP_PARAMETER_NOT_USED(p_state);
    return;
#else
    if ((NULL == p_state) || (NULL == g_screen))
    {
        return;
    }

    if ((!g_ui_built) || (g_current_view != p_state->view))
    {
        mipi_ui_clear_screen();
        mipi_ui_build_view(p_state);
        g_current_view = p_state->view;
        g_ui_built = true;
        return;
    }

    mipi_ui_update_view(p_state);
#endif
}
