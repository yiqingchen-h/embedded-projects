#include "software/ui/lcd_show.h"

#include "FreeRTOS.h"
#include "task.h"
#include "software/ui/ui_app.h"
#include "software/ui/ui_port.h"

#if UI_PORT_USE_MIPI_LCD
#include "hardware/Chassis/Chassis.h"
#include "hardware/Chassis/chassis_avoidance.h"
#include "hardware/Chassis/chassis_param.h"
#include "hardware/Chassis/encoder.h"
#include "hardware/Chassis/kinematics.h"
#include "hardware/INA226/ina226.h"
#include "hardware/Microphone/microphone.h"
#include "hardware/N10/n10_radar.h"
#include "hardware/key/key.h"
#include "software/display_speed_control.h"
#include "software/mission_task.h"
#include "software/voice_ai.h"
#include "software/voice_dataset_capture.h"
#include "hardware/mechanical_arm/Mechanical_arm_feedback.h"
#include "software/ui/mipi_ui.h"
typedef mipi_ui_state_t boot_ui_state_t;
#define boot_ui_init  mipi_ui_init
#define boot_ui_update mipi_ui_update
#else
#include "hardware/Chassis/Chassis.h"
#include "hardware/Chassis/chassis_param.h"
#include "hardware/Chassis/encoder.h"
#include "hardware/INA226/ina226.h"
#include "hardware/key/key.h"
#include "software/display_speed_control.h"
#include "software/ui/st7789_ui.h"
typedef st7789_ui_state_t boot_ui_state_t;
#define boot_ui_init  st7789_ui_init
#define boot_ui_update st7789_ui_update
#endif

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#if UI_PORT_USE_MIPI_LCD
/* MIPI LVGL 刷新周期，40ms 约等于 26Hz，降低竖屏调参页刷新压力。 */
#define LCD_SHOW_REFRESH_PERIOD_MS   (40U)
#else
/* SPI LVGL 刷新周期，33ms 约等于 30Hz，保持旧屏原行为。 */
#define LCD_SHOW_REFRESH_PERIOD_MS   (33U)
#endif

/* 开机页自动进度总时长，单位 ms。 */
#define LCD_SHOW_BOOT_AUTO_TOTAL_MS  (2000U)

#if !UI_PORT_USE_MIPI_LCD
/* SPI 顶部电源状态栏刷新周期，单位 ms。 */
#define LCD_SHOW_POWER_REFRESH_PERIOD_MS (100U)

/* SPI 功能页编码器单次事件调整步进。 */
#define LCD_SHOW_ENCODER_VALUE_STEP      (1)

/* 麦轮车体平移速度编辑步进，单位 m/s。 */
#define LCD_SHOW_MECANUM_V_STEP_MPS      (0.05f)

/* 麦轮车体旋转速度编辑步进，单位 rad/s。 */
#define LCD_SHOW_MECANUM_WZ_STEP_RADPS   (0.10f)

/* 麦轮单轮速度编辑步进，单位 m/s。 */
#define LCD_SHOW_MECANUM_WHEEL_STEP_MPS  (0.05f)

/* 麦轮 PID 参数编辑步进。 */
#define LCD_SHOW_MECANUM_PID_STEP        (5.0f)

/* 麦轮 PWM 计数值编辑步进。 */
#define LCD_SHOW_MECANUM_PWM_STEP        (1000.0f)

/* PWM 界面四轮同步开关。置 1 时编辑任一车轮的 PWM 值，四个轮子同步设为相同值。 */
#define LCD_SHOW_PWM_SYNC_ALL            (1)

/* 麦轮实测轮速显示刷新阈值，单位 m/s。 */
#define LCD_SHOW_MECANUM_SPEED_EPS_MPS   (0.001f)

#endif

#if UI_PORT_USE_MIPI_LCD
/* MIPI 顶部电源状态栏刷新周期，单位 ms。 */
#define LCD_SHOW_POWER_REFRESH_PERIOD_MS (100U)

/* MIPI 电压显示刷新阈值，单位 mV。 */
#define LCD_SHOW_MIPI_POWER_VOLTAGE_EPS_MV (50U)

/* MIPI 电流显示刷新阈值，单位 uA。 */
#define LCD_SHOW_MIPI_POWER_CURRENT_EPS_UA (20000U)

/* MIPI 编码器反馈读取周期，单位 ms。 */
#define LCD_SHOW_MIPI_FEEDBACK_REFRESH_PERIOD_MS (50U)

/* MIPI 雷达点云读取周期，单位 ms。 */
#define LCD_SHOW_MIPI_RADAR_REFRESH_PERIOD_MS    (100U)

/* MIPI 机械臂反馈快照读取周期，单位 ms。 */
#define LCD_SHOW_MIPI_ARM_REFRESH_PERIOD_MS      (40U)

/* MIPI Mission 快照读取周期，单位 ms。 */
#define LCD_SHOW_MIPI_MISSION_REFRESH_PERIOD_MS  (100U)

/* MIPI 语音数据集状态读取周期，单位 ms。 */
#define LCD_SHOW_MIPI_VOICE_REFRESH_PERIOD_MS    (50U)

/* MIPI 顶栏语音 AI 识别结果读取周期，单位 ms。 */
#define LCD_SHOW_MIPI_VOICE_AI_REFRESH_PERIOD_MS (50U)

/* 机械臂坐标编辑步进，单位 mm。 */
#define LCD_SHOW_ARM_COORD_STEP_MM               (1.0f)

/* 机械臂动作时间编辑步进，单位 ms。 */
#define LCD_SHOW_ARM_TIME_STEP_MS                (50.0f)

/* 机械臂动作最短时间，单位 ms。 */
#define LCD_SHOW_ARM_TIME_MIN_MS                 (100U)

/* 机械臂单舵机 PWM 编辑步进。 */
#define LCD_SHOW_ARM_PWM_STEP                    (10.0f)

/* 机械臂页面动作项起始索引。 */
#define LCD_SHOW_ARM_ACTION_BASE_INDEX           (6U)

/* 机械臂页面选中项数量。 */
#define LCD_SHOW_ARM_ITEM_COUNT                  (10U)

/* 机械臂标定页预设点数量。 */
#define LCD_SHOW_ARM_CALIB_POINT_COUNT           (20U)

/* 机械臂标定页目标上方高度偏移，单位 mm。 */
#define LCD_SHOW_ARM_CALIB_ABOVE_OFFSET_MM       (50.0f)

/* 机械臂标定页左右半区避障横向偏移，单位 mm。 */
#define LCD_SHOW_ARM_CALIB_SIDE_CLEAR_OFFSET_MM  (50.0f)

/* 机械臂标定页中线避障后退偏移，单位 mm。 */
#define LCD_SHOW_ARM_CALIB_CENTER_BACK_OFFSET_MM (20.0f)

/* 机械臂标定自动流程动作完成后的等待余量，单位 ms。 */
#define LCD_SHOW_ARM_CALIB_ACTION_SETTLE_MS      (500U)

/* 机械臂标定页可编辑参数数量。 */
#define LCD_SHOW_ARM_CALIB_PARAM_COUNT           (2U)

/* 机械臂标定页动作项起始索引。 */
#define LCD_SHOW_ARM_CALIB_ACTION_BASE_INDEX     (2U)

/* 机械臂标定页选中项数量。 */
#define LCD_SHOW_ARM_CALIB_ITEM_COUNT            (8U)

/* UI 动作等待机械臂总线互斥锁的超时时间，单位 ms。 */
#define LCD_SHOW_ARM_BUS_LOCK_TIMEOUT_MS         (500U)

/* 麦轮车体平移速度编辑步进，单位 m/s。 */
#define LCD_SHOW_MECANUM_V_STEP_MPS      (0.05f)

/* 麦轮车体旋转速度编辑步进，单位 rad/s。 */
#define LCD_SHOW_MECANUM_WZ_STEP_RADPS   (0.10f)

/* 麦轮单轮速度编辑步进，单位 m/s。 */
#define LCD_SHOW_MECANUM_WHEEL_STEP_MPS  (0.05f)

/* 麦轮 PID 参数编辑步进。 */
#define LCD_SHOW_MECANUM_PID_STEP        (5.0f)

/* 麦轮 PWM 计数值编辑步进。 */
#define LCD_SHOW_MECANUM_PWM_STEP        (1000.0f)

/* PWM 界面四轮同步开关。置 1 时编辑任一车轮的 PWM 值，四个轮子同步设为相同值。 */
#define LCD_SHOW_PWM_SYNC_ALL            (1)

/* 麦轮实测轮速显示刷新阈值，单位 m/s。 */
#define LCD_SHOW_MECANUM_SPEED_EPS_MPS   (0.01f)
#endif

/* 开机页进度，范围 0~100。 */
static uint8_t g_lcd_show_boot_progress = 0U;

/* LVGL 刷新累计时间，单位 ms。 */
static uint32_t g_lcd_show_refresh_ms = 0U;

/* 开机页自动进度累计时间，单位 ms。 */
static uint32_t g_lcd_show_boot_auto_ms = 0U;

/* LCD UI 初始化完成标志。 */
static bool g_lcd_show_initialized = false;

/* LCD UI 脏标志，true 表示下一次刷新需要更新 LVGL 对象。 */
static bool g_lcd_show_dirty = true;

#if UI_PORT_USE_MIPI_LCD
typedef enum e_lcd_show_mipi_chassis_action
{
    /* 不需要执行底盘命令。 */
    LCD_SHOW_MIPI_CHASSIS_ACTION_NONE = 0,

    /* 应用当前底盘控制值。 */
    LCD_SHOW_MIPI_CHASSIS_ACTION_APPLY,

    /* 保存 PID 参数。 */
    LCD_SHOW_MIPI_CHASSIS_ACTION_SAVE_PID,

    /* 清零当前模式目标。 */
    LCD_SHOW_MIPI_CHASSIS_ACTION_ZERO,

    /* 停止底盘。 */
    LCD_SHOW_MIPI_CHASSIS_ACTION_STOP,
} lcd_show_mipi_chassis_action_t;

typedef enum e_lcd_show_mipi_arm_action
{
    /* 不需要执行机械臂命令。 */
    LCD_SHOW_MIPI_ARM_ACTION_NONE = 0,

    /* 按夹爪中心 TCP 坐标执行移动。 */
    LCD_SHOW_MIPI_ARM_ACTION_MOVE_TCP,

    /* 执行单舵机 PWM 调试命令。 */
    LCD_SHOW_MIPI_ARM_ACTION_SERVO_PWM,

    /* 切换夹爪开合。 */
    LCD_SHOW_MIPI_ARM_ACTION_GRIP,

    /* 显式执行 Home。 */
    LCD_SHOW_MIPI_ARM_ACTION_HOME,

    /* 停止全部舵机。 */
    LCD_SHOW_MIPI_ARM_ACTION_STOP,

    /* 进入机械臂标定辅助页。 */
    LCD_SHOW_MIPI_ARM_ACTION_CALIB,
} lcd_show_mipi_arm_action_t;

typedef enum e_lcd_show_mipi_arm_calib_action
{
    /* 不需要执行机械臂标定命令。 */
    LCD_SHOW_MIPI_ARM_CALIB_ACTION_NONE = 0,

    /* 移动到当前标定点。 */
    LCD_SHOW_MIPI_ARM_CALIB_ACTION_MOVE,

    /* 移动到当前标定点上方。 */
    LCD_SHOW_MIPI_ARM_CALIB_ACTION_ABOVE,

    /* 回 Home。 */
    LCD_SHOW_MIPI_ARM_CALIB_ACTION_HOME,

    /* 只切换夹爪开合。 */
    LCD_SHOW_MIPI_ARM_CALIB_ACTION_GRIP,

    /* 执行自动松夹爪、上移、避障平移、回 Home 流程。 */
    LCD_SHOW_MIPI_ARM_CALIB_ACTION_PICK,

    /* 停止全部舵机并取消自动流程。 */
    LCD_SHOW_MIPI_ARM_CALIB_ACTION_STOP,
} lcd_show_mipi_arm_calib_action_t;

typedef enum e_lcd_show_mipi_arm_calib_pick_state
{
    /* 自动流程空闲。 */
    LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE = 0,

    /* 等待松开夹爪动作完成。 */
    LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_OPEN,

    /* 下发移动到目标上方动作。 */
    LCD_SHOW_MIPI_ARM_CALIB_PICK_ISSUE_ABOVE,

    /* 等待移动到目标上方动作完成。 */
    LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_ABOVE,

    /* 下发避障平移动作。 */
    LCD_SHOW_MIPI_ARM_CALIB_PICK_ISSUE_CLEAR,

    /* 等待避障平移动作完成。 */
    LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_CLEAR,

    /* 下发回 Home 动作。 */
    LCD_SHOW_MIPI_ARM_CALIB_PICK_ISSUE_HOME,

    /* 等待回 Home 动作完成。 */
    LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_HOME,
} lcd_show_mipi_arm_calib_pick_state_t;

typedef struct st_lcd_show_arm_calib_point
{
    /* 预设 X 坐标，单位 mm。 */
    float x_mm;

    /* 预设 Y 坐标，单位 mm。 */
    float y_mm;
} lcd_show_arm_calib_point_t;

/* 机械臂标定页固定 X/Y 点表，按 Y 分组且 X 从小到大排序。 */
static const lcd_show_arm_calib_point_t g_mipi_arm_calib_points[LCD_SHOW_ARM_CALIB_POINT_COUNT] =
{
    {-70.0f, 150.0f},
    {-35.0f, 150.0f},
    {0.0f, 150.0f},
    {35.0f, 150.0f},
    {70.0f, 150.0f},
    {-70.0f, 180.0f},
    {-35.0f, 180.0f},
    {0.0f, 180.0f},
    {35.0f, 180.0f},
    {70.0f, 180.0f},
    {-70.0f, 210.0f},
    {-35.0f, 210.0f},
    {0.0f, 210.0f},
    {35.0f, 210.0f},
    {70.0f, 210.0f},
    {-70.0f, 250.0f},
    {-35.0f, 250.0f},
    {0.0f, 250.0f},
    {35.0f, 250.0f},
    {70.0f, 250.0f},
};

/* MIPI 当前视图。 */
static mipi_ui_view_t g_mipi_view = MIPI_UI_VIEW_BOOT;

/* MIPI 首页当前选中项。 */
static mipi_home_page_t g_mipi_home_selected = MIPI_HOME_PAGE_MISSION;

/* MIPI 占位页当前功能项。 */
static mipi_home_page_t g_mipi_placeholder_page = MIPI_HOME_PAGE_CHASSIS;

/* MIPI 底盘当前控制模式。 */
static mipi_chassis_mode_t g_mipi_chassis_mode = MIPI_CHASSIS_MODE_BODY;

/* MIPI 当前页面选中项索引。 */
static uint8_t g_mipi_selected_index = 0U;

/* MIPI 当前选中项是否处于编辑态。 */
static bool g_mipi_editing = false;

/* MIPI 编辑中的临时值。 */
static float g_mipi_edit_value = 0.0f;

/* MIPI 进入编辑前的原始值。 */
static float g_mipi_edit_original_value = 0.0f;

/* MIPI 底盘控制是否已启用。 */
static bool g_mipi_chassis_control_enabled = false;

/* MIPI 电源采样是否有效。 */
static bool g_mipi_power_valid = false;

/* MIPI 母线电压缓存，单位 mV。 */
static uint32_t g_mipi_bus_voltage_mv = 0U;

/* MIPI 电流缓存，单位 uA。 */
static int32_t g_mipi_current_ua = 0;

/* MIPI 电源刷新累计时间，单位 ms。 */
static uint32_t g_mipi_power_refresh_ms = 0U;

/* MIPI 语音 AI 识别结果是否有效。 */
static bool g_mipi_voice_ai_valid = false;

/* MIPI 语音 AI 识别类别索引。 */
static uint8_t g_mipi_voice_ai_index = 0U;

/* MIPI 语音 AI 识别分数百分比，范围 0~100。 */
static uint8_t g_mipi_voice_ai_score_percent = 0U;

/* MIPI 语音 AI 识别类别名称。 */
static char g_mipi_voice_ai_label[MIPI_UI_VOICE_AI_LABEL_LEN] = {0};

/* MIPI 语音 AI 识别结果读取累计时间，单位 ms。 */
static uint32_t g_mipi_voice_ai_refresh_ms = 0U;

/* MIPI 编码器反馈刷新累计时间，单位 ms。 */
static uint32_t g_mipi_feedback_refresh_ms = 0U;

/* MIPI 雷达点云刷新累计时间，单位 ms。 */
static uint32_t g_mipi_radar_refresh_ms = 0U;

/* MIPI 车体前进速度，单位 m/s。 */
static float g_mipi_vx_mps = 0.0f;

/* MIPI 车体横移速度，单位 m/s。 */
static float g_mipi_vy_mps = 0.0f;

/* MIPI 车体旋转角速度，单位 rad/s。 */
static float g_mipi_wz_radps = 0.0f;

/* MIPI 四路轮速目标，单位 m/s。 */
static float g_mipi_wheel_target_mps[CHASSIS_WHEEL_COUNT] = {0.0f};

/* MIPI 四路轮速实测值，单位 m/s。 */
static float g_mipi_wheel_measured_mps[CHASSIS_WHEEL_COUNT] = {0.0f};

/* MIPI 四路电机转速，单位 rpm。 */
static float g_mipi_wheel_rpm[CHASSIS_WHEEL_COUNT] = {0.0f};

/* MIPI 四路开环 PWM 计数值。 */
static int32_t g_mipi_pwm_counts[CHASSIS_WHEEL_COUNT] = {0};

/* MIPI 底盘 PID 比例系数。 */
static float g_mipi_kp = CHASSIS_PID_DEFAULT_KP;

/* MIPI 底盘 PID 积分系数。 */
static float g_mipi_ki = CHASSIS_PID_DEFAULT_KI;

/* MIPI PID 保存状态。 */
static mipi_pid_save_status_t g_mipi_pid_save_status = MIPI_PID_SAVE_NONE;

/* MIPI 雷达完整点云是否有效。 */
static bool g_mipi_radar_cloud_valid = false;

/* MIPI 雷达完整点云序号。 */
static uint32_t g_mipi_radar_cloud_sequence = 0U;

/* MIPI 雷达完整点云来源原始帧序号。 */
static uint32_t g_mipi_radar_source_scan_sequence = 0U;

/* MIPI 雷达转速，单位 rpm。 */
static float g_mipi_radar_rpm = 0.0f;

/* MIPI 雷达 72 扇区最近距离，单位 mm。 */
static uint16_t g_mipi_radar_sector_min_mm[N10_RADAR_CLOUD_SECTOR_COUNT] = {0U};

/* MIPI 雷达 72 扇区是否有效。 */
static uint8_t g_mipi_radar_sector_valid[N10_RADAR_CLOUD_SECTOR_COUNT] = {0U};

/* MIPI 雷达最近有效距离，单位 mm；0 表示无有效点。 */
static uint16_t g_mipi_radar_nearest_distance_mm = 0U;

/* MIPI 避障状态。 */
static chassis_avoidance_state_t g_mipi_avoidance_state = CHASSIS_AVOIDANCE_STATE_NORMAL;

/* MIPI 避障雷达未更新时长，单位 ms。 */
static uint32_t g_mipi_avoidance_radar_stale_ms = 0U;

/* MIPI 避障上层目标方向，单位度。 */
static float g_mipi_avoidance_target_angle_deg = 0.0f;

/* MIPI 避障最终选择方向，单位度。 */
static float g_mipi_avoidance_selected_angle_deg = 0.0f;

/* MIPI 避障目标方向是否安全。 */
static bool g_mipi_avoidance_target_safe = false;

/* MIPI 机械臂通信是否已初始化。 */
static bool g_mipi_arm_initialized = false;

/* MIPI 机械臂调试目标 X，单位 mm。 */
static float g_mipi_arm_x_mm = MECH_HAND_HOME_X;

/* MIPI 机械臂调试目标 Y，单位 mm。 */
static float g_mipi_arm_y_mm = MECH_HAND_HOME_Y;

/* MIPI 机械臂调试目标 Z，单位 mm。 */
static float g_mipi_arm_z_mm = MECH_HAND_HOME_Z;

/* MIPI 机械臂动作时间，单位 ms。 */
static uint16_t g_mipi_arm_time_ms = 1000U;

/* MIPI 机械臂单舵机调试序号。 */
static uint8_t g_mipi_arm_servo_index = 0U;

/* MIPI 机械臂单舵机调试目标 PWM。 */
static uint16_t g_mipi_arm_servo_pwm = 1500U;

/* MIPI 机械臂夹爪打开状态。 */
static bool g_mipi_arm_gripper_opened = false;

/* MIPI 机械臂最近一次动作返回码。 */
static int g_mipi_arm_last_result = MECH_HAND_OK;

/* MIPI 机械臂最近一次结构化命令是否触发限位裁剪。 */
static bool g_mipi_arm_last_clamped = false;

/* MIPI 机械臂最近一次命令短文本。 */
static char g_mipi_arm_last_command[MIPI_UI_ARM_LAST_COMMAND_LEN] = {0};

/* MIPI 机械臂 6 路目标 PWM 缓存。 */
static mech_hand_servo_positions_t g_mipi_arm_target_positions = {{1500U, 1500U, 1500U, 1500U, 1500U, 1500U}};

/* MIPI 机械臂独立反馈任务快照。 */
static mechanical_arm_feedback_snapshot_t g_mipi_arm_feedback = {0};

/* MIPI 机械臂反馈快照读取累计时间，单位 ms。 */
static uint32_t g_mipi_arm_refresh_ms = 0U;

/* MIPI 机械臂最近已显示反馈序号。 */
static uint32_t g_mipi_arm_feedback_sequence = 0U;

/* MIPI 机械臂标定页当前预设点序号，0 表示第一组。 */
static uint8_t g_mipi_arm_calib_point_index = 0U;

/* MIPI 机械臂标定页目标 Z，单位 mm。 */
static float g_mipi_arm_calib_z_mm = MECH_HAND_HOME_Z;

/* MIPI 机械臂标定自动流程内部状态。 */
static lcd_show_mipi_arm_calib_pick_state_t g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE;

/* MIPI 机械臂标定自动流程显示状态。 */
static mipi_arm_calib_flow_state_t g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_IDLE;

/* MIPI 机械臂标定自动流程等待累计时间，单位 ms。 */
static uint32_t g_mipi_arm_calib_wait_ms = 0U;

/* MIPI 机械臂标定自动流程当前等待总时间，单位 ms。 */
static uint32_t g_mipi_arm_calib_wait_total_ms = 0U;

/* MIPI Mission 快照缓存。 */
static mission_snapshot_t g_mipi_mission_snapshot = {0};

/* MIPI Mission 快照读取累计时间，单位 ms。 */
static uint32_t g_mipi_mission_refresh_ms = 0U;

/* MIPI 语音数据集状态缓存。 */
static voice_dataset_status_t g_mipi_voice_status =
{
    .state = VOICE_DATASET_STATE_IDLE,
    .progress = 0U,
    .last_error = FSP_SUCCESS,
    .sample_rate_hz = MICROPHONE_SAMPLE_RATE_HZ,
    .samples = 0U,
    .label = VOICE_DATASET_DEFAULT_LABEL,
};

/* MIPI 语音数据集状态读取累计时间，单位 ms。 */
static uint32_t g_mipi_voice_refresh_ms = 0U;

/* MIPI 语音数据集当前选择标签索引，默认 0 对应 wake。 */
static uint8_t g_mipi_voice_label_index = 0U;
#endif

#if !UI_PORT_USE_MIPI_LCD
typedef enum e_lcd_show_spi_chassis_action
{
    /* 不需要执行底盘命令。 */
    LCD_SHOW_SPI_CHASSIS_ACTION_NONE = 0,

    /* 应用整体车体速度。 */
    LCD_SHOW_SPI_CHASSIS_ACTION_BODY,

    /* 应用四轮独立速度。 */
    LCD_SHOW_SPI_CHASSIS_ACTION_WHEEL,

    /* 保存并应用 PID 参数。 */
    LCD_SHOW_SPI_CHASSIS_ACTION_SAVE_PID,

    /* 应用四路开环 PWM 诊断值。 */
    LCD_SHOW_SPI_CHASSIS_ACTION_PWM,

    /* 停止底盘。 */
    LCD_SHOW_SPI_CHASSIS_ACTION_STOP,
} lcd_show_spi_chassis_action_t;

typedef struct st_lcd_show_spi_chassis_command
{
    /* 待执行命令类型。 */
    lcd_show_spi_chassis_action_t action;

    /* 整体前进速度，单位 m/s。 */
    float vx_mps;

    /* 整体横移速度，单位 m/s。 */
    float vy_mps;

    /* 整体旋转角速度，单位 rad/s。 */
    float wz_radps;

    /* 四路轮速目标，单位 m/s。 */
    float wheel_speed_mps[CHASSIS_WHEEL_COUNT];

    /* 四路开环 PWM 计数值。 */
    int32_t pwm_counts[CHASSIS_WHEEL_COUNT];

    /* 待保存或应用的 PID 参数。 */
    chassis_pid_params_t pid_params;
} lcd_show_spi_chassis_command_t;

/* 麦轮整体控制子界面项目顺序。 */
static const st7789_ui_mecanum_item_t g_lcd_show_mecanum_body_items[] =
{
    ST7789_UI_MECANUM_ITEM_MODE,
    ST7789_UI_MECANUM_ITEM_VX,
    ST7789_UI_MECANUM_ITEM_VY,
    ST7789_UI_MECANUM_ITEM_WZ,
    ST7789_UI_MECANUM_ITEM_STOP,
};

/* 麦轮单轮速度控制子界面项目顺序。 */
static const st7789_ui_mecanum_item_t g_lcd_show_mecanum_wheel_items[] =
{
    ST7789_UI_MECANUM_ITEM_MODE,
    ST7789_UI_MECANUM_ITEM_W1,
    ST7789_UI_MECANUM_ITEM_W2,
    ST7789_UI_MECANUM_ITEM_W3,
    ST7789_UI_MECANUM_ITEM_W4,
    ST7789_UI_MECANUM_ITEM_STOP,
};

/* 麦轮 PID 调参子界面项目顺序。 */
static const st7789_ui_mecanum_item_t g_lcd_show_mecanum_pid_items[] =
{
    ST7789_UI_MECANUM_ITEM_MODE,
    ST7789_UI_MECANUM_ITEM_KP,
    ST7789_UI_MECANUM_ITEM_KI,
};

/* 麦轮 PWM 诊断控制子界面项目顺序。 */
static const st7789_ui_mecanum_item_t g_lcd_show_mecanum_pwm_items[] =
{
    ST7789_UI_MECANUM_ITEM_MODE,
    ST7789_UI_MECANUM_ITEM_W1,
    ST7789_UI_MECANUM_ITEM_W2,
    ST7789_UI_MECANUM_ITEM_W3,
    ST7789_UI_MECANUM_ITEM_W4,
    ST7789_UI_MECANUM_ITEM_STOP,
};

/* SPI 当前显示视图。 */
static st7789_ui_view_t g_lcd_show_view = ST7789_UI_VIEW_BOOT;

/* SPI 菜单当前选中项。 */
static uint8_t g_lcd_show_selected_index = 0U;

/* SPI 当前进入的功能页。 */
static st7789_ui_page_t g_lcd_show_current_page = ST7789_UI_PAGE_MECANUM_PID;

/* SPI 各功能页占位调试数值。 */
static int32_t g_lcd_show_page_values[ST7789_UI_PAGE_COUNT] = {0};

/* SPI 功能页编辑状态。 */
static bool g_lcd_show_editing = false;

/* SPI 功能页编辑中的临时数值。 */
static int32_t g_lcd_show_edit_value = 0;

/* SPI 电源状态有效标志。 */
static bool g_lcd_show_power_valid = false;

/* SPI 母线电压缓存，单位 mV。 */
static uint32_t g_lcd_show_bus_voltage_mv = 0U;

/* SPI 电流缓存，单位 uA。 */
static int32_t g_lcd_show_current_ua = 0;

/* SPI 电源状态栏刷新累计时间，单位 ms。 */
static uint32_t g_lcd_show_power_refresh_ms = 0U;

/* 麦轮调试当前控制模式。 */
static st7789_ui_mecanum_mode_t g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_BODY;

/* 麦轮调试当前选中项目。 */
static uint8_t g_lcd_show_mecanum_selected_item = 0U;

/* 麦轮调试编辑状态。 */
static bool g_lcd_show_mecanum_editing = false;

/* 麦轮调试编辑中的临时值。 */
static float g_lcd_show_mecanum_edit_value = 0.0f;

/* 麦轮调试整体前进速度，单位 m/s。 */
static float g_lcd_show_mecanum_vx_mps = 0.0f;

/* 麦轮调试整体横移速度，单位 m/s。 */
static float g_lcd_show_mecanum_vy_mps = 0.0f;

/* 麦轮调试整体旋转角速度，单位 rad/s。 */
static float g_lcd_show_mecanum_wz_radps = 0.0f;

/* 麦轮调试四路轮速，单位 m/s。 */
static float g_lcd_show_mecanum_wheel_speed_mps[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* 麦轮调试四路开环 PWM 计数值。 */
static int32_t g_lcd_show_mecanum_pwm_counts[CHASSIS_WHEEL_COUNT] = {0, 0, 0, 0};

/* 麦轮调试四路实测轮速，单位 m/s。 */
static float g_lcd_show_mecanum_measured_speed_mps[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* 麦轮调试比例系数。 */
static float g_lcd_show_mecanum_kp = CHASSIS_PID_DEFAULT_KP;

/* 麦轮调试积分系数。 */
static float g_lcd_show_mecanum_ki = CHASSIS_PID_DEFAULT_KI;

/* 麦轮调试 PID 保存状态。 */
static st7789_ui_mecanum_save_status_t g_lcd_show_mecanum_save_status = ST7789_UI_MECANUM_SAVE_NONE;
#endif

/**
 * @brief  进入 LCD UI 临界区
 *
 * @return void
 */
static void lcd_show_enter_critical(void)
{
    taskENTER_CRITICAL();
}

/**
 * @brief  退出 LCD UI 临界区
 *
 * @return void
 */
static void lcd_show_exit_critical(void)
{
    taskEXIT_CRITICAL();
}

/**
 * @brief  标记 UI 状态发生变化
 *
 * @return void
 */
static void lcd_show_mark_dirty(void)
{
    g_lcd_show_dirty = true;
}

/**
 * @brief  在已进入临界区时复制当前 UI 状态
 *
 * @param p_state 状态输出指针
 * @return void
 */
static void lcd_show_copy_state_unlocked(boot_ui_state_t *p_state)
{
    p_state->boot_progress = g_lcd_show_boot_progress;

#if UI_PORT_USE_MIPI_LCD
    uint32_t i = 0U;
    uint8_t calib_point_index = g_mipi_arm_calib_point_index;
    float calib_above_z_mm = g_mipi_arm_calib_z_mm + LCD_SHOW_ARM_CALIB_ABOVE_OFFSET_MM;

    if (calib_point_index >= LCD_SHOW_ARM_CALIB_POINT_COUNT)
    {
        calib_point_index = 0U;
    }
    if (calib_above_z_mm > MECH_HAND_MENU_Z_MAX)
    {
        calib_above_z_mm = MECH_HAND_MENU_Z_MAX;
    }

    p_state->view = g_mipi_view;
    p_state->home_selected = g_mipi_home_selected;
    p_state->placeholder_page = g_mipi_placeholder_page;
    p_state->power_valid = g_mipi_power_valid;
    p_state->bus_voltage_mv = g_mipi_bus_voltage_mv;
    p_state->current_ua = g_mipi_current_ua;
    p_state->voice_ai_valid = g_mipi_voice_ai_valid;
    p_state->voice_ai_index = g_mipi_voice_ai_index;
    p_state->voice_ai_score_percent = g_mipi_voice_ai_score_percent;
    (void)strncpy(p_state->voice_ai_label,
                  g_mipi_voice_ai_label,
                  MIPI_UI_VOICE_AI_LABEL_LEN - 1U);
    p_state->voice_ai_label[MIPI_UI_VOICE_AI_LABEL_LEN - 1U] = '\0';
    p_state->chassis_mode = g_mipi_chassis_mode;
    p_state->selected_index = g_mipi_selected_index;
    p_state->editing = g_mipi_editing;
    p_state->edit_value = g_mipi_edit_value;
    p_state->chassis_control_enabled = g_mipi_chassis_control_enabled;
    p_state->vx_mps = g_mipi_vx_mps;
    p_state->vy_mps = g_mipi_vy_mps;
    p_state->wz_radps = g_mipi_wz_radps;
    p_state->kp = g_mipi_kp;
    p_state->ki = g_mipi_ki;
    p_state->pid_save_status = g_mipi_pid_save_status;
    p_state->radar_cloud_valid = g_mipi_radar_cloud_valid;
    p_state->radar_cloud_sequence = g_mipi_radar_cloud_sequence;
    p_state->radar_source_scan_sequence = g_mipi_radar_source_scan_sequence;
    p_state->radar_rpm = g_mipi_radar_rpm;
    p_state->radar_nearest_distance_mm = g_mipi_radar_nearest_distance_mm;
    p_state->avoidance_state = g_mipi_avoidance_state;
    p_state->avoidance_radar_stale_ms = g_mipi_avoidance_radar_stale_ms;
    p_state->avoidance_target_angle_deg = g_mipi_avoidance_target_angle_deg;
    p_state->avoidance_selected_angle_deg = g_mipi_avoidance_selected_angle_deg;
    p_state->avoidance_target_safe = g_mipi_avoidance_target_safe;
    p_state->arm_initialized = g_mipi_arm_initialized;
    p_state->arm_x_mm = g_mipi_arm_x_mm;
    p_state->arm_y_mm = g_mipi_arm_y_mm;
    p_state->arm_z_mm = g_mipi_arm_z_mm;
    p_state->arm_time_ms = g_mipi_arm_time_ms;
    p_state->arm_servo_index = g_mipi_arm_servo_index;
    p_state->arm_servo_pwm = g_mipi_arm_servo_pwm;
    p_state->arm_gripper_opened = g_mipi_arm_gripper_opened;
    p_state->arm_last_result = g_mipi_arm_last_result;
    p_state->arm_last_clamped = g_mipi_arm_last_clamped;
    (void)strncpy(p_state->arm_last_command,
                  g_mipi_arm_last_command,
                  MIPI_UI_ARM_LAST_COMMAND_LEN - 1U);
    p_state->arm_last_command[MIPI_UI_ARM_LAST_COMMAND_LEN - 1U] = '\0';
    p_state->arm_target_positions = g_mipi_arm_target_positions;
    p_state->arm_feedback = g_mipi_arm_feedback;
    p_state->arm_calib_point_index = calib_point_index;
    p_state->arm_calib_point_count = LCD_SHOW_ARM_CALIB_POINT_COUNT;
    p_state->arm_calib_x_mm = g_mipi_arm_calib_points[calib_point_index].x_mm;
    p_state->arm_calib_y_mm = g_mipi_arm_calib_points[calib_point_index].y_mm;
    p_state->arm_calib_z_mm = g_mipi_arm_calib_z_mm;
    p_state->arm_calib_above_z_mm = calib_above_z_mm;
    p_state->arm_calib_auto_running = (LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE != g_mipi_arm_calib_pick_state);
    p_state->arm_calib_flow_state = g_mipi_arm_calib_flow_state;
    p_state->mission_snapshot = g_mipi_mission_snapshot;
    p_state->voice_dataset_status = g_mipi_voice_status;

    if (MIPI_CHASSIS_MODE_BODY == g_mipi_chassis_mode)
    {
        chassis_velocity_t velocity =
        {
            .vx_mps = g_mipi_vx_mps,
            .vy_mps = g_mipi_vy_mps,
            .wz_radps = g_mipi_wz_radps,
        };

        kinematics.inverse(&velocity, p_state->wheel_target_mps);
    }

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (MIPI_CHASSIS_MODE_BODY != g_mipi_chassis_mode)
        {
            p_state->wheel_target_mps[i] = g_mipi_wheel_target_mps[i];
        }
        p_state->wheel_measured_mps[i] = g_mipi_wheel_measured_mps[i];
        p_state->wheel_rpm[i] = g_mipi_wheel_rpm[i];
        p_state->pwm_counts[i] = g_mipi_pwm_counts[i];
    }

    for (i = 0U; i < N10_RADAR_CLOUD_SECTOR_COUNT; i++)
    {
        p_state->radar_sector_min_mm[i] = g_mipi_radar_sector_min_mm[i];
        p_state->radar_sector_valid[i] = g_mipi_radar_sector_valid[i];
    }
#endif

#if !UI_PORT_USE_MIPI_LCD
    p_state->view = g_lcd_show_view;
    p_state->selected_index = g_lcd_show_selected_index;
    p_state->current_page = g_lcd_show_current_page;
    p_state->edit_value = g_lcd_show_editing ? g_lcd_show_edit_value :
                           g_lcd_show_page_values[(uint8_t)g_lcd_show_current_page];
    p_state->editing = g_lcd_show_editing;
    p_state->power_valid = g_lcd_show_power_valid;
    p_state->bus_voltage_mv = g_lcd_show_bus_voltage_mv;
    p_state->current_ua = g_lcd_show_current_ua;
    p_state->mecanum_mode = g_lcd_show_mecanum_mode;
    p_state->mecanum_selected_item = g_lcd_show_mecanum_selected_item;
    p_state->mecanum_editing = g_lcd_show_mecanum_editing;
    p_state->mecanum_edit_value = g_lcd_show_mecanum_edit_value;
    p_state->mecanum_vx_mps = g_lcd_show_mecanum_vx_mps;
    p_state->mecanum_vy_mps = g_lcd_show_mecanum_vy_mps;
    p_state->mecanum_wz_radps = g_lcd_show_mecanum_wz_radps;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        p_state->mecanum_wheel_speed_mps[i] = g_lcd_show_mecanum_wheel_speed_mps[i];
        p_state->mecanum_pwm_counts[i] = g_lcd_show_mecanum_pwm_counts[i];
        p_state->mecanum_measured_speed_mps[i] = g_lcd_show_mecanum_measured_speed_mps[i];
    }
    p_state->mecanum_kp = g_lcd_show_mecanum_kp;
    p_state->mecanum_ki = g_lcd_show_mecanum_ki;
    p_state->mecanum_save_status = g_lcd_show_mecanum_save_status;
#endif
}

/**
 * @brief  复制当前开机页状态
 *
 * @param p_state 状态输出指针
 * @return void
 */
static void lcd_show_copy_state(boot_ui_state_t *p_state)
{
    lcd_show_enter_critical();
    lcd_show_copy_state_unlocked(p_state);
    lcd_show_exit_critical();
}

/**
 * @brief  复制当前开机页状态并清除脏标志
 *
 * @param p_state 状态输出指针
 * @return bool true 表示复制前存在脏数据
 */
static bool lcd_show_copy_state_and_clear_dirty(boot_ui_state_t *p_state)
{
    bool dirty = false;

    lcd_show_enter_critical();
    lcd_show_copy_state_unlocked(p_state);
    dirty = g_lcd_show_dirty;
    g_lcd_show_dirty = false;
    lcd_show_exit_critical();

    return dirty;
}

#if UI_PORT_USE_MIPI_LCD
/**
 * @brief  限制浮点数范围
 *
 * @param value 当前值
 * @param low   最小值
 * @param high  最大值
 * @return float 限幅后的值
 */
static float lcd_show_mipi_limit_float(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}

/**
 * @brief  限制有符号整数范围
 *
 * @param value 当前值
 * @param low   最小值
 * @param high  最大值
 * @return int32_t 限幅后的值
 */
static int32_t lcd_show_mipi_limit_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}

/**
 * @brief  计算浮点绝对值
 *
 * @param value 浮点数
 * @return float 绝对值
 */
static float lcd_show_mipi_abs_float(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

/**
 * @brief  获取当前底盘页选中项数量
 *
 * @return uint8_t 选中项数量
 */
static uint8_t lcd_show_mipi_chassis_item_count_unlocked(void)
{
    if (MIPI_CHASSIS_MODE_BODY == g_mipi_chassis_mode)
    {
        return 12U;
    }

    return 13U;
}

/**
 * @brief  获取当前底盘页动作项起始索引
 *
 * @return uint8_t 动作项起始索引
 */
static uint8_t lcd_show_mipi_chassis_action_base_unlocked(void)
{
    if (MIPI_CHASSIS_MODE_BODY == g_mipi_chassis_mode)
    {
        return 8U;
    }

    return 9U;
}

/**
 * @brief  判断当前底盘选中项是否可编辑
 *
 * @return bool true 表示可编辑
 */
static bool lcd_show_mipi_chassis_selected_editable_unlocked(void)
{
    if (MIPI_CHASSIS_MODE_BODY == g_mipi_chassis_mode)
    {
        return ((3U <= g_mipi_selected_index) && (g_mipi_selected_index <= 7U));
    }

    return ((3U <= g_mipi_selected_index) && (g_mipi_selected_index <= 8U));
}

/**
 * @brief  获取当前选中项编辑步进
 *
 * @return float 编辑步进
 */
static float lcd_show_mipi_chassis_edit_step_unlocked(void)
{
    if (MIPI_CHASSIS_MODE_BODY == g_mipi_chassis_mode)
    {
        if (5U == g_mipi_selected_index)
        {
            return LCD_SHOW_MECANUM_WZ_STEP_RADPS;
        }

        if ((6U == g_mipi_selected_index) || (7U == g_mipi_selected_index))
        {
            return LCD_SHOW_MECANUM_PID_STEP;
        }

        return LCD_SHOW_MECANUM_V_STEP_MPS;
    }

    if ((7U == g_mipi_selected_index) || (8U == g_mipi_selected_index))
    {
        return LCD_SHOW_MECANUM_PID_STEP;
    }

    if (MIPI_CHASSIS_MODE_PWM == g_mipi_chassis_mode)
    {
        return LCD_SHOW_MECANUM_PWM_STEP;
    }

    return LCD_SHOW_MECANUM_WHEEL_STEP_MPS;
}

/**
 * @brief  读取当前选中项数值
 *
 * @return float 当前数值
 */
static float lcd_show_mipi_chassis_get_selected_value_unlocked(void)
{
    if (MIPI_CHASSIS_MODE_BODY == g_mipi_chassis_mode)
    {
        if (3U == g_mipi_selected_index)
        {
            return g_mipi_vx_mps;
        }
        if (4U == g_mipi_selected_index)
        {
            return g_mipi_vy_mps;
        }
        if (5U == g_mipi_selected_index)
        {
            return g_mipi_wz_radps;
        }
        if (6U == g_mipi_selected_index)
        {
            return g_mipi_kp;
        }
        if (7U == g_mipi_selected_index)
        {
            return g_mipi_ki;
        }
    }
    else if ((3U <= g_mipi_selected_index) && (g_mipi_selected_index <= 6U))
    {
        uint8_t wheel = (uint8_t)(g_mipi_selected_index - 3U);

        if (MIPI_CHASSIS_MODE_PWM == g_mipi_chassis_mode)
        {
            return (float)g_mipi_pwm_counts[wheel];
        }

        return g_mipi_wheel_target_mps[wheel];
    }
    else if (7U == g_mipi_selected_index)
    {
        return g_mipi_kp;
    }
    else if (8U == g_mipi_selected_index)
    {
        return g_mipi_ki;
    }

    return 0.0f;
}

/**
 * @brief  写入当前选中项数值
 *
 * @param value 新数值
 * @return void
 */
static void lcd_show_mipi_chassis_set_selected_value_unlocked(float value)
{
    if (MIPI_CHASSIS_MODE_BODY == g_mipi_chassis_mode)
    {
        if (3U == g_mipi_selected_index)
        {
            g_mipi_vx_mps = lcd_show_mipi_limit_float(value, -CHASSIS_MAX_VX_MPS, CHASSIS_MAX_VX_MPS);
        }
        else if (4U == g_mipi_selected_index)
        {
            g_mipi_vy_mps = lcd_show_mipi_limit_float(value, -CHASSIS_MAX_VY_MPS, CHASSIS_MAX_VY_MPS);
        }
        else if (5U == g_mipi_selected_index)
        {
            g_mipi_wz_radps = lcd_show_mipi_limit_float(value, -CHASSIS_MAX_WZ_RADPS, CHASSIS_MAX_WZ_RADPS);
        }
        else if (6U == g_mipi_selected_index)
        {
            g_mipi_kp = lcd_show_mipi_limit_float(value, CHASSIS_PID_PARAM_MIN, CHASSIS_PID_PARAM_MAX);
            g_mipi_pid_save_status = MIPI_PID_SAVE_NONE;
        }
        else if (7U == g_mipi_selected_index)
        {
            g_mipi_ki = lcd_show_mipi_limit_float(value, CHASSIS_PID_PARAM_MIN, CHASSIS_PID_PARAM_MAX);
            g_mipi_pid_save_status = MIPI_PID_SAVE_NONE;
        }
    }
    else if ((3U <= g_mipi_selected_index) && (g_mipi_selected_index <= 6U))
    {
        uint8_t wheel = (uint8_t)(g_mipi_selected_index - 3U);

        if (MIPI_CHASSIS_MODE_PWM == g_mipi_chassis_mode)
        {
            int32_t pwm = lcd_show_mipi_limit_i32((int32_t)value,
                                                  -CHASSIS_PWM_TEST_LIMIT_COUNTS,
                                                  CHASSIS_PWM_TEST_LIMIT_COUNTS);
#if LCD_SHOW_PWM_SYNC_ALL
            for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
            {
                g_mipi_pwm_counts[i] = pwm;
            }
#else
            g_mipi_pwm_counts[wheel] = pwm;
#endif
        }
        else
        {
            g_mipi_wheel_target_mps[wheel] =
                lcd_show_mipi_limit_float(value,
                                          -CHASSIS_MAX_WHEEL_SPEED_MPS,
                                          CHASSIS_MAX_WHEEL_SPEED_MPS);
        }
    }
    else if (7U == g_mipi_selected_index)
    {
        g_mipi_kp = lcd_show_mipi_limit_float(value, CHASSIS_PID_PARAM_MIN, CHASSIS_PID_PARAM_MAX);
        g_mipi_pid_save_status = MIPI_PID_SAVE_NONE;
    }
    else if (8U == g_mipi_selected_index)
    {
        g_mipi_ki = lcd_show_mipi_limit_float(value, CHASSIS_PID_PARAM_MIN, CHASSIS_PID_PARAM_MAX);
        g_mipi_pid_save_status = MIPI_PID_SAVE_NONE;
    }
}

/**
 * @brief  移动底盘页选中项
 *
 * @param direction 正数向下，负数向上
 * @return void
 */
static void lcd_show_mipi_chassis_move_selection_unlocked(int32_t direction)
{
    uint8_t count = lcd_show_mipi_chassis_item_count_unlocked();

    if (g_mipi_editing)
    {
        return;
    }

    if (direction > 0)
    {
        g_mipi_selected_index++;
        if (g_mipi_selected_index >= count)
        {
            g_mipi_selected_index = 0U;
        }
    }
    else
    {
        if (0U == g_mipi_selected_index)
        {
            g_mipi_selected_index = count - 1U;
        }
        else
        {
            g_mipi_selected_index--;
        }
    }

    lcd_show_mark_dirty();
}

/**
 * @brief  开始编辑底盘当前选中项
 *
 * @return void
 */
static void lcd_show_mipi_chassis_begin_edit_unlocked(void)
{
    if (!lcd_show_mipi_chassis_selected_editable_unlocked())
    {
        return;
    }

    g_mipi_edit_original_value = lcd_show_mipi_chassis_get_selected_value_unlocked();
    g_mipi_edit_value = g_mipi_edit_original_value;
    g_mipi_editing = true;
    lcd_show_mark_dirty();
}

/**
 * @brief  确认底盘当前编辑值
 *
 * @return void
 */
static void lcd_show_mipi_chassis_confirm_edit_unlocked(void)
{
    if (!g_mipi_editing)
    {
        return;
    }

    lcd_show_mipi_chassis_set_selected_value_unlocked(g_mipi_edit_value);
    g_mipi_editing = false;
    lcd_show_mark_dirty();
}

/**
 * @brief  取消底盘当前编辑值
 *
 * @return void
 */
static void lcd_show_mipi_chassis_cancel_edit_unlocked(void)
{
    g_mipi_edit_value = g_mipi_edit_original_value;
    g_mipi_editing = false;
    lcd_show_mark_dirty();
}

/**
 * @brief  调整底盘当前编辑值
 *
 * @param direction 正数增大，负数减小
 * @return void
 */
static void lcd_show_mipi_chassis_adjust_edit_unlocked(int32_t direction)
{
    float step = lcd_show_mipi_chassis_edit_step_unlocked();

    if (!g_mipi_editing)
    {
        return;
    }

    g_mipi_edit_value += (direction > 0) ? step : -step;
    lcd_show_mark_dirty();
}

/**
 * @brief  切换底盘控制模式
 *
 * @param mode 目标模式
 * @return void
 */
static void lcd_show_mipi_chassis_switch_mode_unlocked(mipi_chassis_mode_t mode)
{
    g_mipi_chassis_mode = mode;
    g_mipi_selected_index = (uint8_t)mode;
    g_mipi_editing = false;
    lcd_show_mark_dirty();
}

/**
 * @brief  首页切换到下一个功能
 *
 * @return void
 */
static void lcd_show_mipi_home_next_unlocked(void)
{
    uint8_t next = (uint8_t)g_mipi_home_selected + 1U;

    if (next >= (uint8_t)MIPI_HOME_PAGE_COUNT)
    {
        next = 0U;
    }

    g_mipi_home_selected = (mipi_home_page_t)next;
    lcd_show_mark_dirty();
}

/**
 * @brief  首页切换到上一个功能
 *
 * @return void
 */
static void lcd_show_mipi_home_prev_unlocked(void)
{
    if (0U == (uint8_t)g_mipi_home_selected)
    {
        g_mipi_home_selected = (mipi_home_page_t)((uint8_t)MIPI_HOME_PAGE_COUNT - 1U);
    }
    else
    {
        g_mipi_home_selected = (mipi_home_page_t)((uint8_t)g_mipi_home_selected - 1U);
    }

    lcd_show_mark_dirty();
}

/**
 * @brief  进入 MIPI 底盘调试页
 *
 * @return void
 */
static void lcd_show_mipi_enter_chassis_page(void)
{
    chassis_pid_params_t pid_params =
    {
        .kp = CHASSIS_PID_DEFAULT_KP,
        .ki = CHASSIS_PID_DEFAULT_KI,
    };

    chassis.get_pid(&pid_params);

    lcd_show_enter_critical();
    g_mipi_view = MIPI_UI_VIEW_CHASSIS;
    g_mipi_chassis_mode = MIPI_CHASSIS_MODE_BODY;
    g_mipi_selected_index = 0U;
    g_mipi_editing = false;
    g_mipi_kp = pid_params.kp;
    g_mipi_ki = pid_params.ki;
    g_mipi_pid_save_status = MIPI_PID_SAVE_NONE;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  进入 MIPI 雷达点云页
 *
 * @return void
 */
static void lcd_show_mipi_enter_radar_cloud_page(void)
{
    lcd_show_enter_critical();
    g_mipi_view = MIPI_UI_VIEW_RADAR_CLOUD;
    g_mipi_editing = false;
    g_mipi_radar_refresh_ms = LCD_SHOW_MIPI_RADAR_REFRESH_PERIOD_MS;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  进入 MIPI 机械臂调试页
 *
 * @return void
 */
static void lcd_show_mipi_enter_arm_debug_page(void)
{
    mech_hand_servo_positions_t positions = {{0U}};
    mechanical_arm_feedback_snapshot_t snapshot = {0};

    mechanical_hand.get_servo_positions(&positions);
    mechanical_arm_feedback.read_snapshot(&snapshot);

    lcd_show_enter_critical();
    g_mipi_view = MIPI_UI_VIEW_ARM_DEBUG;
    g_mipi_selected_index = 0U;
    g_mipi_editing = false;
    g_mipi_arm_target_positions = positions;
    g_mipi_arm_servo_pwm = positions.servo_pwm[g_mipi_arm_servo_index];
    g_mipi_arm_feedback = snapshot;
    g_mipi_arm_initialized = snapshot.initialized;
    g_mipi_arm_refresh_ms = LCD_SHOW_MIPI_ARM_REFRESH_PERIOD_MS;
    g_mipi_arm_feedback_sequence = snapshot.sequence;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  进入 MIPI 机械臂标定辅助页
 *
 * @return void
 */
static void lcd_show_mipi_enter_arm_calib_page(void)
{
    mech_hand_servo_positions_t positions = {{0U}};
    mechanical_arm_feedback_snapshot_t snapshot = {0};

    mechanical_hand.get_servo_positions(&positions);
    mechanical_arm_feedback.read_snapshot(&snapshot);

    lcd_show_enter_critical();
    g_mipi_view = MIPI_UI_VIEW_ARM_CALIB;
    g_mipi_selected_index = 0U;
    g_mipi_editing = false;
    g_mipi_arm_target_positions = positions;
    g_mipi_arm_feedback = snapshot;
    g_mipi_arm_initialized = snapshot.initialized;
    g_mipi_arm_refresh_ms = LCD_SHOW_MIPI_ARM_REFRESH_PERIOD_MS;
    g_mipi_arm_feedback_sequence = snapshot.sequence;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  进入 MIPI Mission 页面
 *
 * @return void
 */
static void lcd_show_mipi_enter_mission_page(void)
{
    mission_snapshot_t snapshot = {0};

    mission.request_monitor_entered();
    mission.snapshot_read(&snapshot);

    lcd_show_enter_critical();
    g_mipi_view = MIPI_UI_VIEW_MISSION;
    g_mipi_selected_index = 0U;
    g_mipi_editing = false;
    g_mipi_mission_snapshot = snapshot;
    g_mipi_mission_refresh_ms = LCD_SHOW_MIPI_MISSION_REFRESH_PERIOD_MS;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  从 MIPI Mission 页面手动请求启动 Mission
 *
 * @return void
 */
static void lcd_show_mipi_mission_request_start(void)
{
    mission_snapshot_t snapshot = {0};

    (void)mission.request_start();
    mission.snapshot_read(&snapshot);

    lcd_show_enter_critical();
    g_mipi_mission_snapshot = snapshot;
    g_mipi_mission_refresh_ms = LCD_SHOW_MIPI_MISSION_REFRESH_PERIOD_MS;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  在临界区内同步 UI 当前选择的语音标签
 *
 * @return void
 */
static void lcd_show_mipi_voice_sync_selected_label_unlocked(void)
{
    const char *label = voice_dataset_label_name(g_mipi_voice_label_index);

    if (NULL == label)
    {
        label = VOICE_DATASET_DEFAULT_LABEL;
    }

    (void)strncpy(g_mipi_voice_status.label, label, VOICE_DATASET_LABEL_LENGTH - 1U);
    g_mipi_voice_status.label[VOICE_DATASET_LABEL_LENGTH - 1U] = '\0';
}

/**
 * @brief  在临界区内判断语音数据集是否忙碌
 *
 * @return bool true 表示正在录制或发送
 */
static bool lcd_show_mipi_voice_busy_unlocked(void)
{
    return (VOICE_DATASET_STATE_RECORDING == g_mipi_voice_status.state) ||
           (VOICE_DATASET_STATE_SENDING == g_mipi_voice_status.state);
}

/**
 * @brief  切换到下一个语音数据集标签
 *
 * @return void
 */
static void lcd_show_mipi_voice_next_label(void)
{
    uint8_t count = voice_dataset_label_count();

    lcd_show_enter_critical();
    if ((!lcd_show_mipi_voice_busy_unlocked()) && (count > 0U))
    {
        g_mipi_voice_label_index++;
        if (g_mipi_voice_label_index >= count)
        {
            g_mipi_voice_label_index = 0U;
        }

        lcd_show_mipi_voice_sync_selected_label_unlocked();
        lcd_show_mark_dirty();
    }
    lcd_show_exit_critical();
}

/**
 * @brief  切换到上一个语音数据集标签
 *
 * @return void
 */
static void lcd_show_mipi_voice_prev_label(void)
{
    uint8_t count = voice_dataset_label_count();

    lcd_show_enter_critical();
    if ((!lcd_show_mipi_voice_busy_unlocked()) && (count > 0U))
    {
        if (0U == g_mipi_voice_label_index)
        {
            g_mipi_voice_label_index = count - 1U;
        }
        else
        {
            g_mipi_voice_label_index--;
        }

        lcd_show_mipi_voice_sync_selected_label_unlocked();
        lcd_show_mark_dirty();
    }
    lcd_show_exit_critical();
}

/**
 * @brief  进入 MIPI 语音数据集页面
 *
 * @return void
 */
static void lcd_show_mipi_enter_voice_dataset_page(void)
{
    voice_dataset_status_t status = {0};

    (void)voice_ai.set_enabled(false);
    (void)voice_dataset.init();
    voice_dataset.status_get(&status);

    lcd_show_enter_critical();
    g_mipi_view = MIPI_UI_VIEW_VOICE_DATASET;
    g_mipi_selected_index = 0U;
    g_mipi_editing = false;
    g_mipi_voice_status = status;
    if (!lcd_show_mipi_voice_busy_unlocked())
    {
        lcd_show_mipi_voice_sync_selected_label_unlocked();
    }
    g_mipi_voice_refresh_ms = LCD_SHOW_MIPI_VOICE_REFRESH_PERIOD_MS;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  返回 MIPI 首页
 *
 * @param stop_chassis true 表示同时停止底盘调试控制
 * @return void
 */
static void lcd_show_mipi_back_home(bool stop_chassis)
{
    (void)voice_ai.set_enabled(true);

    if (stop_chassis)
    {
        display_speed_control.clear();
    }

    lcd_show_enter_critical();
    g_mipi_view = MIPI_UI_VIEW_HOME;
    g_mipi_editing = false;
    if (stop_chassis)
    {
        g_mipi_chassis_control_enabled = false;
    }
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  应用当前 MIPI 底盘控制值
 *
 * @return void
 */
static void lcd_show_mipi_apply_chassis_control(void)
{
    mipi_chassis_mode_t mode = MIPI_CHASSIS_MODE_BODY;
    ui_speed_control control =
    {
        .vx_mps = 0.0f,
        .vy_mps = 0.0f,
        .wz_radps = 0.0f,
    };
    float wheel_target_mps[CHASSIS_WHEEL_COUNT] = {0.0f};
    int32_t pwm_counts[CHASSIS_WHEEL_COUNT] = {0};

    lcd_show_enter_critical();
    mode = g_mipi_chassis_mode;
    control.vx_mps = g_mipi_vx_mps;
    control.vy_mps = g_mipi_vy_mps;
    control.wz_radps = g_mipi_wz_radps;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        wheel_target_mps[i] = g_mipi_wheel_target_mps[i];
        pwm_counts[i] = g_mipi_pwm_counts[i];
    }
    g_mipi_chassis_control_enabled = true;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();

    if (MIPI_CHASSIS_MODE_WHEEL == mode)
    {
        display_speed_control.set_wheel(wheel_target_mps);
    }
    else if (MIPI_CHASSIS_MODE_PWM == mode)
    {
        display_speed_control.set_pwm(pwm_counts);
    }
    else
    {
        display_speed_control.set_body(&control);
    }

    display_speed_control.set_enable(true);
}

/**
 * @brief  保存当前 MIPI PID 参数
 *
 * @return void
 */
static void lcd_show_mipi_save_pid(void)
{
    fsp_err_t err = FSP_SUCCESS;
    chassis_pid_params_t params =
    {
        .kp = CHASSIS_PID_DEFAULT_KP,
        .ki = CHASSIS_PID_DEFAULT_KI,
    };

    lcd_show_enter_critical();
    params.kp = g_mipi_kp;
    params.ki = g_mipi_ki;
    g_mipi_pid_save_status = MIPI_PID_SAVE_SAVING;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();

    chassis.set_pid(params.kp, params.ki);
    err = chassis_param.save(&params);

    lcd_show_enter_critical();
    g_mipi_pid_save_status = (FSP_SUCCESS == err) ? MIPI_PID_SAVE_SAVED : MIPI_PID_SAVE_ERR;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  清零当前 MIPI 底盘模式目标
 *
 * @return void
 */
static void lcd_show_mipi_clear_current_values(void)
{
    display_speed_control.clear();

    lcd_show_enter_critical();
    if (MIPI_CHASSIS_MODE_BODY == g_mipi_chassis_mode)
    {
        g_mipi_vx_mps = 0.0f;
        g_mipi_vy_mps = 0.0f;
        g_mipi_wz_radps = 0.0f;
    }
    else if (MIPI_CHASSIS_MODE_WHEEL == g_mipi_chassis_mode)
    {
        for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            g_mipi_wheel_target_mps[i] = 0.0f;
        }
    }
    else
    {
        for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            g_mipi_pwm_counts[i] = 0;
        }
    }
    g_mipi_chassis_control_enabled = false;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  停止 MIPI 底盘控制
 *
 * @return void
 */
static void lcd_show_mipi_stop_chassis(void)
{
    display_speed_control.clear();

    lcd_show_enter_critical();
    g_mipi_chassis_control_enabled = false;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  按动作索引执行底盘动作
 *
 * @param action 底盘动作
 * @return void
 */
static void lcd_show_mipi_execute_action(lcd_show_mipi_chassis_action_t action)
{
    switch (action)
    {
        case LCD_SHOW_MIPI_CHASSIS_ACTION_APPLY:
            lcd_show_mipi_apply_chassis_control();
            break;

        case LCD_SHOW_MIPI_CHASSIS_ACTION_SAVE_PID:
            lcd_show_mipi_save_pid();
            break;

        case LCD_SHOW_MIPI_CHASSIS_ACTION_ZERO:
            lcd_show_mipi_clear_current_values();
            break;

        case LCD_SHOW_MIPI_CHASSIS_ACTION_STOP:
            lcd_show_mipi_stop_chassis();
            break;

        case LCD_SHOW_MIPI_CHASSIS_ACTION_NONE:
        default:
            break;
    }
}

/**
 * @brief  获取当前选中动作
 *
 * @return lcd_show_mipi_chassis_action_t 当前动作
 */
static lcd_show_mipi_chassis_action_t lcd_show_mipi_chassis_selected_action_unlocked(void)
{
    uint8_t base = lcd_show_mipi_chassis_action_base_unlocked();

    if (g_mipi_selected_index == base)
    {
        return LCD_SHOW_MIPI_CHASSIS_ACTION_APPLY;
    }
    if (g_mipi_selected_index == (uint8_t)(base + 1U))
    {
        return LCD_SHOW_MIPI_CHASSIS_ACTION_SAVE_PID;
    }
    if (g_mipi_selected_index == (uint8_t)(base + 2U))
    {
        return LCD_SHOW_MIPI_CHASSIS_ACTION_ZERO;
    }
    if (g_mipi_selected_index == (uint8_t)(base + 3U))
    {
        return LCD_SHOW_MIPI_CHASSIS_ACTION_STOP;
    }

    return LCD_SHOW_MIPI_CHASSIS_ACTION_NONE;
}

/**
 * @brief  获取机械臂舵机 PWM 下限
 *
 * @param index 舵机序号
 * @return uint16_t PWM 下限
 */
static uint16_t lcd_show_mipi_arm_servo_min(uint8_t index)
{
    switch (index)
    {
        case 0U:
            return MECH_HAND_SERVO0_PWM_MIN;

        case 1U:
            return MECH_HAND_SERVO1_PWM_MIN;

        case 2U:
            return MECH_HAND_SERVO2_PWM_MIN;

        case 3U:
            return MECH_HAND_SERVO3_PWM_MIN;

        case 4U:
            return MECH_HAND_SERVO4_PWM_MIN;

        case 5U:
            return MECH_HAND_SERVO5_PWM_MIN;

        default:
            return BUS_SERVO_PWM_MIN;
    }
}

/**
 * @brief  获取机械臂舵机 PWM 上限
 *
 * @param index 舵机序号
 * @return uint16_t PWM 上限
 */
static uint16_t lcd_show_mipi_arm_servo_max(uint8_t index)
{
    switch (index)
    {
        case 0U:
            return MECH_HAND_SERVO0_PWM_MAX;

        case 1U:
            return MECH_HAND_SERVO1_PWM_MAX;

        case 2U:
            return MECH_HAND_SERVO2_PWM_MAX;

        case 3U:
            return MECH_HAND_SERVO3_PWM_MAX;

        case 4U:
            return MECH_HAND_SERVO4_PWM_MAX;

        case 5U:
            return MECH_HAND_SERVO5_PWM_MAX;

        default:
            return BUS_SERVO_PWM_MAX;
    }
}

/**
 * @brief  获取机械臂当前选中项数值
 *
 * @return float 当前选中项数值
 */
static float lcd_show_mipi_arm_get_selected_value_unlocked(void)
{
    switch (g_mipi_selected_index)
    {
        case 0U:
            return g_mipi_arm_x_mm;

        case 1U:
            return g_mipi_arm_y_mm;

        case 2U:
            return g_mipi_arm_z_mm;

        case 3U:
            return (float)g_mipi_arm_time_ms;

        case 4U:
            return (float)g_mipi_arm_servo_index;

        case 5U:
            return (float)g_mipi_arm_servo_pwm;

        default:
            return 0.0f;
    }
}

/**
 * @brief  设置机械臂当前选中项数值
 *
 * @param value 新数值
 * @return void
 */
static void lcd_show_mipi_arm_set_selected_value_unlocked(float value)
{
    switch (g_mipi_selected_index)
    {
        case 0U:
            g_mipi_arm_x_mm = lcd_show_mipi_limit_float(value, MECH_HAND_MENU_X_MIN, MECH_HAND_MENU_X_MAX);
            break;

        case 1U:
            g_mipi_arm_y_mm = lcd_show_mipi_limit_float(value, MECH_HAND_MENU_Y_MIN, MECH_HAND_MENU_Y_MAX);
            break;

        case 2U:
            g_mipi_arm_z_mm = lcd_show_mipi_limit_float(value, MECH_HAND_MENU_Z_MIN, MECH_HAND_MENU_Z_MAX);
            break;

        case 3U:
            g_mipi_arm_time_ms = (uint16_t)lcd_show_mipi_limit_i32((int32_t)value,
                                                                   (int32_t)LCD_SHOW_ARM_TIME_MIN_MS,
                                                                   (int32_t)BUS_SERVO_TIME_MAX);
            break;

        case 4U:
            g_mipi_arm_servo_index = (uint8_t)lcd_show_mipi_limit_i32((int32_t)value,
                                                                      0,
                                                                      (int32_t)MECH_HAND_SERVO_COUNT - 1);
            g_mipi_arm_servo_pwm = g_mipi_arm_target_positions.servo_pwm[g_mipi_arm_servo_index];
            break;

        case 5U:
            g_mipi_arm_servo_pwm = (uint16_t)lcd_show_mipi_limit_i32((int32_t)value,
                                                                     (int32_t)lcd_show_mipi_arm_servo_min(g_mipi_arm_servo_index),
                                                                     (int32_t)lcd_show_mipi_arm_servo_max(g_mipi_arm_servo_index));
            break;

        default:
            break;
    }
}

/**
 * @brief  获取机械臂当前编辑步进
 *
 * @return float 编辑步进
 */
static float lcd_show_mipi_arm_edit_step_unlocked(void)
{
    switch (g_mipi_selected_index)
    {
        case 0U:
        case 1U:
        case 2U:
            return LCD_SHOW_ARM_COORD_STEP_MM;

        case 3U:
            return LCD_SHOW_ARM_TIME_STEP_MS;

        case 4U:
            return 1.0f;

        case 5U:
            return LCD_SHOW_ARM_PWM_STEP;

        default:
            return 0.0f;
    }
}

/**
 * @brief  判断机械臂当前选中项是否可编辑
 *
 * @return bool true 表示可编辑
 */
static bool lcd_show_mipi_arm_selected_editable_unlocked(void)
{
    return g_mipi_selected_index < LCD_SHOW_ARM_ACTION_BASE_INDEX;
}

/**
 * @brief  移动机械臂页面选中项
 *
 * @param direction 正数向后，负数向前
 * @return void
 */
static void lcd_show_mipi_arm_move_selection_unlocked(int32_t direction)
{
    if (g_mipi_editing)
    {
        return;
    }

    if (direction > 0)
    {
        g_mipi_selected_index++;
        if (g_mipi_selected_index >= LCD_SHOW_ARM_ITEM_COUNT)
        {
            g_mipi_selected_index = 0U;
        }
    }
    else
    {
        if (0U == g_mipi_selected_index)
        {
            g_mipi_selected_index = LCD_SHOW_ARM_ITEM_COUNT - 1U;
        }
        else
        {
            g_mipi_selected_index--;
        }
    }

    lcd_show_mark_dirty();
}

/**
 * @brief  开始编辑机械臂选中项
 *
 * @return void
 */
static void lcd_show_mipi_arm_begin_edit_unlocked(void)
{
    if (!lcd_show_mipi_arm_selected_editable_unlocked())
    {
        return;
    }

    g_mipi_edit_original_value = lcd_show_mipi_arm_get_selected_value_unlocked();
    g_mipi_edit_value = g_mipi_edit_original_value;
    g_mipi_editing = true;
    lcd_show_mark_dirty();
}

/**
 * @brief  确认机械臂编辑值
 *
 * @return void
 */
static void lcd_show_mipi_arm_confirm_edit_unlocked(void)
{
    if (!g_mipi_editing)
    {
        return;
    }

    lcd_show_mipi_arm_set_selected_value_unlocked(g_mipi_edit_value);
    g_mipi_editing = false;
    lcd_show_mark_dirty();
}

/**
 * @brief  取消机械臂编辑值
 *
 * @return void
 */
static void lcd_show_mipi_arm_cancel_edit_unlocked(void)
{
    g_mipi_edit_value = g_mipi_edit_original_value;
    g_mipi_editing = false;
    lcd_show_mark_dirty();
}

/**
 * @brief  调整机械臂编辑值
 *
 * @param direction 正数增大，负数减小
 * @return void
 */
static void lcd_show_mipi_arm_adjust_edit_unlocked(int32_t direction)
{
    float step = lcd_show_mipi_arm_edit_step_unlocked();

    if (!g_mipi_editing)
    {
        return;
    }

    g_mipi_edit_value += (direction > 0) ? step : -step;
    lcd_show_mark_dirty();
}

/**
 * @brief  获取机械臂当前选中动作
 *
 * @return lcd_show_mipi_arm_action_t 当前动作
 */
static lcd_show_mipi_arm_action_t lcd_show_mipi_arm_selected_action_unlocked(void)
{
    if (g_mipi_selected_index == LCD_SHOW_ARM_ACTION_BASE_INDEX)
    {
        return LCD_SHOW_MIPI_ARM_ACTION_HOME;
    }
    if (g_mipi_selected_index == (uint8_t)(LCD_SHOW_ARM_ACTION_BASE_INDEX + 1U))
    {
        return LCD_SHOW_MIPI_ARM_ACTION_GRIP;
    }
    if (g_mipi_selected_index == (uint8_t)(LCD_SHOW_ARM_ACTION_BASE_INDEX + 2U))
    {
        return LCD_SHOW_MIPI_ARM_ACTION_STOP;
    }
    if (g_mipi_selected_index == (uint8_t)(LCD_SHOW_ARM_ACTION_BASE_INDEX + 3U))
    {
        return LCD_SHOW_MIPI_ARM_ACTION_CALIB;
    }

    return LCD_SHOW_MIPI_ARM_ACTION_NONE;
}

/**
 * @brief  同步机械臂动作执行后的运行状态
 *
 * @param result 机械臂动作返回码
 * @return void
 */
static void lcd_show_mipi_arm_sync_after_action(int result)
{
    mech_hand_servo_positions_t positions = {{0U}};
    const char *p_command = NULL;

    mechanical_hand.get_servo_positions(&positions);
    p_command = mechanical_hand.get_last_command();

    lcd_show_enter_critical();
    g_mipi_arm_initialized = (MECH_HAND_ERR_NOT_READY != mechanical_hand.get_last_error());
    g_mipi_arm_last_result = result;
    g_mipi_arm_last_clamped = mechanical_hand.was_last_command_clamped();
    g_mipi_arm_target_positions = positions;
    if (g_mipi_arm_servo_index < MECH_HAND_SERVO_COUNT)
    {
        g_mipi_arm_servo_pwm = positions.servo_pwm[g_mipi_arm_servo_index];
    }
    if (NULL != p_command)
    {
        (void)strncpy(g_mipi_arm_last_command, p_command, MIPI_UI_ARM_LAST_COMMAND_LEN - 1U);
        g_mipi_arm_last_command[MIPI_UI_ARM_LAST_COMMAND_LEN - 1U] = '\0';
    }
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  执行机械臂当前动作
 *
 * @param action 待执行动作
 * @return void
 */
static void lcd_show_mipi_execute_arm_action(lcd_show_mipi_arm_action_t action)
{
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    uint16_t time_ms = 0U;
    uint8_t servo_index = 0U;
    uint16_t servo_pwm = 0U;
    int result = MECH_HAND_OK;
    bool gripper_opened = false;

    if (LCD_SHOW_MIPI_ARM_ACTION_NONE == action)
    {
        return;
    }

    lcd_show_enter_critical();
    x_mm = g_mipi_arm_x_mm;
    y_mm = g_mipi_arm_y_mm;
    z_mm = g_mipi_arm_z_mm;
    time_ms = g_mipi_arm_time_ms;
    servo_index = g_mipi_arm_servo_index;
    servo_pwm = g_mipi_arm_servo_pwm;
    gripper_opened = g_mipi_arm_gripper_opened;
    lcd_show_exit_critical();

    if (!mechanical_arm_feedback.lock_bus(LCD_SHOW_ARM_BUS_LOCK_TIMEOUT_MS))
    {
        lcd_show_mipi_arm_sync_after_action(MECH_HAND_ERR_TIMEOUT);
        return;
    }

    switch (action)
    {
        case LCD_SHOW_MIPI_ARM_ACTION_MOVE_TCP:
            result = mechanical_hand.move_tcp_to(x_mm, y_mm, z_mm, time_ms);
            mechanical_arm_feedback.request_refresh_all();
            break;

        case LCD_SHOW_MIPI_ARM_ACTION_SERVO_PWM:
            result = mechanical_hand.send_single_servo(servo_index, servo_pwm, time_ms);
            mechanical_arm_feedback.request_refresh(servo_index);
            break;

        case LCD_SHOW_MIPI_ARM_ACTION_GRIP:
            result = gripper_opened ? mechanical_hand.gripper_close(time_ms) :
                     mechanical_hand.gripper_open(time_ms);
            if (MECH_HAND_OK == result)
            {
                lcd_show_enter_critical();
                g_mipi_arm_gripper_opened = !gripper_opened;
                lcd_show_exit_critical();
            }
            mechanical_arm_feedback.request_refresh_all();
            break;

        case LCD_SHOW_MIPI_ARM_ACTION_HOME:
            result = mechanical_hand.home(time_ms);
            mechanical_arm_feedback.request_refresh_all();
            break;

        case LCD_SHOW_MIPI_ARM_ACTION_STOP:
            result = mechanical_hand.stop_all();
            mechanical_arm_feedback.request_refresh_all();
            break;

        case LCD_SHOW_MIPI_ARM_ACTION_CALIB:
            break;

        case LCD_SHOW_MIPI_ARM_ACTION_NONE:
        default:
            break;
    }

    mechanical_arm_feedback.unlock_bus();
    lcd_show_mipi_arm_sync_after_action(result);
}

/**
 * @brief  处理机械臂页面确认键
 *
 * @return void
 */
static void lcd_show_mipi_arm_confirm(void)
{
    lcd_show_mipi_arm_action_t action = LCD_SHOW_MIPI_ARM_ACTION_NONE;
    uint8_t selected_index = 0U;

    lcd_show_enter_critical();
    if (g_mipi_editing)
    {
        selected_index = g_mipi_selected_index;
        lcd_show_mipi_arm_confirm_edit_unlocked();
        lcd_show_exit_critical();

        if (selected_index <= 3U)
        {
            lcd_show_mipi_execute_arm_action(LCD_SHOW_MIPI_ARM_ACTION_MOVE_TCP);
        }
        else if (5U == selected_index)
        {
            lcd_show_mipi_execute_arm_action(LCD_SHOW_MIPI_ARM_ACTION_SERVO_PWM);
        }
        return;
    }

    if (lcd_show_mipi_arm_selected_editable_unlocked())
    {
        lcd_show_mipi_arm_begin_edit_unlocked();
    }
    else
    {
        action = lcd_show_mipi_arm_selected_action_unlocked();
    }
    lcd_show_exit_critical();

    if (LCD_SHOW_MIPI_ARM_ACTION_CALIB == action)
    {
        lcd_show_mipi_enter_arm_calib_page();
    }
    else
    {
        lcd_show_mipi_execute_arm_action(action);
    }
}

/**
 * @brief  获取机械臂标定页当前目标上方 Z
 *
 * @return float 目标上方 Z，单位 mm
 */
static float lcd_show_mipi_arm_calib_above_z_unlocked(void)
{
    float above_z_mm = g_mipi_arm_calib_z_mm + LCD_SHOW_ARM_CALIB_ABOVE_OFFSET_MM;

    if (above_z_mm > MECH_HAND_MENU_Z_MAX)
    {
        above_z_mm = MECH_HAND_MENU_Z_MAX;
    }

    return above_z_mm;
}

/**
 * @brief  获取机械臂标定页当前点位索引
 *
 * @return uint8_t 当前点位索引
 */
static uint8_t lcd_show_mipi_arm_calib_point_index_unlocked(void)
{
    if (g_mipi_arm_calib_point_index >= LCD_SHOW_ARM_CALIB_POINT_COUNT)
    {
        g_mipi_arm_calib_point_index = 0U;
    }

    return g_mipi_arm_calib_point_index;
}

/**
 * @brief  获取机械臂标定页当前选中项数值
 *
 * @return float 当前选中项数值
 */
static float lcd_show_mipi_arm_calib_get_selected_value_unlocked(void)
{
    switch (g_mipi_selected_index)
    {
        case 0U:
            return (float)lcd_show_mipi_arm_calib_point_index_unlocked() + 1.0f;

        case 1U:
            return g_mipi_arm_calib_z_mm;

        default:
            return 0.0f;
    }
}

/**
 * @brief  设置机械臂标定页当前选中项数值
 *
 * @param value 新数值
 * @return void
 */
static void lcd_show_mipi_arm_calib_set_selected_value_unlocked(float value)
{
    switch (g_mipi_selected_index)
    {
        case 0U:
            g_mipi_arm_calib_point_index =
                (uint8_t)(lcd_show_mipi_limit_i32((int32_t)value,
                                                  1,
                                                  (int32_t)LCD_SHOW_ARM_CALIB_POINT_COUNT) - 1);
            break;

        case 1U:
            g_mipi_arm_calib_z_mm = lcd_show_mipi_limit_float(value, MECH_HAND_MENU_Z_MIN, MECH_HAND_MENU_Z_MAX);
            break;

        default:
            break;
    }
}

/**
 * @brief  获取机械臂标定页当前编辑步进
 *
 * @return float 编辑步进
 */
static float lcd_show_mipi_arm_calib_edit_step_unlocked(void)
{
    if (0U == g_mipi_selected_index)
    {
        return 1.0f;
    }

    if (1U == g_mipi_selected_index)
    {
        return LCD_SHOW_ARM_COORD_STEP_MM;
    }

    return 0.0f;
}

/**
 * @brief  判断机械臂标定页当前选中项是否可编辑
 *
 * @return bool true 表示可编辑
 */
static bool lcd_show_mipi_arm_calib_selected_editable_unlocked(void)
{
    return g_mipi_selected_index < LCD_SHOW_ARM_CALIB_PARAM_COUNT;
}

/**
 * @brief  移动机械臂标定页选中项
 *
 * @param direction 正数向后，负数向前
 * @return void
 */
static void lcd_show_mipi_arm_calib_move_selection_unlocked(int32_t direction)
{
    if (g_mipi_editing)
    {
        return;
    }

    if (direction > 0)
    {
        g_mipi_selected_index++;
        if (g_mipi_selected_index >= LCD_SHOW_ARM_CALIB_ITEM_COUNT)
        {
            g_mipi_selected_index = 0U;
        }
    }
    else
    {
        if (0U == g_mipi_selected_index)
        {
            g_mipi_selected_index = LCD_SHOW_ARM_CALIB_ITEM_COUNT - 1U;
        }
        else
        {
            g_mipi_selected_index--;
        }
    }

    lcd_show_mark_dirty();
}

/**
 * @brief  开始编辑机械臂标定页选中项
 *
 * @return void
 */
static void lcd_show_mipi_arm_calib_begin_edit_unlocked(void)
{
    if (!lcd_show_mipi_arm_calib_selected_editable_unlocked())
    {
        return;
    }

    g_mipi_edit_original_value = lcd_show_mipi_arm_calib_get_selected_value_unlocked();
    g_mipi_edit_value = g_mipi_edit_original_value;
    g_mipi_editing = true;
    lcd_show_mark_dirty();
}

/**
 * @brief  确认机械臂标定页编辑值
 *
 * @return void
 */
static void lcd_show_mipi_arm_calib_confirm_edit_unlocked(void)
{
    if (!g_mipi_editing)
    {
        return;
    }

    lcd_show_mipi_arm_calib_set_selected_value_unlocked(g_mipi_edit_value);
    g_mipi_editing = false;
    lcd_show_mark_dirty();
}

/**
 * @brief  取消机械臂标定页编辑值
 *
 * @return void
 */
static void lcd_show_mipi_arm_calib_cancel_edit_unlocked(void)
{
    g_mipi_edit_value = g_mipi_edit_original_value;
    g_mipi_editing = false;
    lcd_show_mark_dirty();
}

/**
 * @brief  调整机械臂标定页编辑值
 *
 * @param direction 正数增大，负数减小
 * @return void
 */
static void lcd_show_mipi_arm_calib_adjust_edit_unlocked(int32_t direction)
{
    float step = lcd_show_mipi_arm_calib_edit_step_unlocked();

    if (!g_mipi_editing)
    {
        return;
    }

    g_mipi_edit_value += (direction > 0) ? step : -step;
    lcd_show_mark_dirty();
}

/**
 * @brief  获取机械臂标定页当前选中动作
 *
 * @return lcd_show_mipi_arm_calib_action_t 当前动作
 */
static lcd_show_mipi_arm_calib_action_t lcd_show_mipi_arm_calib_selected_action_unlocked(void)
{
    if (g_mipi_selected_index == LCD_SHOW_ARM_CALIB_ACTION_BASE_INDEX)
    {
        return LCD_SHOW_MIPI_ARM_CALIB_ACTION_MOVE;
    }
    if (g_mipi_selected_index == (uint8_t)(LCD_SHOW_ARM_CALIB_ACTION_BASE_INDEX + 1U))
    {
        return LCD_SHOW_MIPI_ARM_CALIB_ACTION_PICK;
    }
    if (g_mipi_selected_index == (uint8_t)(LCD_SHOW_ARM_CALIB_ACTION_BASE_INDEX + 2U))
    {
        return LCD_SHOW_MIPI_ARM_CALIB_ACTION_GRIP;
    }
    if (g_mipi_selected_index == (uint8_t)(LCD_SHOW_ARM_CALIB_ACTION_BASE_INDEX + 3U))
    {
        return LCD_SHOW_MIPI_ARM_CALIB_ACTION_ABOVE;
    }
    if (g_mipi_selected_index == (uint8_t)(LCD_SHOW_ARM_CALIB_ACTION_BASE_INDEX + 4U))
    {
        return LCD_SHOW_MIPI_ARM_CALIB_ACTION_HOME;
    }
    if (g_mipi_selected_index == (uint8_t)(LCD_SHOW_ARM_CALIB_ACTION_BASE_INDEX + 5U))
    {
        return LCD_SHOW_MIPI_ARM_CALIB_ACTION_STOP;
    }

    return LCD_SHOW_MIPI_ARM_CALIB_ACTION_NONE;
}

/**
 * @brief  取消机械臂标定自动流程
 *
 * @param state 对外显示状态
 * @return void
 */
static void lcd_show_mipi_arm_calib_cancel_auto_unlocked(mipi_arm_calib_flow_state_t state)
{
    g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE;
    g_mipi_arm_calib_wait_ms = 0U;
    g_mipi_arm_calib_wait_total_ms = 0U;
    g_mipi_arm_calib_flow_state = state;
    lcd_show_mark_dirty();
}

/**
 * @brief  执行机械臂标定页移动命令
 *
 * @param move_above true 表示移动到目标上方
 * @return int 机械臂动作返回码
 */
static int lcd_show_mipi_arm_calib_issue_move(bool move_above)
{
    uint8_t point_index = 0U;
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    uint16_t time_ms = 0U;
    int result = MECH_HAND_OK;

    lcd_show_enter_critical();
    point_index = lcd_show_mipi_arm_calib_point_index_unlocked();
    x_mm = g_mipi_arm_calib_points[point_index].x_mm;
    y_mm = g_mipi_arm_calib_points[point_index].y_mm;
    z_mm = move_above ? lcd_show_mipi_arm_calib_above_z_unlocked() : g_mipi_arm_calib_z_mm;
    time_ms = g_mipi_arm_time_ms;
    lcd_show_exit_critical();

    if (!mechanical_arm_feedback.lock_bus(LCD_SHOW_ARM_BUS_LOCK_TIMEOUT_MS))
    {
        lcd_show_mipi_arm_sync_after_action(MECH_HAND_ERR_TIMEOUT);
        return MECH_HAND_ERR_TIMEOUT;
    }

    result = mechanical_hand.move_tcp_to(x_mm, y_mm, z_mm, time_ms);
    mechanical_arm_feedback.request_refresh_all();
    mechanical_arm_feedback.unlock_bus();

    if (MECH_HAND_OK == result)
    {
        lcd_show_enter_critical();
        g_mipi_arm_x_mm = x_mm;
        g_mipi_arm_y_mm = y_mm;
        g_mipi_arm_z_mm = z_mm;
        lcd_show_exit_critical();
    }

    lcd_show_mipi_arm_sync_after_action(result);
    return result;
}

/**
 * @brief  执行机械臂标定页避障平移命令
 *
 * @return int 机械臂动作返回码
 */
static int lcd_show_mipi_arm_calib_issue_clear(void)
{
    uint8_t point_index = 0U;
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    uint16_t time_ms = 0U;
    int result = MECH_HAND_OK;

    lcd_show_enter_critical();
    point_index = lcd_show_mipi_arm_calib_point_index_unlocked();
    x_mm = g_mipi_arm_calib_points[point_index].x_mm;
    y_mm = g_mipi_arm_calib_points[point_index].y_mm;
    z_mm = lcd_show_mipi_arm_calib_above_z_unlocked();
    time_ms = g_mipi_arm_time_ms;
    lcd_show_exit_critical();

    if (x_mm < 0.0f)
    {
        x_mm += LCD_SHOW_ARM_CALIB_SIDE_CLEAR_OFFSET_MM;
    }
    else if (x_mm > 0.0f)
    {
        x_mm -= LCD_SHOW_ARM_CALIB_SIDE_CLEAR_OFFSET_MM;
    }
    else
    {
        y_mm -= LCD_SHOW_ARM_CALIB_CENTER_BACK_OFFSET_MM;
    }

    x_mm = lcd_show_mipi_limit_float(x_mm, MECH_HAND_MENU_X_MIN, MECH_HAND_MENU_X_MAX);
    y_mm = lcd_show_mipi_limit_float(y_mm, MECH_HAND_MENU_Y_MIN, MECH_HAND_MENU_Y_MAX);

    if (!mechanical_arm_feedback.lock_bus(LCD_SHOW_ARM_BUS_LOCK_TIMEOUT_MS))
    {
        lcd_show_mipi_arm_sync_after_action(MECH_HAND_ERR_TIMEOUT);
        return MECH_HAND_ERR_TIMEOUT;
    }

    result = mechanical_hand.move_tcp_to(x_mm, y_mm, z_mm, time_ms);
    mechanical_arm_feedback.request_refresh_all();
    mechanical_arm_feedback.unlock_bus();

    if (MECH_HAND_OK == result)
    {
        lcd_show_enter_critical();
        g_mipi_arm_x_mm = x_mm;
        g_mipi_arm_y_mm = y_mm;
        g_mipi_arm_z_mm = z_mm;
        lcd_show_exit_critical();
    }

    lcd_show_mipi_arm_sync_after_action(result);
    return result;
}

/**
 * @brief  执行机械臂标定页夹爪命令
 *
 * @param close_gripper true 表示闭合夹爪，false 表示打开夹爪
 * @return int 机械臂动作返回码
 */
static int lcd_show_mipi_arm_calib_issue_gripper(bool close_gripper)
{
    uint16_t time_ms = 0U;
    int result = MECH_HAND_OK;

    lcd_show_enter_critical();
    time_ms = g_mipi_arm_time_ms;
    lcd_show_exit_critical();

    if (!mechanical_arm_feedback.lock_bus(LCD_SHOW_ARM_BUS_LOCK_TIMEOUT_MS))
    {
        lcd_show_mipi_arm_sync_after_action(MECH_HAND_ERR_TIMEOUT);
        return MECH_HAND_ERR_TIMEOUT;
    }

    result = close_gripper ? mechanical_hand.gripper_close(time_ms) : mechanical_hand.gripper_open(time_ms);
    mechanical_arm_feedback.request_refresh_all();
    mechanical_arm_feedback.unlock_bus();

    if (MECH_HAND_OK == result)
    {
        lcd_show_enter_critical();
        g_mipi_arm_gripper_opened = !close_gripper;
        lcd_show_exit_critical();
    }

    lcd_show_mipi_arm_sync_after_action(result);
    return result;
}

/**
 * @brief  执行机械臂标定页 Home 命令
 *
 * @return int 机械臂动作返回码
 */
static int lcd_show_mipi_arm_calib_issue_home(void)
{
    uint16_t time_ms = 0U;
    int result = MECH_HAND_OK;

    lcd_show_enter_critical();
    time_ms = g_mipi_arm_time_ms;
    lcd_show_exit_critical();

    if (!mechanical_arm_feedback.lock_bus(LCD_SHOW_ARM_BUS_LOCK_TIMEOUT_MS))
    {
        lcd_show_mipi_arm_sync_after_action(MECH_HAND_ERR_TIMEOUT);
        return MECH_HAND_ERR_TIMEOUT;
    }

    result = mechanical_hand.home(time_ms);
    mechanical_arm_feedback.request_refresh_all();
    mechanical_arm_feedback.unlock_bus();
    lcd_show_mipi_arm_sync_after_action(result);
    return result;
}

/**
 * @brief  启动机械臂标定页自动松夹爪回收流程
 *
 * @return void
 */
static void lcd_show_mipi_arm_calib_start_pick(void)
{
    uint16_t time_ms = 0U;
    int result = MECH_HAND_OK;

    lcd_show_enter_critical();
    if (LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE != g_mipi_arm_calib_pick_state)
    {
        lcd_show_exit_critical();
        return;
    }
    time_ms = g_mipi_arm_time_ms;
    g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_OPEN;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();

    result = lcd_show_mipi_arm_calib_issue_gripper(false);

    lcd_show_enter_critical();
    if (MECH_HAND_OK == result)
    {
        g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_OPEN;
        g_mipi_arm_calib_wait_ms = 0U;
        g_mipi_arm_calib_wait_total_ms = (uint32_t)time_ms + LCD_SHOW_ARM_CALIB_ACTION_SETTLE_MS;
    }
    else
    {
        lcd_show_mipi_arm_calib_cancel_auto_unlocked(MIPI_ARM_CALIB_FLOW_ERR);
    }
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  执行机械臂标定页动作
 *
 * @param action 待执行动作
 * @return void
 */
static void lcd_show_mipi_execute_arm_calib_action(lcd_show_mipi_arm_calib_action_t action)
{
    bool gripper_opened = false;
    int result = MECH_HAND_OK;

    if (LCD_SHOW_MIPI_ARM_CALIB_ACTION_NONE == action)
    {
        return;
    }

    lcd_show_enter_critical();
    if ((LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE != g_mipi_arm_calib_pick_state) &&
        (LCD_SHOW_MIPI_ARM_CALIB_ACTION_STOP != action))
    {
        lcd_show_exit_critical();
        return;
    }
    gripper_opened = g_mipi_arm_gripper_opened;
    lcd_show_exit_critical();

    switch (action)
    {
        case LCD_SHOW_MIPI_ARM_CALIB_ACTION_MOVE:
            lcd_show_enter_critical();
            g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_IDLE;
            lcd_show_mark_dirty();
            lcd_show_exit_critical();
            result = lcd_show_mipi_arm_calib_issue_gripper(false);
            if (MECH_HAND_OK == result)
            {
                (void)lcd_show_mipi_arm_calib_issue_move(false);
            }
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_ACTION_ABOVE:
            lcd_show_enter_critical();
            g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_IDLE;
            lcd_show_mark_dirty();
            lcd_show_exit_critical();
            (void)lcd_show_mipi_arm_calib_issue_move(true);
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_ACTION_HOME:
            lcd_show_enter_critical();
            g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_IDLE;
            lcd_show_mark_dirty();
            lcd_show_exit_critical();
            (void)lcd_show_mipi_arm_calib_issue_home();
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_ACTION_GRIP:
            lcd_show_enter_critical();
            g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_IDLE;
            lcd_show_mark_dirty();
            lcd_show_exit_critical();
            (void)lcd_show_mipi_arm_calib_issue_gripper(gripper_opened);
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_ACTION_PICK:
            lcd_show_mipi_arm_calib_start_pick();
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_ACTION_STOP:
            lcd_show_enter_critical();
            lcd_show_mipi_arm_calib_cancel_auto_unlocked(MIPI_ARM_CALIB_FLOW_STOP);
            lcd_show_exit_critical();
            lcd_show_mipi_execute_arm_action(LCD_SHOW_MIPI_ARM_ACTION_STOP);
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_ACTION_NONE:
        default:
            break;
    }
}

/**
 * @brief  处理机械臂标定页确认键
 *
 * @return void
 */
static void lcd_show_mipi_arm_calib_confirm(void)
{
    lcd_show_mipi_arm_calib_action_t action = LCD_SHOW_MIPI_ARM_CALIB_ACTION_NONE;

    lcd_show_enter_critical();
    if (g_mipi_editing)
    {
        lcd_show_mipi_arm_calib_confirm_edit_unlocked();
        lcd_show_exit_critical();
        return;
    }

    if (lcd_show_mipi_arm_calib_selected_editable_unlocked())
    {
        if (LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE == g_mipi_arm_calib_pick_state)
        {
            lcd_show_mipi_arm_calib_begin_edit_unlocked();
        }
    }
    else
    {
        action = lcd_show_mipi_arm_calib_selected_action_unlocked();
    }
    lcd_show_exit_critical();

    lcd_show_mipi_execute_arm_calib_action(action);
}

/**
 * @brief  推进机械臂标定页自动松夹爪回收流程
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_mipi_arm_calib_process_auto(uint32_t elapsed_ms)
{
    lcd_show_mipi_arm_calib_pick_state_t state = LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE;
    bool issue_above = false;
    bool issue_clear = false;
    bool issue_home = false;
    uint16_t time_ms = 0U;
    int result = MECH_HAND_OK;

    if (0U == elapsed_ms)
    {
        return;
    }

    lcd_show_enter_critical();
    state = g_mipi_arm_calib_pick_state;
    if (LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE == state)
    {
        lcd_show_exit_critical();
        return;
    }

    switch (state)
    {
        case LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_OPEN:
            g_mipi_arm_calib_wait_ms += elapsed_ms;
            if (g_mipi_arm_calib_wait_ms >= g_mipi_arm_calib_wait_total_ms)
            {
                g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_ISSUE_ABOVE;
                g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_ABOVE;
                g_mipi_arm_calib_wait_ms = 0U;
                issue_above = true;
                lcd_show_mark_dirty();
            }
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_PICK_ISSUE_ABOVE:
            issue_above = true;
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_ABOVE:
            g_mipi_arm_calib_wait_ms += elapsed_ms;
            if (g_mipi_arm_calib_wait_ms >= g_mipi_arm_calib_wait_total_ms)
            {
                g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_ISSUE_CLEAR;
                g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_CLEAR;
                g_mipi_arm_calib_wait_ms = 0U;
                issue_clear = true;
                lcd_show_mark_dirty();
            }
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_PICK_ISSUE_CLEAR:
            issue_clear = true;
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_CLEAR:
            g_mipi_arm_calib_wait_ms += elapsed_ms;
            if (g_mipi_arm_calib_wait_ms >= g_mipi_arm_calib_wait_total_ms)
            {
                g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_ISSUE_HOME;
                g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_HOME;
                g_mipi_arm_calib_wait_ms = 0U;
                issue_home = true;
                lcd_show_mark_dirty();
            }
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_PICK_ISSUE_HOME:
            issue_home = true;
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_HOME:
            g_mipi_arm_calib_wait_ms += elapsed_ms;
            if (g_mipi_arm_calib_wait_ms >= g_mipi_arm_calib_wait_total_ms)
            {
                lcd_show_mipi_arm_calib_cancel_auto_unlocked(MIPI_ARM_CALIB_FLOW_DONE);
            }
            break;

        case LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE:
        default:
            break;
    }
    time_ms = g_mipi_arm_time_ms;
    lcd_show_exit_critical();

    if (issue_above)
    {
        result = lcd_show_mipi_arm_calib_issue_move(true);
        lcd_show_enter_critical();
        if (MECH_HAND_OK == result)
        {
            g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_ABOVE;
            g_mipi_arm_calib_wait_ms = 0U;
            g_mipi_arm_calib_wait_total_ms = (uint32_t)time_ms + LCD_SHOW_ARM_CALIB_ACTION_SETTLE_MS;
        }
        else
        {
            lcd_show_mipi_arm_calib_cancel_auto_unlocked(MIPI_ARM_CALIB_FLOW_ERR);
        }
        lcd_show_mark_dirty();
        lcd_show_exit_critical();
        return;
    }

    if (issue_clear)
    {
        result = lcd_show_mipi_arm_calib_issue_clear();
        lcd_show_enter_critical();
        if (MECH_HAND_OK == result)
        {
            g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_CLEAR;
            g_mipi_arm_calib_wait_ms = 0U;
            g_mipi_arm_calib_wait_total_ms = (uint32_t)time_ms + LCD_SHOW_ARM_CALIB_ACTION_SETTLE_MS;
        }
        else
        {
            lcd_show_mipi_arm_calib_cancel_auto_unlocked(MIPI_ARM_CALIB_FLOW_ERR);
        }
        lcd_show_mark_dirty();
        lcd_show_exit_critical();
        return;
    }

    if (issue_home)
    {
        result = lcd_show_mipi_arm_calib_issue_home();
        lcd_show_enter_critical();
        if (MECH_HAND_OK == result)
        {
            g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_WAIT_HOME;
            g_mipi_arm_calib_wait_ms = 0U;
            g_mipi_arm_calib_wait_total_ms = (uint32_t)time_ms + LCD_SHOW_ARM_CALIB_ACTION_SETTLE_MS;
        }
        else
        {
            lcd_show_mipi_arm_calib_cancel_auto_unlocked(MIPI_ARM_CALIB_FLOW_ERR);
        }
        lcd_show_mark_dirty();
        lcd_show_exit_critical();
    }
}

/**
 * @brief  处理 MIPI 首页按键事件
 *
 * @param p_event 按键事件
 * @return void
 */
static void lcd_show_mipi_home_process_key_event(const key_event_t *p_event)
{
    mipi_home_page_t selected_page = MIPI_HOME_PAGE_CHASSIS;

    if (NULL == p_event)
    {
        return;
    }

    lcd_show_enter_critical();
    if ((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CW == p_event->type))
    {
        lcd_show_mipi_home_next_unlocked();
    }
    else if ((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CCW == p_event->type))
    {
        lcd_show_mipi_home_prev_unlocked();
    }
    else if ((KEY_ID_1 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_mipi_home_prev_unlocked();
    }
    else if ((KEY_ID_2 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_mipi_home_next_unlocked();
    }
    selected_page = g_mipi_home_selected;
    lcd_show_exit_critical();

    if (((KEY_ID_3 == p_event->id) || (KEY_ID_ENCODER_BUTTON == p_event->id)) &&
        (KEY_EVENT_CLICK == p_event->type))
    {
        if (MIPI_HOME_PAGE_CHASSIS == selected_page)
        {
            lcd_show_mipi_enter_chassis_page();
        }
        else if (MIPI_HOME_PAGE_RADAR == selected_page)
        {
            lcd_show_mipi_enter_radar_cloud_page();
        }
        else if (MIPI_HOME_PAGE_ARM == selected_page)
        {
            lcd_show_mipi_enter_arm_debug_page();
        }
        else if (MIPI_HOME_PAGE_MISSION == selected_page)
        {
            lcd_show_mipi_enter_mission_page();
        }
        else if (MIPI_HOME_PAGE_VOICE == selected_page)
        {
            lcd_show_mipi_enter_voice_dataset_page();
        }
    }
}

/**
 * @brief  处理 MIPI 占位页按键事件
 *
 * @param p_event 按键事件
 * @return void
 */
static void lcd_show_mipi_placeholder_process_key_event(const key_event_t *p_event)
{
    if (NULL == p_event)
    {
        return;
    }

    if ((KEY_EVENT_CLICK == p_event->type) || (KEY_EVENT_LONG == p_event->type))
    {
        lcd_show_mipi_back_home(false);
    }
}

/**
 * @brief  处理 MIPI 底盘页点击确认
 *
 * @return void
 */
static void lcd_show_mipi_chassis_confirm(void)
{
    lcd_show_mipi_chassis_action_t action = LCD_SHOW_MIPI_CHASSIS_ACTION_NONE;

    lcd_show_enter_critical();
    if (g_mipi_editing)
    {
        lcd_show_mipi_chassis_confirm_edit_unlocked();
        lcd_show_exit_critical();
        return;
    }

    if (0U == g_mipi_selected_index)
    {
        lcd_show_mipi_chassis_switch_mode_unlocked(MIPI_CHASSIS_MODE_BODY);
    }
    else if (1U == g_mipi_selected_index)
    {
        lcd_show_mipi_chassis_switch_mode_unlocked(MIPI_CHASSIS_MODE_WHEEL);
    }
    else if (2U == g_mipi_selected_index)
    {
        lcd_show_mipi_chassis_switch_mode_unlocked(MIPI_CHASSIS_MODE_PWM);
    }
    else if (lcd_show_mipi_chassis_selected_editable_unlocked())
    {
        lcd_show_mipi_chassis_begin_edit_unlocked();
    }
    else
    {
        action = lcd_show_mipi_chassis_selected_action_unlocked();
    }
    lcd_show_exit_critical();

    lcd_show_mipi_execute_action(action);
}

/**
 * @brief  处理 MIPI 底盘页按键事件
 *
 * @param p_event 按键事件
 * @return void
 */
static void lcd_show_mipi_chassis_process_key_event(const key_event_t *p_event)
{
    if (NULL == p_event)
    {
        return;
    }

    if (KEY_EVENT_LONG == p_event->type)
    {
        if (g_mipi_editing)
        {
            lcd_show_enter_critical();
            lcd_show_mipi_chassis_cancel_edit_unlocked();
            lcd_show_exit_critical();
        }
        else
        {
            lcd_show_mipi_back_home(true);
        }
        return;
    }

    if ((KEY_ID_4 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        if (g_mipi_editing)
        {
            lcd_show_enter_critical();
            lcd_show_mipi_chassis_cancel_edit_unlocked();
            lcd_show_exit_critical();
        }
        else
        {
            lcd_show_mipi_back_home(true);
        }
        return;
    }

    if ((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CW == p_event->type))
    {
        lcd_show_enter_critical();
        if (g_mipi_editing)
        {
            lcd_show_mipi_chassis_adjust_edit_unlocked(1);
        }
        else
        {
            lcd_show_mipi_chassis_move_selection_unlocked(1);
        }
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CCW == p_event->type))
    {
        lcd_show_enter_critical();
        if (g_mipi_editing)
        {
            lcd_show_mipi_chassis_adjust_edit_unlocked(-1);
        }
        else
        {
            lcd_show_mipi_chassis_move_selection_unlocked(-1);
        }
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_1 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_enter_critical();
        lcd_show_mipi_chassis_move_selection_unlocked(-1);
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_2 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_enter_critical();
        lcd_show_mipi_chassis_move_selection_unlocked(1);
        lcd_show_exit_critical();
        return;
    }

    if (((KEY_ID_3 == p_event->id) || (KEY_ID_ENCODER_BUTTON == p_event->id)) &&
        (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_mipi_chassis_confirm();
    }
}

/**
 * @brief  处理 MIPI 雷达点云页按键事件
 *
 * @param p_event 按键事件
 * @return void
 */
static void lcd_show_mipi_radar_cloud_process_key_event(const key_event_t *p_event)
{
    if (NULL == p_event)
    {
        return;
    }

    if ((KEY_EVENT_LONG == p_event->type) ||
        ((KEY_ID_4 == p_event->id) && (KEY_EVENT_CLICK == p_event->type)))
    {
        lcd_show_mipi_back_home(false);
    }
}

/**
 * @brief  退出 Mission 页面并停止 mission
 *
 * @return void
 */
static void lcd_show_mipi_mission_exit(void)
{
    mission.request_stop();
    lcd_show_mipi_back_home(false);
}

/**
 * @brief  处理 MIPI Mission 页按键事件
 *
 * @param p_event 按键事件
 * @return void
 */
static void lcd_show_mipi_mission_process_key_event(const key_event_t *p_event)
{
    if (NULL == p_event)
    {
        return;
    }

    if (KEY_EVENT_LONG == p_event->type)
    {
        lcd_show_mipi_mission_exit();
        return;
    }

    if ((KEY_ID_4 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_mipi_mission_exit();
        return;
    }

    if (((KEY_ID_3 == p_event->id) || (KEY_ID_ENCODER_BUTTON == p_event->id)) &&
        (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_mipi_mission_request_start();
        return;
    }
}

/**
 * @brief  请求开始语音数据集录制
 *
 * @return void
 */
static void lcd_show_mipi_voice_start(void)
{
    voice_dataset_status_t status = {0};
    const char *label = NULL;

    lcd_show_enter_critical();
    label = voice_dataset_label_name(g_mipi_voice_label_index);
    lcd_show_exit_critical();

    if (NULL == label)
    {
        label = VOICE_DATASET_DEFAULT_LABEL;
    }

    (void)voice_ai.set_enabled(false);
    (void)voice_dataset.start(label);
    voice_dataset.status_get(&status);

    lcd_show_enter_critical();
    g_mipi_voice_status = status;
    g_mipi_voice_refresh_ms = LCD_SHOW_MIPI_VOICE_REFRESH_PERIOD_MS;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  处理 MIPI 语音数据集页按键事件
 *
 * @param p_event 按键事件
 * @return void
 */
static void lcd_show_mipi_voice_process_key_event(const key_event_t *p_event)
{
    bool busy = false;

    if (NULL == p_event)
    {
        return;
    }

    if ((KEY_EVENT_LONG == p_event->type) ||
        ((KEY_ID_4 == p_event->id) && (KEY_EVENT_CLICK == p_event->type)))
    {
        lcd_show_mipi_back_home(false);
        return;
    }

    lcd_show_enter_critical();
    busy = lcd_show_mipi_voice_busy_unlocked();
    lcd_show_exit_critical();
    if (busy)
    {
        return;
    }

    if (((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CW == p_event->type)) ||
        ((KEY_ID_2 == p_event->id) && (KEY_EVENT_CLICK == p_event->type)))
    {
        lcd_show_mipi_voice_next_label();
        return;
    }

    if (((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CCW == p_event->type)) ||
        ((KEY_ID_1 == p_event->id) && (KEY_EVENT_CLICK == p_event->type)))
    {
        lcd_show_mipi_voice_prev_label();
        return;
    }

    if (((KEY_ID_3 == p_event->id) || (KEY_ID_ENCODER_BUTTON == p_event->id)) &&
        (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_mipi_voice_start();
        return;
    }
}

/**
 * @brief  处理 MIPI 机械臂页按键事件
 *
 * @param p_event 按键事件
 * @return void
 */
static void lcd_show_mipi_arm_debug_process_key_event(const key_event_t *p_event)
{
    if (NULL == p_event)
    {
        return;
    }

    if (KEY_EVENT_LONG == p_event->type)
    {
        lcd_show_mipi_execute_arm_action(LCD_SHOW_MIPI_ARM_ACTION_STOP);
        if (!g_mipi_editing)
        {
            return;
        }

        lcd_show_enter_critical();
        lcd_show_mipi_arm_cancel_edit_unlocked();
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_4 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        if (g_mipi_editing)
        {
            lcd_show_enter_critical();
            lcd_show_mipi_arm_cancel_edit_unlocked();
            lcd_show_exit_critical();
        }
        else
        {
            lcd_show_mipi_back_home(false);
        }
        return;
    }

    if ((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CW == p_event->type))
    {
        lcd_show_enter_critical();
        if (g_mipi_editing)
        {
            lcd_show_mipi_arm_adjust_edit_unlocked(1);
        }
        else
        {
            lcd_show_mipi_arm_move_selection_unlocked(1);
        }
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CCW == p_event->type))
    {
        lcd_show_enter_critical();
        if (g_mipi_editing)
        {
            lcd_show_mipi_arm_adjust_edit_unlocked(-1);
        }
        else
        {
            lcd_show_mipi_arm_move_selection_unlocked(-1);
        }
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_1 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_enter_critical();
        lcd_show_mipi_arm_move_selection_unlocked(-1);
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_2 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_enter_critical();
        lcd_show_mipi_arm_move_selection_unlocked(1);
        lcd_show_exit_critical();
        return;
    }

    if (((KEY_ID_3 == p_event->id) || (KEY_ID_ENCODER_BUTTON == p_event->id)) &&
        (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_mipi_arm_confirm();
    }
}

/**
 * @brief  处理 MIPI 机械臂标定页按键事件
 *
 * @param p_event 按键事件
 * @return void
 */
static void lcd_show_mipi_arm_calib_process_key_event(const key_event_t *p_event)
{
    bool auto_running = false;

    if (NULL == p_event)
    {
        return;
    }

    if (KEY_EVENT_LONG == p_event->type)
    {
        lcd_show_mipi_execute_arm_calib_action(LCD_SHOW_MIPI_ARM_CALIB_ACTION_STOP);
        if (!g_mipi_editing)
        {
            return;
        }

        lcd_show_enter_critical();
        lcd_show_mipi_arm_calib_cancel_edit_unlocked();
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_4 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_enter_critical();
        auto_running = (LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE != g_mipi_arm_calib_pick_state);
        if (g_mipi_editing)
        {
            lcd_show_mipi_arm_calib_cancel_edit_unlocked();
            lcd_show_exit_critical();
        }
        else if (!auto_running)
        {
            g_mipi_view = MIPI_UI_VIEW_ARM_DEBUG;
            g_mipi_selected_index = LCD_SHOW_ARM_ACTION_BASE_INDEX + 3U;
            lcd_show_mark_dirty();
            lcd_show_exit_critical();
        }
        else
        {
            lcd_show_exit_critical();
        }
        return;
    }

    if ((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CW == p_event->type))
    {
        lcd_show_enter_critical();
        if (g_mipi_editing)
        {
            lcd_show_mipi_arm_calib_adjust_edit_unlocked(1);
        }
        else
        {
            lcd_show_mipi_arm_calib_move_selection_unlocked(1);
        }
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_ENCODER == p_event->id) && (KEY_EVENT_ROT_CCW == p_event->type))
    {
        lcd_show_enter_critical();
        if (g_mipi_editing)
        {
            lcd_show_mipi_arm_calib_adjust_edit_unlocked(-1);
        }
        else
        {
            lcd_show_mipi_arm_calib_move_selection_unlocked(-1);
        }
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_1 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_enter_critical();
        lcd_show_mipi_arm_calib_move_selection_unlocked(-1);
        lcd_show_exit_critical();
        return;
    }

    if ((KEY_ID_2 == p_event->id) && (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_enter_critical();
        lcd_show_mipi_arm_calib_move_selection_unlocked(1);
        lcd_show_exit_critical();
        return;
    }

    if (((KEY_ID_3 == p_event->id) || (KEY_ID_ENCODER_BUTTON == p_event->id)) &&
        (KEY_EVENT_CLICK == p_event->type))
    {
        lcd_show_mipi_arm_calib_confirm();
    }
}

/**
 * @brief  处理一个 MIPI 按键或编码器事件
 *
 * @param p_event 按键事件
 * @return void
 */
static void lcd_show_mipi_process_key_event(const key_event_t *p_event)
{
    mipi_ui_view_t view = MIPI_UI_VIEW_BOOT;

    if (NULL == p_event)
    {
        return;
    }

    lcd_show_enter_critical();
    view = g_mipi_view;
    lcd_show_exit_critical();

    if (MIPI_UI_VIEW_HOME == view)
    {
        lcd_show_mipi_home_process_key_event(p_event);
    }
    else if (MIPI_UI_VIEW_CHASSIS == view)
    {
        lcd_show_mipi_chassis_process_key_event(p_event);
    }
    else if (MIPI_UI_VIEW_RADAR_CLOUD == view)
    {
        lcd_show_mipi_radar_cloud_process_key_event(p_event);
    }
    else if (MIPI_UI_VIEW_ARM_DEBUG == view)
    {
        lcd_show_mipi_arm_debug_process_key_event(p_event);
    }
    else if (MIPI_UI_VIEW_ARM_CALIB == view)
    {
        lcd_show_mipi_arm_calib_process_key_event(p_event);
    }
    else if (MIPI_UI_VIEW_MISSION == view)
    {
        lcd_show_mipi_mission_process_key_event(p_event);
    }
    else if (MIPI_UI_VIEW_VOICE_DATASET == view)
    {
        lcd_show_mipi_voice_process_key_event(p_event);
    }
    else if (MIPI_UI_VIEW_PLACEHOLDER == view)
    {
        lcd_show_mipi_placeholder_process_key_event(p_event);
    }
}

/**
 * @brief  MIPI 分支扫描按键并消费事件
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_mipi_process_key_events(void)
{
    key_event_t event = {0};

    while (key.get_event(&event))
    {
        lcd_show_mipi_process_key_event(&event);
    }
}

/**
 * @brief  MIPI 分支按周期读取电压电流
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_mipi_update_power_status(uint32_t elapsed_ms)
{
    ina226_sample_t sample = {0};
    fsp_err_t err = FSP_SUCCESS;
    uint32_t voltage_delta_mv = 0U;
    uint32_t current_delta_ua = 0U;
    bool valid = false;
    bool changed = false;

    g_mipi_power_refresh_ms += elapsed_ms;
    if (g_mipi_power_refresh_ms < LCD_SHOW_POWER_REFRESH_PERIOD_MS)
    {
        return;
    }
    g_mipi_power_refresh_ms = 0U;

    err = ina226.update();
    if (FSP_SUCCESS == err)
    {
        err = ina226.read(&sample);
    }
    valid = (FSP_SUCCESS == err);

    lcd_show_enter_critical();
    if (valid)
    {
        voltage_delta_mv = (sample.bus_voltage_mv > g_mipi_bus_voltage_mv) ?
                           (sample.bus_voltage_mv - g_mipi_bus_voltage_mv) :
                           (g_mipi_bus_voltage_mv - sample.bus_voltage_mv);
        current_delta_ua = (sample.current_ua > g_mipi_current_ua) ?
                           (uint32_t)(sample.current_ua - g_mipi_current_ua) :
                           (uint32_t)(g_mipi_current_ua - sample.current_ua);
        changed = (!g_mipi_power_valid) ||
                  (voltage_delta_mv >= LCD_SHOW_MIPI_POWER_VOLTAGE_EPS_MV) ||
                  (current_delta_ua >= LCD_SHOW_MIPI_POWER_CURRENT_EPS_UA);
    }
    else
    {
        changed = g_mipi_power_valid;
    }

    if (changed)
    {
        g_mipi_power_valid = valid;
        if (valid)
        {
            g_mipi_bus_voltage_mv = sample.bus_voltage_mv;
            g_mipi_current_ua = sample.current_ua;
        }
        lcd_show_mark_dirty();
    }
    lcd_show_exit_critical();
}

/**
 * @brief  MIPI 分支按周期刷新编码器反馈
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_mipi_update_feedback_status(uint32_t elapsed_ms)
{
    float measured_speed_mps[CHASSIS_WHEEL_COUNT] = {0.0f};
    float rpm[CHASSIS_WHEEL_COUNT] = {0.0f};
    bool need_update = false;
    bool changed = false;

    g_mipi_feedback_refresh_ms += elapsed_ms;
    if (g_mipi_feedback_refresh_ms < LCD_SHOW_MIPI_FEEDBACK_REFRESH_PERIOD_MS)
    {
        return;
    }
    g_mipi_feedback_refresh_ms = 0U;

    lcd_show_enter_critical();
    need_update = (MIPI_UI_VIEW_CHASSIS == g_mipi_view);
    lcd_show_exit_critical();
    if (!need_update)
    {
        return;
    }

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        measured_speed_mps[i] = encoder.get_speed((chassis_wheel_t)i);
        rpm[i] = encoder.get_rpm((chassis_wheel_t)i);
    }

    lcd_show_enter_critical();
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (lcd_show_mipi_abs_float(g_mipi_wheel_measured_mps[i] - measured_speed_mps[i]) >
            LCD_SHOW_MECANUM_SPEED_EPS_MPS)
        {
            g_mipi_wheel_measured_mps[i] = measured_speed_mps[i];
            changed = true;
        }

        if (lcd_show_mipi_abs_float(g_mipi_wheel_rpm[i] - rpm[i]) > 1.0f)
        {
            g_mipi_wheel_rpm[i] = rpm[i];
        }
    }

    if (changed)
    {
        lcd_show_mark_dirty();
    }
    lcd_show_exit_critical();
}

/**
 * @brief  判断雷达扇区是否有有效距离
 *
 * @param p_sector 雷达扇区指针
 * @return bool true 表示该扇区可显示
 */
static bool lcd_show_mipi_radar_sector_valid(const n10_radar_cloud_sector_t *p_sector)
{
    if (NULL == p_sector)
    {
        return false;
    }

    if (0U == p_sector->valid_count)
    {
        return false;
    }

    if ((0U == p_sector->min_distance_mm) ||
        (N10_RADAR_CLOUD_INVALID_DISTANCE_MM == p_sector->min_distance_mm))
    {
        return false;
    }

    return true;
}

/**
 * @brief  MIPI 分支按周期读取雷达点云和避障只读状态
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_mipi_refresh_radar_cloud(uint32_t elapsed_ms)
{
    n10_radar_cloud_t cloud = {0};
    chassis_avoidance_status_t status = {0};
    uint16_t sector_min_mm[N10_RADAR_CLOUD_SECTOR_COUNT] = {0U};
    uint8_t sector_valid[N10_RADAR_CLOUD_SECTOR_COUNT] = {0U};
    uint16_t nearest_mm = N10_RADAR_CLOUD_INVALID_DISTANCE_MM;
    fsp_err_t cloud_err = FSP_SUCCESS;
    fsp_err_t status_err = FSP_SUCCESS;
    bool need_update = false;
    bool cloud_valid = false;
    bool changed = false;

    lcd_show_enter_critical();
    need_update = (MIPI_UI_VIEW_RADAR_CLOUD == g_mipi_view);
    lcd_show_exit_critical();
    if (!need_update)
    {
        return;
    }

    g_mipi_radar_refresh_ms += elapsed_ms;
    if (g_mipi_radar_refresh_ms < LCD_SHOW_MIPI_RADAR_REFRESH_PERIOD_MS)
    {
        return;
    }
    g_mipi_radar_refresh_ms = 0U;

    cloud_err = n10_radar.cloud_read(&cloud);
    cloud_valid = (FSP_SUCCESS == cloud_err);
    if (cloud_valid)
    {
        for (uint32_t i = 0U; i < N10_RADAR_CLOUD_SECTOR_COUNT; i++)
        {
            if (lcd_show_mipi_radar_sector_valid(&cloud.sectors[i]))
            {
                sector_valid[i] = 1U;
                sector_min_mm[i] = cloud.sectors[i].min_distance_mm;
                if (cloud.sectors[i].min_distance_mm < nearest_mm)
                {
                    nearest_mm = cloud.sectors[i].min_distance_mm;
                }
            }
        }

        if (N10_RADAR_CLOUD_INVALID_DISTANCE_MM == nearest_mm)
        {
            nearest_mm = 0U;
        }
    }

    status_err = chassis_avoidance.status_get(&status);

    lcd_show_enter_critical();
    if (cloud_valid)
    {
        changed = (!g_mipi_radar_cloud_valid) ||
                  (g_mipi_radar_cloud_sequence != cloud.sequence) ||
                  (g_mipi_radar_source_scan_sequence != cloud.source_scan_sequence) ||
                  (lcd_show_mipi_abs_float(g_mipi_radar_rpm - cloud.rpm) > 0.1f) ||
                  (g_mipi_radar_nearest_distance_mm != nearest_mm);

        g_mipi_radar_cloud_valid = true;
        g_mipi_radar_cloud_sequence = cloud.sequence;
        g_mipi_radar_source_scan_sequence = cloud.source_scan_sequence;
        g_mipi_radar_rpm = cloud.rpm;
        g_mipi_radar_nearest_distance_mm = nearest_mm;

        for (uint32_t i = 0U; i < N10_RADAR_CLOUD_SECTOR_COUNT; i++)
        {
            if ((g_mipi_radar_sector_valid[i] != sector_valid[i]) ||
                (g_mipi_radar_sector_min_mm[i] != sector_min_mm[i]))
            {
                changed = true;
            }
            g_mipi_radar_sector_valid[i] = sector_valid[i];
            g_mipi_radar_sector_min_mm[i] = sector_min_mm[i];
        }
    }
    else if (g_mipi_radar_cloud_valid)
    {
        g_mipi_radar_cloud_valid = false;
        g_mipi_radar_nearest_distance_mm = 0U;
        for (uint32_t i = 0U; i < N10_RADAR_CLOUD_SECTOR_COUNT; i++)
        {
            g_mipi_radar_sector_valid[i] = 0U;
            g_mipi_radar_sector_min_mm[i] = 0U;
        }
        changed = true;
    }

    if (FSP_SUCCESS == status_err)
    {
        if ((g_mipi_avoidance_state != status.state) ||
            (g_mipi_avoidance_radar_stale_ms != status.radar_stale_ms) ||
            (lcd_show_mipi_abs_float(g_mipi_avoidance_target_angle_deg - status.target_angle_deg) > 0.1f) ||
            (lcd_show_mipi_abs_float(g_mipi_avoidance_selected_angle_deg - status.selected_angle_deg) > 0.1f) ||
            (g_mipi_avoidance_target_safe != status.target_safe))
        {
            changed = true;
        }

        g_mipi_avoidance_state = status.state;
        g_mipi_avoidance_radar_stale_ms = status.radar_stale_ms;
        g_mipi_avoidance_target_angle_deg = status.target_angle_deg;
        g_mipi_avoidance_selected_angle_deg = status.selected_angle_deg;
        g_mipi_avoidance_target_safe = status.target_safe;
    }

    if (changed)
    {
        lcd_show_mark_dirty();
    }
    lcd_show_exit_critical();
}

/**
 * @brief  MIPI 分支按周期读取机械臂反馈快照
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_mipi_refresh_arm_feedback(uint32_t elapsed_ms)
{
    mechanical_arm_feedback_snapshot_t snapshot = {0};
    bool need_update = false;
    bool changed = false;

    lcd_show_enter_critical();
    need_update = ((MIPI_UI_VIEW_ARM_DEBUG == g_mipi_view) ||
                   (MIPI_UI_VIEW_ARM_CALIB == g_mipi_view));
    lcd_show_exit_critical();
    if (!need_update)
    {
        return;
    }

    g_mipi_arm_refresh_ms += elapsed_ms;
    if (g_mipi_arm_refresh_ms < LCD_SHOW_MIPI_ARM_REFRESH_PERIOD_MS)
    {
        return;
    }
    g_mipi_arm_refresh_ms = 0U;

    mechanical_arm_feedback.read_snapshot(&snapshot);

    lcd_show_enter_critical();
    changed = (g_mipi_arm_feedback_sequence != snapshot.sequence) ||
              (g_mipi_arm_initialized != snapshot.initialized) ||
              (g_mipi_arm_feedback.last_error != snapshot.last_error);
    g_mipi_arm_feedback = snapshot;
    g_mipi_arm_initialized = snapshot.initialized;
    g_mipi_arm_feedback_sequence = snapshot.sequence;
    if (changed)
    {
        lcd_show_mark_dirty();
    }
    lcd_show_exit_critical();
}

/**
 * @brief  MIPI 分支只读消费语音 AI 最新识别结果
 *
 * 这里只读取 voice_ai 后台任务已经发布的结果，不调用 process()，避免显示任务参与推理更新。
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_mipi_update_voice_ai_result(uint32_t elapsed_ms)
{
    voice_ai_result_t result = {0};
    mission_voice_ai_actions_t actions = {0};
    const char *p_label = NULL;
    char label_text[MIPI_UI_VOICE_AI_LABEL_LEN] = {0};
    uint32_t score_percent = 0U;
    bool changed = false;
    bool mission_view_active = false;

    g_mipi_voice_ai_refresh_ms += elapsed_ms;
    if (g_mipi_voice_ai_refresh_ms < LCD_SHOW_MIPI_VOICE_AI_REFRESH_PERIOD_MS)
    {
        return;
    }
    g_mipi_voice_ai_refresh_ms = 0U;

    if (!voice_ai.has_new())
    {
        return;
    }

    if (FSP_SUCCESS == voice_ai.latest_read(&result))
    {
        p_label = voice_ai.label_name(result.top_index);
        if (NULL == p_label)
        {
            p_label = "unknown";
        }

        if (result.top_score <= 0.0f)
        {
            score_percent = 0U;
        }
        else if (result.top_score >= 1.0f)
        {
            score_percent = 100U;
        }
        else
        {
            score_percent = (uint32_t)((result.top_score * 100.0f) + 0.5f);
        }

        (void)strncpy(label_text, p_label, MIPI_UI_VOICE_AI_LABEL_LEN - 1U);
        label_text[MIPI_UI_VOICE_AI_LABEL_LEN - 1U] = '\0';

        lcd_show_enter_critical();
        mission_view_active = (MIPI_UI_VIEW_MISSION == g_mipi_view);
        changed = (!g_mipi_voice_ai_valid) ||
                  (g_mipi_voice_ai_index != result.top_index) ||
                  (g_mipi_voice_ai_score_percent != (uint8_t)score_percent) ||
                  (0 != strncmp(g_mipi_voice_ai_label, label_text, MIPI_UI_VOICE_AI_LABEL_LEN));
        if (changed)
        {
            g_mipi_voice_ai_valid = true;
            g_mipi_voice_ai_index = result.top_index;
            g_mipi_voice_ai_score_percent = (uint8_t)score_percent;
            (void)strncpy(g_mipi_voice_ai_label, label_text, MIPI_UI_VOICE_AI_LABEL_LEN - 1U);
            g_mipi_voice_ai_label[MIPI_UI_VOICE_AI_LABEL_LEN - 1U] = '\0';
            lcd_show_mark_dirty();
        }
        lcd_show_exit_critical();

        mission.voice_ai_consume(&result, mission_view_active, &actions);
        if (actions.enter_mission_view)
        {
            lcd_show_mipi_enter_mission_page();
        }

        if (actions.stop_output)
        {
            display_speed_control.clear();
        }
    }

    voice_ai.clear_new();
}

/**
 * @brief  MIPI 分支按周期读取 Mission 快照
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_mipi_refresh_mission(uint32_t elapsed_ms)
{
    mission_snapshot_t snapshot = {0};
    bool need_update = false;

    lcd_show_enter_critical();
    need_update = (MIPI_UI_VIEW_MISSION == g_mipi_view);
    lcd_show_exit_critical();
    if (!need_update)
    {
        return;
    }

    g_mipi_mission_refresh_ms += elapsed_ms;
    if (g_mipi_mission_refresh_ms < LCD_SHOW_MIPI_MISSION_REFRESH_PERIOD_MS)
    {
        return;
    }
    g_mipi_mission_refresh_ms = 0U;

    mission.snapshot_read(&snapshot);

    lcd_show_enter_critical();
    g_mipi_mission_snapshot = snapshot;
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}

/**
 * @brief  MIPI 分支按周期读取语音数据集采集发送状态
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_mipi_refresh_voice_dataset(uint32_t elapsed_ms)
{
    voice_dataset_status_t status = {0};
    bool need_update = false;

    lcd_show_enter_critical();
    need_update = (MIPI_UI_VIEW_VOICE_DATASET == g_mipi_view);
    lcd_show_exit_critical();
    if (!need_update)
    {
        return;
    }

    g_mipi_voice_refresh_ms += elapsed_ms;
    if (g_mipi_voice_refresh_ms < LCD_SHOW_MIPI_VOICE_REFRESH_PERIOD_MS)
    {
        return;
    }
    g_mipi_voice_refresh_ms = 0U;

    voice_dataset.status_get(&status);

    lcd_show_enter_critical();
    g_mipi_voice_status = status;
    if (!lcd_show_mipi_voice_busy_unlocked())
    {
        lcd_show_mipi_voice_sync_selected_label_unlocked();
    }
    lcd_show_mark_dirty();
    lcd_show_exit_critical();
}
#endif

#if !UI_PORT_USE_MIPI_LCD
/**
 * @brief  限制浮点调试参数范围
 *
 * @param value  当前值
 * @param low    最小值
 * @param high   最大值
 * @return float 限幅后的值
 */
static float lcd_show_limit_float(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}

/**
 * @brief  计算浮点数绝对值
 *
 * @param value 浮点数
 * @return float 绝对值
 */
static float lcd_show_abs_float(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

/**
 * @brief  获取当前麦轮子界面项目数量
 *
 * @return uint8_t 当前子界面项目数量
 */
static uint8_t lcd_show_spi_mecanum_item_count_unlocked(void)
{
    if (ST7789_UI_MECANUM_MODE_WHEEL == g_lcd_show_mecanum_mode)
    {
        return (uint8_t)(sizeof(g_lcd_show_mecanum_wheel_items) / sizeof(g_lcd_show_mecanum_wheel_items[0]));
    }

    if (ST7789_UI_MECANUM_MODE_PID == g_lcd_show_mecanum_mode)
    {
        return (uint8_t)(sizeof(g_lcd_show_mecanum_pid_items) / sizeof(g_lcd_show_mecanum_pid_items[0]));
    }

    if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
    {
        return (uint8_t)(sizeof(g_lcd_show_mecanum_pwm_items) / sizeof(g_lcd_show_mecanum_pwm_items[0]));
    }

    return (uint8_t)(sizeof(g_lcd_show_mecanum_body_items) / sizeof(g_lcd_show_mecanum_body_items[0]));
}

/**
 * @brief  获取当前麦轮子界面指定索引的项目
 *
 * @param index 子界面内索引
 * @return st7789_ui_mecanum_item_t 项目编号
 */
static st7789_ui_mecanum_item_t lcd_show_spi_mecanum_item_at_unlocked(uint8_t index)
{
    if (ST7789_UI_MECANUM_MODE_WHEEL == g_lcd_show_mecanum_mode)
    {
        return g_lcd_show_mecanum_wheel_items[index];
    }

    if (ST7789_UI_MECANUM_MODE_PID == g_lcd_show_mecanum_mode)
    {
        return g_lcd_show_mecanum_pid_items[index];
    }

    if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
    {
        return g_lcd_show_mecanum_pwm_items[index];
    }

    return g_lcd_show_mecanum_body_items[index];
}

/**
 * @brief  获取当前麦轮选中项目在子界面中的索引
 *
 * @return uint8_t 当前子界面内索引
 */
static uint8_t lcd_show_spi_mecanum_selected_index_unlocked(void)
{
    uint8_t count = lcd_show_spi_mecanum_item_count_unlocked();
    uint8_t i = 0U;

    for (i = 0U; i < count; i++)
    {
        if ((st7789_ui_mecanum_item_t)g_lcd_show_mecanum_selected_item ==
            lcd_show_spi_mecanum_item_at_unlocked(i))
        {
            return i;
        }
    }

    g_lcd_show_mecanum_selected_item = (uint8_t)ST7789_UI_MECANUM_ITEM_MODE;
    return 0U;
}

/**
 * @brief  初始化一个空的底盘命令
 *
 * @param p_command 命令输出指针
 * @return void
 */
static void lcd_show_spi_clear_chassis_command(lcd_show_spi_chassis_command_t *p_command)
{
    if (NULL == p_command)
    {
        return;
    }

    p_command->action = LCD_SHOW_SPI_CHASSIS_ACTION_NONE;
    p_command->vx_mps = 0.0f;
    p_command->vy_mps = 0.0f;
    p_command->wz_radps = 0.0f;
    p_command->pid_params.kp = CHASSIS_PID_DEFAULT_KP;
    p_command->pid_params.ki = CHASSIS_PID_DEFAULT_KI;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        p_command->wheel_speed_mps[i] = 0.0f;
        p_command->pwm_counts[i] = 0;
    }
}

/**
 * @brief  复制当前麦轮参数到待执行命令
 *
 * @param action    命令类型
 * @param p_command 命令输出指针
 * @return void
 */
static void lcd_show_spi_make_chassis_command_unlocked(lcd_show_spi_chassis_action_t action,
                                                       lcd_show_spi_chassis_command_t *p_command)
{
    if (NULL == p_command)
    {
        return;
    }

    p_command->action = action;
    p_command->vx_mps = g_lcd_show_mecanum_vx_mps;
    p_command->vy_mps = g_lcd_show_mecanum_vy_mps;
    p_command->wz_radps = g_lcd_show_mecanum_wz_radps;
    p_command->pid_params.kp = g_lcd_show_mecanum_kp;
    p_command->pid_params.ki = g_lcd_show_mecanum_ki;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        p_command->wheel_speed_mps[i] = g_lcd_show_mecanum_wheel_speed_mps[i];
        p_command->pwm_counts[i] = g_lcd_show_mecanum_pwm_counts[i];
    }
}

/**
 * @brief  执行底盘命令
 *
 * @param p_command 待执行命令
 * @return void
 */
static void lcd_show_spi_execute_chassis_command(const lcd_show_spi_chassis_command_t *p_command)
{
    fsp_err_t err = FSP_SUCCESS;
    ui_speed_control speed_control =
    {
        .vx_mps = 0.0f,
        .vy_mps = 0.0f,
        .wz_radps = 0.0f,
    };

    if (NULL == p_command)
    {
        return;
    }

    switch (p_command->action)
    {
        case LCD_SHOW_SPI_CHASSIS_ACTION_BODY:
            speed_control.vx_mps = p_command->vx_mps;
            speed_control.vy_mps = p_command->vy_mps;
            speed_control.wz_radps = p_command->wz_radps;
            display_speed_control.set_body(&speed_control);
            display_speed_control.set_enable(true);
            break;

        case LCD_SHOW_SPI_CHASSIS_ACTION_WHEEL:
            display_speed_control.set_wheel(p_command->wheel_speed_mps);
            display_speed_control.set_enable(true);
            break;

        case LCD_SHOW_SPI_CHASSIS_ACTION_PWM:
            display_speed_control.set_pwm(p_command->pwm_counts);
            display_speed_control.set_enable(true);
            break;

        case LCD_SHOW_SPI_CHASSIS_ACTION_SAVE_PID:
            chassis.set_pid(p_command->pid_params.kp, p_command->pid_params.ki);
            err = chassis_param.save(&p_command->pid_params);

            lcd_show_enter_critical();
            g_lcd_show_mecanum_save_status = (FSP_SUCCESS == err) ?
                                             ST7789_UI_MECANUM_SAVE_SAVED :
                                             ST7789_UI_MECANUM_SAVE_ERR;
            lcd_show_mark_dirty();
            lcd_show_exit_critical();
            break;

        case LCD_SHOW_SPI_CHASSIS_ACTION_STOP:
            display_speed_control.clear();
            break;

        case LCD_SHOW_SPI_CHASSIS_ACTION_NONE:
        default:
            break;
    }
}

/**
 * @brief  获取麦轮当前选中项目的数值
 *
 * @param item 麦轮页面项目
 * @return float 当前项目数值
 */
static float lcd_show_spi_mecanum_get_item_value_unlocked(st7789_ui_mecanum_item_t item)
{
    switch (item)
    {
        case ST7789_UI_MECANUM_ITEM_MODE:
            return (float)g_lcd_show_mecanum_mode;

        case ST7789_UI_MECANUM_ITEM_VX:
            return g_lcd_show_mecanum_vx_mps;

        case ST7789_UI_MECANUM_ITEM_VY:
            return g_lcd_show_mecanum_vy_mps;

        case ST7789_UI_MECANUM_ITEM_WZ:
            return g_lcd_show_mecanum_wz_radps;

        case ST7789_UI_MECANUM_ITEM_W1:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                return (float)g_lcd_show_mecanum_pwm_counts[0];
            }
            return g_lcd_show_mecanum_wheel_speed_mps[0];

        case ST7789_UI_MECANUM_ITEM_W2:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                return (float)g_lcd_show_mecanum_pwm_counts[1];
            }
            return g_lcd_show_mecanum_wheel_speed_mps[1];

        case ST7789_UI_MECANUM_ITEM_W3:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                return (float)g_lcd_show_mecanum_pwm_counts[2];
            }
            return g_lcd_show_mecanum_wheel_speed_mps[2];

        case ST7789_UI_MECANUM_ITEM_W4:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                return (float)g_lcd_show_mecanum_pwm_counts[3];
            }
            return g_lcd_show_mecanum_wheel_speed_mps[3];

        case ST7789_UI_MECANUM_ITEM_KP:
            return g_lcd_show_mecanum_kp;

        case ST7789_UI_MECANUM_ITEM_KI:
            return g_lcd_show_mecanum_ki;

        case ST7789_UI_MECANUM_ITEM_STOP:
        default:
            return 0.0f;
    }
}

/**
 * @brief  设置麦轮当前选中项目的数值
 *
 * @param item  麦轮页面项目
 * @param value 新数值
 * @return lcd_show_spi_chassis_action_t  需要执行的底盘命令类型
 */
static lcd_show_spi_chassis_action_t lcd_show_spi_mecanum_set_item_value_unlocked(st7789_ui_mecanum_item_t item,
                                                                                  float value)
{
    switch (item)
    {
        case ST7789_UI_MECANUM_ITEM_VX:
            g_lcd_show_mecanum_vx_mps = lcd_show_limit_float(value,
                                                             -CHASSIS_MAX_VX_MPS,
                                                             CHASSIS_MAX_VX_MPS);
            g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_BODY;
            return LCD_SHOW_SPI_CHASSIS_ACTION_BODY;

        case ST7789_UI_MECANUM_ITEM_VY:
            g_lcd_show_mecanum_vy_mps = lcd_show_limit_float(value,
                                                             -CHASSIS_MAX_VY_MPS,
                                                             CHASSIS_MAX_VY_MPS);
            g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_BODY;
            return LCD_SHOW_SPI_CHASSIS_ACTION_BODY;

        case ST7789_UI_MECANUM_ITEM_WZ:
            g_lcd_show_mecanum_wz_radps = lcd_show_limit_float(value,
                                                               -CHASSIS_MAX_WZ_RADPS,
                                                               CHASSIS_MAX_WZ_RADPS);
            g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_BODY;
            return LCD_SHOW_SPI_CHASSIS_ACTION_BODY;

        case ST7789_UI_MECANUM_ITEM_W1:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                int32_t pwm = (int32_t)lcd_show_limit_float(value,
                    (float)(-CHASSIS_PWM_TEST_LIMIT_COUNTS),
                    (float)(CHASSIS_PWM_TEST_LIMIT_COUNTS));
                g_lcd_show_mecanum_pwm_counts[0] = pwm;
#if LCD_SHOW_PWM_SYNC_ALL
                g_lcd_show_mecanum_pwm_counts[1] = pwm;
                g_lcd_show_mecanum_pwm_counts[2] = pwm;
                g_lcd_show_mecanum_pwm_counts[3] = pwm;
#endif
                g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_PWM;
                return LCD_SHOW_SPI_CHASSIS_ACTION_PWM;
            }
            g_lcd_show_mecanum_wheel_speed_mps[0] = lcd_show_limit_float(value,
                                                                         -CHASSIS_MAX_WHEEL_SPEED_MPS,
                                                                         CHASSIS_MAX_WHEEL_SPEED_MPS);
            g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_WHEEL;
            return LCD_SHOW_SPI_CHASSIS_ACTION_WHEEL;

        case ST7789_UI_MECANUM_ITEM_W2:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                int32_t pwm = (int32_t)lcd_show_limit_float(value,
                    (float)(-CHASSIS_PWM_TEST_LIMIT_COUNTS),
                    (float)(CHASSIS_PWM_TEST_LIMIT_COUNTS));
                g_lcd_show_mecanum_pwm_counts[1] = pwm;
#if LCD_SHOW_PWM_SYNC_ALL
                g_lcd_show_mecanum_pwm_counts[0] = pwm;
                g_lcd_show_mecanum_pwm_counts[2] = pwm;
                g_lcd_show_mecanum_pwm_counts[3] = pwm;
#endif
                g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_PWM;
                return LCD_SHOW_SPI_CHASSIS_ACTION_PWM;
            }
            g_lcd_show_mecanum_wheel_speed_mps[1] = lcd_show_limit_float(value,
                                                                         -CHASSIS_MAX_WHEEL_SPEED_MPS,
                                                                         CHASSIS_MAX_WHEEL_SPEED_MPS);
            g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_WHEEL;
            return LCD_SHOW_SPI_CHASSIS_ACTION_WHEEL;

        case ST7789_UI_MECANUM_ITEM_W3:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                int32_t pwm = (int32_t)lcd_show_limit_float(value,
                    (float)(-CHASSIS_PWM_TEST_LIMIT_COUNTS),
                    (float)(CHASSIS_PWM_TEST_LIMIT_COUNTS));
                g_lcd_show_mecanum_pwm_counts[2] = pwm;
#if LCD_SHOW_PWM_SYNC_ALL
                g_lcd_show_mecanum_pwm_counts[0] = pwm;
                g_lcd_show_mecanum_pwm_counts[1] = pwm;
                g_lcd_show_mecanum_pwm_counts[3] = pwm;
#endif
                g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_PWM;
                return LCD_SHOW_SPI_CHASSIS_ACTION_PWM;
            }
            g_lcd_show_mecanum_wheel_speed_mps[2] = lcd_show_limit_float(value,
                                                                         -CHASSIS_MAX_WHEEL_SPEED_MPS,
                                                                         CHASSIS_MAX_WHEEL_SPEED_MPS);
            g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_WHEEL;
            return LCD_SHOW_SPI_CHASSIS_ACTION_WHEEL;

        case ST7789_UI_MECANUM_ITEM_W4:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                int32_t pwm = (int32_t)lcd_show_limit_float(value,
                    (float)(-CHASSIS_PWM_TEST_LIMIT_COUNTS),
                    (float)(CHASSIS_PWM_TEST_LIMIT_COUNTS));
                g_lcd_show_mecanum_pwm_counts[3] = pwm;
#if LCD_SHOW_PWM_SYNC_ALL
                g_lcd_show_mecanum_pwm_counts[0] = pwm;
                g_lcd_show_mecanum_pwm_counts[1] = pwm;
                g_lcd_show_mecanum_pwm_counts[2] = pwm;
#endif
                g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_PWM;
                return LCD_SHOW_SPI_CHASSIS_ACTION_PWM;
            }
            g_lcd_show_mecanum_wheel_speed_mps[3] = lcd_show_limit_float(value,
                                                                         -CHASSIS_MAX_WHEEL_SPEED_MPS,
                                                                         CHASSIS_MAX_WHEEL_SPEED_MPS);
            g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_WHEEL;
            return LCD_SHOW_SPI_CHASSIS_ACTION_WHEEL;

        case ST7789_UI_MECANUM_ITEM_KP:
            g_lcd_show_mecanum_kp = lcd_show_limit_float(value,
                                                         CHASSIS_PID_PARAM_MIN,
                                                         CHASSIS_PID_PARAM_MAX);
            g_lcd_show_mecanum_save_status = ST7789_UI_MECANUM_SAVE_SAVING;
            return LCD_SHOW_SPI_CHASSIS_ACTION_SAVE_PID;

        case ST7789_UI_MECANUM_ITEM_KI:
            g_lcd_show_mecanum_ki = lcd_show_limit_float(value,
                                                         CHASSIS_PID_PARAM_MIN,
                                                         CHASSIS_PID_PARAM_MAX);
            g_lcd_show_mecanum_save_status = ST7789_UI_MECANUM_SAVE_SAVING;
            return LCD_SHOW_SPI_CHASSIS_ACTION_SAVE_PID;

        case ST7789_UI_MECANUM_ITEM_MODE:
        case ST7789_UI_MECANUM_ITEM_STOP:
        default:
            return LCD_SHOW_SPI_CHASSIS_ACTION_NONE;
    }
}

/**
 * @brief  获取麦轮项目的编辑步进
 *
 * @param item 麦轮页面项目
 * @return float 编辑步进
 */
static float lcd_show_spi_mecanum_item_step(st7789_ui_mecanum_item_t item)
{
    switch (item)
    {
        case ST7789_UI_MECANUM_ITEM_VX:
        case ST7789_UI_MECANUM_ITEM_VY:
            return LCD_SHOW_MECANUM_V_STEP_MPS;

        case ST7789_UI_MECANUM_ITEM_WZ:
            return LCD_SHOW_MECANUM_WZ_STEP_RADPS;

        case ST7789_UI_MECANUM_ITEM_W1:
        case ST7789_UI_MECANUM_ITEM_W2:
        case ST7789_UI_MECANUM_ITEM_W3:
        case ST7789_UI_MECANUM_ITEM_W4:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                return LCD_SHOW_MECANUM_PWM_STEP;
            }
            return LCD_SHOW_MECANUM_WHEEL_STEP_MPS;

        case ST7789_UI_MECANUM_ITEM_KP:
        case ST7789_UI_MECANUM_ITEM_KI:
            return LCD_SHOW_MECANUM_PID_STEP;

        case ST7789_UI_MECANUM_ITEM_MODE:
        case ST7789_UI_MECANUM_ITEM_STOP:
        default:
            return 0.0f;
    }
}

/**
 * @brief  按麦轮项目限制编辑中的数值范围
 *
 * @param item  麦轮页面项目
 * @param value 待限制数值
 * @return float 限幅后的数值
 */
static float lcd_show_spi_mecanum_limit_item_value(st7789_ui_mecanum_item_t item, float value)
{
    switch (item)
    {
        case ST7789_UI_MECANUM_ITEM_VX:
            return lcd_show_limit_float(value, -CHASSIS_MAX_VX_MPS, CHASSIS_MAX_VX_MPS);

        case ST7789_UI_MECANUM_ITEM_VY:
            return lcd_show_limit_float(value, -CHASSIS_MAX_VY_MPS, CHASSIS_MAX_VY_MPS);

        case ST7789_UI_MECANUM_ITEM_WZ:
            return lcd_show_limit_float(value, -CHASSIS_MAX_WZ_RADPS, CHASSIS_MAX_WZ_RADPS);

        case ST7789_UI_MECANUM_ITEM_W1:
        case ST7789_UI_MECANUM_ITEM_W2:
        case ST7789_UI_MECANUM_ITEM_W3:
        case ST7789_UI_MECANUM_ITEM_W4:
            if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
            {
                return lcd_show_limit_float(value,
                                           (float)(-CHASSIS_PWM_TEST_LIMIT_COUNTS),
                                           (float)(CHASSIS_PWM_TEST_LIMIT_COUNTS));
            }
            return lcd_show_limit_float(value,
                                        -CHASSIS_MAX_WHEEL_SPEED_MPS,
                                        CHASSIS_MAX_WHEEL_SPEED_MPS);

        case ST7789_UI_MECANUM_ITEM_KP:
        case ST7789_UI_MECANUM_ITEM_KI:
            return lcd_show_limit_float(value, CHASSIS_PID_PARAM_MIN, CHASSIS_PID_PARAM_MAX);

        case ST7789_UI_MECANUM_ITEM_MODE:
        case ST7789_UI_MECANUM_ITEM_STOP:
        default:
            return value;
    }
}

/**
 * @brief  判断麦轮项目是否为可编辑数值
 *
 * @param item 麦轮页面项目
 * @return bool true 表示可编辑
 */
static bool lcd_show_spi_mecanum_item_is_editable(st7789_ui_mecanum_item_t item)
{
    return ((item != ST7789_UI_MECANUM_ITEM_MODE) &&
            (item != ST7789_UI_MECANUM_ITEM_STOP) &&
            (item < ST7789_UI_MECANUM_ITEM_COUNT));
}

/**
 * @brief  复位麦轮调试速度目标
 *
 * @return void
 */
static void lcd_show_spi_mecanum_clear_speed_unlocked(void)
{
    g_lcd_show_mecanum_vx_mps = 0.0f;
    g_lcd_show_mecanum_vy_mps = 0.0f;
    g_lcd_show_mecanum_wz_radps = 0.0f;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_lcd_show_mecanum_wheel_speed_mps[i] = 0.0f;
        g_lcd_show_mecanum_pwm_counts[i] = 0;
    }
}

/**
 * @brief  切换麦轮控制模式
 *
 * @return lcd_show_spi_chassis_action_t  需要执行的底盘命令类型
 */
static lcd_show_spi_chassis_action_t lcd_show_spi_mecanum_toggle_mode_unlocked(void)
{
    if (ST7789_UI_MECANUM_MODE_BODY == g_lcd_show_mecanum_mode)
    {
        g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_WHEEL;
        g_lcd_show_mecanum_selected_item = (uint8_t)ST7789_UI_MECANUM_ITEM_MODE;
        return LCD_SHOW_SPI_CHASSIS_ACTION_WHEEL;
    }

    if (ST7789_UI_MECANUM_MODE_WHEEL == g_lcd_show_mecanum_mode)
    {
        g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_PID;
        g_lcd_show_mecanum_selected_item = (uint8_t)ST7789_UI_MECANUM_ITEM_MODE;
        return LCD_SHOW_SPI_CHASSIS_ACTION_STOP;
    }

    if (ST7789_UI_MECANUM_MODE_PID == g_lcd_show_mecanum_mode)
    {
        g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_PWM;
        g_lcd_show_mecanum_selected_item = (uint8_t)ST7789_UI_MECANUM_ITEM_MODE;
        return LCD_SHOW_SPI_CHASSIS_ACTION_PWM;
    }

    g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_BODY;
    g_lcd_show_mecanum_selected_item = (uint8_t)ST7789_UI_MECANUM_ITEM_MODE;
    return LCD_SHOW_SPI_CHASSIS_ACTION_BODY;
}

/**
 * @brief  移动麦轮页面选中项目
 *
 * @param delta 移动方向，负数向上，正数向下
 * @return void
 */
static void lcd_show_spi_mecanum_move_selection_unlocked(int32_t delta)
{
    uint8_t item_count = lcd_show_spi_mecanum_item_count_unlocked();
    uint8_t selected_index = lcd_show_spi_mecanum_selected_index_unlocked();

    if (0 == delta)
    {
        return;
    }

    if (delta > 0)
    {
        selected_index++;
        if (selected_index >= item_count)
        {
            selected_index = 0U;
        }
    }
    else
    {
        if (0U == selected_index)
        {
            selected_index = item_count - 1U;
        }
        else
        {
            selected_index--;
        }
    }

    g_lcd_show_mecanum_selected_item = (uint8_t)lcd_show_spi_mecanum_item_at_unlocked(selected_index);
    g_lcd_show_mecanum_save_status = ST7789_UI_MECANUM_SAVE_NONE;
    lcd_show_mark_dirty();
}

/**
 * @brief  开始编辑麦轮当前项目
 *
 * @return void
 */
static void lcd_show_spi_mecanum_begin_edit_unlocked(void)
{
    st7789_ui_mecanum_item_t item = (st7789_ui_mecanum_item_t)g_lcd_show_mecanum_selected_item;

    if (!lcd_show_spi_mecanum_item_is_editable(item))
    {
        return;
    }

    g_lcd_show_mecanum_edit_value = lcd_show_spi_mecanum_get_item_value_unlocked(item);
    g_lcd_show_mecanum_editing = true;
    g_lcd_show_mecanum_save_status = ST7789_UI_MECANUM_SAVE_NONE;
    lcd_show_mark_dirty();
}

/**
 * @brief  保存麦轮当前编辑项目
 *
 * @param p_command 底盘命令输出指针
 * @return void
 */
static void lcd_show_spi_mecanum_save_edit_unlocked(lcd_show_spi_chassis_command_t *p_command)
{
    st7789_ui_mecanum_item_t item = (st7789_ui_mecanum_item_t)g_lcd_show_mecanum_selected_item;
    lcd_show_spi_chassis_action_t action = LCD_SHOW_SPI_CHASSIS_ACTION_NONE;

    action = lcd_show_spi_mecanum_set_item_value_unlocked(item, g_lcd_show_mecanum_edit_value);
    g_lcd_show_mecanum_editing = false;
    lcd_show_spi_make_chassis_command_unlocked(action, p_command);
    lcd_show_mark_dirty();
}

/**
 * @brief  取消麦轮当前编辑项目
 *
 * @return void
 */
static void lcd_show_spi_mecanum_cancel_edit_unlocked(void)
{
    g_lcd_show_mecanum_edit_value =
        lcd_show_spi_mecanum_get_item_value_unlocked((st7789_ui_mecanum_item_t)g_lcd_show_mecanum_selected_item);
    g_lcd_show_mecanum_editing = false;
    g_lcd_show_mecanum_save_status = ST7789_UI_MECANUM_SAVE_NONE;
    lcd_show_mark_dirty();
}

/**
 * @brief  调整麦轮当前编辑项目
 *
 * @param direction 调整方向，正数增大，负数减小
 * @return void
 */
static void lcd_show_spi_mecanum_adjust_edit_unlocked(int32_t direction)
{
    st7789_ui_mecanum_item_t item = (st7789_ui_mecanum_item_t)g_lcd_show_mecanum_selected_item;
    float step = 0.0f;

    if (!g_lcd_show_mecanum_editing)
    {
        return;
    }

    step = lcd_show_spi_mecanum_item_step(item);
    g_lcd_show_mecanum_edit_value += ((direction > 0) ? step : -step);
    g_lcd_show_mecanum_edit_value =
        lcd_show_spi_mecanum_limit_item_value(item, g_lcd_show_mecanum_edit_value);
    lcd_show_mark_dirty();
}

/**
 * @brief  处理麦轮页面确认键
 *
 * @param p_command 底盘命令输出指针
 * @return void
 */
static void lcd_show_spi_mecanum_confirm_unlocked(lcd_show_spi_chassis_command_t *p_command)
{
    st7789_ui_mecanum_item_t item = (st7789_ui_mecanum_item_t)g_lcd_show_mecanum_selected_item;
    lcd_show_spi_chassis_action_t action = LCD_SHOW_SPI_CHASSIS_ACTION_NONE;

    if (g_lcd_show_mecanum_editing)
    {
        lcd_show_spi_mecanum_save_edit_unlocked(p_command);
        return;
    }

    if (ST7789_UI_MECANUM_ITEM_MODE == item)
    {
        action = lcd_show_spi_mecanum_toggle_mode_unlocked();
        lcd_show_spi_make_chassis_command_unlocked(action, p_command);
        lcd_show_mark_dirty();
        return;
    }

    if (ST7789_UI_MECANUM_ITEM_STOP == item)
    {
        lcd_show_spi_mecanum_clear_speed_unlocked();
        g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_BODY;
        lcd_show_spi_make_chassis_command_unlocked(LCD_SHOW_SPI_CHASSIS_ACTION_STOP, p_command);
        lcd_show_mark_dirty();
        return;
    }

    lcd_show_spi_mecanum_begin_edit_unlocked();
}

/**
 * @brief  处理麦轮页面返回键
 *
 * @param p_command 底盘命令输出指针
 * @return void
 */
static void lcd_show_spi_mecanum_back_unlocked(lcd_show_spi_chassis_command_t *p_command)
{
    if (g_lcd_show_mecanum_editing)
    {
        lcd_show_spi_mecanum_cancel_edit_unlocked();
        return;
    }

    lcd_show_spi_mecanum_clear_speed_unlocked();
    g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_BODY;
    g_lcd_show_view = ST7789_UI_VIEW_MENU;
    lcd_show_spi_make_chassis_command_unlocked(LCD_SHOW_SPI_CHASSIS_ACTION_STOP, p_command);
    lcd_show_mark_dirty();
}

/**
 * @brief  处理 SPI 菜单向下选择
 *
 * @return void
 */
static void lcd_show_spi_menu_down(void)
{
    g_lcd_show_selected_index++;
    if (g_lcd_show_selected_index >= (uint8_t)ST7789_UI_PAGE_COUNT)
    {
        g_lcd_show_selected_index = 0U;
    }
    lcd_show_mark_dirty();
}

/**
 * @brief  处理 SPI 菜单向上选择
 *
 * @return void
 */
static void lcd_show_spi_menu_up(void)
{
    if (0U == g_lcd_show_selected_index)
    {
        g_lcd_show_selected_index = (uint8_t)ST7789_UI_PAGE_COUNT - 1U;
    }
    else
    {
        g_lcd_show_selected_index--;
    }
    lcd_show_mark_dirty();
}

/**
 * @brief  进入 SPI 当前选中的功能页
 *
 * @return void
 */
static void lcd_show_spi_enter_selected_page(void)
{
    g_lcd_show_current_page = (st7789_ui_page_t)g_lcd_show_selected_index;
    g_lcd_show_view = ST7789_UI_VIEW_PAGE;
    g_lcd_show_editing = false;
    g_lcd_show_edit_value = g_lcd_show_page_values[(uint8_t)g_lcd_show_current_page];
    if (ST7789_UI_PAGE_MECANUM_PID == g_lcd_show_current_page)
    {
        ui_speed_control speed_control =
        {
            .vx_mps = 0.0f,
            .vy_mps = 0.0f,
            .wz_radps = 0.0f,
        };

        g_lcd_show_mecanum_selected_item = 0U;
        g_lcd_show_mecanum_editing = false;
        g_lcd_show_mecanum_edit_value =
            lcd_show_spi_mecanum_get_item_value_unlocked(ST7789_UI_MECANUM_ITEM_MODE);
        g_lcd_show_mecanum_save_status = ST7789_UI_MECANUM_SAVE_NONE;
        speed_control.vx_mps = g_lcd_show_mecanum_vx_mps;
        speed_control.vy_mps = g_lcd_show_mecanum_vy_mps;
        speed_control.wz_radps = g_lcd_show_mecanum_wz_radps;
        if (ST7789_UI_MECANUM_MODE_WHEEL == g_lcd_show_mecanum_mode)
        {
            display_speed_control.set_wheel(g_lcd_show_mecanum_wheel_speed_mps);
            display_speed_control.set_enable(true);
        }
        else if (ST7789_UI_MECANUM_MODE_PID == g_lcd_show_mecanum_mode)
        {
            display_speed_control.clear();
        }
        else if (ST7789_UI_MECANUM_MODE_PWM == g_lcd_show_mecanum_mode)
        {
            display_speed_control.set_pwm(g_lcd_show_mecanum_pwm_counts);
            display_speed_control.set_enable(true);
        }
        else
        {
            display_speed_control.set_body(&speed_control);
            display_speed_control.set_enable(true);
        }
    }
    lcd_show_mark_dirty();
}

/**
 * @brief  从 SPI 功能页返回菜单
 *
 * @return void
 */
static void lcd_show_spi_back_to_menu(void)
{
    g_lcd_show_editing = false;
    g_lcd_show_view = ST7789_UI_VIEW_MENU;
    lcd_show_mark_dirty();
}

/**
 * @brief  开始编辑 SPI 当前功能页占位数值
 *
 * @return void
 */
static void lcd_show_spi_begin_edit(void)
{
    g_lcd_show_edit_value = g_lcd_show_page_values[(uint8_t)g_lcd_show_current_page];
    g_lcd_show_editing = true;
    lcd_show_mark_dirty();
}

/**
 * @brief  保存 SPI 当前功能页占位数值
 *
 * @return void
 */
static void lcd_show_spi_save_edit(void)
{
    g_lcd_show_page_values[(uint8_t)g_lcd_show_current_page] = g_lcd_show_edit_value;
    g_lcd_show_editing = false;
    lcd_show_mark_dirty();
}

/**
 * @brief  取消 SPI 当前功能页占位数值编辑
 *
 * @return void
 */
static void lcd_show_spi_cancel_edit(void)
{
    g_lcd_show_edit_value = g_lcd_show_page_values[(uint8_t)g_lcd_show_current_page];
    g_lcd_show_editing = false;
    lcd_show_mark_dirty();
}

/**
 * @brief  切换 SPI 当前功能页编辑状态
 *
 * @return void
 */
static void lcd_show_spi_toggle_edit(void)
{
    if (g_lcd_show_editing)
    {
        lcd_show_spi_save_edit();
    }
    else
    {
        lcd_show_spi_begin_edit();
    }
}

/**
 * @brief  调整 SPI 当前功能页编辑中的临时数值
 *
 * @param delta 调整数值
 * @return void
 */
static void lcd_show_spi_adjust_current_value(int32_t delta)
{
    if (!g_lcd_show_editing)
    {
        return;
    }

    g_lcd_show_edit_value += delta;
    lcd_show_mark_dirty();
}

/**
 * @brief  处理 SPI 菜单页点击事件
 *
 * @param event_id 按键编号
 * @return void
 */
static void lcd_show_spi_handle_menu_click(key_id_t event_id)
{
    switch (event_id)
    {
        case KEY_ID_1:
            lcd_show_spi_menu_up();
            break;

        case KEY_ID_2:
            lcd_show_spi_menu_down();
            break;

        case KEY_ID_3:
            lcd_show_spi_enter_selected_page();
            break;

        case KEY_ID_ENCODER_BUTTON:
            lcd_show_spi_enter_selected_page();
            break;

        default:
            break;
    }
}

/**
 * @brief  处理 SPI 菜单页旋转编码器事件
 *
 * @param event_type 事件类型
 * @return void
 */
static void lcd_show_spi_handle_menu_rotation(key_event_type_t event_type)
{
    if (KEY_EVENT_ROT_CW == event_type)
    {
        lcd_show_spi_menu_down();
    }
    else if (KEY_EVENT_ROT_CCW == event_type)
    {
        lcd_show_spi_menu_up();
    }
}

/**
 * @brief  处理 SPI 功能页点击事件
 *
 * @param event_id 按键编号
 * @return void
 */
static void lcd_show_spi_handle_page_click(key_id_t event_id, lcd_show_spi_chassis_command_t *p_command)
{
    if (ST7789_UI_PAGE_MECANUM_PID == g_lcd_show_current_page)
    {
        if (!g_lcd_show_mecanum_editing)
        {
            if (KEY_ID_1 == event_id)
            {
                lcd_show_spi_mecanum_move_selection_unlocked(-1);
                return;
            }

            if (KEY_ID_2 == event_id)
            {
                lcd_show_spi_mecanum_move_selection_unlocked(1);
                return;
            }
        }

        if ((KEY_ID_3 == event_id) || (KEY_ID_ENCODER_BUTTON == event_id))
        {
            lcd_show_spi_mecanum_confirm_unlocked(p_command);
            return;
        }

        if (KEY_ID_4 == event_id)
        {
            lcd_show_spi_mecanum_back_unlocked(p_command);
        }

        return;
    }

    if ((KEY_ID_3 == event_id) || (KEY_ID_ENCODER_BUTTON == event_id))
    {
        lcd_show_spi_toggle_edit();
    }
    else if (KEY_ID_4 == event_id)
    {
        if (g_lcd_show_editing)
        {
            lcd_show_spi_cancel_edit();
        }
        else
        {
            lcd_show_spi_back_to_menu();
        }
    }
}

/**
 * @brief  处理一个 SPI 按键或编码器事件
 *
 * @param p_event 按键事件指针
 * @return void
 */
static void lcd_show_spi_handle_key_event(const key_event_t *p_event)
{
    lcd_show_spi_chassis_command_t command;

    lcd_show_spi_clear_chassis_command(&command);

    if (NULL == p_event)
    {
        return;
    }

    lcd_show_enter_critical();

    if (KEY_EVENT_CLICK == p_event->type)
    {
        if (ST7789_UI_VIEW_MENU == g_lcd_show_view)
        {
            lcd_show_spi_handle_menu_click(p_event->id);
        }
        else if (ST7789_UI_VIEW_PAGE == g_lcd_show_view)
        {
            lcd_show_spi_handle_page_click(p_event->id, &command);
        }
    }
    else if (KEY_ID_ENCODER == p_event->id)
    {
        if (ST7789_UI_VIEW_MENU == g_lcd_show_view)
        {
            lcd_show_spi_handle_menu_rotation(p_event->type);
        }
        else if (ST7789_UI_VIEW_PAGE == g_lcd_show_view)
        {
            if (ST7789_UI_PAGE_MECANUM_PID == g_lcd_show_current_page)
            {
                if (g_lcd_show_mecanum_editing)
                {
                    if (KEY_EVENT_ROT_CW == p_event->type)
                    {
                        lcd_show_spi_mecanum_adjust_edit_unlocked(1);
                    }
                    else if (KEY_EVENT_ROT_CCW == p_event->type)
                    {
                        lcd_show_spi_mecanum_adjust_edit_unlocked(-1);
                    }
                }
                else
                {
                    if (KEY_EVENT_ROT_CW == p_event->type)
                    {
                        lcd_show_spi_mecanum_move_selection_unlocked(1);
                    }
                    else if (KEY_EVENT_ROT_CCW == p_event->type)
                    {
                        lcd_show_spi_mecanum_move_selection_unlocked(-1);
                    }
                }
            }
            else if (KEY_EVENT_ROT_CW == p_event->type)
            {
                lcd_show_spi_adjust_current_value(LCD_SHOW_ENCODER_VALUE_STEP);
            }
            else if (KEY_EVENT_ROT_CCW == p_event->type)
            {
                lcd_show_spi_adjust_current_value(-LCD_SHOW_ENCODER_VALUE_STEP);
            }
        }
        else
        {
            /* 开机页旋钮事件不处理，避免开机动画期间误触发。 */
        }
    }
    else
    {
        /* 长按事件本轮不参与 UI 状态变化，避免误触发。 */
    }

    lcd_show_exit_critical();
    lcd_show_spi_execute_chassis_command(&command);
}

/**
 * @brief  读取已有按键任务写入的事件队列
 *
 * @return void
 */
static void lcd_show_spi_process_key_events(void)
{
    key_event_t event = {0};

    while (key.get_event(&event))
    {
        lcd_show_spi_handle_key_event(&event);
    }
}

/**
 * @brief  按周期读取 INA226 最近一次缓存数据
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_spi_update_power_status(uint32_t elapsed_ms)
{
    ina226_sample_t sample = {0};
    fsp_err_t err = FSP_SUCCESS;
    bool power_valid = false;
    bool changed = false;

    g_lcd_show_power_refresh_ms += elapsed_ms;
    if (g_lcd_show_power_refresh_ms < LCD_SHOW_POWER_REFRESH_PERIOD_MS)
    {
        return;
    }
    g_lcd_show_power_refresh_ms = 0U;

    err = ina226.read(&sample);
    power_valid = (FSP_SUCCESS == err);

    lcd_show_enter_critical();
    if (g_lcd_show_power_valid != power_valid)
    {
        changed = true;
    }

    if ((g_lcd_show_bus_voltage_mv != sample.bus_voltage_mv) || (g_lcd_show_current_ua != sample.current_ua))
    {
        changed = true;
    }

    if (changed)
    {
        g_lcd_show_power_valid = power_valid;
        g_lcd_show_bus_voltage_mv = sample.bus_voltage_mv;
        g_lcd_show_current_ua = sample.current_ua;
        lcd_show_mark_dirty();
    }
    lcd_show_exit_critical();
}

/**
 * @brief  刷新麦轮页四路实测轮速缓存
 *
 * @return void
 */
static void lcd_show_spi_update_mecanum_speed_status(void)
{
    float measured_speed_mps[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool changed = false;

    if ((ST7789_UI_VIEW_PAGE != g_lcd_show_view) ||
        (ST7789_UI_PAGE_MECANUM_PID != g_lcd_show_current_page))
    {
        return;
    }

    if ((ST7789_UI_MECANUM_MODE_BODY != g_lcd_show_mecanum_mode) &&
        (ST7789_UI_MECANUM_MODE_WHEEL != g_lcd_show_mecanum_mode) &&
        (ST7789_UI_MECANUM_MODE_PWM != g_lcd_show_mecanum_mode))
    {
        return;
    }

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        measured_speed_mps[i] = encoder.get_speed((chassis_wheel_t)i);
    }

    lcd_show_enter_critical();
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (lcd_show_abs_float(g_lcd_show_mecanum_measured_speed_mps[i] - measured_speed_mps[i]) >
            LCD_SHOW_MECANUM_SPEED_EPS_MPS)
        {
            g_lcd_show_mecanum_measured_speed_mps[i] = measured_speed_mps[i];
            changed = true;
        }
    }

    if (changed)
    {
        lcd_show_mark_dirty();
    }
    lcd_show_exit_critical();
}
#endif

/**
 * @brief  按实际经过时间推进 3 秒开机进度
 *
 * @param elapsed_ms 距离上次处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_auto_boot_progress(uint32_t elapsed_ms)
{
    uint8_t next_progress = 0U;

    lcd_show_enter_critical();

#if !UI_PORT_USE_MIPI_LCD
    if (ST7789_UI_VIEW_BOOT != g_lcd_show_view)
    {
        lcd_show_exit_critical();
        return;
    }
#else
    if (MIPI_UI_VIEW_BOOT != g_mipi_view)
    {
        lcd_show_exit_critical();
        return;
    }
#endif

    if (g_lcd_show_boot_progress < 100U)
    {
        g_lcd_show_boot_auto_ms += elapsed_ms;
        if (g_lcd_show_boot_auto_ms > LCD_SHOW_BOOT_AUTO_TOTAL_MS)
        {
            g_lcd_show_boot_auto_ms = LCD_SHOW_BOOT_AUTO_TOTAL_MS;
        }

        next_progress = (uint8_t)((g_lcd_show_boot_auto_ms * 100U) / LCD_SHOW_BOOT_AUTO_TOTAL_MS);
        if (g_lcd_show_boot_progress != next_progress)
        {
            g_lcd_show_boot_progress = next_progress;
            lcd_show_mark_dirty();
        }
    }

#if !UI_PORT_USE_MIPI_LCD
    if (100U == g_lcd_show_boot_progress)
    {
        g_lcd_show_view = ST7789_UI_VIEW_MENU;
        lcd_show_mark_dirty();
    }
#else
    if (100U == g_lcd_show_boot_progress)
    {
        g_mipi_view = MIPI_UI_VIEW_HOME;
        lcd_show_mark_dirty();
    }
#endif

    lcd_show_exit_critical();
}

/**
 * @brief  执行一次 LVGL 开机页刷新
 *
 * @param elapsed_ms 距离上次 LVGL 处理经过的时间，单位 ms
 * @return void
 */
static void lcd_show_refresh(uint32_t elapsed_ms)
{
    boot_ui_state_t state;
    bool dirty = false;

    dirty = lcd_show_copy_state_and_clear_dirty(&state);
    if (dirty)
    {
        boot_ui_update(&state);
    }

    ui_app_process(elapsed_ms);
}

/**
 * @brief  初始化 LCD 显示和开机页
 *
 * @return fsp_err_t FSP_SUCCESS 表示初始化成功
 */
static fsp_err_t lcd_show_init_impl(void)
{
    boot_ui_state_t state;
    fsp_err_t err = FSP_SUCCESS;
#if !UI_PORT_USE_MIPI_LCD
    uint8_t i = 0U;
    chassis_pid_params_t pid_params =
    {
        .kp = CHASSIS_PID_DEFAULT_KP,
        .ki = CHASSIS_PID_DEFAULT_KI,
    };
#else
    uint8_t i = 0U;
    chassis_pid_params_t pid_params =
    {
        .kp = CHASSIS_PID_DEFAULT_KP,
        .ki = CHASSIS_PID_DEFAULT_KI,
    };
#endif

    if (g_lcd_show_initialized)
    {
        return FSP_SUCCESS;
    }

    err = ui_app_init();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

#if UI_PORT_USE_MIPI_LCD
    (void)ina226.init();
    chassis.get_pid(&pid_params);
#endif

    lcd_show_enter_critical();
    g_lcd_show_refresh_ms = 0U;
    g_lcd_show_boot_auto_ms = 0U;
    g_lcd_show_boot_progress = 0U;
    g_lcd_show_dirty = true;
#if !UI_PORT_USE_MIPI_LCD
    g_lcd_show_view = ST7789_UI_VIEW_BOOT;
    g_lcd_show_selected_index = 0U;
    g_lcd_show_current_page = ST7789_UI_PAGE_MECANUM_PID;
    g_lcd_show_editing = false;
    g_lcd_show_edit_value = 0;
    g_lcd_show_power_valid = false;
    g_lcd_show_bus_voltage_mv = 0U;
    g_lcd_show_current_ua = 0;
    g_lcd_show_power_refresh_ms = LCD_SHOW_POWER_REFRESH_PERIOD_MS;
    chassis.get_pid(&pid_params);
    g_lcd_show_mecanum_mode = ST7789_UI_MECANUM_MODE_BODY;
    g_lcd_show_mecanum_selected_item = 0U;
    g_lcd_show_mecanum_editing = false;
    g_lcd_show_mecanum_edit_value = 0.0f;
    g_lcd_show_mecanum_vx_mps = 0.0f;
    g_lcd_show_mecanum_vy_mps = 0.0f;
    g_lcd_show_mecanum_wz_radps = 0.0f;
    g_lcd_show_mecanum_kp = pid_params.kp;
    g_lcd_show_mecanum_ki = pid_params.ki;
    g_lcd_show_mecanum_save_status = ST7789_UI_MECANUM_SAVE_NONE;
    for (i = 0U; i < (uint8_t)ST7789_UI_PAGE_COUNT; i++)
    {
        g_lcd_show_page_values[i] = 0;
    }
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_lcd_show_mecanum_wheel_speed_mps[i] = 0.0f;
        g_lcd_show_mecanum_pwm_counts[i] = 0;
        g_lcd_show_mecanum_measured_speed_mps[i] = 0.0f;
    }
#else
    g_mipi_view = MIPI_UI_VIEW_BOOT;
    g_mipi_home_selected = MIPI_HOME_PAGE_MISSION;
    g_mipi_placeholder_page = MIPI_HOME_PAGE_CHASSIS;
    g_mipi_chassis_mode = MIPI_CHASSIS_MODE_BODY;
    g_mipi_selected_index = 0U;
    g_mipi_editing = false;
    g_mipi_edit_value = 0.0f;
    g_mipi_edit_original_value = 0.0f;
    g_mipi_chassis_control_enabled = false;
    g_mipi_power_valid = false;
    g_mipi_bus_voltage_mv = 0U;
    g_mipi_current_ua = 0;
    g_mipi_power_refresh_ms = LCD_SHOW_POWER_REFRESH_PERIOD_MS;
    g_mipi_voice_ai_valid = false;
    g_mipi_voice_ai_index = 0U;
    g_mipi_voice_ai_score_percent = 0U;
    g_mipi_voice_ai_label[0] = '\0';
    g_mipi_voice_ai_refresh_ms = LCD_SHOW_MIPI_VOICE_AI_REFRESH_PERIOD_MS;
    g_mipi_feedback_refresh_ms = LCD_SHOW_MIPI_FEEDBACK_REFRESH_PERIOD_MS;
    g_mipi_radar_refresh_ms = LCD_SHOW_MIPI_RADAR_REFRESH_PERIOD_MS;
    g_mipi_vx_mps = 0.0f;
    g_mipi_vy_mps = 0.0f;
    g_mipi_wz_radps = 0.0f;
    g_mipi_kp = pid_params.kp;
    g_mipi_ki = pid_params.ki;
    g_mipi_pid_save_status = MIPI_PID_SAVE_NONE;
    g_mipi_radar_cloud_valid = false;
    g_mipi_radar_cloud_sequence = 0U;
    g_mipi_radar_source_scan_sequence = 0U;
    g_mipi_radar_rpm = 0.0f;
    g_mipi_radar_nearest_distance_mm = 0U;
    g_mipi_avoidance_state = CHASSIS_AVOIDANCE_STATE_NORMAL;
    g_mipi_avoidance_radar_stale_ms = 0U;
    g_mipi_avoidance_target_angle_deg = 0.0f;
    g_mipi_avoidance_selected_angle_deg = 0.0f;
    g_mipi_avoidance_target_safe = false;
    g_mipi_arm_initialized = false;
    g_mipi_arm_x_mm = MECH_HAND_HOME_X;
    g_mipi_arm_y_mm = MECH_HAND_HOME_Y;
    g_mipi_arm_z_mm = MECH_HAND_HOME_Z;
    g_mipi_arm_time_ms = 1000U;
    g_mipi_arm_servo_index = 0U;
    g_mipi_arm_servo_pwm = 1500U;
    g_mipi_arm_gripper_opened = false;
    g_mipi_arm_last_result = MECH_HAND_OK;
    g_mipi_arm_last_clamped = false;
    g_mipi_arm_last_command[0] = '\0';
    mechanical_hand.get_servo_positions(&g_mipi_arm_target_positions);
    (void)memset(&g_mipi_arm_feedback, 0, sizeof(g_mipi_arm_feedback));
    g_mipi_arm_refresh_ms = LCD_SHOW_MIPI_ARM_REFRESH_PERIOD_MS;
    g_mipi_arm_feedback_sequence = 0U;
    g_mipi_arm_calib_point_index = 0U;
    g_mipi_arm_calib_z_mm = MECH_HAND_HOME_Z;
    g_mipi_arm_calib_pick_state = LCD_SHOW_MIPI_ARM_CALIB_PICK_IDLE;
    g_mipi_arm_calib_flow_state = MIPI_ARM_CALIB_FLOW_IDLE;
    g_mipi_arm_calib_wait_ms = 0U;
    g_mipi_arm_calib_wait_total_ms = 0U;
    (void)memset(&g_mipi_mission_snapshot, 0, sizeof(g_mipi_mission_snapshot));
    g_mipi_mission_refresh_ms = LCD_SHOW_MIPI_MISSION_REFRESH_PERIOD_MS;
    (void)voice_dataset.init();
    voice_dataset.status_get(&g_mipi_voice_status);
    g_mipi_voice_label_index = 0U;
    lcd_show_mipi_voice_sync_selected_label_unlocked();
    g_mipi_voice_refresh_ms = LCD_SHOW_MIPI_VOICE_REFRESH_PERIOD_MS;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_mipi_wheel_target_mps[i] = 0.0f;
        g_mipi_wheel_measured_mps[i] = 0.0f;
        g_mipi_wheel_rpm[i] = 0.0f;
        g_mipi_pwm_counts[i] = 0;
    }
    for (i = 0U; i < N10_RADAR_CLOUD_SECTOR_COUNT; i++)
    {
        g_mipi_radar_sector_min_mm[i] = 0U;
        g_mipi_radar_sector_valid[i] = 0U;
    }
#endif
    lcd_show_exit_critical();

    lcd_show_copy_state(&state);
    err = boot_ui_init(&state);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    g_lcd_show_initialized = true;

    return FSP_SUCCESS;
}

/**
 * @brief  处理显示周期时钟
 *
 * @param elapsed_ms 距离上次调用经过的时间，单位 ms
 * @return void
 */
static void lcd_show_process_1ms_tick_impl(uint32_t elapsed_ms)
{
    uint32_t handler_elapsed_ms = 0U;

    if (!g_lcd_show_initialized)
    {
        return;
    }

    if (0U == elapsed_ms)
    {
        return;
    }

    g_lcd_show_refresh_ms += elapsed_ms;
#if !UI_PORT_USE_MIPI_LCD
    lcd_show_spi_process_key_events();
    lcd_show_spi_update_power_status(elapsed_ms);
    lcd_show_spi_update_mecanum_speed_status();
#else
    lcd_show_mipi_process_key_events();
    lcd_show_mipi_arm_calib_process_auto(elapsed_ms);
    lcd_show_mipi_update_power_status(elapsed_ms);
    lcd_show_mipi_update_voice_ai_result(elapsed_ms);
    lcd_show_mipi_update_feedback_status(elapsed_ms);
    lcd_show_mipi_refresh_radar_cloud(elapsed_ms);
    lcd_show_mipi_refresh_arm_feedback(elapsed_ms);
    lcd_show_mipi_refresh_mission(elapsed_ms);
    lcd_show_mipi_refresh_voice_dataset(elapsed_ms);
#endif
    lcd_show_auto_boot_progress(elapsed_ms);

    if (g_lcd_show_refresh_ms >= LCD_SHOW_REFRESH_PERIOD_MS)
    {
        handler_elapsed_ms = g_lcd_show_refresh_ms;
        g_lcd_show_refresh_ms = 0U;
        lcd_show_refresh(handler_elapsed_ms);
    }
}

/**
 * @brief  设置开机页进度
 *
 * @param percent 进度百分比
 * @return void
 */
static void lcd_show_set_boot_progress_impl(uint8_t percent)
{
    if (percent > 100U)
    {
        percent = 100U;
    }

    lcd_show_enter_critical();
    if (g_lcd_show_boot_progress != percent)
    {
        g_lcd_show_boot_progress = percent;
        g_lcd_show_boot_auto_ms = ((uint32_t)percent * LCD_SHOW_BOOT_AUTO_TOTAL_MS) / 100U;
        lcd_show_mark_dirty();
    }
#if !UI_PORT_USE_MIPI_LCD
    if ((100U == percent) && (ST7789_UI_VIEW_BOOT == g_lcd_show_view))
    {
        g_lcd_show_view = ST7789_UI_VIEW_MENU;
        lcd_show_mark_dirty();
    }
#else
    if ((100U == percent) && (MIPI_UI_VIEW_BOOT == g_mipi_view))
    {
        g_mipi_view = MIPI_UI_VIEW_HOME;
        lcd_show_mark_dirty();
    }
#endif
    lcd_show_exit_critical();
}

const lcd_show_api_t lcd_show =
{
    .init = lcd_show_init_impl,
    .process_1ms_tick = lcd_show_process_1ms_tick_impl,
    .set_boot_progress = lcd_show_set_boot_progress_impl,
};
