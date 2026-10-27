#ifndef SOFTWARE_UI_MIPI_UI_H_
#define SOFTWARE_UI_MIPI_UI_H_

#include "hal_data.h"
#include "hardware/Chassis/Chassis_config.h"
#include "hardware/Chassis/chassis_avoidance.h"
#include "hardware/N10/n10_radar.h"
#include "hardware/mechanical_arm/Mechanical_hand_control.h"
#include "hardware/mechanical_arm/Mechanical_arm_feedback.h"
#include "software/mission_task.h"
#include "software/voice_dataset_capture.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief MIPI 竖屏当前视图 */
typedef enum e_mipi_ui_view
{
    /* 开机页。 */
    MIPI_UI_VIEW_BOOT = 0,

    /* 首页功能选择页。 */
    MIPI_UI_VIEW_HOME,

    /* 底盘调试页。 */
    MIPI_UI_VIEW_CHASSIS,

    /* 雷达点云页。 */
    MIPI_UI_VIEW_RADAR_CLOUD,

    /* 机械臂调试页。 */
    MIPI_UI_VIEW_ARM_DEBUG,

    /* 机械臂标定辅助页。 */
    MIPI_UI_VIEW_ARM_CALIB,

    /* Mission 任务页。 */
    MIPI_UI_VIEW_MISSION,

    /* 语音数据集采集页。 */
    MIPI_UI_VIEW_VOICE_DATASET,

    /* 未实现功能占位页。 */
    MIPI_UI_VIEW_PLACEHOLDER,
} mipi_ui_view_t;

/** @brief MIPI 首页功能项 */
typedef enum e_mipi_home_page
{
    /* 底盘调试。 */
    MIPI_HOME_PAGE_CHASSIS = 0,

    /* 雷达。 */
    MIPI_HOME_PAGE_RADAR,

    /* 机械臂调试。 */
    MIPI_HOME_PAGE_ARM,

    /* Mission 视觉任务。 */
    MIPI_HOME_PAGE_MISSION,

    /* 语音数据集串口采集。 */
    MIPI_HOME_PAGE_VOICE,

    /* 首页功能数量。 */
    MIPI_HOME_PAGE_COUNT,
} mipi_home_page_t;

/** @brief MIPI 底盘调试模式 */
typedef enum e_mipi_chassis_mode
{
    /* 车体速度控制。 */
    MIPI_CHASSIS_MODE_BODY = 0,

    /* 四轮独立速度控制。 */
    MIPI_CHASSIS_MODE_WHEEL,

    /* 四轮开环 PWM 控制。 */
    MIPI_CHASSIS_MODE_PWM,
} mipi_chassis_mode_t;

/** @brief MIPI PID 保存状态 */
typedef enum e_mipi_pid_save_status
{
    /* 无保存提示。 */
    MIPI_PID_SAVE_NONE = 0,

    /* 正在保存。 */
    MIPI_PID_SAVE_SAVING,

    /* 保存成功。 */
    MIPI_PID_SAVE_SAVED,

    /* 保存失败。 */
    MIPI_PID_SAVE_ERR,
} mipi_pid_save_status_t;

/** @brief MIPI 机械臂标定自动流程状态 */
typedef enum e_mipi_arm_calib_flow_state
{
    /* 自动流程空闲。 */
    MIPI_ARM_CALIB_FLOW_IDLE = 0,

    /* 自动流程正在松开夹爪。 */
    MIPI_ARM_CALIB_FLOW_OPEN,

    /* 自动流程正在移动到目标上方。 */
    MIPI_ARM_CALIB_FLOW_ABOVE,

    /* 自动流程正在避障平移。 */
    MIPI_ARM_CALIB_FLOW_CLEAR,

    /* 自动流程正在回 Home。 */
    MIPI_ARM_CALIB_FLOW_HOME,

    /* 自动流程已完成。 */
    MIPI_ARM_CALIB_FLOW_DONE,

    /* 自动流程已停止。 */
    MIPI_ARM_CALIB_FLOW_STOP,

    /* 自动流程发生错误。 */
    MIPI_ARM_CALIB_FLOW_ERR,
} mipi_arm_calib_flow_state_t;

/* 机械臂最近命令显示缓冲区长度。 */
#define MIPI_UI_ARM_LAST_COMMAND_LEN (32U)

/* 顶栏语音 AI 标签显示缓冲区长度。 */
#define MIPI_UI_VOICE_AI_LABEL_LEN   (24U)

/** @brief MIPI 竖屏 UI 状态 */
typedef struct st_mipi_ui_state
{
    /* 开机页进度，范围 0~100。 */
    uint8_t boot_progress;

    /* 当前显示视图。 */
    mipi_ui_view_t view;

    /* 首页当前选中的功能。 */
    mipi_home_page_t home_selected;

    /* 占位页显示的功能项。 */
    mipi_home_page_t placeholder_page;

    /* 电源采样是否有效。 */
    bool power_valid;

    /* 母线电压，单位 mV。 */
    uint32_t bus_voltage_mv;

    /* 电流，单位 uA。 */
    int32_t current_ua;

    /* 语音 AI 识别结果是否有效。 */
    bool voice_ai_valid;

    /* 语音 AI 识别类别索引。 */
    uint8_t voice_ai_index;

    /* 语音 AI 识别分数百分比，范围 0~100。 */
    uint8_t voice_ai_score_percent;

    /* 语音 AI 识别类别名称。 */
    char voice_ai_label[MIPI_UI_VOICE_AI_LABEL_LEN];

    /* 底盘调试当前模式。 */
    mipi_chassis_mode_t chassis_mode;

    /* 当前页面统一选中项索引。 */
    uint8_t selected_index;

    /* 当前选中项是否处于编辑态。 */
    bool editing;

    /* 编辑中的临时值。 */
    float edit_value;

    /* 底盘调试控制是否已启用。 */
    bool chassis_control_enabled;

    /* 车体前进速度，单位 m/s。 */
    float vx_mps;

    /* 车体横移速度，单位 m/s。 */
    float vy_mps;

    /* 车体旋转角速度，单位 rad/s。 */
    float wz_radps;

    /* 四路轮速目标，单位 m/s。 */
    float wheel_target_mps[CHASSIS_WHEEL_COUNT];

    /* 四路轮速实测值，单位 m/s。 */
    float wheel_measured_mps[CHASSIS_WHEEL_COUNT];

    /* 四路电机转速，单位 rpm。 */
    float wheel_rpm[CHASSIS_WHEEL_COUNT];

    /* 四路开环 PWM 计数值。 */
    int32_t pwm_counts[CHASSIS_WHEEL_COUNT];

    /* 速度环比例系数。 */
    float kp;

    /* 速度环积分系数。 */
    float ki;

    /* PID 保存状态。 */
    mipi_pid_save_status_t pid_save_status;

    /* 雷达完整点云是否有效。 */
    bool radar_cloud_valid;

    /* 雷达完整点云序号。 */
    uint32_t radar_cloud_sequence;

    /* 雷达完整点云来源原始帧序号。 */
    uint32_t radar_source_scan_sequence;

    /* 雷达转速，单位 rpm。 */
    float radar_rpm;

    /* 雷达 72 个扇区最近距离，单位 mm。 */
    uint16_t radar_sector_min_mm[N10_RADAR_CLOUD_SECTOR_COUNT];

    /* 雷达 72 个扇区是否存在有效点。 */
    uint8_t radar_sector_valid[N10_RADAR_CLOUD_SECTOR_COUNT];

    /* 雷达最近有效障碍距离，单位 mm；0 表示无有效点。 */
    uint16_t radar_nearest_distance_mm;

    /* 避障模块当前状态。 */
    chassis_avoidance_state_t avoidance_state;

    /* 避障模块雷达数据未更新时长，单位 ms。 */
    uint32_t avoidance_radar_stale_ms;

    /* 避障模块上层目标方向，单位度。 */
    float avoidance_target_angle_deg;

    /* 避障模块选中方向，单位度。 */
    float avoidance_selected_angle_deg;

    /* 避障模块目标方向是否安全。 */
    bool avoidance_target_safe;

    /* 机械臂通信是否已初始化。 */
    bool arm_initialized;

    /* 机械臂调试目标 X，单位 mm。 */
    float arm_x_mm;

    /* 机械臂调试目标 Y，单位 mm。 */
    float arm_y_mm;

    /* 机械臂调试目标 Z，单位 mm。 */
    float arm_z_mm;

    /* 机械臂动作时间，单位 ms。 */
    uint16_t arm_time_ms;

    /* 单舵机调试序号。 */
    uint8_t arm_servo_index;

    /* 单舵机调试目标 PWM。 */
    uint16_t arm_servo_pwm;

    /* 夹爪是否处于打开状态。 */
    bool arm_gripper_opened;

    /* 机械臂最近一次动作返回码。 */
    int arm_last_result;

    /* 机械臂最近一次结构化命令是否触发限位裁剪。 */
    bool arm_last_clamped;

    /* 机械臂最近一次下发命令短文本。 */
    char arm_last_command[MIPI_UI_ARM_LAST_COMMAND_LEN];

    /* 机械臂 6 路目标 PWM 缓存。 */
    mech_hand_servo_positions_t arm_target_positions;

    /* 机械臂独立反馈任务快照。 */
    mechanical_arm_feedback_snapshot_t arm_feedback;

    /* 机械臂标定页当前点位序号，0 表示第一组。 */
    uint8_t arm_calib_point_index;

    /* 机械臂标定页当前点位总数。 */
    uint8_t arm_calib_point_count;

    /* 机械臂标定页当前 X 坐标，单位 mm。 */
    float arm_calib_x_mm;

    /* 机械臂标定页当前 Y 坐标，单位 mm。 */
    float arm_calib_y_mm;

    /* 机械臂标定页当前 Z 坐标，单位 mm。 */
    float arm_calib_z_mm;

    /* 机械臂标定页目标上方 Z 坐标，单位 mm。 */
    float arm_calib_above_z_mm;

    /* 机械臂标定自动流程是否运行中。 */
    bool arm_calib_auto_running;

    /* 机械臂标定自动流程显示状态。 */
    mipi_arm_calib_flow_state_t arm_calib_flow_state;

    /* Mission 状态机快照。 */
    mission_snapshot_t mission_snapshot;

    /* 语音数据集采集发送状态。 */
    voice_dataset_status_t voice_dataset_status;
} mipi_ui_state_t;

/**
 * @brief  初始化 MIPI 竖屏 UI 对象
 *
 * @param p_state 初始 UI 状态
 * @return fsp_err_t FSP_SUCCESS 表示初始化成功
 */
fsp_err_t mipi_ui_init(const mipi_ui_state_t *p_state);

/**
 * @brief  按当前状态刷新 MIPI 竖屏 UI
 *
 * @param p_state 当前 UI 状态
 * @return void
 */
void mipi_ui_update(const mipi_ui_state_t *p_state);

#endif /* SOFTWARE_UI_MIPI_UI_H_ */
