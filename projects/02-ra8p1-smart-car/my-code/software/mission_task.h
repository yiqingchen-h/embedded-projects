#ifndef SOFTWARE_MISSION_TASK_H_
#define SOFTWARE_MISSION_TASK_H_

#include "hal_data.h"
#include "hardware/Chassis/chassis_avoidance.h"
#include "hardware/mechanical_arm/Mechanical_hand_control.h"
#include "hardware/vision/Vision_Uart8_Comm.h"
#include "software/voice_ai.h"

#include <stdbool.h>
#include <stdint.h>

/* 机械臂到目标上方的安全接近高度，单位 mm。 */
#ifndef MISSION_ARM_APPROACH_HEIGHT_MM
#define MISSION_ARM_APPROACH_HEIGHT_MM (50.0f)
#endif

/* 机械臂 TCP 坐标移动动作时间，单位 ms。 */
#ifndef MISSION_ARM_MOVE_TIME_MS
#define MISSION_ARM_MOVE_TIME_MS (1000U)
#endif

/* 机械臂夹爪开合动作时间，单位 ms。 */
#ifndef MISSION_ARM_GRIPPER_TIME_MS
#define MISSION_ARM_GRIPPER_TIME_MS (500U)
#endif

/* 机械臂回 Home 动作时间，单位 ms。 */
#ifndef MISSION_ARM_HOME_TIME_MS
#define MISSION_ARM_HOME_TIME_MS (1000U)
#endif

/* Mission 最近抓取目标标签缓存长度。 */
#define MISSION_PICK_LABEL_LEN (24U)

/* 每个机械臂动作完成后的额外等待时间，单位 ms。 */
#ifndef MISSION_ARM_ACTION_SETTLE_MS
#define MISSION_ARM_ACTION_SETTLE_MS (500U)
#endif

/* 放置目标 X 方向补偿，单位 mm。 */
#define MISSION_PLACE_OFFSET_X_MM (0.0f)

/* 放置目标 Y 方向补偿，单位 mm。 */
#define MISSION_PLACE_OFFSET_Y_MM (0.0f)

/* 放置目标 Z 方向补偿，单位 mm。 */
#define MISSION_PLACE_OFFSET_Z_MM (0.0f)

typedef enum e_mission_state
{
    /* Mission 未执行流程，底盘保持停车。 */
    MISSION_STATE_IDLE = 0,

    /* 无避障循迹。 */
    MISSION_STATE_TRACE_NO_AVOID,

    /* 有避障循迹。 */
    MISSION_STATE_TRACE_AVOID,

    /* 等待抓取坐标。 */
    MISSION_STATE_WAIT_PICK_POS,

    /* 等待放置坐标。 */
    MISSION_STATE_WAIT_PLACE_POS,

    /* 等待机械臂上一条动作自然完成。 */
    MISSION_STATE_WAIT_ACTION_DONE,

    /* 放置完成后保持不动。 */
    MISSION_STATE_PLACE_AFTER_HOLD,

    /* Mission 发生错误并停车。 */
    MISSION_STATE_ERROR,
} mission_state_t;

typedef enum e_mission_trace_mode
{
    /* 当前循迹模式为无避障。 */
    MISSION_TRACE_MODE_NO_AVOID = 0,

    /* 当前循迹模式为有避障。 */
    MISSION_TRACE_MODE_AVOID,
} mission_trace_mode_t;

typedef struct st_mission_chassis_control
{
    /* true 表示 mission 当前接管底盘控制源。 */
    bool enabled;

    /* true 表示底盘速度需要经过避障入口。 */
    bool use_avoidance;

    /* 前进速度，单位 m/s。 */
    float vx_mps;

    /* 横移速度，单位 m/s。 */
    float vy_mps;

    /* 旋转角速度，单位 rad/s。 */
    float wz_radps;
} mission_chassis_control_t;

typedef struct st_mission_snapshot
{
    /* true 表示 mission 已启用并接管底盘控制源。 */
    bool enabled;

    /* 当前 mission 状态。 */
    mission_state_t state;

    /* 当前循迹模式。 */
    mission_trace_mode_t trace_mode;

    /* 最近一次 9900 速度帧中的 vx，单位 m/s。 */
    float vx_mps;

    /* 最近一次 9900 速度帧中的 vy，单位 m/s。 */
    float vy_mps;

    /* 最近一次 9900 速度帧中的 wz，单位 rad/s。 */
    float wz_radps;

    /* 距离最近一次 9900 速度帧的时间，单位 ms。 */
    uint32_t speed_frame_age_ms;

    /* 最近一次视觉协议帧快照。 */
    vision_uart8_frame_t latest_frame;

    /* true 表示已有本轮抓取或放置目标。 */
    bool target_valid;

    /* true 表示最近一次语音抓取目标已经发送给视觉端。 */
    bool pick_send_valid;

    /* 最近一次语音抓取目标发送是否成功。 */
    bool pick_send_ok;

    /* 最近一次语音抓取目标选择码。 */
    uint16_t pick_target_code;

    /* 最近一次语音抓取目标分数百分比，范围 0~100。 */
    uint8_t pick_score_percent;

    /* 最近一次语音抓取目标标签。 */
    char pick_label[MISSION_PICK_LABEL_LEN];

    /* 目标图像横坐标。 */
    float target_u;

    /* 目标图像纵坐标。 */
    float target_v;

    /* 目标深度原始值。 */
    float target_depth;

    /* 目标基座 X 坐标，单位 mm。 */
    float target_x_mm;

    /* 目标基座 Y 坐标，单位 mm。 */
    float target_y_mm;

    /* 目标基座 Z 坐标，单位 mm。 */
    float target_z_mm;

    /* true 表示 9803 完成标志正在等待 9907 ACK。 */
    bool pending_done_ack;

    /* 最近一次 mission 错误码。 */
    int last_error;

    /* 最近一次读取到的避障状态。 */
    chassis_avoidance_state_t avoidance_state;

    /* true 表示 mission 认为夹爪已闭合。 */
    bool gripper_is_closed;

    /* 最近一次夹爪动作返回码。 */
    int last_gripper_result;

    /* 最近一次机械臂移动或 Home 返回码。 */
    int last_arm_result;

    /* 机械臂 6 路目标 PWM 缓存。 */
    mech_hand_servo_positions_t servo_positions;

    /* true 表示最近一次结构化命令发生过限位裁剪。 */
    bool was_last_command_clamped;
} mission_snapshot_t;

typedef struct st_mission_voice_ai_actions
{
    /* true 表示 UI 需要切换到 Mission 页面。 */
    bool enter_mission_view;

    /* true 表示 UI 需要清空底盘调试控制源。 */
    bool stop_output;
} mission_voice_ai_actions_t;

typedef struct st_mission_api
{
    /**
     * @brief  初始化 mission 状态和机械臂通信
     *
     * @return fsp_err_t FSP_SUCCESS 表示初始化完成
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  请求启动 mission
     *
     * @return fsp_err_t FSP_SUCCESS 表示启动请求已记录
     */
    fsp_err_t (*request_start)(void);

    /**
     * @brief  请求停止 mission 并停车
     *
     * @return void
     */
    void (*request_stop)(void);

    /**
     * @brief  请求发送 mission 监视界面进入标志
     *
     * @return void
     */
    void (*request_monitor_entered)(void);

    /**
     * @brief  推进 mission 状态机
     *
     * @param interval_ms 距离上次调用经过的时间，单位 ms
     * @return void
     */
    void (*process)(uint32_t interval_ms);

    /**
     * @brief  读取 mission 快照供 UI 显示
     *
     * @param p_snapshot 快照输出指针
     * @return void
     */
    void (*snapshot_read)(mission_snapshot_t *p_snapshot);

    /**
     * @brief  读取 mission 底盘控制源
     *
     * @param p_control 底盘控制源输出指针
     * @return bool true 表示 mission 当前接管底盘控制
     */
    bool (*chassis_control_read)(mission_chassis_control_t *p_control);

    /**
     * @brief  消费语音 AI 识别结果并转换为 Mission 动作
     *
     * @param p_result 语音 AI 识别结果
     * @param mission_view_active true 表示 UI 当前位于 Mission 页面
     * @param p_actions UI 后续动作输出
     * @return void
     */
    void (*voice_ai_consume)(const voice_ai_result_t *p_result,
                             bool mission_view_active,
                             mission_voice_ai_actions_t *p_actions);
} mission_api_t;

extern const mission_api_t mission;

#endif /* SOFTWARE_MISSION_TASK_H_ */
