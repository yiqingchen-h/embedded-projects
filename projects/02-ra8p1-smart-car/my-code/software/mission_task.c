#include "software/mission_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "hardware/Chassis/Chassis.h"
#include "hardware/mechanical_arm/Calibration.h"
#include "hardware/mechanical_arm/Mechanical_arm_feedback.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* 上位机普通速度帧。 */
#define MISSION_CODE_PC_SPEED (9900U)

/* 上位机切换无避障循迹。 */
#define MISSION_CODE_PC_TRACE_NO_AVOID (9901U)

/* 上位机切换有避障循迹。 */
#define MISSION_CODE_PC_TRACE_AVOID (9902U)

/* 上位机请求等待抓取坐标。 */
#define MISSION_CODE_PC_WAIT_PICK (9903U)

/* 上位机请求等待放置坐标。 */
#define MISSION_CODE_PC_WAIT_PLACE (9904U)

/* 上位机坐标帧有效。 */
#define MISSION_CODE_PC_POSITION_VALID (9905U)

/* 上位机立即停车。 */
#define MISSION_CODE_PC_STOP_NOW (9906U)

/* 上位机确认收到动作完成标志。 */
#define MISSION_CODE_PC_DONE_ACK (9907U)

/* 下位机已进入 mission。 */
#define MISSION_CODE_MCU_ENTERED (9801U)

/* 下位机请求发送抓取或放置坐标。 */
#define MISSION_CODE_MCU_REQUEST_POSITION (9802U)

/* 下位机抓取或放置动作完成。 */
#define MISSION_CODE_MCU_ACTION_DONE (9803U)

/* 下位机避障完成。 */
#define MISSION_CODE_MCU_AVOID_DONE (9804U)

/* 下位机 mission 错误。 */
#define MISSION_CODE_MCU_ERROR (9805U)

/* 下位机已进入 mission 监视界面。 */
#define MISSION_CODE_MCU_MONITOR_ENTERED (9806U)

/* 下位机视觉目标选择码起点，对应 select_cube。 */
#define MISSION_CODE_MCU_SELECT_BASE (9811U)
/*
select_cube	3	9811
select_hippo	4	9812
select_cat	5	9813
select_stone_pillar	6	9814
select_capybara	7	9815
select_corn	8	9816
select_jiaotailang	9	9817
select_dog	10	9818
select_tnt	11	9819
*/
/* 循迹速度帧超时时间，单位 ms。 */
#define MISSION_SPEED_TIMEOUT_MS (200U)

/* 9803 完成标志重发周期，单位 ms。 */
#define MISSION_DONE_RESEND_PERIOD_MS (200U)

/* mission 获取机械臂总线互斥锁的超时时间，单位 ms。 */
#define MISSION_ARM_BUS_LOCK_TIMEOUT_MS (500U)

/* 语音唤醒后的有效窗口，单位 ms。 */
#define MISSION_VOICE_WAKE_WINDOW_MS (5000U)

/* 语音候选一致性接受数量。 */
#define MISSION_VOICE_ACCEPT_MATCH_COUNT (4U)

/* 语音分数换算为千分比的满分值。 */
#define MISSION_VOICE_SCORE_MILLI (1000.0f)

/* 浮点协议码转整数时允许的半区间。 */
#define MISSION_CODE_ROUND_OFFSET (0.5f)

typedef enum e_mission_arm_step
{
    /* 当前没有待执行机械臂步骤。 */
    MISSION_ARM_STEP_NONE = 0,

    /* 抓取流程下一步打开夹爪。 */
    MISSION_ARM_STEP_PICK_OPEN,

    /* 抓取流程下一步下探到目标点。 */
    MISSION_ARM_STEP_PICK_DESCEND,

    /* 抓取流程下一步闭合夹爪。 */
    MISSION_ARM_STEP_PICK_CLOSE,

    /* 抓取流程下一步抬起到目标上方。 */
    MISSION_ARM_STEP_PICK_LIFT,

    /* 抓取流程下一步回 Home。 */
    MISSION_ARM_STEP_PICK_HOME,

    /* 抓取流程下一步发送完成并恢复循迹。 */
    MISSION_ARM_STEP_PICK_DONE,

    /* 放置流程下一步下探到目标点。 */
    MISSION_ARM_STEP_PLACE_DESCEND,

    /* 放置流程下一步打开夹爪释放。 */
    MISSION_ARM_STEP_PLACE_OPEN,

    /* 放置流程下一步回 Home。 */
    MISSION_ARM_STEP_PLACE_HOME,

    /* 放置流程下一步发送完成并保持不动。 */
    MISSION_ARM_STEP_PLACE_DONE,
} mission_arm_step_t;

/* 语音标签接受阈值，单位为千分比分数，顺序必须与 voice_ai_label_t 一致。 */
static const uint16_t g_mission_voice_score_thresholds[VOICE_AI_LABEL_COUNT] =
    {
        750U, /* 唤醒词*/
        800U,/* 开始*/
        800U,/* 停止*/
        750U,/* 抓取方块*/
        700U,/*抓河马*/
        700U,/*抓小猫*/
        500U,/*抓石墩子*/
        700U,/*抓水豚*/
        700U,/*抓玉米*/
        700U,/*抓蕉太狼*/
        600U,/*抓小狗*/
        600U,/*抓TNT*/
        800U,
        800U,
};

/* mission 是否已完成初始化。 */
static bool g_mission_initialized = false;

/* mission 是否已启用。 */
static bool g_mission_enabled = false;

/* mission 当前状态。 */
static mission_state_t g_mission_state = MISSION_STATE_IDLE;

/* mission 当前循迹模式。 */
static mission_trace_mode_t g_mission_trace_mode = MISSION_TRACE_MODE_NO_AVOID;

/* 抓取完成后需要恢复的循迹模式。 */
static mission_trace_mode_t g_mission_resume_trace_mode = MISSION_TRACE_MODE_NO_AVOID;

/* mission 内部累计时间，单位 ms。 */
static uint32_t g_mission_now_ms = 0U;

/* true 表示语音唤醒窗口仍然有效。 */
static bool g_mission_voice_wake_active = false;

/* 最近一次语音唤醒时间，单位 ms。 */
static uint32_t g_mission_voice_wake_tick_ms = 0U;

/* 最近一次速度帧时间，单位 ms。 */
static uint32_t g_mission_last_speed_rx_ms = 0U;

/* 最近一次速度帧 vx，单位 m/s。 */
static float g_mission_vx_mps = 0.0f;

/* 最近一次速度帧 vy，单位 m/s。 */
static float g_mission_vy_mps = 0.0f;

/* 最近一次速度帧 wz，单位 rad/s。 */
static float g_mission_wz_radps = 0.0f;

/* mission 已处理到的视觉帧计数。 */
static uint32_t g_mission_last_rx_ok_count = 0U;

/* 进入等待坐标状态时的视觉帧计数。 */
static uint32_t g_mission_wait_start_rx_ok_count = 0U;

/* 最近一次视觉协议帧快照。 */
static vision_uart8_frame_t g_mission_latest_frame = {0};

/* 当前目标图像横坐标。 */
static float g_mission_target_u = 0.0f;

/* 当前目标图像纵坐标。 */
static float g_mission_target_v = 0.0f;

/* 当前目标深度原始值。 */
static float g_mission_target_depth = 0.0f;

/* 当前目标基座 X 坐标，单位 mm。 */
static float g_mission_target_x_mm = 0.0f;

/* 当前目标基座 Y 坐标，单位 mm。 */
static float g_mission_target_y_mm = 0.0f;

/* 当前目标基座 Z 坐标，单位 mm。 */
static float g_mission_target_z_mm = 0.0f;

/* true 表示已有本轮目标点。 */
static bool g_mission_target_valid = false;

/* true 表示最近一次语音抓取目标已经发送给视觉端。 */
static bool g_mission_pick_send_valid = false;

/* 最近一次语音抓取目标发送是否成功。 */
static bool g_mission_pick_send_ok = false;

/* 最近一次语音抓取目标选择码。 */
static uint16_t g_mission_pick_target_code = 0U;

/* 最近一次语音抓取目标分数百分比，范围 0~100。 */
static uint8_t g_mission_pick_score_percent = 0U;

/* 最近一次语音抓取目标标签。 */
static char g_mission_pick_label[MISSION_PICK_LABEL_LEN] = {0};

/* true 表示 9801 进入标志待发送。 */
static bool g_mission_pending_enter_status = false;

/* true 表示启动时的旧视觉帧已经丢弃。 */
static bool g_mission_start_frame_cleared = false;

/* true 表示 9802 请求坐标待发送。 */
static bool g_mission_pending_request_position = false;

/* true 表示 9804 避障完成待发送。 */
static bool g_mission_pending_avoid_done = false;

/* true 表示 9805 mission 错误待发送。 */
static bool g_mission_pending_error = false;

/* true 表示 9806 mission 监视界面进入标志待发送。 */
static bool g_mission_pending_monitor_entered = false;

/* true 表示 9803 动作完成标志等待 ACK。 */
static bool g_mission_pending_done_ack = false;

/* 最近一次 9803 发送时间，单位 ms。 */
static uint32_t g_mission_last_done_send_ms = 0U;

/* 最近一次 mission 错误码。 */
static int g_mission_last_error = MECH_HAND_OK;

/* 最近一次避障状态。 */
static chassis_avoidance_state_t g_mission_avoidance_state = CHASSIS_AVOIDANCE_STATE_NORMAL;

/* 上一次用于检测 9804 的避障状态。 */
static chassis_avoidance_state_t g_mission_last_avoidance_state = CHASSIS_AVOIDANCE_STATE_NORMAL;

/* true 表示 mission 认为夹爪当前闭合。 */
static bool g_mission_gripper_is_closed = false;

/* 最近一次夹爪动作返回码。 */
static int g_mission_last_gripper_result = MECH_HAND_OK;

/* 最近一次机械臂移动或 Home 返回码。 */
static int g_mission_last_arm_result = MECH_HAND_OK;

/* 机械臂 6 路目标 PWM 缓存。 */
static mech_hand_servo_positions_t g_mission_servo_positions = {{1500U, 1500U, 1500U, 1500U, 1500U, 1500U}};

/* true 表示最近一次结构化命令发生过限位裁剪。 */
static bool g_mission_was_last_command_clamped = false;

/* WAIT_ACTION_DONE 结束后要执行的下一步机械臂动作。 */
static mission_arm_step_t g_mission_next_arm_step = MISSION_ARM_STEP_NONE;

/* 当前机械臂等待开始时间，单位 ms。 */
static uint32_t g_mission_action_start_ms = 0U;

/* 当前机械臂动作需要等待的总时间，单位 ms。 */
static uint32_t g_mission_action_wait_ms = 0U;

/**
 * @brief  进入 mission 临界区
 *
 * @return void
 */
static void mission_enter_critical(void)
{
    taskENTER_CRITICAL();
}

/**
 * @brief  退出 mission 临界区
 *
 * @return void
 */
static void mission_exit_critical(void)
{
    taskEXIT_CRITICAL();
}

/**
 * @brief  将 4 float 协议码转换为整数命令
 *
 * @param value 第 4 个浮点字段
 * @return uint16_t 四舍五入后的协议码
 */
static uint16_t mission_frame_code(float value)
{
    if (value < 0.0f)
    {
        return 0U;
    }

    return (uint16_t)(value + MISSION_CODE_ROUND_OFFSET);
}

/**
 * @brief  发送一个 mission 状态帧
 *
 * @param code 98xx 状态码
 * @param data1 第 1 个数据字段
 * @return bool true 表示发送成功
 */
static bool mission_send_status(uint16_t code, float data1)
{
    float values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    fsp_err_t err = FSP_SUCCESS;

    values[0] = data1;
    values[3] = (float)code;
    err = vision_uart8.send4(values, 4U);
    if (FSP_SUCCESS != err)
    {
        mission_enter_critical();
        g_mission_last_error = (int)err;
        mission_exit_critical();
        return false;
    }

    return true;
}

/**
 * @brief  清空当前目标点
 *
 * @return void
 */
static void mission_clear_target_unlocked(void)
{
    g_mission_target_u = 0.0f;
    g_mission_target_v = 0.0f;
    g_mission_target_depth = 0.0f;
    g_mission_target_x_mm = 0.0f;
    g_mission_target_y_mm = 0.0f;
    g_mission_target_z_mm = 0.0f;
    g_mission_target_valid = false;
}

/**
 * @brief  清空最近一次语音抓取目标发送显示状态
 *
 * @return void
 */
static void mission_clear_pick_status_unlocked(void)
{
    g_mission_pick_send_valid = false;
    g_mission_pick_send_ok = false;
    g_mission_pick_target_code = 0U;
    g_mission_pick_score_percent = 0U;
    g_mission_pick_label[0] = '\0';
}

/**
 * @brief  把循迹模式转换为状态
 *
 * @param trace_mode 循迹模式
 * @return mission_state_t 对应状态
 */
static mission_state_t mission_trace_mode_to_state(mission_trace_mode_t trace_mode)
{
    if (MISSION_TRACE_MODE_AVOID == trace_mode)
    {
        return MISSION_STATE_TRACE_AVOID;
    }

    return MISSION_STATE_TRACE_NO_AVOID;
}

/**
 * @brief  进入 mission 错误状态并停车
 *
 * @param error 错误码
 * @return void
 */
static void mission_enter_error(int error)
{
    mission_enter_critical();
    g_mission_last_error = error;
    g_mission_pending_error = true;
    g_mission_state = MISSION_STATE_ERROR;
    g_mission_vx_mps = 0.0f;
    g_mission_vy_mps = 0.0f;
    g_mission_wz_radps = 0.0f;
    g_mission_next_arm_step = MISSION_ARM_STEP_NONE;
    mission_exit_critical();
}

/**
 * @brief  记录机械臂状态供 UI 显示
 *
 * @return void
 */
static void mission_refresh_arm_debug_state(void)
{
    mech_hand_servo_positions_t positions = {{0U}};
    bool clamped = false;

    mechanical_hand.get_servo_positions(&positions);
    clamped = mechanical_hand.was_last_command_clamped();

    mission_enter_critical();
    g_mission_servo_positions = positions;
    g_mission_was_last_command_clamped = clamped;
    mission_exit_critical();
}

/**
 * @brief  开始等待机械臂动作完成
 *
 * @param next_step 等待结束后的下一步
 * @param action_time_ms 已下发动作的设定时间，单位 ms
 * @return void
 */
static void mission_begin_action_wait(mission_arm_step_t next_step, uint32_t action_time_ms)
{
    mission_enter_critical();
    g_mission_state = MISSION_STATE_WAIT_ACTION_DONE;
    g_mission_next_arm_step = next_step;
    g_mission_action_start_ms = g_mission_now_ms;
    g_mission_action_wait_ms = action_time_ms + MISSION_ARM_ACTION_SETTLE_MS;
    mission_exit_critical();
}

/**
 * @brief  下发机械臂 TCP 移动命令
 *
 * @param x_mm 目标 X，单位 mm
 * @param y_mm 目标 Y，单位 mm
 * @param z_mm 目标 Z，单位 mm
 * @param next_step 等待结束后的下一步
 * @return bool true 表示命令已成功下发
 */
static bool mission_issue_move(float x_mm, float y_mm, float z_mm, mission_arm_step_t next_step)
{
    int result = MECH_HAND_OK;

    if (!mechanical_arm_feedback.lock_bus(MISSION_ARM_BUS_LOCK_TIMEOUT_MS))
    {
        mission_enter_error(MECH_HAND_ERR_TIMEOUT);
        return false;
    }

    result = mechanical_hand.move_tcp_to(x_mm, y_mm, z_mm, (uint16_t)MISSION_ARM_MOVE_TIME_MS);
    mechanical_arm_feedback.request_refresh_all();
    mechanical_arm_feedback.unlock_bus();
    mission_refresh_arm_debug_state();

    mission_enter_critical();
    g_mission_last_arm_result = result;
    mission_exit_critical();

    if (MECH_HAND_OK != result)
    {
        mission_enter_error(result);
        return false;
    }

    mission_begin_action_wait(next_step, MISSION_ARM_MOVE_TIME_MS);
    return true;
}

/**
 * @brief  下发机械臂 Home 命令
 *
 * @param next_step 等待结束后的下一步
 * @return bool true 表示命令已成功下发
 */
static bool mission_issue_home(mission_arm_step_t next_step)
{
    int result = MECH_HAND_OK;

    if (!mechanical_arm_feedback.lock_bus(MISSION_ARM_BUS_LOCK_TIMEOUT_MS))
    {
        mission_enter_error(MECH_HAND_ERR_TIMEOUT);
        return false;
    }

    result = mechanical_hand.home((uint16_t)MISSION_ARM_HOME_TIME_MS);
    mechanical_arm_feedback.request_refresh_all();
    mechanical_arm_feedback.unlock_bus();
    mission_refresh_arm_debug_state();

    mission_enter_critical();
    g_mission_last_arm_result = result;
    mission_exit_critical();

    if (MECH_HAND_OK != result)
    {
        mission_enter_error(result);
        return false;
    }

    mission_begin_action_wait(next_step, MISSION_ARM_HOME_TIME_MS);
    return true;
}

/**
 * @brief  下发夹爪动作命令
 *
 * @param close_gripper true 表示闭合夹爪，false 表示打开夹爪
 * @param next_step 等待结束后的下一步
 * @return bool true 表示命令已成功下发
 */
static bool mission_issue_gripper(bool close_gripper, mission_arm_step_t next_step)
{
    int result = MECH_HAND_OK;

    if (!mechanical_arm_feedback.lock_bus(MISSION_ARM_BUS_LOCK_TIMEOUT_MS))
    {
        mission_enter_error(MECH_HAND_ERR_TIMEOUT);
        return false;
    }

    if (close_gripper)
    {
        result = mechanical_hand.gripper_close((uint16_t)MISSION_ARM_GRIPPER_TIME_MS);
    }
    else
    {
        result = mechanical_hand.gripper_open((uint16_t)MISSION_ARM_GRIPPER_TIME_MS);
    }
    mechanical_arm_feedback.request_refresh_all();
    mechanical_arm_feedback.unlock_bus();
    mission_refresh_arm_debug_state();

    mission_enter_critical();
    g_mission_last_gripper_result = result;
    if (MECH_HAND_OK == result)
    {
        g_mission_gripper_is_closed = close_gripper;
    }
    mission_exit_critical();

    if (MECH_HAND_OK != result)
    {
        mission_enter_error(result);
        return false;
    }

    mission_begin_action_wait(next_step, MISSION_ARM_GRIPPER_TIME_MS);
    return true;
}

/**
 * @brief  设置 9803 完成标志等待 ACK
 *
 * @return void
 */
static void mission_mark_done_pending(void)
{
    mission_enter_critical();
    g_mission_pending_done_ack = true;
    g_mission_last_done_send_ms = g_mission_now_ms - MISSION_DONE_RESEND_PERIOD_MS;
    mission_exit_critical();
}

/**
 * @brief  获取当前目标上方 Z 坐标
 *
 * @return float 目标上方 Z，单位 mm
 */
static float mission_target_approach_z(void)
{
    return g_mission_target_z_mm + MISSION_ARM_APPROACH_HEIGHT_MM;
}

/**
 * @brief  执行 WAIT_ACTION_DONE 结束后的下一步
 *
 * @return void
 */
static void mission_run_next_arm_step(void)
{
    mission_arm_step_t next_step = MISSION_ARM_STEP_NONE;
    mission_trace_mode_t resume_trace_mode = MISSION_TRACE_MODE_NO_AVOID;
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    float approach_z_mm = 0.0f;

    mission_enter_critical();
    next_step = g_mission_next_arm_step;
    resume_trace_mode = g_mission_resume_trace_mode;
    x_mm = g_mission_target_x_mm;
    y_mm = g_mission_target_y_mm;
    z_mm = g_mission_target_z_mm;
    approach_z_mm = mission_target_approach_z();
    g_mission_next_arm_step = MISSION_ARM_STEP_NONE;
    mission_exit_critical();

    switch (next_step)
    {
        case MISSION_ARM_STEP_PICK_OPEN:
            (void)mission_issue_gripper(false, MISSION_ARM_STEP_PICK_DESCEND);
            break;

        case MISSION_ARM_STEP_PICK_DESCEND:
            (void)mission_issue_move(x_mm, y_mm, z_mm, MISSION_ARM_STEP_PICK_CLOSE);
            break;

        case MISSION_ARM_STEP_PICK_CLOSE:
            (void)mission_issue_gripper(true, MISSION_ARM_STEP_PICK_LIFT);
            break;

        case MISSION_ARM_STEP_PICK_LIFT:
            (void)mission_issue_move(x_mm, y_mm, approach_z_mm, MISSION_ARM_STEP_PICK_HOME);
            break;

        case MISSION_ARM_STEP_PICK_HOME:
            (void)mission_issue_home(MISSION_ARM_STEP_PICK_DONE);
            break;

        case MISSION_ARM_STEP_PICK_DONE:
            mission_mark_done_pending();
            mission_enter_critical();
            g_mission_state = mission_trace_mode_to_state(resume_trace_mode);
            g_mission_trace_mode = resume_trace_mode;
            mission_exit_critical();
            break;

        case MISSION_ARM_STEP_PLACE_DESCEND:
            (void)mission_issue_move(x_mm, y_mm, z_mm, MISSION_ARM_STEP_PLACE_OPEN);
            break;

        case MISSION_ARM_STEP_PLACE_OPEN:
            (void)mission_issue_gripper(false, MISSION_ARM_STEP_PLACE_HOME);
            break;

        case MISSION_ARM_STEP_PLACE_HOME:
            (void)mission_issue_home(MISSION_ARM_STEP_PLACE_DONE);
            break;

        case MISSION_ARM_STEP_PLACE_DONE:
            mission_mark_done_pending();
            mission_enter_critical();
            g_mission_state = MISSION_STATE_PLACE_AFTER_HOLD;
            mission_exit_critical();
            break;

        case MISSION_ARM_STEP_NONE:
        default:
            break;
    }
}

/**
 * @brief  处理机械臂等待时间到期
 *
 * @return void
 */
static void mission_process_action_wait(void)
{
    uint32_t now_ms = 0U;
    uint32_t start_ms = 0U;
    uint32_t wait_ms = 0U;
    mission_state_t state = MISSION_STATE_IDLE;

    mission_enter_critical();
    now_ms = g_mission_now_ms;
    start_ms = g_mission_action_start_ms;
    wait_ms = g_mission_action_wait_ms;
    state = g_mission_state;
    mission_exit_critical();

    if (MISSION_STATE_WAIT_ACTION_DONE != state)
    {
        return;
    }

    if ((uint32_t)(now_ms - start_ms) < wait_ms)
    {
        return;
    }

    mission_run_next_arm_step();
}

/**
 * @brief  根据位置帧启动抓取或放置流程
 *
 * @param place_mode true 表示放置流程，false 表示抓取流程
 * @param p_frame 位置帧
 * @return void
 */
static void mission_start_position_action(bool place_mode, const vision_uart8_frame_t *p_frame)
{
    int calib_result = CALIB_OK;
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    float approach_z_mm = 0.0f;

    if (NULL == p_frame)
    {
        mission_enter_error(FSP_ERR_INVALID_POINTER);
        return;
    }

    calib_result = calibration.image_to_base(p_frame->data1,
                                             p_frame->data2,
                                             p_frame->data3,
                                             &x_mm,
                                             &y_mm,
                                             &z_mm);
    if (CALIB_OK != calib_result)
    {
        mission_enter_error(calib_result);
        return;
    }

    if (place_mode)
    {
        x_mm += MISSION_PLACE_OFFSET_X_MM;
        y_mm += MISSION_PLACE_OFFSET_Y_MM;
        z_mm += MISSION_PLACE_OFFSET_Z_MM;
    }

    mission_enter_critical();
    g_mission_target_u = p_frame->data1;
    g_mission_target_v = p_frame->data2;
    g_mission_target_depth = p_frame->data3;
    g_mission_target_x_mm = x_mm;
    g_mission_target_y_mm = y_mm;
    g_mission_target_z_mm = z_mm;
    g_mission_target_valid = true;
    approach_z_mm = mission_target_approach_z();
    mission_exit_critical();

    if (place_mode)
    {
        (void)mission_issue_move(x_mm, y_mm, approach_z_mm, MISSION_ARM_STEP_PLACE_DESCEND);
    }
    else
    {
        (void)mission_issue_move(x_mm, y_mm, approach_z_mm, MISSION_ARM_STEP_PICK_OPEN);
    }
}

/**
 * @brief  处理进入等待抓取或放置坐标
 *
 * @param state 等待状态
 * @return void
 */
static void mission_enter_wait_position(mission_state_t state)
{
    mission_enter_critical();
    mission_clear_target_unlocked();
    g_mission_wait_start_rx_ok_count = g_mission_last_rx_ok_count;
    g_mission_state = state;
    g_mission_pending_request_position = true;
    mission_exit_critical();
}

/**
 * @brief  处理上位机 99xx 命令帧
 *
 * @param p_frame 上位机命令帧
 * @return void
 */
static void mission_handle_pc_frame(const vision_uart8_frame_t *p_frame)
{
    uint16_t code = 0U;
    mission_state_t state = MISSION_STATE_IDLE;
    bool enabled = false;
    bool position_frame_is_new = false;

    if (NULL == p_frame)
    {
        return;
    }

    code = mission_frame_code(p_frame->data4);

    mission_enter_critical();
    enabled = g_mission_enabled;
    state = g_mission_state;
    mission_exit_critical();

    if (!enabled)
    {
        return;
    }

    switch (code)
    {
        case MISSION_CODE_PC_SPEED:
            mission_enter_critical();
            g_mission_vx_mps = p_frame->data1;
            g_mission_vy_mps = p_frame->data2;
            g_mission_wz_radps = p_frame->data3;
            g_mission_last_speed_rx_ms = g_mission_now_ms;
            mission_exit_critical();
            break;

        case MISSION_CODE_PC_TRACE_NO_AVOID:
            mission_enter_critical();
            if (MISSION_STATE_WAIT_ACTION_DONE != g_mission_state)
            {
                g_mission_trace_mode = MISSION_TRACE_MODE_NO_AVOID;
                g_mission_state = MISSION_STATE_TRACE_NO_AVOID;
            }
            mission_exit_critical();
            break;

        case MISSION_CODE_PC_TRACE_AVOID:
            mission_enter_critical();
            if (MISSION_STATE_WAIT_ACTION_DONE != g_mission_state)
            {
                g_mission_trace_mode = MISSION_TRACE_MODE_AVOID;
                g_mission_state = MISSION_STATE_TRACE_AVOID;
            }
            mission_exit_critical();
            break;

        case MISSION_CODE_PC_WAIT_PICK:
            mission_enter_critical();
            g_mission_resume_trace_mode = g_mission_trace_mode;
            mission_exit_critical();
            mission_enter_wait_position(MISSION_STATE_WAIT_PICK_POS);
            break;

        case MISSION_CODE_PC_WAIT_PLACE:
            mission_enter_wait_position(MISSION_STATE_WAIT_PLACE_POS);
            break;

        case MISSION_CODE_PC_POSITION_VALID:
            mission_enter_critical();
            state = g_mission_state;
            position_frame_is_new = (p_frame->rx_ok_count > g_mission_wait_start_rx_ok_count);
            mission_exit_critical();
            if (position_frame_is_new && (MISSION_STATE_WAIT_PICK_POS == state))
            {
                mission_start_position_action(false, p_frame);
            }
            else if (position_frame_is_new && (MISSION_STATE_WAIT_PLACE_POS == state))
            {
                mission_start_position_action(true, p_frame);
            }
            break;

        case MISSION_CODE_PC_STOP_NOW:
            mission_enter_critical();
            g_mission_vx_mps = 0.0f;
            g_mission_vy_mps = 0.0f;
            g_mission_wz_radps = 0.0f;
            g_mission_last_speed_rx_ms = g_mission_now_ms;
            mission_exit_critical();
            break;

        case MISSION_CODE_PC_DONE_ACK:
            mission_enter_critical();
            g_mission_pending_done_ack = false;
            mission_exit_critical();
            break;

        default:
            break;
    }
}

/**
 * @brief  处理 UART8 最新视觉帧
 *
 * @return void
 */
static void mission_process_vision_frame(void)
{
    vision_uart8_frame_t frame = {0};
    fsp_err_t err = FSP_SUCCESS;

    if (!vision_uart8.has_new())
    {
        return;
    }

    err = vision_uart8.latest_read(&frame);
    vision_uart8.clear_new();
    if (FSP_SUCCESS != err)
    {
        mission_enter_error((int)err);
        return;
    }

    mission_enter_critical();
    g_mission_latest_frame = frame;
    if (frame.rx_ok_count == g_mission_last_rx_ok_count)
    {
        mission_exit_critical();
        return;
    }
    g_mission_last_rx_ok_count = frame.rx_ok_count;
    mission_exit_critical();

    mission_handle_pc_frame(&frame);
}

/**
 * @brief  处理避障完成检测
 *
 * @return void
 */
static void mission_process_avoidance_status(void)
{
    chassis_avoidance_status_t status = {0};
    fsp_err_t err = FSP_SUCCESS;
    mission_state_t state = MISSION_STATE_IDLE;
    bool avoid_done = false;

    err = chassis_avoidance.status_get(&status);
    if (FSP_SUCCESS != err)
    {
        return;
    }

    mission_enter_critical();
    state = g_mission_state;
    g_mission_avoidance_state = status.state;
    if ((MISSION_STATE_TRACE_AVOID == state) &&
        (CHASSIS_AVOIDANCE_STATE_NORMAL == status.state) &&
        ((CHASSIS_AVOIDANCE_STATE_AVOID == g_mission_last_avoidance_state) ||
         (CHASSIS_AVOIDANCE_STATE_RETURN_TRACK == g_mission_last_avoidance_state)))
    {
        avoid_done = true;
        g_mission_pending_avoid_done = true;
    }
    g_mission_last_avoidance_state = status.state;
    mission_exit_critical();

    (void)avoid_done;
}

/**
 * @brief  处理待发送的 98xx 状态帧
 *
 * @return void
 */
static void mission_process_pending_tx(void)
{
    bool pending_error = false;
    bool pending_enter = false;
    bool pending_request_position = false;
    bool pending_avoid_done = false;
    bool pending_monitor_entered = false;
    bool pending_done_ack = false;
    int last_error = MECH_HAND_OK;
    uint32_t now_ms = 0U;
    uint32_t last_done_send_ms = 0U;

    mission_enter_critical();
    pending_error = g_mission_pending_error;
    pending_enter = g_mission_pending_enter_status;
    pending_request_position = g_mission_pending_request_position;
    pending_avoid_done = g_mission_pending_avoid_done;
    pending_monitor_entered = g_mission_pending_monitor_entered;
    pending_done_ack = g_mission_pending_done_ack;
    last_error = g_mission_last_error;
    now_ms = g_mission_now_ms;
    last_done_send_ms = g_mission_last_done_send_ms;
    mission_exit_critical();

    if (pending_error)
    {
        if (mission_send_status(MISSION_CODE_MCU_ERROR, (float)last_error))
        {
            mission_enter_critical();
            g_mission_pending_error = false;
            mission_exit_critical();
        }
        return;
    }

    if (pending_monitor_entered)
    {
        if (mission_send_status(MISSION_CODE_MCU_MONITOR_ENTERED, 0.0f))
        {
            mission_enter_critical();
            g_mission_pending_monitor_entered = false;
            mission_exit_critical();
        }
        return;
    }

    if (pending_enter)
    {
        if (mission_send_status(MISSION_CODE_MCU_ENTERED, 0.0f))
        {
            mission_enter_critical();
            g_mission_pending_enter_status = false;
            mission_exit_critical();
        }
        return;
    }

    if (pending_request_position)
    {
        if (mission_send_status(MISSION_CODE_MCU_REQUEST_POSITION, 0.0f))
        {
            mission_enter_critical();
            g_mission_pending_request_position = false;
            mission_exit_critical();
        }
        return;
    }

    if (pending_avoid_done)
    {
        if (mission_send_status(MISSION_CODE_MCU_AVOID_DONE, 0.0f))
        {
            mission_enter_critical();
            g_mission_pending_avoid_done = false;
            mission_exit_critical();
        }
        return;
    }

    if (pending_done_ack &&
        ((uint32_t)(now_ms - last_done_send_ms) >= MISSION_DONE_RESEND_PERIOD_MS))
    {
        if (mission_send_status(MISSION_CODE_MCU_ACTION_DONE, 0.0f))
        {
            mission_enter_critical();
            g_mission_last_done_send_ms = g_mission_now_ms;
            mission_exit_critical();
        }
    }
}

/**
 * @brief  消费启动请求并丢弃启动前旧帧
 *
 * @return void
 */
static void mission_process_start_pending(void)
{
    vision_uart8_frame_t frame = {0};
    fsp_err_t err = FSP_SUCCESS;
    bool pending_enter = false;
    bool start_frame_cleared = false;

    mission_enter_critical();
    pending_enter = g_mission_pending_enter_status;
    start_frame_cleared = g_mission_start_frame_cleared;
    mission_exit_critical();

    if ((!pending_enter) || start_frame_cleared)
    {
        return;
    }

    err = vision_uart8.latest_read(&frame);
    if (FSP_SUCCESS == err)
    {
        mission_enter_critical();
        g_mission_latest_frame = frame;
        g_mission_last_rx_ok_count = frame.rx_ok_count;
        mission_exit_critical();
    }
    vision_uart8.clear_new();

    mission_enter_critical();
    g_mission_start_frame_cleared = true;
    mission_exit_critical();
}

/**
 * @brief  初始化 mission 模块
 *
 * @return fsp_err_t FSP_SUCCESS 表示初始化完成
 */
static fsp_err_t mission_init_impl(void)
{
    uint8_t hand_ok = 0U;

    hand_ok = mechanical_hand.init();
    mechanical_hand.get_servo_positions(&g_mission_servo_positions);

    mission_enter_critical();
    g_mission_initialized = true;
    g_mission_enabled = false;
    g_mission_state = MISSION_STATE_IDLE;
    g_mission_trace_mode = MISSION_TRACE_MODE_NO_AVOID;
    g_mission_resume_trace_mode = MISSION_TRACE_MODE_NO_AVOID;
    g_mission_last_error = (0U != hand_ok) ? MECH_HAND_OK : mechanical_hand.get_last_error();
    g_mission_was_last_command_clamped = mechanical_hand.was_last_command_clamped();
    mission_clear_target_unlocked();
    mission_clear_pick_status_unlocked();
    mission_exit_critical();

    return (0U != hand_ok) ? FSP_SUCCESS : FSP_ERR_NOT_OPEN;
}

/**
 * @brief  请求启动 mission
 *
 * @return fsp_err_t FSP_SUCCESS 表示启动请求已记录
 */
static fsp_err_t mission_request_start_impl(void)
{
    mission_enter_critical();
    if (!g_mission_initialized)
    {
        mission_exit_critical();
        return FSP_ERR_NOT_OPEN;
    }

    g_mission_enabled = true;
    g_mission_state = MISSION_STATE_IDLE;
    g_mission_trace_mode = MISSION_TRACE_MODE_NO_AVOID;
    g_mission_resume_trace_mode = MISSION_TRACE_MODE_NO_AVOID;
    g_mission_vx_mps = 0.0f;
    g_mission_vy_mps = 0.0f;
    g_mission_wz_radps = 0.0f;
    g_mission_last_speed_rx_ms = g_mission_now_ms;
    g_mission_pending_enter_status = true;
    g_mission_start_frame_cleared = false;
    g_mission_pending_request_position = false;
    g_mission_pending_avoid_done = false;
    g_mission_pending_error = false;
    g_mission_pending_done_ack = false;
    g_mission_next_arm_step = MISSION_ARM_STEP_NONE;
    mission_clear_target_unlocked();
    mission_clear_pick_status_unlocked();
    mission_exit_critical();

    return FSP_SUCCESS;
}

/**
 * @brief  请求停止 mission
 *
 * @return void
 */
static void mission_request_stop_impl(void)
{
    mission_enter_critical();
    g_mission_enabled = false;
    g_mission_state = MISSION_STATE_IDLE;
    g_mission_vx_mps = 0.0f;
    g_mission_vy_mps = 0.0f;
    g_mission_wz_radps = 0.0f;
    g_mission_pending_enter_status = false;
    g_mission_start_frame_cleared = false;
    g_mission_pending_request_position = false;
    g_mission_pending_avoid_done = false;
    g_mission_pending_error = false;
    g_mission_pending_done_ack = false;
    g_mission_next_arm_step = MISSION_ARM_STEP_NONE;
    mission_clear_target_unlocked();
    mission_clear_pick_status_unlocked();
    mission_exit_critical();
}

/**
 * @brief  请求发送 mission 监视界面进入标志
 *
 * @return void
 */
static void mission_request_monitor_entered_impl(void)
{
    mission_enter_critical();
    g_mission_pending_monitor_entered = true;
    mission_exit_critical();
}

/**
 * @brief  将语音分数转换为千分比分数
 *
 * @param score 原始模型分数
 * @return uint16_t 0~1000 的分数
 */
static uint16_t mission_voice_score_to_milli(float score)
{
    if (score <= 0.0f)
    {
        return 0U;
    }

    if (score >= 1.0f)
    {
        return (uint16_t)MISSION_VOICE_SCORE_MILLI;
    }

    return (uint16_t)((score * MISSION_VOICE_SCORE_MILLI) + 0.5f);
}

/**
 * @brief  判断语音唤醒窗口是否仍有效
 *
 * @return bool true 表示唤醒窗口有效
 */
static bool mission_voice_wake_is_valid(void)
{
    bool active = false;

    mission_enter_critical();
    active = g_mission_voice_wake_active;
    if (active &&
        ((uint32_t)(g_mission_now_ms - g_mission_voice_wake_tick_ms) > MISSION_VOICE_WAKE_WINDOW_MS))
    {
        g_mission_voice_wake_active = false;
        active = false;
    }
    mission_exit_critical();

    return active;
}

/**
 * @brief  标记语音唤醒窗口开始
 *
 * @return void
 */
static void mission_voice_mark_wake_active(void)
{
    mission_enter_critical();
    g_mission_voice_wake_active = true;
    g_mission_voice_wake_tick_ms = g_mission_now_ms;
    mission_exit_critical();
}

/**
 * @brief  将抓取标签转换为视觉目标选择码
 *
 * @param label 语音标签
 * @return uint16_t 0 表示该标签不是抓取目标
 */
static uint16_t mission_voice_target_code(voice_ai_label_t label)
{
    if ((label < VOICE_AI_LABEL_SELECT_CUBE) || (label > VOICE_AI_LABEL_SELECT_TNT))
    {
        return 0U;
    }

    return (uint16_t)(MISSION_CODE_MCU_SELECT_BASE +
                      ((uint16_t)label - (uint16_t)VOICE_AI_LABEL_SELECT_CUBE));
}

/**
 * @brief  获取语音抓取目标的显示分数
 *
 * @param p_result 语音 AI 结果
 * @param label    语音标签
 * @return uint8_t 0~100 的百分比分数
 */
static uint8_t mission_voice_pick_score_percent(const voice_ai_result_t *p_result, voice_ai_label_t label)
{
    uint16_t best_score_milli = 0U;
    uint16_t score_milli = 0U;
    uint32_t candidate_count = 0U;

    if ((NULL == p_result) || ((uint8_t)label >= VOICE_AI_LABEL_COUNT))
    {
        return 0U;
    }

    candidate_count = p_result->candidate_count;
    if (candidate_count > VOICE_AI_MAX_CANDIDATES)
    {
        candidate_count = VOICE_AI_MAX_CANDIDATES;
    }

    for (uint32_t i = 0U; i < candidate_count; i++)
    {
        if (p_result->candidate_top_indices[i] == (uint8_t)label)
        {
            score_milli = mission_voice_score_to_milli(p_result->candidate_top_scores[i]);
            if (score_milli > best_score_milli)
            {
                best_score_milli = score_milli;
            }
        }
    }

    if (0U == best_score_milli)
    {
        best_score_milli = mission_voice_score_to_milli(p_result->scores[(uint8_t)label]);
    }

    return (uint8_t)((best_score_milli + 5U) / 10U);
}

/**
 * @brief  记录最近一次语音抓取目标发送结果
 *
 * @param label         语音标签
 * @param code          发送给视觉端的目标选择码
 * @param send_ok       true 表示发送成功
 * @param score_percent 0~100 的百分比分数
 * @return void
 */
static void mission_record_pick_send_status(voice_ai_label_t label,
                                            uint16_t code,
                                            bool send_ok,
                                            uint8_t score_percent)
{
    const char *p_label_name = voice_ai.label_name((uint8_t)label);

    if (NULL == p_label_name)
    {
        p_label_name = "unknown";
    }

    mission_enter_critical();
    g_mission_pick_send_valid = true;
    g_mission_pick_send_ok = send_ok;
    g_mission_pick_target_code = code;
    g_mission_pick_score_percent = score_percent;
    (void)strncpy(g_mission_pick_label, p_label_name, MISSION_PICK_LABEL_LEN - 1U);
    g_mission_pick_label[MISSION_PICK_LABEL_LEN - 1U] = '\0';
    mission_exit_critical();
}

/**
 * @brief  发送视觉目标选择码
 *
 * @param code 981x 视觉目标选择码
 * @return bool true 表示发送成功
 */
static bool mission_voice_send_target_select(uint16_t code)
{
    float values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    fsp_err_t err = FSP_SUCCESS;

    values[3] = (float)code;
    err = vision_uart8.send4(values, 4U);
    if (FSP_SUCCESS != err)
    {
        mission_enter_critical();
        g_mission_last_error = (int)err;
        mission_exit_critical();
        return false;
    }

    return true;
}

/**
 * @brief  从 5 候选结果中查找满足 4/5 和阈值规则的标签
 *
 * @param p_result 语音 AI 结果
 * @param p_label 接受标签输出
 * @param p_matches 匹配候选数量输出
 * @return bool true 表示找到可接受标签
 */
static bool mission_voice_find_accepted_label(const voice_ai_result_t *p_result,
                                              voice_ai_label_t *p_label,
                                              uint32_t *p_matches)
{
    uint32_t match_counts[VOICE_AI_LABEL_COUNT] = {0U};
    uint32_t candidate_count = 0U;

    if ((NULL == p_result) || (NULL == p_label) || (NULL == p_matches))
    {
        return false;
    }

    candidate_count = p_result->candidate_count;
    if (candidate_count > VOICE_AI_MAX_CANDIDATES)
    {
        candidate_count = VOICE_AI_MAX_CANDIDATES;
    }

    if (candidate_count < MISSION_VOICE_ACCEPT_MATCH_COUNT)
    {
        return false;
    }

    for (uint32_t i = 0U; i < candidate_count; i++)
    {
        uint8_t index = p_result->candidate_top_indices[i];
        uint16_t score_milli = mission_voice_score_to_milli(p_result->candidate_top_scores[i]);

        if (index >= VOICE_AI_LABEL_COUNT)
        {
            continue;
        }

        if (score_milli >= g_mission_voice_score_thresholds[index])
        {
            match_counts[index]++;
        }
    }

    for (uint32_t i = 0U; i < VOICE_AI_LABEL_COUNT; i++)
    {
        if (match_counts[i] >= MISSION_VOICE_ACCEPT_MATCH_COUNT)
        {
            *p_label = (voice_ai_label_t)i;
            *p_matches = match_counts[i];
            return true;
        }
    }

    return false;
}

/**
 * @brief  打印 Mission 语音接受日志
 *
 * @param label 接受标签
 * @param matches 匹配候选数量
 * @return void
 */
static void mission_voice_print_accept(voice_ai_label_t label, uint32_t matches)
{
    const char *p_label_name = voice_ai.label_name((uint8_t)label);

    if (NULL == p_label_name)
    {
        p_label_name = "unknown";
    }

    printf("mission:voice_ai:accept label=%s idx=%u matches=%lu\r\n",
           p_label_name,
           (unsigned int)label,
           (unsigned long)matches);
}

/**
 * @brief  消费语音 AI 识别结果并转换为 Mission 动作
 *
 * @param p_result 语音 AI 识别结果
 * @param mission_view_active true 表示 UI 当前位于 Mission 页面
 * @param p_actions UI 后续动作输出
 * @return void
 */
static void mission_voice_ai_consume_impl(const voice_ai_result_t *p_result,
                                          bool mission_view_active,
                                          mission_voice_ai_actions_t *p_actions)
{
    voice_ai_label_t label = VOICE_AI_LABEL_UNKNOWN;
    uint32_t matches = 0U;
    uint16_t target_code = 0U;
    bool send_ok = false;

    if (NULL == p_actions)
    {
        return;
    }

    p_actions->enter_mission_view = false;
    p_actions->stop_output = false;

    if (!mission_voice_find_accepted_label(p_result, &label, &matches))
    {
        return;
    }

    mission_voice_print_accept(label, matches);

    if (VOICE_AI_LABEL_WAKE == label)
    {
        mission_voice_mark_wake_active();
        p_actions->enter_mission_view = true;
        return;
    }

    if (!mission_view_active || !mission_voice_wake_is_valid())
    {
        return;
    }

    if (VOICE_AI_LABEL_START == label)
    {
        (void)mission_request_start_impl();
        return;
    }

    if (VOICE_AI_LABEL_STOP == label)
    {
        mission_request_stop_impl();
        p_actions->stop_output = true;
        return;
    }

    target_code = mission_voice_target_code(label);
    if (0U != target_code)
    {
        send_ok = mission_voice_send_target_select(target_code);

        mission_record_pick_send_status(label,
                                        target_code,
                                        send_ok,
                                        mission_voice_pick_score_percent(p_result, label));
    }
}

/**
 * @brief  推进 mission 状态机
 *
 * @param interval_ms 距离上次调用经过的时间，单位 ms
 * @return void
 */
static void mission_process_impl(uint32_t interval_ms)
{
    if (0U == interval_ms)
    {
        return;
    }

    mission_enter_critical();
    g_mission_now_ms += interval_ms;
    mission_exit_critical();

    mission_process_start_pending();
    mission_process_vision_frame();
    mission_process_action_wait();
    mission_process_avoidance_status();
    mission_process_pending_tx();
}

/**
 * @brief  读取 mission 快照
 *
 * @param p_snapshot 快照输出指针
 * @return void
 */
static void mission_snapshot_read_impl(mission_snapshot_t *p_snapshot)
{
    uint32_t now_ms = 0U;
    uint32_t last_speed_rx_ms = 0U;

    if (NULL == p_snapshot)
    {
        return;
    }

    mission_enter_critical();
    now_ms = g_mission_now_ms;
    last_speed_rx_ms = g_mission_last_speed_rx_ms;
    p_snapshot->enabled = g_mission_enabled;
    p_snapshot->state = g_mission_state;
    p_snapshot->trace_mode = g_mission_trace_mode;
    p_snapshot->vx_mps = g_mission_vx_mps;
    p_snapshot->vy_mps = g_mission_vy_mps;
    p_snapshot->wz_radps = g_mission_wz_radps;
    p_snapshot->speed_frame_age_ms = (uint32_t)(now_ms - last_speed_rx_ms);
    p_snapshot->latest_frame = g_mission_latest_frame;
    p_snapshot->target_valid = g_mission_target_valid;
    p_snapshot->pick_send_valid = g_mission_pick_send_valid;
    p_snapshot->pick_send_ok = g_mission_pick_send_ok;
    p_snapshot->pick_target_code = g_mission_pick_target_code;
    p_snapshot->pick_score_percent = g_mission_pick_score_percent;
    (void)strncpy(p_snapshot->pick_label, g_mission_pick_label, MISSION_PICK_LABEL_LEN - 1U);
    p_snapshot->pick_label[MISSION_PICK_LABEL_LEN - 1U] = '\0';
    p_snapshot->target_u = g_mission_target_u;
    p_snapshot->target_v = g_mission_target_v;
    p_snapshot->target_depth = g_mission_target_depth;
    p_snapshot->target_x_mm = g_mission_target_x_mm;
    p_snapshot->target_y_mm = g_mission_target_y_mm;
    p_snapshot->target_z_mm = g_mission_target_z_mm;
    p_snapshot->pending_done_ack = g_mission_pending_done_ack;
    p_snapshot->last_error = g_mission_last_error;
    p_snapshot->avoidance_state = g_mission_avoidance_state;
    p_snapshot->gripper_is_closed = g_mission_gripper_is_closed;
    p_snapshot->last_gripper_result = g_mission_last_gripper_result;
    p_snapshot->last_arm_result = g_mission_last_arm_result;
    p_snapshot->servo_positions = g_mission_servo_positions;
    p_snapshot->was_last_command_clamped = g_mission_was_last_command_clamped;
    mission_exit_critical();
}

/**
 * @brief  读取 mission 底盘控制源
 *
 * @param p_control 底盘控制源输出指针
 * @return bool true 表示 mission 当前接管底盘控制
 */
static bool mission_chassis_control_read_impl(mission_chassis_control_t *p_control)
{
    uint32_t speed_age_ms = 0U;
    mission_state_t state = MISSION_STATE_IDLE;
    mission_trace_mode_t trace_mode = MISSION_TRACE_MODE_NO_AVOID;
    bool enabled = false;

    if (NULL == p_control)
    {
        return false;
    }

    mission_enter_critical();
    enabled = g_mission_enabled;
    state = g_mission_state;
    trace_mode = g_mission_trace_mode;
    speed_age_ms = (uint32_t)(g_mission_now_ms - g_mission_last_speed_rx_ms);
    p_control->enabled = enabled;
    p_control->use_avoidance = (MISSION_TRACE_MODE_AVOID == trace_mode);
    p_control->vx_mps = 0.0f;
    p_control->vy_mps = 0.0f;
    p_control->wz_radps = 0.0f;

    if (enabled &&
        ((MISSION_STATE_TRACE_NO_AVOID == state) || (MISSION_STATE_TRACE_AVOID == state)) &&
        (speed_age_ms <= MISSION_SPEED_TIMEOUT_MS))
    {
        p_control->vx_mps = g_mission_vx_mps;
        p_control->vy_mps = g_mission_vy_mps;
        p_control->wz_radps = g_mission_wz_radps;
    }
    mission_exit_critical();

    return enabled;
}

const mission_api_t mission =
{
    .init = mission_init_impl,
    .request_start = mission_request_start_impl,
    .request_stop = mission_request_stop_impl,
    .request_monitor_entered = mission_request_monitor_entered_impl,
    .process = mission_process_impl,
    .snapshot_read = mission_snapshot_read_impl,
    .chassis_control_read = mission_chassis_control_read_impl,
    .voice_ai_consume = mission_voice_ai_consume_impl,
};
