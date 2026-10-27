#include "hardware/mechanical_arm/Mechanical_hand_control.h"

#include "hardware/mechanical_arm/Calibration.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

/* 连杆参数沿用旧工程模型，内部坐标会先放大 10 倍。 */
#define MECH_HAND_L0 (1000.0f)
#define MECH_HAND_L1 (1050.0f)
#define MECH_HAND_L2 (880.0f)
#define MECH_HAND_L3 (1550.0f)

#define MECH_HAND_SERVO_BASE            (0U)
#define MECH_HAND_SERVO_BIG_ARM         (1U)
#define MECH_HAND_SERVO_FOREARM         (2U)
#define MECH_HAND_SERVO_WRIST           (3U)
#define MECH_HAND_SERVO_GRIPPER_ROTATE  (4U)
#define MECH_HAND_SERVO_GRIPPER_CLAMP   (5U)

#define MECH_HAND_PWM_MIN   BUS_SERVO_PWM_MIN
#define MECH_HAND_PWM_MAX   BUS_SERVO_PWM_MAX
#define MECH_HAND_TIME_MAX  BUS_SERVO_TIME_MAX

#define MECH_HAND_GRIPPER_OPEN_PWM             (1500U)
#define MECH_HAND_GRIPPER_CLOSE_PWM            (1900U)
#define MECH_HAND_GRIPPER_ROTATE_CENTER_PWM    (1500U)

#define MECH_HAND_FLOAT_EPSILON (0.0001f)
#define MECH_HAND_PI            (3.1415926f)

static volatile bool g_mech_hand_initialized = false;
static int g_mech_hand_last_error = MECH_HAND_OK;
static bool g_mech_hand_last_cmd_clamped = false;
static mech_hand_servo_positions_t g_mech_hand_positions = {{1500U, 1500U, 1500U, 1500U, 1500U, 1500U}};

static int mechanical_hand_check_ready(void);
static int mechanical_hand_check_servo_index(uint8_t index);
static int mechanical_hand_check_pwm(uint16_t pwm);
static int mechanical_hand_check_time(uint16_t time_ms);
static uint16_t mechanical_hand_get_servo_min(uint8_t index);
static uint16_t mechanical_hand_get_servo_max(uint8_t index);
static uint16_t mechanical_hand_clamp_servo_pwm(uint8_t index, uint16_t pwm, bool *clamped);
static void mechanical_hand_update_servo_position(uint8_t index, uint16_t pwm);
static void mechanical_hand_update_joint_positions(const mech_hand_joint_pwm_t *pwm);
static void mechanical_hand_reset_clamp_state(void);
static int mechanical_hand_send_joint_command(const mech_hand_joint_pwm_t *pwm, uint16_t time_ms);
static int mechanical_hand_try_parse_servo_command(const char *cmd, uint8_t *index, uint16_t *pwm, uint16_t *time_ms);
static int mechanical_hand_try_parse_joint_command(const char *cmd, mech_hand_joint_pwm_t *pwm, uint16_t *time_ms);
static int mechanical_hand_kinematics_analysis(float x, float y, float z, float alpha_deg, mech_hand_joint_pwm_t *out_pwm);
static int mechanical_hand_kinematics_move(float x, float y, float z, uint16_t time_ms);
static float mechanical_hand_calc_yaw_deg(float x, float y);
static int mechanical_hand_tcp_to_reference(float tcp_x, float tcp_y, float tcp_z, float alpha_deg,
                                         float *ref_x, float *ref_y, float *ref_z);
static int mechanical_hand_kinematics_move_tcp(float x, float y, float z, uint16_t time_ms);

/**
 * @brief  初始化机械臂动作层；这里只打开 UART4，不自动回 Home
 */
static uint8_t mechanical_hand_init(void)
{
    if (MECH_HAND_OK != bus_servo.init())
    {
        g_mech_hand_last_error = bus_servo.get_last_error();
        return 0U;
    }

    mechanical_hand_reset_clamp_state();
    g_mech_hand_initialized = true;
    g_mech_hand_last_error = MECH_HAND_OK;
    return 1U;
}

static void mechanical_hand_deinit(void)
{
    bus_servo.deinit();
    g_mech_hand_initialized = false;
    g_mech_hand_last_error = bus_servo.get_last_error();
}

/**
 * @brief  按旧机械模型参考点移动，保持旧 MoveTo 语义，不叠加 TCP 工具偏移
 */
static int mechanical_hand_move_to(float x, float y, float z, uint16_t time_ms)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = mechanical_hand_kinematics_move(x, y, z, time_ms);
    return g_mech_hand_last_error;
}

/**
 * @brief  按旧机械模型参考点移动，并使用固定 alpha 求解
 */
static int mechanical_hand_move_to_with_alpha(float x, float y, float z, float alpha_deg, uint16_t time_ms)
{
    mech_hand_joint_pwm_t pwm;
    int status;

    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    status = mechanical_hand_kinematics_analysis(x, y, z, alpha_deg, &pwm);
    if (MECH_HAND_OK != status)
    {
        g_mech_hand_last_error = status;
        return status;
    }

    status = mechanical_hand_send_joint_command(&pwm, time_ms);
    g_mech_hand_last_error = status;
    return status;
}

/**
 * @brief  按夹爪中心 TCP 目标点移动，内部会同步扫描 alpha 和工具偏移
 */
static int mechanical_hand_move_tcp_to(float x, float y, float z, uint16_t time_ms)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = mechanical_hand_kinematics_move_tcp(x, y, z, time_ms);
    return g_mech_hand_last_error;
}

/**
 * @brief  按夹爪中心 TCP 目标点移动，并使用固定 alpha 换算工具偏移
 */
static int mechanical_hand_move_tcp_to_with_alpha(float x, float y, float z, float alpha_deg, uint16_t time_ms)
{
    float ref_x;
    float ref_y;
    float ref_z;
    mech_hand_joint_pwm_t pwm;
    int status;

    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    status = mechanical_hand_tcp_to_reference(x, y, z, alpha_deg, &ref_x, &ref_y, &ref_z);
    if (MECH_HAND_OK != status)
    {
        g_mech_hand_last_error = status;
        return status;
    }

    status = mechanical_hand_kinematics_analysis(ref_x, ref_y, ref_z, alpha_deg, &pwm);
    if (MECH_HAND_OK != status)
    {
        g_mech_hand_last_error = status;
        return status;
    }

    status = mechanical_hand_send_joint_command(&pwm, time_ms);
    g_mech_hand_last_error = status;
    return status;
}

static int mechanical_hand_send_single_servo(uint8_t index, uint16_t pwm, uint16_t time_ms)
{
    bool clamped = false;
    int status;

    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if ((MECH_HAND_OK != mechanical_hand_check_servo_index(index)) ||
        (MECH_HAND_OK != mechanical_hand_check_time(time_ms)))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    mechanical_hand_reset_clamp_state();
    pwm = mechanical_hand_clamp_servo_pwm(index, pwm, &clamped);
    g_mech_hand_last_cmd_clamped = clamped;

    status = bus_servo.move(index, pwm, time_ms);
    if (MECH_HAND_OK == status)
    {
        mechanical_hand_update_servo_position(index, pwm);
    }

    g_mech_hand_last_error = status;
    return status;
}

static int mechanical_hand_send_multi_servo(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t time_ms)
{
    mech_hand_joint_pwm_t pwm;

    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    pwm.servo0_pwm = pwm0;
    pwm.servo1_pwm = pwm1;
    pwm.servo2_pwm = pwm2;
    pwm.servo3_pwm = pwm3;

    g_mech_hand_last_error = mechanical_hand_send_joint_command(&pwm, time_ms);
    return g_mech_hand_last_error;
}

static int mechanical_hand_send_raw_debug(const char *cmd)
{
    uint8_t index;
    uint16_t pwm;
    uint16_t time_ms;
    mech_hand_joint_pwm_t joints;

    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if ((0 == cmd) || ('\0' == cmd[0]))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    /* 能识别为结构化舵机命令时，优先走本层限位和位置缓存。 */
    if (MECH_HAND_OK == mechanical_hand_try_parse_servo_command(cmd, &index, &pwm, &time_ms))
    {
        g_mech_hand_last_error = mechanical_hand_send_single_servo(index, pwm, time_ms);
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK == mechanical_hand_try_parse_joint_command(cmd, &joints, &time_ms))
    {
        g_mech_hand_last_error = mechanical_hand_send_multi_servo(joints.servo0_pwm,
                                                               joints.servo1_pwm,
                                                               joints.servo2_pwm,
                                                               joints.servo3_pwm,
                                                               time_ms);
        return g_mech_hand_last_error;
    }

    /* 不能识别的原始命令直接透传到底层总线，调试时必须确认命令安全。 */
    mechanical_hand_reset_clamp_state();
    g_mech_hand_last_error = bus_servo.send_raw(cmd, NULL, 0U, 0U);
    return g_mech_hand_last_error;
}

static int mechanical_hand_gripper_rotate(uint16_t pwm, uint16_t time_ms)
{
    return mechanical_hand_send_single_servo(MECH_HAND_SERVO_GRIPPER_ROTATE, pwm, time_ms);
}

static int mechanical_hand_gripper_rotate_center(uint16_t time_ms)
{
    return mechanical_hand_gripper_rotate(MECH_HAND_GRIPPER_ROTATE_CENTER_PWM, time_ms);
}

static int mechanical_hand_gripper_open(uint16_t time_ms)
{
    return mechanical_hand_send_single_servo(MECH_HAND_SERVO_GRIPPER_CLAMP, MECH_HAND_GRIPPER_OPEN_PWM, time_ms);
}

static int mechanical_hand_gripper_close(uint16_t time_ms)
{
    return mechanical_hand_send_single_servo(MECH_HAND_SERVO_GRIPPER_CLAMP, MECH_HAND_GRIPPER_CLOSE_PWM, time_ms);
}

static int mechanical_hand_stop_all(void)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.stop_all();
    return g_mech_hand_last_error;
}

static int mechanical_hand_stop_servo(uint8_t index)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.stop(index);
    return g_mech_hand_last_error;
}

static int mechanical_hand_home(uint16_t time_ms)
{
    return mechanical_hand_move_tcp_to(MECH_HAND_HOME_X, MECH_HAND_HOME_Y, MECH_HAND_HOME_Z, time_ms);
}

static int mechanical_hand_read_servo_position(uint8_t index, uint16_t *pwm)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if ((0 == pwm) || (MECH_HAND_OK != mechanical_hand_check_servo_index(index)))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.read_position(index, pwm);
    return g_mech_hand_last_error;
}

static int mechanical_hand_read_servo_telemetry(uint8_t index, bus_servo_telemetry_t *telemetry)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if ((0 == telemetry) || (MECH_HAND_OK != mechanical_hand_check_servo_index(index)))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.read_temperature_voltage(index, telemetry);
    return g_mech_hand_last_error;
}

static int mechanical_hand_read_servo_mode(uint8_t index, uint8_t *mode)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if ((0 == mode) || (MECH_HAND_OK != mechanical_hand_check_servo_index(index)))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.read_mode(index, mode);
    return g_mech_hand_last_error;
}

static int mechanical_hand_set_servo_mode(uint8_t index, uint8_t mode)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.set_mode(index, mode);
    return g_mech_hand_last_error;
}

static int mechanical_hand_set_servo_id(uint8_t index, uint8_t new_id)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if ((MECH_HAND_OK != mechanical_hand_check_servo_index(index)) ||
        (new_id > BUS_SERVO_ID_MAX))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.set_id(index, new_id);
    return g_mech_hand_last_error;
}

static int mechanical_hand_release_servo_torque(uint8_t index)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.release_torque(index);
    return g_mech_hand_last_error;
}

static int mechanical_hand_restore_servo_torque(uint8_t index)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.restore_torque(index);
    return g_mech_hand_last_error;
}

static int mechanical_hand_set_servo_current_as_startup(uint8_t index)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.set_current_as_startup(index);
    return g_mech_hand_last_error;
}

static int mechanical_hand_set_servo_startup_mode(uint8_t index, uint8_t mode)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.set_startup_mode(index, mode);
    return g_mech_hand_last_error;
}

static int mechanical_hand_read_servo_startup_mode(uint8_t index, uint8_t *mode)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if ((0 == mode) || (MECH_HAND_OK != mechanical_hand_check_servo_index(index)))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.read_startup_mode(index, mode);
    return g_mech_hand_last_error;
}

static int mechanical_hand_correct_servo_middle(uint8_t index)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.correct_middle(index);
    return g_mech_hand_last_error;
}

static int mechanical_hand_set_servo_current_as_min(uint8_t index)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.set_current_as_min(index);
    return g_mech_hand_last_error;
}

static int mechanical_hand_set_servo_current_as_max(uint8_t index)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.set_current_as_max(index);
    return g_mech_hand_last_error;
}

static int mechanical_hand_factory_reset_keep_id(uint8_t index)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.factory_reset_keep_id(index);
    return g_mech_hand_last_error;
}

static int mechanical_hand_factory_reset_all(uint8_t index)
{
    if (MECH_HAND_OK != mechanical_hand_check_ready())
    {
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != mechanical_hand_check_servo_index(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = bus_servo.factory_reset_all(index);
    return g_mech_hand_last_error;
}

static int mechanical_hand_get_last_error(void)
{
    return g_mech_hand_last_error;
}

static const char *mechanical_hand_get_last_command(void)
{
    return bus_servo.get_last_command();
}

static const char *mechanical_hand_get_last_response(void)
{
    return bus_servo.get_last_response();
}

static void mechanical_hand_get_servo_positions(mech_hand_servo_positions_t *positions)
{
    if (0 == positions)
    {
        return;
    }

    *positions = g_mech_hand_positions;
}

static bool mechanical_hand_was_last_command_clamped(void)
{
    return g_mech_hand_last_cmd_clamped;
}

/**
 * @brief  机械臂对外对象，应用层通过 mechanical_hand.xxx() 调用
 */
const mechanical_hand_api_t mechanical_hand =
{
    .init                         = mechanical_hand_init,
    .deinit                       = mechanical_hand_deinit,
    .move_to                      = mechanical_hand_move_to,
    .move_to_with_alpha           = mechanical_hand_move_to_with_alpha,
    .move_tcp_to                  = mechanical_hand_move_tcp_to,
    .move_tcp_to_with_alpha       = mechanical_hand_move_tcp_to_with_alpha,
    .send_single_servo            = mechanical_hand_send_single_servo,
    .send_multi_servo             = mechanical_hand_send_multi_servo,
    .send_raw_debug               = mechanical_hand_send_raw_debug,
    .gripper_rotate               = mechanical_hand_gripper_rotate,
    .gripper_rotate_center        = mechanical_hand_gripper_rotate_center,
    .gripper_open                 = mechanical_hand_gripper_open,
    .gripper_close                = mechanical_hand_gripper_close,
    .stop_all                     = mechanical_hand_stop_all,
    .stop_servo                   = mechanical_hand_stop_servo,
    .home                         = mechanical_hand_home,
    .read_servo_position          = mechanical_hand_read_servo_position,
    .read_servo_telemetry         = mechanical_hand_read_servo_telemetry,
    .read_servo_mode              = mechanical_hand_read_servo_mode,
    .set_servo_mode               = mechanical_hand_set_servo_mode,
    .set_servo_id                 = mechanical_hand_set_servo_id,
    .release_servo_torque         = mechanical_hand_release_servo_torque,
    .restore_servo_torque         = mechanical_hand_restore_servo_torque,
    .set_servo_current_as_startup = mechanical_hand_set_servo_current_as_startup,
    .set_servo_startup_mode       = mechanical_hand_set_servo_startup_mode,
    .read_servo_startup_mode      = mechanical_hand_read_servo_startup_mode,
    .correct_servo_middle         = mechanical_hand_correct_servo_middle,
    .set_servo_current_as_min     = mechanical_hand_set_servo_current_as_min,
    .set_servo_current_as_max     = mechanical_hand_set_servo_current_as_max,
    .factory_reset_keep_id        = mechanical_hand_factory_reset_keep_id,
    .factory_reset_all            = mechanical_hand_factory_reset_all,
    .get_last_error               = mechanical_hand_get_last_error,
    .get_last_command             = mechanical_hand_get_last_command,
    .get_last_response            = mechanical_hand_get_last_response,
    .get_servo_positions          = mechanical_hand_get_servo_positions,
    .was_last_command_clamped     = mechanical_hand_was_last_command_clamped,
};

static int mechanical_hand_check_ready(void)
{
    if (!g_mech_hand_initialized)
    {
        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
        return g_mech_hand_last_error;
    }

    return MECH_HAND_OK;
}

static int mechanical_hand_check_servo_index(uint8_t index)
{
    if (index >= MECH_HAND_SERVO_COUNT)
    {
        return MECH_HAND_ERR_PARAM;
    }

    return MECH_HAND_OK;
}

static int mechanical_hand_check_pwm(uint16_t pwm)
{
    if ((pwm < MECH_HAND_PWM_MIN) || (pwm > MECH_HAND_PWM_MAX))
    {
        return MECH_HAND_ERR_PARAM;
    }

    return MECH_HAND_OK;
}

static int mechanical_hand_check_time(uint16_t time_ms)
{
    if (time_ms > MECH_HAND_TIME_MAX)
    {
        return MECH_HAND_ERR_PARAM;
    }

    return MECH_HAND_OK;
}

static uint16_t mechanical_hand_get_servo_min(uint8_t index)
{
    switch (index)
    {
    case MECH_HAND_SERVO_BASE:
        return MECH_HAND_SERVO0_PWM_MIN;
    case MECH_HAND_SERVO_BIG_ARM:
        return MECH_HAND_SERVO1_PWM_MIN;
    case MECH_HAND_SERVO_FOREARM:
        return MECH_HAND_SERVO2_PWM_MIN;
    case MECH_HAND_SERVO_WRIST:
        return MECH_HAND_SERVO3_PWM_MIN;
    case MECH_HAND_SERVO_GRIPPER_ROTATE:
        return MECH_HAND_SERVO4_PWM_MIN;
    case MECH_HAND_SERVO_GRIPPER_CLAMP:
        return MECH_HAND_SERVO5_PWM_MIN;
    default:
        return MECH_HAND_PWM_MIN;
    }
}

static uint16_t mechanical_hand_get_servo_max(uint8_t index)
{
    switch (index)
    {
    case MECH_HAND_SERVO_BASE:
        return MECH_HAND_SERVO0_PWM_MAX;
    case MECH_HAND_SERVO_BIG_ARM:
        return MECH_HAND_SERVO1_PWM_MAX;
    case MECH_HAND_SERVO_FOREARM:
        return MECH_HAND_SERVO2_PWM_MAX;
    case MECH_HAND_SERVO_WRIST:
        return MECH_HAND_SERVO3_PWM_MAX;
    case MECH_HAND_SERVO_GRIPPER_ROTATE:
        return MECH_HAND_SERVO4_PWM_MAX;
    case MECH_HAND_SERVO_GRIPPER_CLAMP:
        return MECH_HAND_SERVO5_PWM_MAX;
    default:
        return MECH_HAND_PWM_MAX;
    }
}

static uint16_t mechanical_hand_clamp_servo_pwm(uint8_t index, uint16_t pwm, bool *clamped)
{
    uint16_t min_pwm = mechanical_hand_get_servo_min(index);
    uint16_t max_pwm = mechanical_hand_get_servo_max(index);

    if (0 != clamped)
    {
        *clamped = false;
    }

    if (pwm < min_pwm)
    {
        if (0 != clamped)
        {
            *clamped = true;
        }
        return min_pwm;
    }

    if (pwm > max_pwm)
    {
        if (0 != clamped)
        {
            *clamped = true;
        }
        return max_pwm;
    }

    return pwm;
}

static void mechanical_hand_update_servo_position(uint8_t index, uint16_t pwm)
{
    if (index < MECH_HAND_SERVO_COUNT)
    {
        g_mech_hand_positions.servo_pwm[index] = pwm;
    }
}

static void mechanical_hand_update_joint_positions(const mech_hand_joint_pwm_t *pwm)
{
    if (0 == pwm)
    {
        return;
    }

    g_mech_hand_positions.servo_pwm[0] = pwm->servo0_pwm;
    g_mech_hand_positions.servo_pwm[1] = pwm->servo1_pwm;
    g_mech_hand_positions.servo_pwm[2] = pwm->servo2_pwm;
    g_mech_hand_positions.servo_pwm[3] = pwm->servo3_pwm;
}

static void mechanical_hand_reset_clamp_state(void)
{
    g_mech_hand_last_cmd_clamped = false;
}

static int mechanical_hand_send_joint_command(const mech_hand_joint_pwm_t *pwm, uint16_t time_ms)
{
    mech_hand_joint_pwm_t limited_pwm;
    bus_servo_move_t moves[4];
    bool clamped = false;
    int status;

    if (0 == pwm)
    {
        return MECH_HAND_ERR_PARAM;
    }

    if (MECH_HAND_OK != mechanical_hand_check_time(time_ms))
    {
        return MECH_HAND_ERR_PARAM;
    }

    /* 四个主关节统一在这里做软件限位，避免 IK 或调试入口直接越界。 */
    mechanical_hand_reset_clamp_state();
    limited_pwm = *pwm;
    limited_pwm.servo0_pwm = mechanical_hand_clamp_servo_pwm(0U, limited_pwm.servo0_pwm, &clamped);
    g_mech_hand_last_cmd_clamped |= clamped;
    limited_pwm.servo1_pwm = mechanical_hand_clamp_servo_pwm(1U, limited_pwm.servo1_pwm, &clamped);
    g_mech_hand_last_cmd_clamped |= clamped;
    limited_pwm.servo2_pwm = mechanical_hand_clamp_servo_pwm(2U, limited_pwm.servo2_pwm, &clamped);
    g_mech_hand_last_cmd_clamped |= clamped;
    limited_pwm.servo3_pwm = mechanical_hand_clamp_servo_pwm(3U, limited_pwm.servo3_pwm, &clamped);
    g_mech_hand_last_cmd_clamped |= clamped;

    if ((MECH_HAND_OK != mechanical_hand_check_pwm(limited_pwm.servo0_pwm)) ||
        (MECH_HAND_OK != mechanical_hand_check_pwm(limited_pwm.servo1_pwm)) ||
        (MECH_HAND_OK != mechanical_hand_check_pwm(limited_pwm.servo2_pwm)) ||
        (MECH_HAND_OK != mechanical_hand_check_pwm(limited_pwm.servo3_pwm)))
    {
        return MECH_HAND_ERR_PARAM;
    }

    moves[0].id = 0U;
    moves[0].pwm = limited_pwm.servo0_pwm;
    moves[0].time_ms = time_ms;
    moves[1].id = 1U;
    moves[1].pwm = limited_pwm.servo1_pwm;
    moves[1].time_ms = time_ms;
    moves[2].id = 2U;
    moves[2].pwm = limited_pwm.servo2_pwm;
    moves[2].time_ms = time_ms;
    moves[3].id = 3U;
    moves[3].pwm = limited_pwm.servo3_pwm;
    moves[3].time_ms = time_ms;

    status = bus_servo.move_multi(moves, 4U);
    if (MECH_HAND_OK == status)
    {
        mechanical_hand_update_joint_positions(&limited_pwm);
    }

    return status;
}

static int mechanical_hand_try_parse_servo_command(const char *cmd, uint8_t *index, uint16_t *pwm, uint16_t *time_ms)
{
    unsigned int temp_index;
    unsigned int temp_pwm;
    unsigned int temp_time;

    if ((0 == cmd) || (0 == index) || (0 == pwm) || (0 == time_ms))
    {
        return MECH_HAND_ERR_PARAM;
    }

    if (3 == sscanf(cmd, "#%3uP%4uT%4u!", &temp_index, &temp_pwm, &temp_time))
    {
        if (temp_index > BUS_SERVO_ID_MAX)
        {
            return MECH_HAND_ERR_PARAM;
        }

        *index = (uint8_t)temp_index;
        *pwm = (uint16_t)temp_pwm;
        *time_ms = (uint16_t)temp_time;
        return MECH_HAND_OK;
    }

    return MECH_HAND_ERR_FORMAT;
}

static int mechanical_hand_try_parse_joint_command(const char *cmd, mech_hand_joint_pwm_t *pwm, uint16_t *time_ms)
{
    unsigned int s0;
    unsigned int t0;
    unsigned int s1;
    unsigned int t1;
    unsigned int s2;
    unsigned int t2;
    unsigned int s3;
    unsigned int t3;

    if ((0 == cmd) || (0 == pwm) || (0 == time_ms))
    {
        return MECH_HAND_ERR_PARAM;
    }

    if (8 == sscanf(cmd,
                    "{#000P%4uT%4u!#001P%4uT%4u!#002P%4uT%4u!#003P%4uT%4u!}",
                    &s0,
                    &t0,
                    &s1,
                    &t1,
                    &s2,
                    &t2,
                    &s3,
                    &t3))
    {
        if ((t1 != t0) || (t2 != t0) || (t3 != t0))
        {
            return MECH_HAND_ERR_FORMAT;
        }

        pwm->servo0_pwm = (uint16_t)s0;
        pwm->servo1_pwm = (uint16_t)s1;
        pwm->servo2_pwm = (uint16_t)s2;
        pwm->servo3_pwm = (uint16_t)s3;
        *time_ms = (uint16_t)t0;
        return MECH_HAND_OK;
    }

    return MECH_HAND_ERR_FORMAT;
}

static int mechanical_hand_kinematics_analysis(float x, float y, float z, float alpha_deg, mech_hand_joint_pwm_t *out_pwm)
{
    float l0;
    float l1;
    float l2;
    float l3;
    float theta6;
    float ccc;
    float bbb;
    float theta5;
    float aaa;
    float theta4;
    float theta3;
    float zf_flag;
    float servo_angle0;
    float servo_angle1;
    float servo_angle2;
    float servo_angle3;
    float distance;

    if (0 == out_pwm)
    {
        return MECH_HAND_ERR_PARAM;
    }

    /* 旧 RA6M5/OpenMV 模型内部使用 0.1mm 量级，输入 mm 需要放大 10 倍。 */
    x *= 10.0f;
    y *= 10.0f;
    z *= 10.0f;

    l0 = MECH_HAND_L0;
    l1 = MECH_HAND_L1;
    l2 = MECH_HAND_L2;
    l3 = MECH_HAND_L3;

    if (fabsf(x) < MECH_HAND_FLOAT_EPSILON)
    {
        theta6 = 0.0f;
    }
    else
    {
        if (fabsf(y) < MECH_HAND_FLOAT_EPSILON)
        {
            theta6 = (x > 0.0f) ? 135.0f : -135.0f;
        }
        else
        {
            theta6 = atanf(x / y) * 270.0f / MECH_HAND_PI;
        }
    }

    /*
     * 先把平面 x/y 折算成机械臂前向距离，再扣除末端连杆在 alpha 姿态下的投影。
     * 这里输出的是旧模型参考点需要满足的四轴关节角。
     */
    y = sqrtf((x * x) + (y * y));
    y = y - l3 * cosf(alpha_deg * MECH_HAND_PI / 180.0f);
    z = z - l0 - l3 * sinf(alpha_deg * MECH_HAND_PI / 180.0f);

    if (z < (-l0))
    {
        return MECH_HAND_IK_ERR_1;
    }

    distance = sqrtf((y * y) + (z * z));
    if (distance < MECH_HAND_FLOAT_EPSILON)
    {
        return MECH_HAND_IK_ERR_2;
    }

    if (distance > (l1 + l2))
    {
        return MECH_HAND_IK_ERR_2;
    }

    ccc = acosf(y / distance);
    bbb = ((y * y) + (z * z) + (l1 * l1) - (l2 * l2)) / (2.0f * l1 * distance);
    if ((bbb > 1.0f) || (bbb < -1.0f))
    {
        return MECH_HAND_IK_ERR_5;
    }

    zf_flag = (z < 0.0f) ? -1.0f : 1.0f;

    theta5 = ccc * zf_flag + acosf(bbb);
    theta5 = theta5 * 180.0f / MECH_HAND_PI;
    if ((theta5 > 180.0f) || (theta5 < 0.0f))
    {
        return MECH_HAND_IK_ERR_6;
    }

    aaa = -((y * y) + (z * z) - (l1 * l1) - (l2 * l2)) / (2.0f * l1 * l2);
    if ((aaa > 1.0f) || (aaa < -1.0f))
    {
        return MECH_HAND_IK_ERR_3;
    }

    theta4 = acosf(aaa);
    theta4 = 180.0f - theta4 * 180.0f / MECH_HAND_PI;
    if ((theta4 > 135.0f) || (theta4 < -135.0f))
    {
        return MECH_HAND_IK_ERR_4;
    }

    theta3 = alpha_deg - theta5 + theta4;
    if ((theta3 > 90.0f) || (theta3 < -90.0f))
    {
        return MECH_HAND_IK_ERR_7;
    }

    servo_angle0 = theta6;
    servo_angle1 = theta5 - 90.0f;
    servo_angle2 = theta4;
    servo_angle3 = theta3;

    /* 角度到总线舵机 PWM 的映射沿用旧工程标定，后续只在发送前做软件限位。 */
    out_pwm->servo0_pwm = (uint16_t)(1500.0f - 2000.0f * servo_angle0 / 270.0f);
    out_pwm->servo1_pwm = (uint16_t)(1500.0f + 2000.0f * servo_angle1 / 270.0f);
    out_pwm->servo2_pwm = (uint16_t)(1500.0f + 2000.0f * servo_angle2 / 270.0f);
    out_pwm->servo3_pwm = (uint16_t)(1500.0f + 2000.0f * servo_angle3 / 270.0f);

    if ((MECH_HAND_OK != mechanical_hand_check_pwm(out_pwm->servo0_pwm)) ||
        (MECH_HAND_OK != mechanical_hand_check_pwm(out_pwm->servo1_pwm)) ||
        (MECH_HAND_OK != mechanical_hand_check_pwm(out_pwm->servo2_pwm)) ||
        (MECH_HAND_OK != mechanical_hand_check_pwm(out_pwm->servo3_pwm)))
    {
        return MECH_HAND_ERR_PARAM;
    }

    return MECH_HAND_OK;
}

static int mechanical_hand_kinematics_move(float x, float y, float z, uint16_t time_ms)
{
    int alpha_deg;
    int best_alpha = 0;
    bool found = false;
    int status = MECH_HAND_ERR_PARAM;
    mech_hand_joint_pwm_t pwm;

    if (y < 0.0f)
    {
        return MECH_HAND_ERR_PARAM;
    }

    /* 自动姿态从水平开始向下扫描，最后选择更小的可达 alpha。 */
    for (alpha_deg = 0; alpha_deg >= -135; alpha_deg--)
    {
        status = mechanical_hand_kinematics_analysis(x, y, z, (float)alpha_deg, &pwm);
        if (MECH_HAND_OK == status)
        {
            if (alpha_deg < best_alpha)
            {
                best_alpha = alpha_deg;
            }
            found = true;
        }
    }

    if (!found)
    {
        return status;
    }

    status = mechanical_hand_kinematics_analysis(x, y, z, (float)best_alpha, &pwm);
    if (MECH_HAND_OK != status)
    {
        return status;
    }

    return mechanical_hand_send_joint_command(&pwm, time_ms);
}

static float mechanical_hand_calc_yaw_deg(float x, float y)
{
    if ((fabsf(x) < MECH_HAND_FLOAT_EPSILON) && (fabsf(y) < MECH_HAND_FLOAT_EPSILON))
    {
        return 0.0f;
    }

    return atan2f(x, y) * 180.0f / MECH_HAND_PI;
}

static int mechanical_hand_tcp_to_reference(float tcp_x, float tcp_y, float tcp_z, float alpha_deg,
                                         float *ref_x, float *ref_y, float *ref_z)
{
    float yaw_deg;
    float offset_x;
    float offset_y;
    float offset_z;
    int status;

    if ((0 == ref_x) || (0 == ref_y) || (0 == ref_z))
    {
        return MECH_HAND_ERR_PARAM;
    }

    /*
     * 工具偏移定义为旧模型参考点到 TCP 的向量。
     * 运动求解需要旧模型参考点，所以这里用 TCP 目标点减去基座坐标下的偏移。
     */
    yaw_deg = mechanical_hand_calc_yaw_deg(tcp_x, tcp_y);
    status = calibration.tool_offset_to_base(yaw_deg,
                                          alpha_deg,
                                          MECH_HAND_TOOL_REF_OFFSET_X_MM,
                                          MECH_HAND_TOOL_REF_OFFSET_Y_MM,
                                          MECH_HAND_TOOL_REF_OFFSET_Z_MM,
                                          &offset_x,
                                          &offset_y,
                                          &offset_z);
    if (CALIB_OK != status)
    {
        return MECH_HAND_ERR_PARAM;
    }

    *ref_x = tcp_x - offset_x;
    *ref_y = tcp_y - offset_y;
    *ref_z = tcp_z - offset_z;

    return MECH_HAND_OK;
}

static int mechanical_hand_kinematics_move_tcp(float x, float y, float z, uint16_t time_ms)
{
    int alpha_deg;
    int best_alpha = 0;
    bool found = false;
    int status = MECH_HAND_ERR_PARAM;
    float ref_x;
    float ref_y;
    float ref_z;
    mech_hand_joint_pwm_t pwm;

    if (y < 0.0f)
    {
        return MECH_HAND_ERR_PARAM;
    }

    /* TCP 自动姿态必须每个 alpha 都重新计算工具偏移，不能复用固定参考点。 */
    for (alpha_deg = 0; alpha_deg >= -135; alpha_deg--)
    {
        status = mechanical_hand_tcp_to_reference(x, y, z, (float)alpha_deg, &ref_x, &ref_y, &ref_z);
        if (MECH_HAND_OK != status)
        {
            return status;
        }

        if (ref_y < 0.0f)
        {
            status = MECH_HAND_ERR_PARAM;
            continue;
        }

        status = mechanical_hand_kinematics_analysis(ref_x, ref_y, ref_z, (float)alpha_deg, &pwm);
        if (MECH_HAND_OK == status)
        {
            if (alpha_deg < best_alpha)
            {
                best_alpha = alpha_deg;
            }
            found = true;
        }
    }

    if (!found)
    {
        return status;
    }

    status = mechanical_hand_tcp_to_reference(x, y, z, (float)best_alpha, &ref_x, &ref_y, &ref_z);
    if (MECH_HAND_OK != status)
    {
        return status;
    }

    status = mechanical_hand_kinematics_analysis(ref_x, ref_y, ref_z, (float)best_alpha, &pwm);
    if (MECH_HAND_OK != status)
    {
        return status;
    }

    return mechanical_hand_send_joint_command(&pwm, time_ms);
}
