#include "Mechanical_hand_control.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*
 * 机械臂动作层：逆运动学、UART4 舵机命令、六路 PWM 限位及状态缓存。
 * MechanicalHand_Init() 打开 UART4，并调用 MechanicalHand_Home(1000U)。
 * 默认回零目标由 MECH_HAND_HOME_X/Y/Z 确定，MoveTo/MoveToWithAlpha 换算关节 PWM。
 * 外部目标需先转换到机械臂基座坐标系；本模块不执行视觉识别或标定求解。
 * 六路 PWM 缓存供菜单 S0~S5 状态显示使用。
 */

/*
 * 连杆长度参数说明
 * 1. 这 4 个参数沿用 OpenMV 版本的机械臂模型。
 * 2. 当前内部计算前会把输入坐标放大 10 倍，因此这里是“放大 10 倍后的内部长度单位”。
 * 3. 机械结构变化时需同步校准连杆长度。
 * 4. 修改后会影响可达范围、逆运动学结果以及最终舵机姿态。
 */
#define MECH_HAND_L0 (1000.0f)
#define MECH_HAND_L1 (1050.0f)
#define MECH_HAND_L2 (880.0f)
#define MECH_HAND_L3 (1550.0f)

/* L0：基座高度补偿，表示底座到主运动平面的基准高度。 */
/* L1：大臂有效长度，影响前伸能力和抬升能力。 */
/* L2：小臂有效长度，影响末端继续伸展的能力。 */
/* L3：末端前向补偿长度，影响末端姿态和目标点换算。 */


/*
 * X/Y/Z 范围说明
 * 1. 当前菜单调试页对输入坐标的限幅范围是：
 *    - X：-200 ~ 200 mm
 *    - Y：0 ~ 250 mm
 *    - Z：0 ~ 200 mm
 * 2. 上述范围只是“菜单允许输入的调试范围”，不是机械臂保证可达的绝对工作范围。
 * 3. 即使坐标还在这个范围内，也可能因为逆运动学不可达、Alpha 扫描失败或 #000 ~ #003 舵机限位裁剪而不动作。
 * 4. 如果轻微改动 X/Y/Z 后不动作，应同时查看：
 *    - RET：是否为 1 ~ 7 的逆运动学错误码
 *    - SAFE：是否为 CLAMP，表示关节输出被软件限位裁剪
 */

/* 舵机通道定义。 */
#define MECH_HAND_SERVO_BASE (0U)
#define MECH_HAND_SERVO_BIG_ARM (1U)
#define MECH_HAND_SERVO_FOREARM (2U)
#define MECH_HAND_SERVO_WRIST (3U)
#define MECH_HAND_SERVO_GRIPPER_ROTATE (4U)
#define MECH_HAND_SERVO_GRIPPER_CLAMP (5U)

/* 协议允许的总 PWM 范围和时间范围，属于底层协议边界。 */
#define MECH_HAND_PWM_MIN (500U)
#define MECH_HAND_PWM_MAX (2500U)
#define MECH_HAND_TIME_MAX (9999U)

/*
 * 夹爪默认动作 PWM
 * 1. OPEN / CLOSE 对应 #005 的默认张开和闭合位置。
 * 2. ROTATE_CENTER 对应 #004 的默认回中位置。
 */
#define MECH_HAND_GRIPPER_OPEN_PWM (1000U)
#define MECH_HAND_GRIPPER_CLOSE_PWM (1550U)
#define MECH_HAND_GRIPPER_ROTATE_CENTER_PWM (1500U)

/* 字符串命令缓冲区大小。 */
#define MECH_HAND_CMD_BUFFER_SIZE (96U)
#define MECH_HAND_LAST_CMD_SIZE (96U)

/* UART4 发送完成等待超时，单位 ms。 */
#define MECH_HAND_UART_TX_TIMEOUT_MS (200U)

/* 浮点比较和角度计算常量。 */
#define MECH_HAND_FLOAT_EPSILON (0.0001f)
#define MECH_HAND_PI (3.1415926f)

static volatile bool g_mech_hand_uart4_tx_complete = false;
static volatile bool g_mech_hand_uart4_opened = false;
static volatile bool g_mech_hand_initialized = false;
static int g_mech_hand_last_error = MECH_HAND_OK;
static char g_mech_hand_last_cmd[MECH_HAND_LAST_CMD_SIZE] = {0};
static bool g_mech_hand_last_cmd_clamped = false;
static mech_hand_servo_positions_t g_mech_hand_positions = {{1500U, 1500U, 1500U, 1500U, 1500U, 1500U}};

static int MechanicalHand_UartOpen(void);
static int MechanicalHand_UartWrite(const char *cmd);
static int MechanicalHand_UartWaitTxComplete(uint32_t timeout_ms);
static int MechanicalHand_CheckServoIndex(uint8_t index);
static int MechanicalHand_CheckPwm(uint16_t pwm);
static int MechanicalHand_CheckTime(uint16_t time_ms);
static uint16_t MechanicalHand_GetServoMin(uint8_t index);
static uint16_t MechanicalHand_GetServoMax(uint8_t index);
static uint16_t MechanicalHand_ClampServoPwm(uint8_t index, uint16_t pwm, bool *clamped);
static void MechanicalHand_UpdateServoPosition(uint8_t index, uint16_t pwm);
static void MechanicalHand_UpdateJointPositions(const mech_hand_joint_pwm_t *pwm);
static void MechanicalHand_ResetClampState(void);
static int MechanicalHand_SendJointCommand(const mech_hand_joint_pwm_t *pwm, uint16_t time_ms);
static int MechanicalHand_TryParseServoCommand(const char *cmd, uint8_t *index, uint16_t *pwm, uint16_t *time_ms);
static int MechanicalHand_TryParseJointCommand(const char *cmd, mech_hand_joint_pwm_t *pwm, uint16_t *time_ms);
static void MechanicalHand_RecordLastCommand(const char *cmd);
static int MechanicalHand_BuildJointCmd(const mech_hand_joint_pwm_t *pwm, uint16_t time_ms, char *buf, uint32_t buf_size);
static int MechanicalHand_KinematicsAnalysis(float x, float y, float z, float alpha_deg, mech_hand_joint_pwm_t *out_pwm);
static int MechanicalHand_KinematicsMove(float x, float y, float z, uint16_t time_ms);

/*
 * 函数说明：初始化机械手模块
 * 1. 打开 UART4。
 * 2. 清除限位标志。
 * 3. 执行一次默认回零动作。
 * 4. 默认上电姿态来源于 MechanicalHand_Home() 和默认回零坐标宏。
 */
uint8_t MechanicalHand_Init(void)
{
    if (MECH_HAND_OK != MechanicalHand_UartOpen())
    {
        return 0U;
    }

    MechanicalHand_ResetClampState();
    g_mech_hand_initialized = true;
    g_mech_hand_last_error = MECH_HAND_OK;

    if (MECH_HAND_OK != MechanicalHand_Home(1000U))
    {
        return 0U;
    }

    return 1U;
}

/* 函数说明：关闭 UART4，并清除机械手初始化标志。 */
void MechanicalHand_Deinit(void)
{
    if (g_mech_hand_uart4_opened)
    {
        fsp_err_t err = g_uart4.p_api->close(g_uart4.p_ctrl);
        if ((FSP_SUCCESS != err) && (FSP_ERR_NOT_OPEN != err))
        {
            g_mech_hand_last_error = MECH_HAND_ERR_UART_CLOSE;
            return;
        }
    }

    g_mech_hand_uart4_opened = false;
    g_mech_hand_initialized = false;
    g_mech_hand_last_error = MECH_HAND_OK;
}

/* 函数说明：按目标 x/y/z 运动，自动扫描末端俯仰角。
 * 参数范围说明：
 * 菜单输入范围由头文件 MECH_HAND_MENU_X/Y/Z_MIN/MAX 定义，单位 mm。
 * 输入范围不代表所有点可达，运动仍受逆运动学、Alpha 扫描和关节软件限位约束。
 * 运动失败时通过 RET 和 SAFE 检查返回状态。
 */
int MechanicalHand_MoveTo(float x, float y, float z, uint16_t time_ms)
{
    if (!g_mech_hand_initialized)
    {
        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = MechanicalHand_KinematicsMove(x, y, z, time_ms);
    return g_mech_hand_last_error;
}

/* 函数说明：按目标 x/y/z 和固定 alpha 运动，不自动扫描俯仰角。 */
int MechanicalHand_MoveToWithAlpha(float x, float y, float z, float alpha_deg, uint16_t time_ms)
{
    mech_hand_joint_pwm_t pwm;
    int status;

    if (!g_mech_hand_initialized)
    {
        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
        return g_mech_hand_last_error;
    }

    status = MechanicalHand_KinematicsAnalysis(x, y, z, alpha_deg, &pwm);
    if (MECH_HAND_OK != status)
    {
        g_mech_hand_last_error = status;
        return status;
    }

    status = MechanicalHand_SendJointCommand(&pwm, time_ms);
    g_mech_hand_last_error = status;
    return status;
}

/* 函数说明：直接控制单舵机，不经过逆运动学，但会经过软件限位。 */
int MechanicalHand_SendSingleServo(uint8_t index, uint16_t pwm, uint16_t time_ms)
{
    char cmd[32];
    int len;
    bool clamped = false;

    if (!g_mech_hand_initialized)
    {
        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
        return g_mech_hand_last_error;
    }

    if ((MECH_HAND_OK != MechanicalHand_CheckServoIndex(index)) ||
        (MECH_HAND_OK != MechanicalHand_CheckTime(time_ms)))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    MechanicalHand_ResetClampState();
    pwm = MechanicalHand_ClampServoPwm(index, pwm, &clamped);
    g_mech_hand_last_cmd_clamped = clamped;
    MechanicalHand_UpdateServoPosition(index, pwm);

    len = snprintf(cmd, sizeof(cmd), "#%03uP%04uT%04u!", index, pwm, time_ms);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_FORMAT;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = MechanicalHand_UartWrite(cmd);
    return g_mech_hand_last_error;
}

/* 函数说明：直接发送 #000 ~ #003 四轴组合命令。 */
int MechanicalHand_SendMultiServo(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t time_ms)
{
    mech_hand_joint_pwm_t pwm;

    if (!g_mech_hand_initialized)
    {
        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
        return g_mech_hand_last_error;
    }

    pwm.servo0_pwm = pwm0;
    pwm.servo1_pwm = pwm1;
    pwm.servo2_pwm = pwm2;
    pwm.servo3_pwm = pwm3;

    g_mech_hand_last_error = MechanicalHand_SendJointCommand(&pwm, time_ms);
    return g_mech_hand_last_error;
}

/* 函数说明：发送原始调试命令，能识别标准单舵机或四轴组合命令并施加限位。 */
int MechanicalHand_SendRawDebug(const char *cmd)
{
    uint8_t index;
    uint16_t pwm;
    uint16_t time_ms;
    mech_hand_joint_pwm_t joints;

    if (!g_mech_hand_initialized)
    {
        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
        return g_mech_hand_last_error;
    }

    if ((NULL == cmd) || ('\0' == cmd[0]))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK == MechanicalHand_TryParseServoCommand(cmd, &index, &pwm, &time_ms))
    {
        g_mech_hand_last_error = MechanicalHand_SendSingleServo(index, pwm, time_ms);
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK == MechanicalHand_TryParseJointCommand(cmd, &joints, &time_ms))
    {
        g_mech_hand_last_error = MechanicalHand_SendMultiServo(joints.servo0_pwm,
                                                               joints.servo1_pwm,
                                                               joints.servo2_pwm,
                                                               joints.servo3_pwm,
                                                               time_ms);
        return g_mech_hand_last_error;
    }

    MechanicalHand_ResetClampState();
    g_mech_hand_last_error = MechanicalHand_UartWrite(cmd);
    return g_mech_hand_last_error;
}

/* 函数说明：控制 #004 夹爪旋转。 */
int MechanicalHand_GripperRotate(uint16_t pwm, uint16_t time_ms)
{
    return MechanicalHand_SendSingleServo(MECH_HAND_SERVO_GRIPPER_ROTATE, pwm, time_ms);
}

/* 函数说明：将 #004 旋转舵机回到默认中心位置。 */
int MechanicalHand_GripperRotateCenter(uint16_t time_ms)
{
    return MechanicalHand_GripperRotate(MECH_HAND_GRIPPER_ROTATE_CENTER_PWM, time_ms);
}

//=====================================================================
//============替换为PWM舵机后续换回===================
/* 函数说明：执行 #005 夹爪张开动作。 */
 int MechanicalHand_GripperOpen(uint16_t time_ms)
 {
     return MechanicalHand_SendSingleServo(MECH_HAND_SERVO_GRIPPER_CLAMP, MECH_HAND_GRIPPER_OPEN_PWM, time_ms);
 }
 /* 函数说明：执行 #005 夹爪闭合动作。 */
 int MechanicalHand_GripperClose(uint16_t time_ms)
 {
     return MechanicalHand_SendSingleServo(MECH_HAND_SERVO_GRIPPER_CLAMP, MECH_HAND_GRIPPER_CLOSE_PWM, time_ms);
 }

//int MechanicalHand_GripperOpen(uint16_t time_ms)
//{
//    fsp_err_t err;
//    uint32_t period_counts;
//    uint32_t duty_counts;

//    (void)time_ms;

//    if (!g_mech_hand_initialized)
//    {
//        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
//        return g_mech_hand_last_error;
//    }

//    /* 当前宏值按普通 PWM 舵机脉宽处理，单位 us。 */
//    period_counts = g_timer4.p_cfg->period_counts;
//    duty_counts = (uint32_t)(((uint64_t)MECH_HAND_GRIPPER_OPEN_PWM * period_counts) / 20000U);

//    err = g_timer4.p_api->dutyCycleSet(g_timer4.p_ctrl, duty_counts, GPT_IO_PIN_GTIOCA);
//    if (FSP_SUCCESS != err)
//    {
//        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
//        return g_mech_hand_last_error;
//    }

//    /* 保持软件缓存一致，便于菜单和状态机读取最近一次目标值。 */
//    MechanicalHand_ResetClampState();
//    g_mech_hand_last_cmd_clamped = false;
//    g_mech_hand_positions.servo_pwm[MECH_HAND_SERVO_GRIPPER_CLAMP] = MECH_HAND_GRIPPER_OPEN_PWM;
//    g_mech_hand_last_error = MECH_HAND_OK;

//    return g_mech_hand_last_error;
//}

//int MechanicalHand_GripperClose(uint16_t time_ms)
//{
//    fsp_err_t err;
//    uint32_t period_counts;
//    uint32_t duty_counts;

//    (void)time_ms;

//    if (!g_mech_hand_initialized)
//    {
//        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
//        return g_mech_hand_last_error;
//    }

//    /* 当前宏值按普通 PWM 舵机脉宽处理，单位 us。 */
//    period_counts = g_timer4.p_cfg->period_counts;
//    duty_counts = (uint32_t)(((uint64_t)MECH_HAND_GRIPPER_CLOSE_PWM * period_counts) / 20000U);

//    err = g_timer4.p_api->dutyCycleSet(g_timer4.p_ctrl, duty_counts, GPT_IO_PIN_GTIOCA);
//    if (FSP_SUCCESS != err)
//    {
//        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
//        return g_mech_hand_last_error;
//    }

//    /* 保持软件缓存一致，便于菜单和状态机读取最近一次目标值。 */
//    MechanicalHand_ResetClampState();
//    g_mech_hand_last_cmd_clamped = false;
//    g_mech_hand_positions.servo_pwm[MECH_HAND_SERVO_GRIPPER_CLAMP] = MECH_HAND_GRIPPER_CLOSE_PWM;
//    g_mech_hand_last_error = MECH_HAND_OK;

//    return g_mech_hand_last_error;
//}

//====================================================================
/* 函数说明：发送 $DST!，停止所有舵机。 */
int MechanicalHand_StopAll(void)
{
    if (!g_mech_hand_initialized)
    {
        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = MechanicalHand_UartWrite("$DST!");
    return g_mech_hand_last_error;
}

/* 函数说明：发送 $DST:x!，停止指定舵机。 */
int MechanicalHand_StopServo(uint8_t index)
{
    char cmd[16];
    int len;

    if (!g_mech_hand_initialized)
    {
        g_mech_hand_last_error = MECH_HAND_ERR_NOT_READY;
        return g_mech_hand_last_error;
    }

    if (MECH_HAND_OK != MechanicalHand_CheckServoIndex(index))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_PARAM;
        return g_mech_hand_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "$DST:%u!", index);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_FORMAT;
        return g_mech_hand_last_error;
    }

    g_mech_hand_last_error = MechanicalHand_UartWrite(cmd);
    return g_mech_hand_last_error;
}

/*
 * 函数说明：执行默认回零动作
 * 1. 当前默认姿态不是直接写角度，而是调用默认回零坐标。
 * 2. 路径为 Home -> MoveTo -> 逆运动学 -> 4 轴 PWM。
 */
int MechanicalHand_Home(uint16_t time_ms)
{
    return MechanicalHand_MoveTo(MECH_HAND_HOME_X, MECH_HAND_HOME_Y, MECH_HAND_HOME_Z, time_ms);
}

/* 函数说明：读取最近一次动作的返回码。 */
int MechanicalHand_GetLastError(void)
{
    return g_mech_hand_last_error;
}

/* 函数说明：读取最近一次发送的原始协议字符串。 */
const char *MechanicalHand_GetLastCommand(void)
{
    return g_mech_hand_last_cmd;
}

/* 函数说明：读取 6 路舵机目标 PWM 缓存，供菜单显示 S0~S5。 */
void MechanicalHand_GetServoPositions(mech_hand_servo_positions_t *positions)
{
    if (NULL == positions)
    {
        return;
    }

    *positions = g_mech_hand_positions;
}

/* 函数说明：判断最近一次结构化命令是否触发过限位裁剪。 */
bool MechanicalHand_WasLastCommandClamped(void)
{
    return g_mech_hand_last_cmd_clamped;
}

/* 私有函数说明：打开 UART4，作为机械手协议发送通道。 */
static int MechanicalHand_UartOpen(void)
{
    fsp_err_t err = g_uart4.p_api->open(g_uart4.p_ctrl, g_uart4.p_cfg);

    /* 鑷姩鐢熸垚鐨?UART4 褰撳墠鐢?FSP 閰嶇疆鍐冲畾銆?     * 鏈ā鍧楀彧璐熻矗浣跨敤锛屼笉鍘讳慨鏀硅嚜鍔ㄧ敓鎴愭枃浠躲€?     */
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        g_mech_hand_last_error = MECH_HAND_ERR_UART_OPEN;
        return g_mech_hand_last_error;
    }

    g_mech_hand_uart4_opened = true;
    g_mech_hand_uart4_tx_complete = false;

    return MECH_HAND_OK;
}

/* 私有函数说明：发送完整协议字符串，并等待发送完成。 */
static int MechanicalHand_UartWrite(const char *cmd)
{
    fsp_err_t err;
    uint32_t len;

    if ((NULL == cmd) || (!g_mech_hand_uart4_opened))
    {
        return MECH_HAND_ERR_NOT_READY;
    }

    len = (uint32_t)strlen(cmd);
    if (0U == len)
    {
        return MECH_HAND_ERR_PARAM;
    }

    MechanicalHand_RecordLastCommand(cmd);
    g_mech_hand_uart4_tx_complete = false;

    err = g_uart4.p_api->write(g_uart4.p_ctrl, (uint8_t const *)cmd, len);
    if (FSP_SUCCESS != err)
    {
        return MECH_HAND_ERR_UART_WRITE;
    }

    return MechanicalHand_UartWaitTxComplete(MECH_HAND_UART_TX_TIMEOUT_MS);
}

/* 私有函数说明：轮询等待 UART_EVENT_TX_COMPLETE。 */
static int MechanicalHand_UartWaitTxComplete(uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0U;

    while (!g_mech_hand_uart4_tx_complete)
    {
        if (elapsed_ms >= timeout_ms)
        {
            return MECH_HAND_ERR_TIMEOUT;
        }

        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
        elapsed_ms++;
    }

    g_mech_hand_uart4_tx_complete = false;
    return MECH_HAND_OK;
}

/* 私有函数说明：检查舵机编号是否合法。 */
static int MechanicalHand_CheckServoIndex(uint8_t index)
{
    if (index > 254U)
    {
        return MECH_HAND_ERR_PARAM;
    }

    return MECH_HAND_OK;
}

/* 私有函数说明：检查 PWM 是否在协议底层允许范围内。 */
static int MechanicalHand_CheckPwm(uint16_t pwm)
{
    if ((pwm < MECH_HAND_PWM_MIN) || (pwm > MECH_HAND_PWM_MAX))
    {
        return MECH_HAND_ERR_PARAM;
    }

    return MECH_HAND_OK;
}

/* 私有函数说明：检查动作时间 time_ms 是否在协议允许范围内。 */
static int MechanicalHand_CheckTime(uint16_t time_ms)
{
    if (time_ms > MECH_HAND_TIME_MAX)
    {
        return MECH_HAND_ERR_PARAM;
    }

    return MECH_HAND_OK;
}

/* 私有函数说明：读取指定舵机的软件最小限位。 */
static uint16_t MechanicalHand_GetServoMin(uint8_t index)
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
        return MECH_HAND_PWM_MIN;
    }
}

/* 私有函数说明：读取指定舵机的软件最大限位。 */
static uint16_t MechanicalHand_GetServoMax(uint8_t index)
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
        return MECH_HAND_PWM_MAX;
    }
}

/* 私有函数说明：对单个舵机 PWM 执行限位裁剪，并返回是否发生裁剪。 */
static uint16_t MechanicalHand_ClampServoPwm(uint8_t index, uint16_t pwm, bool *clamped)
{
    uint16_t min_pwm = MechanicalHand_GetServoMin(index);
    uint16_t max_pwm = MechanicalHand_GetServoMax(index);

    if (NULL != clamped)
    {
        *clamped = false;
    }

    if (pwm < min_pwm)
    {
        if (NULL != clamped)
        {
            *clamped = true;
        }
        return min_pwm;
    }

    if (pwm > max_pwm)
    {
        if (NULL != clamped)
        {
            *clamped = true;
        }
        return max_pwm;
    }

    return pwm;
}

/* 私有函数说明：更新单路舵机的目标 PWM 缓存。 */
static void MechanicalHand_UpdateServoPosition(uint8_t index, uint16_t pwm)
{
    if (index < 6U)
    {
        g_mech_hand_positions.servo_pwm[index] = pwm;
    }
}

/* 私有函数说明：批量更新四个主关节的目标 PWM 缓存。 */
static void MechanicalHand_UpdateJointPositions(const mech_hand_joint_pwm_t *pwm)
{
    if (NULL == pwm)
    {
        return;
    }

    g_mech_hand_positions.servo_pwm[0] = pwm->servo0_pwm;
    g_mech_hand_positions.servo_pwm[1] = pwm->servo1_pwm;
    g_mech_hand_positions.servo_pwm[2] = pwm->servo2_pwm;
    g_mech_hand_positions.servo_pwm[3] = pwm->servo3_pwm;
}

/* 私有函数说明：清除最近一次命令的限位裁剪标志。 */
static void MechanicalHand_ResetClampState(void)
{
    g_mech_hand_last_cmd_clamped = false;
}

/* 私有函数说明：记录最近一次发送的原始协议字符串。 */
static void MechanicalHand_RecordLastCommand(const char *cmd)
{
    size_t i;

    if (NULL == cmd)
    {
        g_mech_hand_last_cmd[0] = '\0';
        return;
    }

    for (i = 0; (i < (MECH_HAND_LAST_CMD_SIZE - 1U)) && (cmd[i] != '\0'); i++)
    {
        g_mech_hand_last_cmd[i] = cmd[i];
    }

    g_mech_hand_last_cmd[i] = '\0';
}

/*
 * 私有函数说明：四轴统一发送出口
 * 1. 负责对 #000 ~ #003 做限位裁剪。
 * 2. 负责更新主关节目标缓存。
 * 3. 负责拼接四轴组合协议并发送。
 */
static int MechanicalHand_SendJointCommand(const mech_hand_joint_pwm_t *pwm, uint16_t time_ms)
{
    mech_hand_joint_pwm_t limited_pwm;
    char cmd[MECH_HAND_CMD_BUFFER_SIZE];
    bool clamped = false;
    int status;

    if (NULL == pwm)
    {
        return MECH_HAND_ERR_PARAM;
    }

    MechanicalHand_ResetClampState();
    limited_pwm = *pwm;
    limited_pwm.servo0_pwm = MechanicalHand_ClampServoPwm(0U, limited_pwm.servo0_pwm, &clamped);
    g_mech_hand_last_cmd_clamped |= clamped;
    limited_pwm.servo1_pwm = MechanicalHand_ClampServoPwm(1U, limited_pwm.servo1_pwm, &clamped);
    g_mech_hand_last_cmd_clamped |= clamped;
    limited_pwm.servo2_pwm = MechanicalHand_ClampServoPwm(2U, limited_pwm.servo2_pwm, &clamped);
    g_mech_hand_last_cmd_clamped |= clamped;
    limited_pwm.servo3_pwm = MechanicalHand_ClampServoPwm(3U, limited_pwm.servo3_pwm, &clamped);
    g_mech_hand_last_cmd_clamped |= clamped;

    status = MechanicalHand_BuildJointCmd(&limited_pwm, time_ms, cmd, sizeof(cmd));
    if (MECH_HAND_OK != status)
    {
        return status;
    }

    MechanicalHand_UpdateJointPositions(&limited_pwm);
    return MechanicalHand_UartWrite(cmd);
}

/* 私有函数说明：解析标准单舵机协议 #IndexPpwmTtime!。 */
static int MechanicalHand_TryParseServoCommand(const char *cmd, uint8_t *index, uint16_t *pwm, uint16_t *time_ms)
{
    unsigned int temp_index;
    unsigned int temp_pwm;
    unsigned int temp_time;

    if ((NULL == cmd) || (NULL == index) || (NULL == pwm) || (NULL == time_ms))
    {
        return MECH_HAND_ERR_PARAM;
    }

    if (3 == sscanf(cmd, "#%3uP%4uT%4u!", &temp_index, &temp_pwm, &temp_time))
    {
        *index = (uint8_t)temp_index;
        *pwm = (uint16_t)temp_pwm;
        *time_ms = (uint16_t)temp_time;
        return MECH_HAND_OK;
    }

    return MECH_HAND_ERR_FORMAT;
}

/* 私有函数说明：解析标准四轴组合协议 {#000...#003...}。 */
static int MechanicalHand_TryParseJointCommand(const char *cmd, mech_hand_joint_pwm_t *pwm, uint16_t *time_ms)
{
    unsigned int s0;
    unsigned int t0;
    unsigned int s1;
    unsigned int t1;
    unsigned int s2;
    unsigned int t2;
    unsigned int s3;
    unsigned int t3;

    if ((NULL == cmd) || (NULL == pwm) || (NULL == time_ms))
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
        pwm->servo0_pwm = (uint16_t)s0;
        pwm->servo1_pwm = (uint16_t)s1;
        pwm->servo2_pwm = (uint16_t)s2;
        pwm->servo3_pwm = (uint16_t)s3;
        *time_ms = (uint16_t)t0;
        return MECH_HAND_OK;
    }

    return MECH_HAND_ERR_FORMAT;
}

/* 私有函数说明：把四轴 PWM 组装成标准组合舵机命令字符串。 */
static int MechanicalHand_BuildJointCmd(const mech_hand_joint_pwm_t *pwm, uint16_t time_ms, char *buf, uint32_t buf_size)
{
    int len;

    if ((NULL == pwm) || (NULL == buf))
    {
        return MECH_HAND_ERR_PARAM;
    }

    if ((MECH_HAND_OK != MechanicalHand_CheckPwm(pwm->servo0_pwm)) ||
        (MECH_HAND_OK != MechanicalHand_CheckPwm(pwm->servo1_pwm)) ||
        (MECH_HAND_OK != MechanicalHand_CheckPwm(pwm->servo2_pwm)) ||
        (MECH_HAND_OK != MechanicalHand_CheckPwm(pwm->servo3_pwm)) ||
        (MECH_HAND_OK != MechanicalHand_CheckTime(time_ms)))
    {
        return MECH_HAND_ERR_PARAM;
    }

    len = snprintf(buf,
                   buf_size,
                   "{#000P%04uT%04u!#001P%04uT%04u!#002P%04uT%04u!#003P%04uT%04u!}",
                   pwm->servo0_pwm,
                   time_ms,
                   pwm->servo1_pwm,
                   time_ms,
                   pwm->servo2_pwm,
                   time_ms,
                   pwm->servo3_pwm,
                   time_ms);

    if ((len <= 0) || ((uint32_t)len >= buf_size))
    {
        return MECH_HAND_ERR_FORMAT;
    }

    return MECH_HAND_OK;
}

/*
 * 私有函数说明：逆运动学单次求解
 * 1. 输入固定 alpha_deg。
 * 2. 输出该姿态下的四轴 PWM。
 * 3. 不发送命令，只负责求解。
 */
/* 补充说明：
 * 1. 这里对单个 alpha 姿态做一次逆运动学求解。
 * 2. x/y/z 输入单位为 mm，内部会统一放大 10 倍，再按 OpenMV 原始公式计算。
 * 3. 该函数只负责求解，不负责发送串口命令。
 */
static int MechanicalHand_KinematicsAnalysis(float x, float y, float z, float alpha_deg, mech_hand_joint_pwm_t *out_pwm)
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

    if (NULL == out_pwm)
    {
        return MECH_HAND_ERR_PARAM;
    }

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

    y = sqrtf((x * x) + (y * y));
    y = y - l3 * cosf(alpha_deg * MECH_HAND_PI / 180.0f);
    z = z - l0 - l3 * sinf(alpha_deg * MECH_HAND_PI / 180.0f);

    if (z < (-l0))
    {
        return MECH_HAND_IK_ERR_1;
    }

    if (sqrtf((y * y) + (z * z)) > (l1 + l2))
    {
        return MECH_HAND_IK_ERR_2;
    }

    ccc = acosf(y / sqrtf((y * y) + (z * z)));
    bbb = ((y * y) + (z * z) + (l1 * l1) - (l2 * l2)) / (2.0f * l1 * sqrtf((y * y) + (z * z)));
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

    out_pwm->servo0_pwm = (uint16_t)(1500.0f - 2000.0f * servo_angle0 / 270.0f);
    out_pwm->servo1_pwm = (uint16_t)(1500.0f + 2000.0f * servo_angle1 / 270.0f);
    out_pwm->servo2_pwm = (uint16_t)(1500.0f + 2000.0f * servo_angle2 / 270.0f);
    out_pwm->servo3_pwm = (uint16_t)(1500.0f + 2000.0f * servo_angle3 / 270.0f);

    if ((MECH_HAND_OK != MechanicalHand_CheckPwm(out_pwm->servo0_pwm)) ||
        (MECH_HAND_OK != MechanicalHand_CheckPwm(out_pwm->servo1_pwm)) ||
        (MECH_HAND_OK != MechanicalHand_CheckPwm(out_pwm->servo2_pwm)) ||
        (MECH_HAND_OK != MechanicalHand_CheckPwm(out_pwm->servo3_pwm)))
    {
        return MECH_HAND_ERR_PARAM;
    }

    return MECH_HAND_OK;
}

/*
 * 私有函数说明：逆运动学自动扫描求解
 * 1. 自动扫描 alpha=0 到 -135 度。
 * 2. 找到可达解后，再通过 SendJointCommand() 发出四轴动作。
 */
/* 补充说明：
 * 1. 这里会自动扫描 alpha=0 到 -135 度，寻找一个可达姿态。
 * 2. 如果坐标还在菜单允许输入范围内却不动作，常见原因不是菜单本身，而是：
 *    - 当前点对机械结构不可达
 *    - 求解后的 #000 ~ #003 PWM 被软件限位裁剪
 */
static int MechanicalHand_KinematicsMove(float x, float y, float z, uint16_t time_ms)
{
    int alpha_deg;
    int best_alpha = 0;
    bool found = false;
    int status;
    mech_hand_joint_pwm_t pwm;

    if (y < 0.0f)
    {
        return MECH_HAND_ERR_PARAM;
    }

    for (alpha_deg = 0; alpha_deg >= -135; alpha_deg--)
    {
        status = MechanicalHand_KinematicsAnalysis(x, y, z, (float)alpha_deg, &pwm);
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

    status = MechanicalHand_KinematicsAnalysis(x, y, z, (float)best_alpha, &pwm);
    if (MECH_HAND_OK != status)
    {
        return status;
    }

    return MechanicalHand_SendJointCommand(&pwm, time_ms);
}

/*
 * 回调说明：UART4 发送完成回调
 * 1. 当前只用于通知 MCU 侧发送完成。
 * 2. 不代表舵机控制板已经执行完成动作。
 */
void UART4_Callback(uart_callback_args_t *p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
    case UART_EVENT_TX_COMPLETE: {
        g_mech_hand_uart4_tx_complete = true;
        break;
    }

    case UART_EVENT_RX_COMPLETE:
    case UART_EVENT_RX_CHAR:
    case UART_EVENT_TX_DATA_EMPTY:
    case UART_EVENT_ERR_PARITY:
    case UART_EVENT_ERR_FRAMING:
    case UART_EVENT_ERR_OVERFLOW:
    case UART_EVENT_BREAK_DETECT:
    default: {
        break;
    }
    }
}
