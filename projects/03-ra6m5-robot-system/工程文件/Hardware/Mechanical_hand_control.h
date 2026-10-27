#ifndef MECHANICAL_HAND_CONTROL_H_
#define MECHANICAL_HAND_CONTROL_H_

#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * 机械手模块对外说明
 * 1. 本模块是“执行层”，负责把目标坐标或目标 PWM 转成 UART4 协议命令发送给机械手控制板。
 * 2. 本模块不负责摄像头采图、视觉识别、图像坐标计算，只负责执行已经换算好的目标。
 * 3. 坐标输入统一使用机械臂基座坐标系，单位为 mm。
 * 4. 当前工程中，上电初始化入口在 src/hal_entry.c 中，通过 MechanicalHand_Init() 完成机械手初始化。
 * 5. 上电后的默认姿态不是直接写“关节角度”，而是通过默认回零坐标触发逆运动学得到的。
 */

/*
 * 机械手模块对外接口说明
 * 1. 先在系统初始化完成后调用 MechanicalHand_Init()。
 * 2. 正常按坐标运动时，调用 MechanicalHand_MoveTo(x, y, z, time_ms)。
 * 3. 需要固定末端俯仰角时，调用 MechanicalHand_MoveToWithAlpha()。
 * 4. 需要单独测试某一路舵机时，调用 MechanicalHand_SendSingleServo()。
 * 5. 菜单调试页通过 MechanicalHand_GetServoPositions() 读取 6 路当前位置。
 *
 * 标定建议
 * 1. 推荐固定相机，把视觉结果先转换到机械臂基座坐标系。
 * 2. 采集多个标定点，记录图像坐标 (u, v) 和机械臂坐标 (x, y, z)。
 * 3. 先完成 (u, v) 到 (x, y) 的平面映射，z 先按任务固定。
 * 4. 标定完成后，视觉模块直接输出目标 x、y、z，本模块只负责执行。
 */

/* 机械手模块返回值约定：
 * 1. 0 表示执行成功。
 * 2. 负数表示模块自身错误，例如参数错误、串口未初始化、发送失败、超时等。
 * 3. 正数 1~7 表示逆运动学求解失败，语义与 OpenMV 原版保持一致。
 */
 
typedef enum {
    MECH_HAND_OK             = 0,
    MECH_HAND_ERR_PARAM      = -1,
    MECH_HAND_ERR_UART_OPEN  = -2,
    MECH_HAND_ERR_UART_WRITE = -3,
    MECH_HAND_ERR_NOT_READY  = -4,
    MECH_HAND_ERR_UART_CLOSE = -5,
    MECH_HAND_ERR_FORMAT     = -6,
    MECH_HAND_ERR_TIMEOUT    = -7,

    MECH_HAND_IK_ERR_1 = 1,
    MECH_HAND_IK_ERR_2 = 2,
    MECH_HAND_IK_ERR_3 = 3,
    MECH_HAND_IK_ERR_4 = 4,
    MECH_HAND_IK_ERR_5 = 5,
    MECH_HAND_IK_ERR_6 = 6,
    MECH_HAND_IK_ERR_7 = 7
} mech_hand_status_t;

/*
 * 返回码说明
 * 1. 0 表示执行成功。
 * 2. 负数表示模块级错误，例如参数错误、UART 未准备好、格式错误、超时等。
 * 3. 1~7 表示逆运动学错误码，含义沿用 OpenMV 版本。
 * 4. 如果菜单里看到 RET=1~7，优先怀疑当前 x/y/z 不可达，而不是串口问题。
 */

typedef struct
{
    uint16_t servo0_pwm;
    uint16_t servo1_pwm;
    uint16_t servo2_pwm;
    uint16_t servo3_pwm;
} mech_hand_joint_pwm_t;

/*
 * 结构体说明：
 * 名称：mech_hand_joint_pwm_t
 * 参数字段：
 * 1. servo0_pwm：主关节 0 的目标 PWM，对应通道 #000。
 * 2. servo1_pwm：主关节 1 的目标 PWM，对应通道 #001。
 * 3. servo2_pwm：主关节 2 的目标 PWM，对应通道 #002。
 * 4. servo3_pwm：主关节 3 的目标 PWM，对应通道 #003。
 * 返回值：无，本结构体仅作为数据载体使用。
 * 调用链：
 * 1. MechanicalHand_MoveTo() / MechanicalHand_MoveToWithAlpha() 内部先得到该结构体。
 * 2. 然后由 4 轴组合发送函数统一下发。
 */

typedef struct
{
    uint16_t servo_pwm[6];
} mech_hand_servo_positions_t;

/*
 * 结构体说明：
 * 名称：mech_hand_servo_positions_t
 * 参数字段：
 * 1. servo_pwm[0] ~ servo_pwm[5] 分别对应通道 #000 ~ #005。
 * 返回值：无，本结构体仅作为数据载体使用。
 * 调用链：
 * 1. 机械手模块内部在每次成功下发命令前后更新该缓存。
 * 2. 菜单调试页通过 MechanicalHand_GetServoPositions() 读取该结构体。
 * 注意：
 * 1. 这里保存的是软件最近一次下发的目标 PWM。
 * 2. 这不是编码器反馈值，也不是舵机真实闭环位置。
 */

/* 6 路舵机独立软件限位。
 * 后续如果某一路仍有发热或逼近机械极限，优先修改这里的最小值和最大值。
 */

#define MECH_HAND_SERVO0_PWM_MIN (700U)
#define MECH_HAND_SERVO0_PWM_MAX (2300U)
#define MECH_HAND_SERVO1_PWM_MIN (750U)
#define MECH_HAND_SERVO1_PWM_MAX (2250U)
#define MECH_HAND_SERVO2_PWM_MIN (750U)
#define MECH_HAND_SERVO2_PWM_MAX (2250U)
#define MECH_HAND_SERVO3_PWM_MIN (750U)
#define MECH_HAND_SERVO3_PWM_MAX (2250U)
#define MECH_HAND_SERVO4_PWM_MIN (1000U)
#define MECH_HAND_SERVO4_PWM_MAX (2000U)
#define MECH_HAND_SERVO5_PWM_MIN (1100U)
#define MECH_HAND_SERVO5_PWM_MAX (1600U)

/*
 * 机械手菜单坐标输入限幅
 * 1. 这组宏只用于菜单调试界面的输入范围限制，防止编码器把坐标调得过大。
 * 2. 这不是机械臂的真实可达范围。即便坐标落在这个范围内，
 *    仍可能因为逆运动学不可达、Alpha 扫描失败或关节软件限位而无法运动。
 * 3. menu.c 引用下方宏限制 X/Y/Z 的调节范围。
 */

#define MECH_HAND_MENU_X_MIN (-250.0f)
#define MECH_HAND_MENU_X_MAX (200.0f)
#define MECH_HAND_MENU_Y_MIN (0.0f)
#define MECH_HAND_MENU_Y_MAX (300.0f)
#define MECH_HAND_MENU_Z_MIN (-100.0f)
#define MECH_HAND_MENU_Z_MAX (200.0f)

/*
 * 输入坐标约束：目标 x/y/z 使用机械臂基座坐标系，单位 mm。
 * 图像坐标及深度需先由 Calibration 模块转换；本模块只执行目标运动。
 */


/*
 * 默认回零位说明
 * 1. 这是上电初始化后自动执行的默认目标坐标，单位 mm。
 * 2. 当前默认上电姿态就是由这 3 个坐标决定的。
 * 3. 如果想改上电默认姿态，优先修改这里。
 * 4. 当前默认 Home 坐标为：X=0, Y=100, Z=70。
 * 5. 这里写的是末端目标坐标，不是直接写死的关节角度。
 */
#define MECH_HAND_HOME_X (-20.0f)
#define MECH_HAND_HOME_Y (69.0f)
#define MECH_HAND_HOME_Z (85.0f)

/*
 * 函数说明：
 * 名称：MechanicalHand_Init
 * 作用：
 * 1. 打开 UART4。
 * 2. 清空最近一次错误码和限位裁剪状态。
 * 3. 执行一次默认 Home 姿态动作。
 * 参数：无。
 * 返回值：
 * 1. 1：初始化成功。
 * 2. 0：初始化失败。
 * 调用链：
 * hal_entry() -> MechanicalHand_Init() -> MechanicalHand_Home() -> MechanicalHand_MoveTo()
 */
uint8_t MechanicalHand_Init(void);

/*
 * 函数说明：
 * 名称：MechanicalHand_Deinit
 * 作用：关闭本模块使用的 UART4，并清除模块初始化标记。
 * 参数：无。
 * 返回值：无。
 * 调用链：停机流程/重新初始化流程 -> MechanicalHand_Deinit()
 */
void MechanicalHand_Deinit(void);

/*
 * 函数说明：
 * 名称：MechanicalHand_MoveTo
 * 作用：输入目标 x/y/z 后，自动扫描末端俯仰角并执行坐标运动。
 * 参数：
 * 1. x：机械臂基座坐标系下的 X 坐标，单位 mm。
 * 2. y：机械臂基座坐标系下的 Y 坐标，单位 mm。
 * 3. z：机械臂基座坐标系下的 Z 坐标，单位 mm。
 * 4. time_ms：动作时间，单位 ms。
 * 当前菜单调试推荐输入范围：
 * 1. X：-200 ~ 200 mm
 * 2. Y：0 ~ 250 mm
 * 3. Z：0 ~ 200 mm
 * 注意：
 * 1. 这只是当前菜单调试页采用的软件输入范围。
 * 2. 处于这个范围内的点不一定全部可达，仍可能触发逆运动学错误码或关节限位裁剪。
 * 返回值：
 * 1. 0：执行成功。
 * 2. 负数：模块级错误。
 * 3. 1 ~ 7：逆运动学错误码。
 * 调用链：
 * 视觉模块/菜单 -> MechanicalHand_MoveTo() -> MechanicalHand_KinematicsMove() -> UART4 发送
 */
int MechanicalHand_MoveTo(float x, float y, float z, uint16_t time_ms);

/*
 * 函数说明：
 * 名称：MechanicalHand_MoveToWithAlpha
 * 作用：输入目标 x/y/z，并强制指定末端俯仰角 alpha 后执行运动。
 * 参数：
 * 1. x/y/z：机械臂基座坐标系下的目标点，单位 mm。
 * 2. alpha_deg：末端俯仰角，单位度。
 * 3. time_ms：动作时间，单位 ms。
 * 返回值：
 * 1. 0：执行成功。
 * 2. 负数：模块级错误。
 * 3. 1 ~ 7：逆运动学错误码。
 * 调用链：
 * 上层业务 -> MechanicalHand_MoveToWithAlpha() -> MechanicalHand_KinematicsAnalysis() -> UART4 发送
 */
int MechanicalHand_MoveToWithAlpha(float x, float y, float z, float alpha_deg, uint16_t time_ms);

/*
 * 函数说明：
 * 名称：MechanicalHand_SendSingleServo
 * 作用：直接发送单路舵机命令，不经过逆运动学。
 * 参数：
 * 1. index：舵机编号，当前机械手使用 #000 ~ #005。
 * 2. pwm：目标 PWM，会先经过该通道的软件限位裁剪。
 * 3. time_ms：动作时间，单位 ms。
 * 返回值：
 * 1. 0：发送成功。
 * 2. 负数：模块级错误。
 * 调用链：
 * 菜单/调试接口 -> MechanicalHand_SendSingleServo() -> UART4 发送
 */
int MechanicalHand_SendSingleServo(uint8_t index, uint16_t pwm, uint16_t time_ms);

/*
 * 函数说明：
 * 名称：MechanicalHand_SendMultiServo
 * 作用：直接发送 4 轴组合命令，对应 #000 ~ #003。
 * 参数：
 * 1. pwm0 ~ pwm3：4 路主关节目标 PWM。
 * 2. time_ms：动作时间，单位 ms。
 * 返回值：
 * 1. 0：发送成功。
 * 2. 负数：模块级错误。
 * 调用链：
 * 调试页/上层业务 -> MechanicalHand_SendMultiServo() -> MechanicalHand_SendJointCommand()
 */
int MechanicalHand_SendMultiServo(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t time_ms);
/*
 * 函数说明：
 * 名称：MechanicalHand_SendRawDebug
 * 作用：发送原始字符串协议用于调试。
 * 参数：
 * 1. cmd：待发送的协议字符串。
 * 返回值：
 * 1. 0：发送成功。
 * 2. 负数：模块级错误。
 * 调用链：
 * 调试页/串口测试 -> MechanicalHand_SendRawDebug() -> 解析已知格式并限位 -> UART4 发送
 */
int MechanicalHand_SendRawDebug(const char *cmd);

/*
 * 函数说明：
 * 名称：MechanicalHand_GripperRotate
 * 作用：控制 #004 夹爪旋转舵机。
 * 参数：
 * 1. pwm：目标 PWM。
 * 2. time_ms：动作时间，单位 ms。
 * 返回值：
 * 1. 0：发送成功。
 * 2. 负数：模块级错误。
 * 调用链：上层业务/调试页 -> MechanicalHand_GripperRotate() -> MechanicalHand_SendSingleServo()
 */
int MechanicalHand_GripperRotate(uint16_t pwm, uint16_t time_ms);

/*
 * 函数说明：
 * 名称：MechanicalHand_GripperRotateCenter
 * 作用：把 #004 旋转舵机回到默认中心位。
 * 参数：
 * 1. time_ms：动作时间，单位 ms。
 * 返回值：
 * 1. 0：发送成功。
 * 2. 负数：模块级错误。
 * 调用链：上层业务 -> MechanicalHand_GripperRotateCenter() -> MechanicalHand_GripperRotate()
 */
int MechanicalHand_GripperRotateCenter(uint16_t time_ms);

/*
 * 函数说明：
 * 名称：MechanicalHand_GripperOpen
 * 作用：执行默认夹爪张开动作，对应 #005。
 * 参数：
 * 1. time_ms：动作时间，单位 ms。
 * 返回值：
 * 1. 0：发送成功。
 * 2. 负数：模块级错误。
 * 调用链：上层业务/调试页 -> MechanicalHand_GripperOpen() -> MechanicalHand_SendSingleServo()
 */
int MechanicalHand_GripperOpen(uint16_t time_ms);

/*
 * 函数说明：
 * 名称：MechanicalHand_GripperClose
 * 作用：执行默认夹爪闭合动作，对应 #005。
 * 参数：
 * 1. time_ms：动作时间，单位 ms。
 * 返回值：
 * 1. 0：发送成功。
 * 2. 负数：模块级错误。
 * 调用链：上层业务/调试页 -> MechanicalHand_GripperClose() -> MechanicalHand_SendSingleServo()
 */
int MechanicalHand_GripperClose(uint16_t time_ms);

/*
 * 函数说明：
 * 名称：MechanicalHand_StopAll
 * 作用：发送 $DST!，让所有舵机停在当前位置。
 * 参数：无。
 * 返回值：
 * 1. 0：发送成功。
 * 2. 负数：模块级错误。
 * 调用链：上层业务 -> MechanicalHand_StopAll() -> UART4 发送
 */
int MechanicalHand_StopAll(void);

/*
 * 函数说明：
 * 名称：MechanicalHand_StopServo
 * 作用：发送 $DST:x!，让指定舵机停在当前位置。
 * 参数：
 * 1. index：待停止的舵机编号。
 * 返回值：
 * 1. 0：发送成功。
 * 2. 负数：模块级错误。
 * 调用链：上层业务 -> MechanicalHand_StopServo() -> UART4 发送
 */
int MechanicalHand_StopServo(uint8_t index);

/*
 * 函数说明：
 * 名称：MechanicalHand_Home
 * 作用：执行默认 Home 姿态动作。
 * 参数：
 * 1. time_ms：动作时间，单位 ms。
 * 返回值：
 * 1. 0：执行成功。
 * 2. 负数：模块级错误。
 * 3. 1 ~ 7：逆运动学错误码。
 * 调用链：
 * MechanicalHand_Init() -> MechanicalHand_Home() -> MechanicalHand_MoveTo(MECH_HAND_HOME_X, MECH_HAND_HOME_Y, MECH_HAND_HOME_Z, ...)
 */
int MechanicalHand_Home(uint16_t time_ms);

/*
 * 函数说明：
 * 名称：MechanicalHand_GetLastError
 * 作用：读取最近一次动作返回码。
 * 参数：无。
 * 返回值：最近一次动作的返回码。
 * 调用链：菜单/上层状态显示 -> MechanicalHand_GetLastError()
 */
int MechanicalHand_GetLastError(void);
/*
 * 函数说明：
 * 名称：MechanicalHand_GetLastCommand
 * 作用：返回最近一次通过 UART4 发送的原始命令字符串。
 * 参数：无。
 * 返回值：指向内部缓存字符串的只读指针。
 * 调用链：菜单调试页 -> MechanicalHand_GetLastCommand()
 */
const char *MechanicalHand_GetLastCommand(void);
/*
 * 函数说明：
 * 名称：MechanicalHand_GetServoPositions
 * 作用：读取当前缓存的 6 路舵机目标 PWM。
 * 参数：
 * 1. positions：输出结构体指针，函数会把 S0 ~ S5 当前缓存写入这里。
 * 返回值：无。
 * 调用链：菜单调试页 -> MechanicalHand_GetServoPositions()
 */
void MechanicalHand_GetServoPositions(mech_hand_servo_positions_t *positions);
/*
 * 函数说明：
 * 名称：MechanicalHand_WasLastCommandClamped
 * 作用：判断最近一次结构化命令是否触发过软件限位裁剪。
 * 参数：无。
 * 返回值：
 * 1. true：最近一次命令触发过限位裁剪。
 * 2. false：最近一次命令未触发限位裁剪。
 * 调用链：菜单调试页 -> MechanicalHand_WasLastCommandClamped()
 */
bool MechanicalHand_WasLastCommandClamped(void);

#endif
