#include "Robot_Control.h"

#include <string.h>

#include "Calibration.h"
#include "GPT.h"
#include "Mechanical_hand_control.h"
#include "Navigation.h"
#include "SysTick.h"
#include "UART.h"

/*
 * Robot_Control 模块实现说明
 *
 * 1. 模块职责
 *    协调 UART9 视觉输入、Navigation、机械臂和夹爪状态机。
 *    主循环消费视觉帧并推进动作，10ms 周期执行底盘控制。
 *
 * 2. 当前完整业务链
 *    - 循迹模式：
 *      普通视觉帧中的 u 作为左轮目标速度，v 作为右轮目标速度，yaw 作为循迹偏差。
 *      主循环中把视觉数据送给 Navigation，10ms 周期里真正执行底盘控制。
 *
 *    - 机械臂模式切换：
 *      收到 9999,9999,9999,9999 后进入机械臂模式。
 *      进入该模式后：
 *      1) 小车停车
 *      2) 屏蔽雷达/避障分支
 *      3) 停止把视觉循迹数据继续送给底盘
 *
 *    - 抓取流程：
 *      在机械臂模式下，若当前夹爪为空，等待 u,v,depth,9988。
 *      收到后执行：
 *      目标坐标转换 -> 到目标上方 -> 张爪 -> 下探 -> 闭合 -> 抬起 -> 回默认位 -> 恢复循迹
 *
 *    - 放置流程：
 *      抓取完成并继续循迹后，再次收到 9999 进入机械臂模式。
 *      若当前夹爪已夹住物体，则等待 u,v,depth,8899。
 *      收到后执行：
 *      目标坐标转换 -> 到放置点上方 -> 下探 -> 张爪释放 -> 抬起 -> 停在预留状态
 *
 * 3. 模块边界
 *    - Vision_Uart9_Comm：UART9 收帧与解帧。
 *    - Navigation：循迹控制与底盘输出。
 *    - Mechanical_hand_control：机械臂与夹爪动作执行。
 *    - Calibration：视觉坐标到机械臂基座坐标的转换。
 *
 * 4. 内部入口
 *    - RobotControl_ProcessVisionInput()：视觉字段解释与特殊帧分流。
 *    - RobotControl_ProcessArmTarget()：机械臂状态机推进。
 *    - RobotControl_ProcessGripperRequest()：手动夹爪请求。
 *    - RobotControl_RunDriveControl()：底盘执行与停机条件。
 */

/*
 * 夹爪请求枚举
 *
 * 作用：
 * - 表示“上层想让夹爪做什么”，属于主循环侧的请求缓存。
 * - 对外接口只负责写入请求，真正执行在 RobotControl_ProcessGripperRequest()。
 */
typedef enum
{
    /* 当前没有挂起的夹爪请求。 */
    ROBOT_CTRL_GRIPPER_REQ_NONE = 0,
    /* 请求后续执行一次张爪动作。 */
    ROBOT_CTRL_GRIPPER_REQ_OPEN,
    /* 请求后续执行一次闭爪动作。 */
    ROBOT_CTRL_GRIPPER_REQ_CLOSE,
    /* 请求后续根据当前夹爪状态自动切换开/合。 */
    ROBOT_CTRL_GRIPPER_REQ_TOGGLE
} robot_control_gripper_req_t;

/*
 * 控制模式枚举
 *
 * 作用：
 * - 表示整车当前处于“循迹模式”还是“机械臂模式”。
 * - 这是整个 Robot_Control 的最高层模式开关。
 */
typedef enum
{
    /* 普通循迹模式：允许视觉继续喂给 Navigation。 */
    ROBOT_CTRL_MODE_LINE_TRACK = 0,
    /* 机械臂模式：小车停车，主循环转而推进抓取/放置流程。 */
    ROBOT_CTRL_MODE_ARM_GRAB
} robot_control_mode_t;

/*
 * 机械臂状态机枚举
 *
 * 说明：
 * - 只有 control_mode == ROBOT_CTRL_MODE_ARM_GRAB 时，这个状态机才会被推进。
 * - 每个状态只做当前阶段该做的动作，动作成功后切到下一个状态，失败则进入 ERROR。
 */
typedef enum
{
    /* 空闲态：当前没有抓取/放置任务。 */
    ROBOT_CTRL_ARM_STATE_IDLE = 0,

    /* 已进入机械臂模式，等待视觉发送抓取目标帧 u,v,depth,9988。 */
    ROBOT_CTRL_ARM_STATE_WAIT_PICK_TARGET,
    /* 已拿到抓取目标坐标，准备先移动到目标上方的安全高度。 */
    ROBOT_CTRL_ARM_STATE_MOVE_ABOVE_PICK_TARGET,
    /* 已到达目标上方，先张开夹爪，为下探抓取做准备。 */
    ROBOT_CTRL_ARM_STATE_OPEN_GRIPPER_BEFORE_PICK,
    /* 张爪后向下移动到目标位置。 */
    ROBOT_CTRL_ARM_STATE_MOVE_DOWN_TO_PICK_TARGET,
    /* 到达目标位置后闭合夹爪夹取物体。 */
    ROBOT_CTRL_ARM_STATE_CLOSE_GRIPPER_AFTER_PICK,
    /* 抓取完成后抬起到安全高度，避免拖碰物体。 */
    ROBOT_CTRL_ARM_STATE_LIFT_AFTER_PICK,
    /* 抬起后返回机械臂默认位。 */
    ROBOT_CTRL_ARM_STATE_RETURN_HOME_AFTER_PICK,
    /* 返回默认位后恢复循迹模式，让小车带着物体继续运行。 */
    ROBOT_CTRL_ARM_STATE_RESUME_LINE_TRACK_AFTER_PICK,

    /* 已进入机械臂模式且当前夹爪已闭合，等待放置目标帧 u,v,depth,8899。 */
    ROBOT_CTRL_ARM_STATE_WAIT_PLACE_TARGET,
    /* 已拿到放置目标坐标，先移动到放置点上方。 */
    ROBOT_CTRL_ARM_STATE_MOVE_ABOVE_PLACE_TARGET,
    /* 从放置点上方向下移动到实际放置高度。 */
    ROBOT_CTRL_ARM_STATE_MOVE_DOWN_TO_PLACE_TARGET,
    /* 到达放置点后张开夹爪释放物体。 */
    ROBOT_CTRL_ARM_STATE_OPEN_GRIPPER_TO_PLACE,
    /* 放置完成后抬离目标位置。 */
    ROBOT_CTRL_ARM_STATE_LIFT_AFTER_PLACE,
    /* 放置完成后保持当前状态，不自动恢复循迹。 */
    ROBOT_CTRL_ARM_STATE_AFTER_PLACE_HOLD,

    /* 任意阶段动作失败后进入的错误态。 */
    ROBOT_CTRL_ARM_STATE_ERROR,
    /* 等待上一条机械臂动作执行到位，避免连续指令重叠。 */
    ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE
} robot_control_arm_state_t;

/*
 * 特殊视觉标志位宏
 *
 * 说明：
 * - 这些宏不是普通视觉测量值，而是上层协议约定的“业务标志帧”。
 * - 它们通过 vision_uart9_frame_t 的数值字段传进来，由本模块解释业务含义。
 */

/* 9999,9999,9999,9999：进入机械臂模式并停车。 */
#define ROBOT_CTRL_VISION_ARM_MODE_MARK (9999)
/* u,v,depth,9988：表示当前帧是抓取目标帧。 */
#define ROBOT_CTRL_VISION_ARM_PICK_MARK (9988)
/* u,v,depth,8899：表示当前帧是放置目标帧。 */
#define ROBOT_CTRL_VISION_ARM_PLACE_MARK (8899)
#define ROBOT_CTRL_VISION_PICK_DONE_ACK_MARK (9889)
#define ROBOT_CTRL_PICK_DONE_RESEND_INTERVAL_MS (200U)

/*
 * 机械臂流程参数宏
 *
 * 这些宏都是“动作流程参数”，不是底层驱动参数。
 * 后续调试抓取/放置效果时，优先改这里。
 */

/*
 * 机械臂接近目标时，在目标 z 之上额外抬高的安全高度，单位 mm。
 *
 * 调大：
 * - 更不容易碰撞目标
 * - 但动作路径更长、总时间更慢
 *
 * 调小：
 * - 动作更快
 * - 但接近时更容易擦碰目标或放置区域
 */
#define ROBOT_CTRL_ARM_APPROACH_HEIGHT_MM (60.0f)

/*
 * 单次机械臂 MoveTo 动作的规划时间，单位 ms。
 *
 * 调大：
 * - 动作更慢、更平稳
 * - 整个流程总时间变长
 *
 * 调小：
 * - 动作更快
 * - 若机械臂跟随能力不足，可能造成动作不稳或不到位
 */
#define ROBOT_CTRL_ARM_MOVE_TIME_MS (1000U)

/*
 * 夹爪开合动作保持时间，单位 ms。
 *
 * 调大：
 * - 更有利于确保张爪/闭爪到位
 * - 但流程耗时变长
 *
 * 调小：
 * - 流程更快
 * - 若机构较慢，可能出现夹爪未完全动作就进入下一步
 */
#define ROBOT_CTRL_ARM_GRIPPER_TIME_MS (1000U)
/* 机械臂位移动作的额外等待裕量，便于调试时观察状态切换。 */
#define ROBOT_CTRL_ARM_MOVE_SETTLE_MS (150U)
/* 夹爪开合动作的额外等待裕量，防止下一条命令过早覆盖。 */
#define ROBOT_CTRL_ARM_GRIPPER_SETTLE_MS (100U)

/*
 * 协调层上下文
 *
 * 保存跨控制周期使用的视觉帧、整车模式、机械臂状态机阶段和目标坐标。
 * 字段的更新与读取由下方协调层接口完成。
 */
typedef struct
{
    /* 最近一次缓存的完整视觉帧。由 RobotControl_UpdateVisionFrame() 写入。 */
    vision_uart9_frame_t latest_vision_frame;
    /* 是否至少已经缓存过一帧有效视觉数据。 */
    uint8_t has_valid_vision_frame;

    /*
     * 是否允许把视觉循迹输入继续送入 Navigation。
     * 1：允许。
     * 0：禁止。
     * 典型场景：进入机械臂模式时关闭，恢复循迹时重新打开。
     */
    uint8_t line_track_enabled;

    /* 当前整车控制模式。 */
    robot_control_mode_t control_mode;
    /* 当前机械臂状态机所处阶段。 */
    robot_control_arm_state_t arm_state;

    /* 最近一次坐标转换得到的机械臂基座系目标 x，单位 mm。 */
    float arm_target_x_mm;
    /* 最近一次坐标转换得到的机械臂基座系目标 y，单位 mm。 */
    float arm_target_y_mm;
    /* 最近一次坐标转换得到的机械臂基座系目标 z，单位 mm。 */
    float arm_target_z_mm;
    /* 当前 arm_target_* 是否有效。 */
    uint8_t has_valid_arm_target;
    /* 最近一次坐标转换或机械臂移动相关的返回结果。 */
    int last_arm_target_result;
    /* 机械臂是否正在等待上一条动作完成。 */
    uint8_t arm_wait_active;
    /* 等待结束后要切换到的下一状态。 */
    robot_control_arm_state_t arm_wait_next_state;
    /* 当前等待动作的截止时刻，单位 ms。 */
    uint32_t arm_wait_deadline_ms;

    /* 当前挂起的夹爪请求，由对外请求接口写入。 */
    robot_control_gripper_req_t pending_gripper_req;
    /*
     * 夹爪当前软件状态：
     * 0：软件认为夹爪张开。
     * 1：软件认为夹爪闭合。
     * 该值由夹爪动作成功后更新，不是硬件实时反馈。
     */
    uint8_t gripper_is_closed;
    /* 最近一次夹爪动作的返回结果。 */
    int last_gripper_result;
    uint8_t pick_done_wait_ack;
    uint32_t pick_done_last_send_ms;
} robot_control_context_t;

/* Robot_Control 模块唯一的内部上下文实例。 */
static robot_control_context_t g_robot_control_ctx = {0};

/*
 * 私有函数声明
 *
 * 说明：
 * - 这些函数只在本文件内部使用。
 * - 通过拆分这些函数，可以让主循环入口和 10ms 入口保持简洁。
 */
static void RobotControl_ProcessVisionInput(void);
static void RobotControl_ProcessGripperRequest(void);
static void RobotControl_ProcessArmTarget(void);
static void RobotControl_RunDriveControl(void);
static void RobotControl_ResetArmWait(void);
static void RobotControl_StartArmWait(uint32_t wait_ms, robot_control_arm_state_t next_state);
static uint8_t RobotControl_IsArmWaitDone(void);
static uint32_t RobotControl_GetArmWaitRemainingMsInternal(void);
static uint8_t RobotControl_IsArmModeSwitchFrame(const vision_uart9_frame_t *frame);
static uint8_t RobotControl_IsArmPickFrame(const vision_uart9_frame_t *frame);
static uint8_t RobotControl_IsArmPlaceFrame(const vision_uart9_frame_t *frame);
static uint8_t RobotControl_IsPickDoneAckFrame(const vision_uart9_frame_t *frame);
static float RobotControl_FilterYaw(float raw_yaw); // 偏航角滤波
static int RobotControl_FilterSpeedLeft(int raw_speed);
static int RobotControl_FilterSpeedRight(int raw_speed);
static void RobotControl_SendPickDoneFlag(void);
static void RobotControl_ProcessPickDoneAckTx(void);

static void RobotControl_ResetArmWait(void)
{
    g_robot_control_ctx.arm_wait_active = 0U;
    g_robot_control_ctx.arm_wait_next_state = ROBOT_CTRL_ARM_STATE_IDLE;
    g_robot_control_ctx.arm_wait_deadline_ms = 0U;
}

static void RobotControl_StartArmWait(uint32_t wait_ms, robot_control_arm_state_t next_state)
{
    g_robot_control_ctx.arm_wait_active = 1U;
    g_robot_control_ctx.arm_wait_next_state = next_state;
    g_robot_control_ctx.arm_wait_deadline_ms = HAL_GetTick() + wait_ms;
}

static uint8_t RobotControl_IsArmWaitDone(void)
{
    if (0U == g_robot_control_ctx.arm_wait_active)
    {
        return 1U;
    }

    return ((int32_t)(HAL_GetTick() - g_robot_control_ctx.arm_wait_deadline_ms) >= 0) ? 1U : 0U;
}

static uint32_t RobotControl_GetArmWaitRemainingMsInternal(void)
{
    uint32_t now_ms;

    if (0U == g_robot_control_ctx.arm_wait_active)
    {
        return 0U;
    }

    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - g_robot_control_ctx.arm_wait_deadline_ms) >= 0)
    {
        return 0U;
    }

    return g_robot_control_ctx.arm_wait_deadline_ms - now_ms;
}

void RobotControl_Init(void)
{
    /* 先清空整个上下文，避免旧状态残留。 */
    memset(&g_robot_control_ctx, 0, sizeof(g_robot_control_ctx));

    /*
     * 默认上电后处于正常循迹模式。
     * 这表示视觉 yaw 可以继续送给 Navigation。
     */
    g_robot_control_ctx.line_track_enabled = 1U;
    g_robot_control_ctx.control_mode = ROBOT_CTRL_MODE_LINE_TRACK;
    g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_IDLE;

    /*
     * 夹爪软件状态默认认为“张开”。
     * 这里是软件记忆值，不是读取硬件传感器得到的真实反馈。
     */
    g_robot_control_ctx.gripper_is_closed = 0U;

    /*
     * 初始化机械臂目标缓存和最近返回值。
     * CALIB_ERR_PARAM 只是一个占位初值，表示目前还没有成功执行过坐标转换。
     */
    g_robot_control_ctx.has_valid_arm_target = 0U;
    g_robot_control_ctx.last_arm_target_result = CALIB_ERR_PARAM;
    RobotControl_ResetArmWait();

    /* 初始化最近一次夹爪执行结果。 */
    g_robot_control_ctx.last_gripper_result = MECH_HAND_OK;
}

void RobotControl_MainLoopTask(void)
{
    float avoid_done_data[4];

    /*
     * 解析雷达数据。
     * 当前调用这个函数的目的只是保持感知数据更新，真正是否采用雷达逻辑由 Navigation 决定。
     */
    Lidar_Process_Data();

    /*
     * 解析 UART9 收到的视觉字节流，完成组帧。
     * 这里仅负责让 Vision_Uart9_Comm 产生“最新完整帧”，不解释字段业务含义。
     */
    VisionUart9_Process();
    RobotControl_ProcessPickDoneAckTx();

    if (0U != Navigation_FetchAvoidExitEvent())
    {
        avoid_done_data[0] = 8888.0f;
        avoid_done_data[1] = 8888.0f;
        avoid_done_data[2] = 8888.0f;
        avoid_done_data[3] = 8888.0f;
        (void)VisionUart9_Send4(avoid_done_data, 4U);
    }

    /* 消费最新视觉帧，并根据当前模式决定是用于循迹还是用于机械臂模式切换。 */
    RobotControl_ProcessVisionInput();

    /* 若当前处于机械臂模式，则推进抓取/放置状态机。 */
    RobotControl_ProcessArmTarget();

    /* 执行外部登记的夹爪请求。机械臂模式下这里会主动旁路，避免与自动抓取冲突。 */
    RobotControl_ProcessGripperRequest();
}

void RobotControl_TimerTask_10ms(void)
{
    /*
     * 每 10ms 更新一次左右编码器测速结果。
     * 这里传入的 10U 必须与 GPT 的实际调度周期保持一致。
     */
    Encoder_Update_Speed(&g_left_encoder, 10U);
    Encoder_Update_Speed(&g_right_encoder, 10U);

    /* 在快周期中执行一次底盘输出。 */
    RobotControl_RunDriveControl();
}

void RobotControl_SetLineTrackEnable(uint8_t enable)
{
    g_robot_control_ctx.line_track_enabled = (enable != 0U) ? 1U : 0U;
}

uint8_t RobotControl_GetLineTrackEnable(void)
{
    return g_robot_control_ctx.line_track_enabled;
}

int RobotControl_UpdateVisionFrame(const vision_uart9_frame_t *frame)
{
    if (NULL == frame)
    {
        return VISION_UART9_ERR_PARAM;
    }

    /* 把最新视觉帧复制到协调层自己的缓存里。 */
    memcpy(&g_robot_control_ctx.latest_vision_frame, frame, sizeof(*frame));
    g_robot_control_ctx.has_valid_vision_frame = 1U;

    return VISION_UART9_OK;
}

int RobotControl_GetLastVisionFrame(vision_uart9_frame_t *frame)
{
    if (NULL == frame)
    {
        return VISION_UART9_ERR_PARAM;
    }

    if (0U == g_robot_control_ctx.has_valid_vision_frame)
    {
        return VISION_UART9_ERR_NOT_READY;
    }

    memcpy(frame, &g_robot_control_ctx.latest_vision_frame, sizeof(*frame));
    return VISION_UART9_OK;
}

void RobotControl_RequestGripperOpen(void)
{
    /* 这里只登记请求，不立即动作。 */
    g_robot_control_ctx.pending_gripper_req = ROBOT_CTRL_GRIPPER_REQ_OPEN;
}

void RobotControl_RequestGripperClose(void)
{
    /* 这里只登记请求，不立即动作。 */
    g_robot_control_ctx.pending_gripper_req = ROBOT_CTRL_GRIPPER_REQ_CLOSE;
}

void RobotControl_RequestGripperToggle(void)
{
    /* 这里只登记请求，不立即动作。 */
    g_robot_control_ctx.pending_gripper_req = ROBOT_CTRL_GRIPPER_REQ_TOGGLE;
}

int RobotControl_GetLastGripperResult(void)
{
    return g_robot_control_ctx.last_gripper_result;
}

int RobotControl_GetLastArmTargetResult(void)
{
    return g_robot_control_ctx.last_arm_target_result;
}

int RobotControl_GetArmStateRaw(void)
{
    return (int)g_robot_control_ctx.arm_state;
}

uint8_t RobotControl_IsArmWaiting(void)
{
    return g_robot_control_ctx.arm_wait_active;
}

uint32_t RobotControl_GetArmWaitRemainingMs(void)
{
    return RobotControl_GetArmWaitRemainingMsInternal();
}

int RobotControl_TestSimulateLineTrackOnce(float u, float v, float depth, float yaw, int target_speed)
{
    int status;

    /*
     * 测试前强制恢复为循迹模式，避免被上一次机械臂流程残留状态影响。
     */
    g_robot_control_ctx.control_mode = ROBOT_CTRL_MODE_LINE_TRACK;
    g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_IDLE;
    g_robot_control_ctx.line_track_enabled = 1U;
    RobotControl_ResetArmWait();
    g_nav_target_speed = target_speed;
    g_car_state = CAR_STATE_RUN;

    /* 测试循迹时恢复雷达分支默认状态。 */
    Navigation_SetBadgeEnable(0U);

    /*
     * 向 Vision_Uart9_Comm 注入一帧假的视觉数据。
     * 这样后续主循环仍然走正常的“接收 -> 解析 -> 消费”链路。
     */
    status = VisionUart9_TestInjectFrame(u, v, depth, yaw);
    if (VISION_UART9_OK != status)
    {
        return status;
    }

    RobotControl_MainLoopTask();
    RobotControl_TimerTask_10ms();

    return VISION_UART9_OK;
}

/*
 * 对视觉 yaw 做滤波。
 *
 * 当前策略：
 * 1. 先做 5 点中值滤波，用来压掉偶发尖峰。
 * 2. 再做一次一阶低通，让舵机输入更平滑。
 *
 * 为什么这样做：
 * - 仅用低通时，异常点会被拖成“长尾”，对当前这种跳变数据不够稳。
 * - 先做中值，再做轻低通，更适合当前视觉串口里偶发坏点的情况。
 */
static float RobotControl_FilterYaw(float raw_yaw)
{
    enum
    {
        ROBOT_CTRL_YAW_FILTER_WINDOW = 5
    };

    static float s_window[ROBOT_CTRL_YAW_FILTER_WINDOW] = {0.0f};
    static uint8_t s_count = 0U;
    static uint8_t s_index = 0U;
    static uint8_t s_has_last = 0U;
    static float s_last_filtered = 0.0f;

    float sorted[ROBOT_CTRL_YAW_FILTER_WINDOW];
    float median_value;
    float filtered_value;
    uint8_t valid_count;
    uint8_t i;
    uint8_t j;
    s_window[s_index] = raw_yaw;
    s_index = (uint8_t)((s_index + 1U) % ROBOT_CTRL_YAW_FILTER_WINDOW);

    if (s_count < ROBOT_CTRL_YAW_FILTER_WINDOW)
    {
        s_count++;
    }

    valid_count = s_count;

    for (i = 0U; i < valid_count; i++)
    {
        sorted[i] = s_window[i];
    }

    for (i = 0U; i < valid_count; i++)
    {
        for (j = (uint8_t)(i + 1U); j < valid_count; j++)
        {
            if (sorted[j] < sorted[i])
            {
                float temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }

    median_value = sorted[valid_count / 2U];

    if (0U == s_has_last)
    {
        s_last_filtered = median_value;
        s_has_last = 1U;
        return median_value;
    }

    filtered_value = (s_last_filtered * 0.7f) + (median_value * 0.3f);
    s_last_filtered = filtered_value;

    return filtered_value;
}

/*
 * 对目标速度做轻度低通滤波。
 *
 * 作用：
 * - 避免视觉速度值跳变时，底盘目标速度一下子变化过猛。
 * - 这里只负责平滑，不负责限幅。
 */
static int RobotControl_FilterSpeedLeft(int raw_speed)
{
    static uint8_t s_has_last = 0U;
    static int s_last_speed = 0;
    int filtered_speed;
    if (0U == s_has_last)
    {
        s_last_speed = raw_speed;
        s_has_last = 1U;
        return raw_speed;
    }

    filtered_speed = ((s_last_speed * 7) / 10) + ((raw_speed * 3) / 10);
    s_last_speed = filtered_speed;

    return filtered_speed;
}

static int RobotControl_FilterSpeedRight(int raw_speed)
{
    static uint8_t s_has_last = 0U;
    static int s_last_speed = 0;
    int filtered_speed;
    if (0U == s_has_last)
    {
        s_last_speed = raw_speed;
        s_has_last = 1U;
        return raw_speed;
    }

    filtered_speed = ((s_last_speed * 7) / 10) + ((raw_speed * 3) / 10);
    s_last_speed = filtered_speed;

    return filtered_speed;
}

/*
 * 处理视觉输入并解释业务含义。
 *
 * 作用：
 * - 从 Vision_Uart9_Comm 读取最新完整帧。
 * - 缓存到协调层上下文。
 * - 判断是否为机械臂模式切换帧。
 * - 在循迹模式下，把速度和 yaw 送给 Navigation。
 *
 * 为什么放在主循环：
 * - 视觉帧解析和模式切换都属于慢任务，不适合塞进中断。
 *
 */
static void RobotControl_ProcessVisionInput(void)
{
    vision_uart9_frame_t frame;
    float yaw_offset;

    if (!VisionUart9_HasNewData())
    {
        return;
    }

    if (VISION_UART9_OK != VisionUart9_GetLatest(&frame))
    {
        /* 当前帧读取失败时清掉标志，等待下一帧。 */
        VisionUart9_ClearNewDataFlag();
        return;
    }

    /* 协调层保存一份自己的视觉副本，后续状态机统一读这里。 */
    (void)RobotControl_UpdateVisionFrame(&frame);

    /* 当前帧已经复制完成，可以清掉视觉模块的新数据标志。 */
    VisionUart9_ClearNewDataFlag();

    if (0U == g_robot_control_ctx.has_valid_vision_frame)
    {
        return;
    }

    if ((0U != g_robot_control_ctx.pick_done_wait_ack) &&
        (0U != RobotControl_IsPickDoneAckFrame(&g_robot_control_ctx.latest_vision_frame)))
    {
        g_robot_control_ctx.pick_done_wait_ack = 0U;
        g_robot_control_ctx.pick_done_last_send_ms = 0U;
    }

    /*
     * 9999,9999,9999,9999：
     * 进入机械臂模式。
     *
     * 进入后要做的事：
     * 1. 停止底盘。
     * 2. 屏蔽雷达/避障。
     * 3. 停止把视觉数据继续送给 Navigation。
     * 4. 根据当前夹爪是否已闭合，决定后续等待抓取目标还是放置目标。
     */
    if (((int)g_robot_control_ctx.latest_vision_frame.depth == 9997))
    {
        Navigation_SetBadgeEnable(0U);
        return;
    }
    if (((int)g_robot_control_ctx.latest_vision_frame.depth == 9996))
    {
        Navigation_SetBadgeEnable(1U);
        return;
    }
    if (0U != RobotControl_IsArmModeSwitchFrame(&g_robot_control_ctx.latest_vision_frame))
    {
        g_robot_control_ctx.control_mode = ROBOT_CTRL_MODE_ARM_GRAB;
        g_robot_control_ctx.line_track_enabled = 0U;
        g_robot_control_ctx.has_valid_arm_target = 0U;
        g_robot_control_ctx.pending_gripper_req = ROBOT_CTRL_GRIPPER_REQ_NONE;
        RobotControl_ResetArmWait();

        g_robot_control_ctx.arm_state = (0U != g_robot_control_ctx.gripper_is_closed) ? ROBOT_CTRL_ARM_STATE_WAIT_PLACE_TARGET : ROBOT_CTRL_ARM_STATE_WAIT_PICK_TARGET;

        g_car_state = CAR_STATE_STOP;
        float data[4] = {7878, 7878, 7878, 7878};
        VisionUart9_Send4(data, 4);
        /*
         * 这里调用 Navigation_SetBadgeEnable(1U) 的目的：
         * 让 Navigation 在当前阶段旁路雷达/避障分支，避免机械臂模式下被底盘逻辑打断。
         */
        Navigation_SetBadgeEnable(1U);
        return;
    }

    /*
     * 机械臂模式下仍然继续接收并缓存视觉帧，
     * 但这些帧只服务于机械臂流程，不再继续喂给底盘循迹。
     */
    if (ROBOT_CTRL_MODE_ARM_GRAB == g_robot_control_ctx.control_mode)
    {
        g_robot_control_ctx.line_track_enabled = 0U;
        g_car_state = CAR_STATE_STOP;
        Navigation_SetBadgeEnable(1U);
        return;
    }

    /*
     * 普通循迹模式下：
     * - u 作为左轮目标速度
     * - v 作为右轮目标速度
     * - yaw 作为循迹偏差
     */
    //    static float last_yaw = 0;
    yaw_offset = g_robot_control_ctx.latest_vision_frame.yaw * 40.0f;
    static float yaw_last = 0;
    if (fabs(yaw_offset - 0) <= 0.0000001)
    {
        yaw_offset = yaw_last;
    }
    yaw_last = yaw_offset;
    //    if (yaw_offset == 0.0f)
    //    {
    //        yaw_offset = last_yaw;
    //    }
    //    else
    //    {
    //        yaw_offset = RobotControl_FilterYaw(yaw_offset);
    //        last_yaw = yaw_offset;
    //    }
    /*
     * 当前项目约定：
     * - u 作为左轮目标速度
     * - v 作为右轮目标速度
     * 这里先做限幅，避免视觉侧发错值导致速度异常。
     */
    int target_left_speed = (int)g_robot_control_ctx.latest_vision_frame.u;
    int target_right_speed = (int)g_robot_control_ctx.latest_vision_frame.v;
    static int left_speed_last = 0;
    static int right_speed_last = 0;

    if (target_left_speed == 0 && target_right_speed == 0)
    {
        target_left_speed = left_speed_last;
        target_right_speed = right_speed_last;
    }else if(target_left_speed == 1 && target_right_speed == 1){
				target_left_speed = 0;
        target_right_speed = 0;
		}

    target_left_speed = target_left_speed > 100 ? 100 : (target_left_speed < -100 ? -100 : target_left_speed);
    target_right_speed = target_right_speed > 100 ? 100 : (target_right_speed < -100 ? -100 : target_right_speed);
    target_left_speed = RobotControl_FilterSpeedLeft(target_left_speed);
    target_right_speed = RobotControl_FilterSpeedRight(target_right_speed);

    g_nav_target_speed = (target_left_speed + target_right_speed) / 2;
    left_speed_last = target_left_speed;
    right_speed_last = target_right_speed;

    if (0U != g_robot_control_ctx.line_track_enabled)
    {
        /*
         * 这里只更新 Navigation 的“最新输入”。
         * 真正的底盘执行输出不在这里做，而是在 10ms 周期中的 Navigation_Run_Logic() 完成。
         */
        Navigation_LineTrack_Update(yaw_offset,
                                    g_robot_control_ctx.latest_vision_frame.frame_valid,
                                    target_left_speed,
                                    target_right_speed);
    }
}

/*
 * 处理夹爪手动请求。
 *
 * 作用：
 * - 读取 pending_gripper_req。
 * - 把请求转换成真正的夹爪动作。
 * - 更新最近一次夹爪执行结果和软件状态。
 *
 * 为什么仍保留这个函数：
 * - 这样菜单手动测试夹爪、后续外部触发夹爪时都有统一入口。
 *
 * 注意：
 * - 机械臂自动抓取/放置流程进行中，这里会直接旁路，避免和自动状态机冲突。
 */
static void RobotControl_ProcessGripperRequest(void)
{
    robot_control_gripper_req_t request_to_run;

    /*
     * 机械臂自动流程执行时，不允许这里再插入独立夹爪请求，
     * 否则可能破坏“移动 -> 张爪/闭爪 -> 再移动”的动作顺序。
     */
    if (ROBOT_CTRL_MODE_ARM_GRAB == g_robot_control_ctx.control_mode)
    {
        return;
    }

    request_to_run = g_robot_control_ctx.pending_gripper_req;
    if (ROBOT_CTRL_GRIPPER_REQ_NONE == request_to_run)
    {
        return;
    }

    /* 先清掉请求，避免执行失败后重复进入同一请求。 */
    g_robot_control_ctx.pending_gripper_req = ROBOT_CTRL_GRIPPER_REQ_NONE;

    /*
     * TOGGLE 不直接下发，而是先根据当前软件状态展开成 OPEN 或 CLOSE。
     */
    if (ROBOT_CTRL_GRIPPER_REQ_TOGGLE == request_to_run)
    {
        request_to_run = (0U != g_robot_control_ctx.gripper_is_closed) ? ROBOT_CTRL_GRIPPER_REQ_OPEN : ROBOT_CTRL_GRIPPER_REQ_CLOSE;
    }

    if (ROBOT_CTRL_GRIPPER_REQ_OPEN == request_to_run)
    {
        /* 调用 Mechanical_hand_control 执行真正的张爪动作。 */
        g_robot_control_ctx.last_gripper_result = MechanicalHand_GripperOpen(1000U);
        if (MECH_HAND_OK == g_robot_control_ctx.last_gripper_result)
        {
            g_robot_control_ctx.gripper_is_closed = 0U;
        }
    }
    else if (ROBOT_CTRL_GRIPPER_REQ_CLOSE == request_to_run)
    {
        /* 调用 Mechanical_hand_control 执行真正的闭爪动作。 */
        g_robot_control_ctx.last_gripper_result = MechanicalHand_GripperClose(1000U);
        if (MECH_HAND_OK == g_robot_control_ctx.last_gripper_result)
        {
            g_robot_control_ctx.gripper_is_closed = 1U;
        }
    }
}

/*
 * 推进机械臂抓取/放置状态机。
 *
 * 作用：
 * - 只有处于机械臂模式时才运行。
 * - 根据当前 arm_state 推进抓取或放置流程。
 * - 在需要时调用 Calibration、Mechanical_hand_control 等模块。
 *
 * 为什么放在主循环：
 * - MechanicalHand_MoveTo()、MechanicalHand_Home()、夹爪开合都是阻塞型慢动作，
 *   这些动作不能放在中断里执行。
 *
 * 后续修改建议：
 * - 抓取动作顺序改这里。
 * - 放置完成后是恢复循迹还是原地停止，也在这里改。
 * - 目标筛选、重试、回原点等扩展都从这里继续加。
 */
static void RobotControl_ProcessArmTarget(void)
{
    float x_base;
    float y_base;
    float z_base;
    float target_x;
    float target_y;
    float target_z;
    float safe_z;
    int status;
    robot_control_arm_state_t next_state;

    if (ROBOT_CTRL_MODE_ARM_GRAB != g_robot_control_ctx.control_mode)
    {
        return;
    }

    /*
     * 只要机械臂模式还在进行，就强制保持：
     * - 小车停止
     * - 雷达/避障旁路
     * - 不允许视觉继续输入到底盘
     */
    g_car_state = CAR_STATE_STOP;
    Navigation_SetBadgeEnable(1U);
    g_robot_control_ctx.line_track_enabled = 0U;

    switch (g_robot_control_ctx.arm_state)
    {
    case ROBOT_CTRL_ARM_STATE_IDLE:
        break;

    case ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE:
        if (0U == RobotControl_IsArmWaitDone())
        {
            return;
        }

        next_state = g_robot_control_ctx.arm_wait_next_state;
        RobotControl_ResetArmWait();
        g_robot_control_ctx.arm_state = next_state;
        break;

    case ROBOT_CTRL_ARM_STATE_WAIT_PICK_TARGET:
        /*
         * 等待抓取目标帧：
         * 协议格式为 u,v,depth,9988
         *
         * 这里先做三件事：
         * 1. 判断当前是否已有有效视觉帧。
         * 2. 判断当前帧是否有效。
         * 3. 判断当前帧是否真的是抓取标志帧。
         */
        if (0U == g_robot_control_ctx.has_valid_vision_frame)
        {
            return;
        }

        if (0U == g_robot_control_ctx.latest_vision_frame.frame_valid)
        {
            return;
        }

        if (0U == RobotControl_IsArmPickFrame(&g_robot_control_ctx.latest_vision_frame))
        {
            return;
        }

        /*
         * 调用 Calibration_ImageToBase() 的目的：
         * 把视觉帧中的 u、v、depth 转成机械臂基座坐标系下的目标点。
         */
        status = Calibration_ImageToBase(g_robot_control_ctx.latest_vision_frame.u,
                                         g_robot_control_ctx.latest_vision_frame.v,
                                         g_robot_control_ctx.latest_vision_frame.depth,
                                         &x_base,
                                         &y_base,
                                         &z_base);
        g_robot_control_ctx.last_arm_target_result = status;
        if (CALIB_OK != status)
        {
            g_robot_control_ctx.has_valid_arm_target = 0U;
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
            return;
        }

        g_robot_control_ctx.arm_target_x_mm = x_base;
        g_robot_control_ctx.arm_target_y_mm = y_base;
        g_robot_control_ctx.arm_target_z_mm = z_base;
        g_robot_control_ctx.has_valid_arm_target = 1U;
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_MOVE_ABOVE_PICK_TARGET;
        break;

    case ROBOT_CTRL_ARM_STATE_MOVE_ABOVE_PICK_TARGET:
        if (0U == g_robot_control_ctx.has_valid_arm_target)
        {

            return;
        }

        target_x = g_robot_control_ctx.arm_target_x_mm;
        target_y = g_robot_control_ctx.arm_target_y_mm;
        target_z = g_robot_control_ctx.arm_target_z_mm;
        safe_z = target_z + ROBOT_CTRL_ARM_APPROACH_HEIGHT_MM;

        /*
         * 先移动到目标上方，而不是直接下探到目标点，
         * 这样可以减少机械臂或夹爪直接碰撞目标的概率。
         */
        status = MechanicalHand_MoveTo(target_x, target_y, safe_z, ROBOT_CTRL_ARM_MOVE_TIME_MS);
        g_robot_control_ctx.last_arm_target_result = status;
        if (MECH_HAND_OK != status)
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
            break;
        }

        RobotControl_StartArmWait(ROBOT_CTRL_ARM_MOVE_TIME_MS + ROBOT_CTRL_ARM_MOVE_SETTLE_MS,
                                  ROBOT_CTRL_ARM_STATE_OPEN_GRIPPER_BEFORE_PICK);
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        break;

    case ROBOT_CTRL_ARM_STATE_OPEN_GRIPPER_BEFORE_PICK:
        /* 到达目标上方后先张爪，给后续下探夹取留出空间。 */
        g_robot_control_ctx.last_gripper_result = MechanicalHand_GripperOpen(ROBOT_CTRL_ARM_GRIPPER_TIME_MS);
        if (MECH_HAND_OK == g_robot_control_ctx.last_gripper_result)
        {
            g_robot_control_ctx.gripper_is_closed = 0U;
            RobotControl_StartArmWait(ROBOT_CTRL_ARM_GRIPPER_TIME_MS + ROBOT_CTRL_ARM_GRIPPER_SETTLE_MS,
                                      ROBOT_CTRL_ARM_STATE_MOVE_DOWN_TO_PICK_TARGET);
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        }
        else
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
        }
        break;

    case ROBOT_CTRL_ARM_STATE_MOVE_DOWN_TO_PICK_TARGET:
        target_x = g_robot_control_ctx.arm_target_x_mm;
        target_y = g_robot_control_ctx.arm_target_y_mm;
        target_z = g_robot_control_ctx.arm_target_z_mm;

        /* 从安全高度下探到实际抓取点。 */
        status = MechanicalHand_MoveTo(target_x, target_y, target_z, ROBOT_CTRL_ARM_MOVE_TIME_MS);
        g_robot_control_ctx.last_arm_target_result = status;
        if (MECH_HAND_OK != status)
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
            break;
        }

        RobotControl_StartArmWait(ROBOT_CTRL_ARM_MOVE_TIME_MS + ROBOT_CTRL_ARM_MOVE_SETTLE_MS,
                                  ROBOT_CTRL_ARM_STATE_CLOSE_GRIPPER_AFTER_PICK);
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        break;

    case ROBOT_CTRL_ARM_STATE_CLOSE_GRIPPER_AFTER_PICK:
        /* 到达目标点后闭合夹爪夹取物体。 */
        g_robot_control_ctx.last_gripper_result = MechanicalHand_GripperClose(ROBOT_CTRL_ARM_GRIPPER_TIME_MS);
        if (MECH_HAND_OK == g_robot_control_ctx.last_gripper_result)
        {
            g_robot_control_ctx.gripper_is_closed = 1U;
            RobotControl_StartArmWait(ROBOT_CTRL_ARM_GRIPPER_TIME_MS + ROBOT_CTRL_ARM_GRIPPER_SETTLE_MS,
                                      ROBOT_CTRL_ARM_STATE_LIFT_AFTER_PICK);
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        }
        else
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
        }
        break;

    case ROBOT_CTRL_ARM_STATE_LIFT_AFTER_PICK:
        target_x = g_robot_control_ctx.arm_target_x_mm;
        target_y = g_robot_control_ctx.arm_target_y_mm;
        target_z = g_robot_control_ctx.arm_target_z_mm;
        safe_z = target_z + ROBOT_CTRL_ARM_APPROACH_HEIGHT_MM;

        /* 夹住物体后先抬起，避免拖碰目标周围环境。 */
        status = MechanicalHand_MoveTo(target_x, target_y, safe_z, ROBOT_CTRL_ARM_MOVE_TIME_MS);
        g_robot_control_ctx.last_arm_target_result = status;
        if (MECH_HAND_OK != status)
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
            break;
        }

        RobotControl_StartArmWait(ROBOT_CTRL_ARM_MOVE_TIME_MS + ROBOT_CTRL_ARM_MOVE_SETTLE_MS,
                                  ROBOT_CTRL_ARM_STATE_RETURN_HOME_AFTER_PICK);
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        break;

    case ROBOT_CTRL_ARM_STATE_RETURN_HOME_AFTER_PICK:
        /*
         * 抓取完成后返回机械臂默认位。
         * 这样后续小车恢复循迹时，机械臂不会一直停在工作区附近。
         */
        status = MechanicalHand_Home(ROBOT_CTRL_ARM_MOVE_TIME_MS);
        g_robot_control_ctx.last_arm_target_result = status;
        if (MECH_HAND_OK != status)
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
            break;
        }

        RobotControl_StartArmWait(ROBOT_CTRL_ARM_MOVE_TIME_MS + ROBOT_CTRL_ARM_MOVE_SETTLE_MS,
                                  ROBOT_CTRL_ARM_STATE_RESUME_LINE_TRACK_AFTER_PICK);
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        break;

    case ROBOT_CTRL_ARM_STATE_RESUME_LINE_TRACK_AFTER_PICK:
        /*
         * 抓取完成后恢复正常循迹模式。
         *
         * 这里恢复的内容包括：
         * 1. 控制模式切回循迹模式。
         * 2. 状态机回到 IDLE。
         * 3. 重新允许视觉输入 Navigation。
         * 4. 清掉机械臂目标缓存和挂起夹爪请求。
         * 5. 恢复小车 RUN。
         * 6. 恢复 Navigation 的正常雷达/避障分支。
         */
        g_robot_control_ctx.control_mode = ROBOT_CTRL_MODE_LINE_TRACK;
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_IDLE;
        g_robot_control_ctx.line_track_enabled = 1U;
        g_robot_control_ctx.has_valid_arm_target = 0U;
        g_robot_control_ctx.pending_gripper_req = ROBOT_CTRL_GRIPPER_REQ_NONE;
        RobotControl_ResetArmWait();

        g_car_state = CAR_STATE_RUN;
        g_robot_control_ctx.pick_done_wait_ack = 1U;
        g_robot_control_ctx.pick_done_last_send_ms = 0U;
        RobotControl_SendPickDoneFlag();
        break;

    case ROBOT_CTRL_ARM_STATE_WAIT_PLACE_TARGET:
        /*
         * 等待放置目标帧：
         * 协议格式为 u,v,depth,8899
         */
        if (0U == g_robot_control_ctx.has_valid_vision_frame)
        {
            return;
        }

        if (0U == g_robot_control_ctx.latest_vision_frame.frame_valid)
        {
            return;
        }

        if (0U == RobotControl_IsArmPlaceFrame(&g_robot_control_ctx.latest_vision_frame))
        {
            return;
        }

        /* 把放置目标的视觉坐标转换成机械臂可用的基座坐标。 */
        status = Calibration_ImageToBase(g_robot_control_ctx.latest_vision_frame.u,
                                         g_robot_control_ctx.latest_vision_frame.v,
                                         g_robot_control_ctx.latest_vision_frame.depth,
                                         &x_base,
                                         &y_base,
                                         &z_base);
        g_robot_control_ctx.last_arm_target_result = status;
        if (CALIB_OK != status)
        {
            g_robot_control_ctx.has_valid_arm_target = 0U;
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
            return;
        }

        g_robot_control_ctx.arm_target_x_mm = x_base;
        g_robot_control_ctx.arm_target_y_mm = y_base;
        g_robot_control_ctx.arm_target_z_mm = z_base;
        g_robot_control_ctx.has_valid_arm_target = 1U;
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_MOVE_ABOVE_PLACE_TARGET;
        break;

    case ROBOT_CTRL_ARM_STATE_MOVE_ABOVE_PLACE_TARGET:
        target_x = g_robot_control_ctx.arm_target_x_mm;
        target_y = g_robot_control_ctx.arm_target_y_mm;
        target_z = g_robot_control_ctx.arm_target_z_mm;
        safe_z = target_z + ROBOT_CTRL_ARM_APPROACH_HEIGHT_MM;

        /* 放置时同样先到目标上方，避免直接碰撞放置区域。 */
        status = MechanicalHand_MoveTo(target_x, target_y, safe_z, ROBOT_CTRL_ARM_MOVE_TIME_MS);
        g_robot_control_ctx.last_arm_target_result = status;
        if (MECH_HAND_OK != status)
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
            break;
        }

        RobotControl_StartArmWait(ROBOT_CTRL_ARM_MOVE_TIME_MS + ROBOT_CTRL_ARM_MOVE_SETTLE_MS,
                                  ROBOT_CTRL_ARM_STATE_MOVE_DOWN_TO_PLACE_TARGET);
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        break;

    case ROBOT_CTRL_ARM_STATE_MOVE_DOWN_TO_PLACE_TARGET:
        target_x = g_robot_control_ctx.arm_target_x_mm;
        target_y = g_robot_control_ctx.arm_target_y_mm;
        target_z = g_robot_control_ctx.arm_target_z_mm;

        /* 从放置点上方下降到实际放置高度。 */
        status = MechanicalHand_MoveTo(target_x, target_y, target_z, ROBOT_CTRL_ARM_MOVE_TIME_MS);
        g_robot_control_ctx.last_arm_target_result = status;
        if (MECH_HAND_OK != status)
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
            break;
        }

        RobotControl_StartArmWait(ROBOT_CTRL_ARM_MOVE_TIME_MS + ROBOT_CTRL_ARM_MOVE_SETTLE_MS,
                                  ROBOT_CTRL_ARM_STATE_OPEN_GRIPPER_TO_PLACE);
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        break;

    case ROBOT_CTRL_ARM_STATE_OPEN_GRIPPER_TO_PLACE:
        /* 到达放置位置后张开夹爪，把物体留在目标点。 */
        g_robot_control_ctx.last_gripper_result = MechanicalHand_GripperOpen(ROBOT_CTRL_ARM_GRIPPER_TIME_MS);
        if (MECH_HAND_OK == g_robot_control_ctx.last_gripper_result)
        {
            g_robot_control_ctx.gripper_is_closed = 0U;
            RobotControl_StartArmWait(ROBOT_CTRL_ARM_GRIPPER_TIME_MS + ROBOT_CTRL_ARM_GRIPPER_SETTLE_MS,
                                      ROBOT_CTRL_ARM_STATE_LIFT_AFTER_PLACE);
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        }
        else
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
        }
        break;

    case ROBOT_CTRL_ARM_STATE_LIFT_AFTER_PLACE:
        status = MechanicalHand_Home(ROBOT_CTRL_ARM_MOVE_TIME_MS);
        g_robot_control_ctx.last_arm_target_result = status;
        if (MECH_HAND_OK != status)
        {
            RobotControl_ResetArmWait();
            g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_ERROR;
            break;
        }

        RobotControl_StartArmWait(ROBOT_CTRL_ARM_MOVE_TIME_MS + ROBOT_CTRL_ARM_MOVE_SETTLE_MS,
                                  ROBOT_CTRL_ARM_STATE_AFTER_PLACE_HOLD);
        g_robot_control_ctx.arm_state = ROBOT_CTRL_ARM_STATE_WAIT_ACTION_DONE;
        break;

    case ROBOT_CTRL_ARM_STATE_AFTER_PLACE_HOLD:
        /* 放置完成后保持当前状态，不自动恢复循迹。 */
        break;

    case ROBOT_CTRL_ARM_STATE_ERROR:
        RobotControl_ResetArmWait();
    default:
        /* 错误状态不自动恢复；动作等待已在上方清除。 */
        break;
    }
}

/*
 * 执行底盘输出。
 *
 * 作用：
 * - 根据 g_car_state 决定小车是执行 Navigation 还是立即停车。
 *
 * 当前规则：
 * - 不是 RUN：电机停转，舵机回中。
 * - 是 RUN：调用 Navigation_Run_Logic() 执行真正的循迹控制输出。
 *
 * 为什么要单独拆成函数：
 * - 这样 GPT 的 10ms 入口里只保留一个简洁调用点。
 * - 后续若要加“某模式下禁止底盘”“抓取时低速行走”等策略，可以优先改这里。
 */
static void RobotControl_RunDriveControl(void)
{
    if (CAR_STATE_RUN != g_car_state)
    {
        /* 当前不是运行态时，确保底盘确实停住。 */
        wheel_stop();
        Servo_SetAngle(0);
        return;
    }

    /*
     * 调用 Navigation_Run_Logic() 的目的：
     * 真正根据最新循迹输入、PID 状态、编码器反馈生成底盘输出。
     * Robot_Control 本身不直接算 PID，也不直接发左右轮命令。
     */
    Navigation_Run_Logic(g_nav_target_speed);
}

/*
 * 判断当前帧是否为机械臂模式切换帧。
 *
 * 规则：
 * - 必须四个字段都等于 9999。
 *
 * 后续若要改协议格式，例如改成单独的 mode 字段，优先改这里。
 */
static uint8_t RobotControl_IsArmModeSwitchFrame(const vision_uart9_frame_t *frame)
{
    if (NULL == frame)
    {
        return 0U;
    }

    return (((int)frame->u == ROBOT_CTRL_VISION_ARM_MODE_MARK) &&
            ((int)frame->v == ROBOT_CTRL_VISION_ARM_MODE_MARK) &&
            ((int)frame->depth == ROBOT_CTRL_VISION_ARM_MODE_MARK) &&
            ((int)frame->yaw == ROBOT_CTRL_VISION_ARM_MODE_MARK))
               ? 1U
               : 0U;
}

/*
 * 判断当前帧是否为抓取目标帧。
 *
 * 规则：
 * - yaw 字段等于 9988。
 * - frame_valid 必须非 0。
 *
 * 当前协议约定下：
 * - u、v、depth 仍然是坐标数据。
 * - yaw 仅被拿来当抓取标志位。
 */
static uint8_t RobotControl_IsArmPickFrame(const vision_uart9_frame_t *frame)
{
    if (NULL == frame)
    {
        return 0U;
    }

    return (((int)frame->yaw == ROBOT_CTRL_VISION_ARM_PICK_MARK) &&
            (0U != frame->frame_valid))
               ? 1U
               : 0U;
}

/*
 * 判断当前帧是否为放置目标帧。
 *
 * 规则：
 * - yaw 字段等于 8899。
 * - frame_valid 必须非 0。
 *
 * 当前协议约定下：
 * - u、v、depth 仍然是坐标数据。
 * - yaw 仅被拿来当放置标志位。
 */
static uint8_t RobotControl_IsArmPlaceFrame(const vision_uart9_frame_t *frame)
{
    if (NULL == frame)
    {
        return 0U;
    }

    return (((int)frame->yaw == ROBOT_CTRL_VISION_ARM_PLACE_MARK) &&
            (0U != frame->frame_valid))
               ? 1U
               : 0U;
}

static uint8_t RobotControl_IsPickDoneAckFrame(const vision_uart9_frame_t *frame)
{
    if (NULL == frame)
    {
        return 0U;
    }

    if (0U == frame->frame_valid)
    {
        return 0U;
    }

    return (((int)frame->u == ROBOT_CTRL_VISION_PICK_DONE_ACK_MARK) ||
            ((int)frame->v == ROBOT_CTRL_VISION_PICK_DONE_ACK_MARK) ||
            ((int)frame->depth == ROBOT_CTRL_VISION_PICK_DONE_ACK_MARK) ||
            ((int)frame->yaw == ROBOT_CTRL_VISION_PICK_DONE_ACK_MARK))
               ? 1U
               : 0U;
}

static void RobotControl_SendPickDoneFlag(void)
{
    float data[4];

    data[0] = 9999.0f;
    data[1] = 9999.0f;
    data[2] = 9999.0f;
    data[3] = 9999.0f;
    (void)VisionUart9_Send4(data, 4U);
    g_robot_control_ctx.pick_done_last_send_ms = HAL_GetTick();
}

static void RobotControl_ProcessPickDoneAckTx(void)
{
    uint32_t now_ms;

    if (0U == g_robot_control_ctx.pick_done_wait_ack)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((g_robot_control_ctx.pick_done_last_send_ms == 0U) ||
        ((now_ms - g_robot_control_ctx.pick_done_last_send_ms) >= ROBOT_CTRL_PICK_DONE_RESEND_INTERVAL_MS))
    {
        RobotControl_SendPickDoneFlag();
    }
}
