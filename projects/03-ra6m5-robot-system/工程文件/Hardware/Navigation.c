#include "Navigation.h"
#include "SysTick.h"
#include <math.h>

Nav_PID_t g_pid_avoid;
Nav_PID_t g_pid_line;

volatile float g_nav_line_offset = 0.0f;
volatile uint8_t g_nav_line_valid = 0U;
volatile uint8_t g_nav_badge_enable = 0U;
volatile int g_nav_target_left_speed = 0;
volatile int g_nav_target_right_speed = 0;
volatile float g_nav_debug_servo_angle = 0.0f;
volatile float g_nav_debug_line_pid_output = 0.0f;
volatile float g_nav_debug_diff_rpm = 0.0f;

extern int g_nav_target_speed;

/*
 * 运行期私有状态
 * 1. g_nav_line_last_update_ms:
 *    记录最近一次收到视觉输入的时刻，用于超时判定。
 * 2. g_nav_diff_assist_active:
 *    记录当前是否已经进入“大偏移差速辅助”状态，用于实现滞回。
 */
static uint32_t g_nav_line_last_update_ms = 0U;
static uint8_t g_nav_diff_assist_active = 0U;
static uint8_t g_nav_avoid_active = 0U;
static uint8_t g_nav_avoid_exit_report_pending = 0U;

/* 调试量，供串口或菜单排查控制行为时观察。 */
static float debug_left_push = 0.0f;
static float debug_right_push = 0.0f;
static float debug_net_force = 0.0f;
static int debug_left_target_rpm = 0;
static int debug_right_target_rpm = 0;
static float g_nav_avoid_max_left = 30.0f;
static float g_nav_avoid_max_right = 30.0f;
static float g_nav_line_max_left = 30.0f;
static float g_nav_line_max_right = 30.0f;

static void Nav_PID_Reset(Nav_PID_t *pid);
static void Navigation_ResetLineRuntime(void);
static uint8_t Navigation_IsLineDataFresh(void);
static void Navigation_UpdateDiffAssistState(float abs_offset);
static float Navigation_ClampFloat(float value, float min_value, float max_value);
static int Navigation_RoundFloatToInt(float value);
static void Navigation_StopOutput(void);
static float Calculate_Lidar_Force(void);
static float Nav_PID_Compute(Nav_PID_t *pid, float error);

/*
 * 初始化说明
 * 1. g_pid_avoid 保持旧避障 PID 默认值，不改它的角色。
 * 2. g_pid_line 是新的循迹 PID，完全独立，不和旧 PID 共用参数。
 * 3. 上电时清空视觉状态、差速状态和调试状态，避免沿用历史脏值。
 */
void Navigation_Init(void)
{
    g_nav_avoid_max_left = 35.0f;
    g_nav_avoid_max_right = 50.0f;
    g_nav_line_max_left = 35.0f;
    g_nav_line_max_right = 50.0f;

    g_pid_avoid.Kp = 1.0f;
    g_pid_avoid.Ki = 0.00f;
    g_pid_avoid.Kd = 0.6f;
    g_pid_avoid.max_output = (g_nav_avoid_max_left > g_nav_avoid_max_right) ? g_nav_avoid_max_left : g_nav_avoid_max_right;
    g_pid_avoid.max_integral = 0.0f;
    Nav_PID_Reset(&g_pid_avoid);

    g_pid_line.Kp = 0.85f;
    g_pid_line.Ki = 0.002f;
    g_pid_line.Kd = 0.1f;
    g_pid_line.max_output = (g_nav_line_max_left > g_nav_line_max_right) ? g_nav_line_max_left : g_nav_line_max_right;
    g_pid_line.max_integral = 10.0f;
    Nav_PID_Reset(&g_pid_line);

    g_nav_line_offset = 0.0f;
    g_nav_line_valid = 0U;
    g_nav_badge_enable = 0U;
    g_nav_target_left_speed = 0;
    g_nav_target_right_speed = 0;
    g_nav_target_speed = 0;
    g_nav_line_last_update_ms = HAL_GetTick();
    g_nav_diff_assist_active = 0U;
    g_nav_avoid_active = 0U;
    g_nav_avoid_exit_report_pending = 0U;

    debug_left_push = 0.0f;
    debug_right_push = 0.0f;
    debug_net_force = 0.0f;
    g_nav_debug_servo_angle = 0.0f;
    g_nav_debug_line_pid_output = 0.0f;
    g_nav_debug_diff_rpm = 0.0f;
    debug_left_target_rpm = 0;
    debug_right_target_rpm = 0;
}

/*
 * 正式视觉循迹输入接口
 * 1. 该函数只负责接收并缓存最新一帧视觉输入，不直接输出舵机和轮速。
 * 2. 每次调用都会同步更新：
 *    - 视觉偏移量
 *    - 视觉有效位
 *    - 左右轮目标 RPM
 *    - 全局平均目标速度 g_nav_target_speed
 *    - 最近更新时间戳
 * 3. 即使本帧视觉无效，也会更新左右轮目标 RPM，方便外部模块统一查阅。
 * 4. vision_valid=0 时，当前循迹积分会立即清零，等待下一帧有效视觉重新接管。
 */
void Navigation_LineTrack_Update(float vision_offset, uint8_t vision_valid, int left_target_rpm, int right_target_rpm)
{
    float signed_offset;
    int32_t speed_sum;

    signed_offset = vision_offset * NAV_LINE_OFFSET_SIGN;
    speed_sum = (int32_t)left_target_rpm + (int32_t)right_target_rpm;

    g_nav_target_left_speed = left_target_rpm;
    g_nav_target_right_speed = right_target_rpm;
    g_nav_target_speed = (int)(speed_sum / 2L);
    g_nav_line_last_update_ms = HAL_GetTick();

    if (vision_valid != 0U)
    {
        if (fabsf(signed_offset) < NAV_LINE_DEADBAND)
        {
            signed_offset = 0.0f;
        }

        g_nav_line_offset = signed_offset;
        g_nav_line_valid = 1U;
    }
    else
    {
        g_nav_line_offset = 0.0f;
        g_nav_line_valid = 0U;
        Navigation_ResetLineRuntime();
    }
}

/*
 * 臂章阶段雷达屏蔽开关
 * 1. enable=0:
 *    雷达避障恢复有效，允许旧避障 PID 接管。
 * 2. enable!=0:
 *    雷达避障整段旁路，避免机械臂抓取时雷达误检。
 * 3. 这里只切换“是否允许雷达接管”，不直接发出停车或运动命令。
 */
void Navigation_SetBadgeEnable(uint8_t enable)
{
    g_nav_badge_enable = (enable != 0U) ? 1U : 0U;

    if (g_nav_badge_enable != 0U)
    {
        Nav_PID_Reset(&g_pid_avoid);
        debug_net_force = 0.0f;
        g_nav_avoid_active = 0U;
        g_nav_avoid_exit_report_pending = 0U;
    }
}

uint8_t Navigation_FetchAvoidExitEvent(void)
{
    if (g_nav_avoid_exit_report_pending != 0U)
    {
        g_nav_avoid_exit_report_pending = 0U;
        return 1U;
    }

    return 0U;
}

void Navigation_ClearAvoidExitEvent(void)
{
    g_nav_avoid_exit_report_pending = 0U;
}

/*
 * 激光雷达净推力计算
 * 1. 在 ROI 内统计有效障碍点，按距离和角度权重计算左右推力。
 * 2. 返回值含义：
 *    - -9999.0f: 障碍过近，触发急停。
 *    - 其他值: 左右净推力，正负号代表障碍偏向哪一侧。
 * 3. 该函数只负责“感知结果量化”，不直接控制舵机和电机。
 */
static float Calculate_Lidar_Force(void)
{
    float push_from_left;
    float push_from_right;
    float min_dist;
    float raw_net_force;
    float scaled_force;
    float dist_diff;
    float dist_from_center;
    float weight;
    float weighted_force;
    uint16_t dist;
    int valid_cnt;
    int i;

    push_from_left = 0.0f;
    push_from_right = 0.0f;
    min_dist = 10000.0f;
    valid_cnt = 0;

    for (i = LIDAR_ROI_MIN_IDX; i <= LIDAR_ROI_MAX_IDX; i++)
    {
        if (i < 0 || i >= TOTAL_SLOTS)
        {
            continue;
        }

        if (g_lidar_map[i].valid == 0U)
        {
            continue;
        }

        dist = g_lidar_map[i].distance_mm;
        if ((dist == 0U) || (dist == 0xFFFFU))
        {
            continue;
        }

        if (dist > LIDAR_DETECT_DIST_MM)
        {
            continue;
        }

        if ((float)dist < min_dist)
        {
            min_dist = (float)dist;
        }

        dist_diff = (float)(LIDAR_DETECT_DIST_MM - dist);
        dist_from_center = fabsf((float)i - 25.0f);
        weight = WEIGHT_CENTER - (dist_from_center / 25.0f) * (WEIGHT_CENTER - WEIGHT_SIDE);
        weighted_force = dist_diff * weight;

        if (i < 25)
        {
            push_from_left += weighted_force;
        }
        else
        {
            push_from_right += weighted_force;
        }

        valid_cnt++;
    }

    if (min_dist < (float)LIDAR_STOP_DIST_MM)
    {
        return -9999.0f;
    }

    if (valid_cnt == 0)
    {
        debug_left_push = 0.0f;
        debug_right_push = 0.0f;
        return 0.0f;
    }

    raw_net_force = push_from_left - push_from_right;
    scaled_force = raw_net_force * FORCE_SCALING;

    debug_left_push = push_from_left * FORCE_SCALING;
    debug_right_push = push_from_right * FORCE_SCALING;

    scaled_force = Navigation_ClampFloat(scaled_force, -50.0f, 50.0f);
    return scaled_force;
}

/*
 * PID 状态清零
 * 1. 只清运行期误差、积分和输出，不动 PID 参数本身。
 * 2. 在分支切换、视觉失效、急停、臂章屏蔽切换时都可以安全调用。
 */
static void Nav_PID_Reset(Nav_PID_t *pid)
{
    if (!pid)
    {
        return;
    }

    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/*
 * 循迹分支运行期状态复位
 * 1. 清新的循迹 PID 积分和历史误差。
 * 2. 清差速辅助状态，避免下一次重新进入时沿用旧状态。
 * 3. 不清左右轮目标 RPM，因为这些值需要保留给外部模块查询。
 */
static void Navigation_ResetLineRuntime(void)
{
    Nav_PID_Reset(&g_pid_line);
    g_nav_diff_assist_active = 0U;
    g_nav_debug_line_pid_output = 0.0f;
    g_nav_debug_diff_rpm = 0.0f;
    debug_left_target_rpm = 0;
    debug_right_target_rpm = 0;
}

/*
 * 判断视觉数据是否仍然有效
 * 1. 必须同时满足：
 *    - g_nav_line_valid == 1
 *    - 当前时间减去最近更新时间不超过超时阈值
 * 2. 任何一项不满足，都直接判定为视觉无效。
 */
static uint8_t Navigation_IsLineDataFresh(void)
{
    uint32_t now_ms;

    if (g_nav_line_valid == 0U)
    {
        return 0U;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - g_nav_line_last_update_ms) > NAV_LINE_VALID_TIMEOUT_MS)
    {
        g_nav_line_valid = 0U;
        g_nav_line_offset = 0.0f;
        return 0U;
    }

    return 1U;
}

/*
 * 大偏移差速辅助滞回状态机
 * 1. abs(offset) 超过进入阈值时置位。
 * 2. abs(offset) 下降到退出阈值以下时清零。
 * 3. 这样做是为了避免偏移在单阈值附近来回抖动。
 */
static void Navigation_UpdateDiffAssistState(float abs_offset)
{
    if (g_nav_diff_assist_active != 0U)
    {
        if (abs_offset <= NAV_LINE_DIFF_EXIT_ABS)
        {
            g_nav_diff_assist_active = 0U;
        }
    }
    else if (abs_offset >= NAV_LINE_DIFF_ENTER_ABS)
    {
        g_nav_diff_assist_active = 1U;
    }
}

/*
 * 浮点限幅辅助
 * 1. 用于舵机输出限角和差速修正限幅。
 * 2. 单独拆出后，主流程更容易读。
 */
static float Navigation_ClampFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

/*
 * 浮点转整型四舍五入
 * 1. 差速修正内部先按浮点计算，再转成整数 RPM。
 * 2. 相比直接强制截断，更适合控制量过渡。
 */
static int Navigation_RoundFloatToInt(float value)
{
    if (value >= 0.0f)
    {
        return (int)(value + 0.5f);
    }

    return (int)(value - 0.5f);
}

/*
 * 停止输出
 * 1. 舵机回中。
 * 2. 双轮停车。
 * 3. 用于视觉超时和雷达急停这类安全出口。
 */
static void Navigation_StopOutput(void)
{
    g_nav_debug_servo_angle = 0.0f;
    debug_left_target_rpm = 0;
    debug_right_target_rpm = 0;
    g_nav_debug_diff_rpm = 0.0f;
    Servo_SetAngle(0);
    wheel_stop();
}

/*
 * 通用 PID 计算
 * 1. 负责 P、I、D 计算、积分限幅、输出限幅。
 * 2. 每次计算后都必须回写 pid->output，便于调试和后续状态查看。
 */
static float Nav_PID_Compute(Nav_PID_t *pid, float error)
{
    float p_term;
    float d_term;
    float output;

    if (!pid)
    {
        return 0.0f;
    }

    pid->error = error;
    p_term = pid->Kp * pid->error;
    pid->integral += pid->Ki * pid->error;

    if (pid->integral > pid->max_integral)
    {
        pid->integral = pid->max_integral;
    }
    else if (pid->integral < -pid->max_integral)
    {
        pid->integral = -pid->max_integral;
    }

    d_term = pid->Kd * (pid->error - pid->prev_error);
    pid->prev_error = pid->error;

    output = p_term + pid->integral + d_term;
    if (output > pid->max_output)
    {
        output = pid->max_output;
    }
    else if (output < -pid->max_output)
    {
        output = -pid->max_output;
    }

    pid->output = output;
    return output;
}

/*
 * 运行逻辑详细说明
 * 1. 先判断视觉数据是否仍然有效，超时后直接按失效处理。
 * 2. 若臂章标志关闭，则优先保留雷达急停和旧避障 PID 逻辑。
 * 3. 若臂章标志开启，则整段雷达逻辑直接旁路，避免机械臂抓取物体时误触发。
 * 4. 若雷达未接管且视觉有效，则进入新的循迹 PID：
 *    - 小偏移：左右轮 RPM 直接使用外部传入目标值
 *    - 大偏移：在外部目标值基础上叠加差速辅助
 * 5. 若视觉无效或超时，则清循迹状态并执行舵机回中、双轮停车。
 */
void Navigation_Run_Logic(int target_speed)
{
    float steering_cmd;
    float final_servo_angle;
    float obstacle_force;
    float abs_offset;
    float diff_rpm;
    uint8_t line_data_valid;
    int left_target_rpm;
    int right_target_rpm;
    int diff_rpm_int;

    steering_cmd = 0.0f;
    final_servo_angle = 0.0f;
    obstacle_force = 0.0f;
    abs_offset = 0.0f;
    diff_rpm = 0.0f;
    diff_rpm_int = 0;
    line_data_valid = Navigation_IsLineDataFresh();
    left_target_rpm = g_nav_target_left_speed;
    right_target_rpm = g_nav_target_right_speed;

    g_nav_debug_line_pid_output = 0.0f;
    g_nav_debug_diff_rpm = 0.0f;

    if (g_nav_badge_enable == 0U)
    {
        obstacle_force = Calculate_Lidar_Force();
        debug_net_force = obstacle_force;

        if (obstacle_force == -9999.0f)
        {
            g_nav_avoid_active = 1U;
            Nav_PID_Reset(&g_pid_avoid);
            Navigation_ResetLineRuntime();
            debug_left_target_rpm = 0;
            debug_right_target_rpm = 0;
            g_nav_debug_diff_rpm = 0.0f;
            wheel_stop();
            return;
        }

        if (fabsf(obstacle_force) > 3.0f)
        {
            g_nav_avoid_active = 1U;
            steering_cmd = Nav_PID_Compute(&g_pid_avoid, obstacle_force);
            if (steering_cmd > g_nav_avoid_max_right)
            {
                steering_cmd = g_nav_avoid_max_right;
            }
            else if (steering_cmd < -g_nav_avoid_max_left)
            {
                steering_cmd = -g_nav_avoid_max_left;
            }
            g_pid_avoid.output = steering_cmd;
            Navigation_ResetLineRuntime();

            left_target_rpm = target_speed;
            right_target_rpm = target_speed;
            debug_left_target_rpm = left_target_rpm;
            debug_right_target_rpm = right_target_rpm;

#if SERVO_DIRECTION_INVERT == 1
            final_servo_angle = -steering_cmd;
#else
            final_servo_angle = steering_cmd;
#endif

            final_servo_angle = Navigation_ClampFloat(final_servo_angle, -50.0f, 50.0f);
            g_nav_debug_servo_angle = final_servo_angle;

            Servo_SetAngle((int16_t)final_servo_angle);
            wheel_pid_control_loop(left_target_rpm, g_left_encoder.current_rpm,
                                   right_target_rpm, g_right_encoder.current_rpm);
            return;
        }

        if (g_nav_avoid_active != 0U)
        {
            g_nav_avoid_active = 0U;
            g_nav_avoid_exit_report_pending = 1U;
        }
    }
    else
    {
        debug_net_force = 0.0f;
        Nav_PID_Reset(&g_pid_avoid);
        g_nav_avoid_active = 0U;
    }

    if (line_data_valid == 0U)
    {
        Navigation_ResetLineRuntime();
        Navigation_StopOutput();
        return;
    }

    steering_cmd = Nav_PID_Compute(&g_pid_line, g_nav_line_offset);
    if (steering_cmd > g_nav_line_max_right)
    {
        steering_cmd = g_nav_line_max_right;
    }
    else if (steering_cmd < -g_nav_line_max_left)
    {
        steering_cmd = -g_nav_line_max_left;
    }
    g_pid_line.output = steering_cmd;
    g_nav_debug_line_pid_output = steering_cmd;

    abs_offset = fabsf(g_nav_line_offset);
    Navigation_UpdateDiffAssistState(abs_offset);

#if SERVO_DIRECTION_INVERT == 1
    final_servo_angle = -steering_cmd;
#else
    final_servo_angle = steering_cmd;
#endif

    final_servo_angle = Navigation_ClampFloat(final_servo_angle, -50.0f, 50.0f);
    g_nav_debug_servo_angle = final_servo_angle;

    /*
     * 大偏移差速辅助说明
     * 1. 先以舵机 PID 作为主转向手段。
     * 2. 当偏移绝对值足够大时，再额外叠加左右轮差速。
     * 3. 这里只做一层附加 RPM 修正，不再额外引入第二套轮速 PID。
     */
    if (g_nav_diff_assist_active != 0U)
    {
        diff_rpm = g_nav_line_offset * NAV_LINE_DIFF_KP;
        diff_rpm = Navigation_ClampFloat(diff_rpm, -NAV_LINE_DIFF_MAX_RPM, NAV_LINE_DIFF_MAX_RPM);
        diff_rpm_int = Navigation_RoundFloatToInt(diff_rpm);

        left_target_rpm -= diff_rpm_int;
        right_target_rpm += diff_rpm_int;
        g_nav_debug_diff_rpm = diff_rpm;
    }

    debug_left_target_rpm = left_target_rpm;
    debug_right_target_rpm = right_target_rpm;
    Nav_PID_Reset(&g_pid_avoid);

    Servo_SetAngle((int16_t)final_servo_angle);
    wheel_pid_control_loop(left_target_rpm, g_left_encoder.current_rpm,
                           right_target_rpm, g_right_encoder.current_rpm);
}

/*
 * 假视觉测试喂数函数
 * 1. 用途：
 *    在没有真实视觉模块时，周期性给导航模块喂一组预设测试数据。
 * 2. 使用方式：
 *    把这个函数放进主循环里反复调用，不要只调用一次。
 * 3. 当前测试节拍：
 *    - 第 0 段：居中直行
 *    - 第 1 段：小正偏移
 *    - 第 2 段：大正偏移
 *    - 第 3 段：小负偏移
 *    - 第 4 段：大负偏移
 *    - 第 5 段：明确给出 valid=0，测试立即失效保护
 *    - 第 6 段：故意不再喂新数据，测试超时保护
 * 4. 当前演示启用臂章屏蔽并旁路雷达，仅用于循迹与视觉失效测试。
 *    验证雷达逻辑时需将本函数中的 Navigation_SetBadgeEnable(1U) 改为 0U。
 */
void Navigation_TestFeedDemo(void)
{
    static uint32_t s_test_start_ms = 0U;
    uint32_t now_ms;
    uint32_t elapsed_ms;
    uint32_t phase;

    now_ms = HAL_GetTick();
    if (s_test_start_ms == 0U)
    {
        s_test_start_ms = now_ms;
    }

    elapsed_ms = now_ms - s_test_start_ms;
    phase = (elapsed_ms / 2000U) % 7U;

    Navigation_SetBadgeEnable(1U);

    switch (phase)
    {
    case 0U:
        Navigation_LineTrack_Update(0.0f, 1U, g_nav_target_speed, g_nav_target_speed);
        break;

    case 1U:
        Navigation_LineTrack_Update(8.0f, 1U, g_nav_target_speed, g_nav_target_speed);
        break;

    case 2U:
        Navigation_LineTrack_Update(30.0f, 1U, g_nav_target_speed, g_nav_target_speed);
        break;

    case 3U:
        Navigation_LineTrack_Update(-8.0f, 1U, g_nav_target_speed, g_nav_target_speed);
        break;

    case 4U:
        Navigation_LineTrack_Update(-30.0f, 1U, g_nav_target_speed, g_nav_target_speed);
        break;

    case 5U:
        Navigation_LineTrack_Update(0.0f, 0U, g_nav_target_speed, g_nav_target_speed);
        break;

    default:
        /*
         * 这里故意不调用 Navigation_LineTrack_Update()，
         * 让 Navigation_Run_Logic() 自己走超时失效保护分支。
         */
        break;
    }
}

void Navigation_Debug_Print(void)
{
    printf("[NAV] Badge:%u Line:%u Off:%5.1f Ld:%4.1f Rd:%4.1f Net:%4.1f Servo:%4.1f LinePID:%4.1f Diff:%4.1f TL:%4d TR:%4d\r\n",
           (unsigned int)g_nav_badge_enable,
           (unsigned int)g_nav_line_valid,
           g_nav_line_offset,
           debug_left_push,
           debug_right_push,
           debug_net_force,
           g_nav_debug_servo_angle,
           g_nav_debug_line_pid_output,
           g_nav_debug_diff_rpm,
           debug_left_target_rpm,
           debug_right_target_rpm);
}
