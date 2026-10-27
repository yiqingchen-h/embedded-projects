#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "hal_data.h"
#include "steering_command.h" // 舵机控制
#include "Wheel_control.h"    // 电机控制
#include "UART.h"             // 雷达数据 g_lidar_map
#include "Encoder.h"

/* ================= 用户可调宏 ================= */

// --- 激光雷达感兴趣区域 ROI ---
// 总槽位仍然按照 0~49 使用，25 视为正前方中心。
// 当前只取中间区域参与避障，避免两侧边缘噪声过多影响转向。
#define LIDAR_ROI_MIN_IDX       4
#define LIDAR_ROI_MAX_IDX       45

// --- 舵机方向宏 ---
// 0: PID 输出为正时舵机直接取正角度。
// 1: PID 输出为正时舵机取反角度。
// 如果发现“误差越大，车头反而朝错误方向转”，优先改这里，不要先改 PID。
#define SERVO_DIRECTION_INVERT  1

// --- 旧导航速度宏，保留兼容 ---
#define NAV_CRUISE_SPEED        0
#define NAV_OBSTACLE_SPEED      0

// --- 雷达距离阈值 ---
// LIDAR_DETECT_DIST_MM:
// 进入避障计算的最远距离，调大后更早开始躲障，但也更容易被远处物体干扰。
// LIDAR_STOP_DIST_MM:
// 进入强制停车的最近距离，调小后更激进，调大后更保守。
#define LIDAR_DETECT_DIST_MM    350
#define LIDAR_STOP_DIST_MM      100

// --- 雷达区域加权 ---
// 中间区域权重大，两侧区域权重小，用于让正前方障碍更容易触发转向。
#define WEIGHT_CENTER           1.5f
#define WEIGHT_SIDE             0.5f

// --- 雷达净推力缩放系数 ---
// 参与计算的雷达点数量变少后，原始净推力数值也会变小，因此需要单独缩放。
// 调大后雷达避障更敏感，调小后雷达分支更温和。
#define FORCE_SCALING           0.025f

/*
 * 视觉循迹总体链路
 * 1. 外部视觉模块拿到偏移量后，调用 Navigation_LineTrack_Update() 只更新“最新输入”。
 * 2. GPT 10ms 周期继续统一调用 Navigation_Run_Logic()，这里只有这一处真正输出舵机和轮速。
 * 3. Navigation_Run_Logic() 的先后顺序固定为：
 *    - 先判断视觉是否仍有效
 *    - 再判断臂章标志位是否要屏蔽雷达
 *    - 若未屏蔽雷达，则优先处理雷达急停和旧避障 PID
 *    - 若雷达未接管，再进入新的循迹 PID
 *    - 偏移过大时，再在左右轮目标 RPM 上叠加差速辅助
 * 4. 视觉无效或超时时，默认安全策略是清循迹积分、舵机回中、双轮停车。
 */

/*
 * 调参指南
 * 1. 第一步只校正方向：
 *    如果视觉偏移越大，车头却往相反方向修正，先改 NAV_LINE_OFFSET_SIGN。
 * 2. 第二步调舵机循迹 PID：
 *    - Kp 调大：修正更快，但过大容易左右摆。
 *    - Ki 调大：能消除长期小偏差，但过大容易累计后突然甩头。
 *    - Kd 调大：抑制摆动更明显，但过大容易抖或放大噪声。
 * 3. 第三步调差速辅助：
 *    - NAV_LINE_DIFF_ENTER_ABS 调小：更早介入差速。
 *    - NAV_LINE_DIFF_EXIT_ABS 必须小于进入阈值：这样才能形成滞回，避免边界抖动。
 *    - NAV_LINE_DIFF_KP 调大：大偏移时回正更猛。
 *    - NAV_LINE_DIFF_MAX_RPM 调大：极限修正更强，但更容易破坏原始速度分配。
 * 4. 第四步调死区和超时：
 *    - NAV_LINE_DEADBAND 调大：更稳，但细调能力下降。
 *    - NAV_LINE_VALID_TIMEOUT_MS 调小：丢帧后停得更快，但帧率低时更容易误停。
 */

/*
 * 视觉偏移符号控制
 * 1. 作用：统一控制“视觉偏移量正负”和“期望转向方向”的对应关系。
 * 2. 如果确认视觉模块输出方向和底盘修正方向相反，直接把 1.0f 改成 -1.0f。
 * 3. 推荐先校这个宏，再动 PID。
 */
#define NAV_LINE_OFFSET_SIGN      (1.0f)

/*
 * 视觉偏移死区
 * 1. 当偏移绝对值小于该值时，按 0 处理，避免小噪声来回抖舵机。
 * 2. 调大更稳，调小更灵敏。
 */
#define NAV_LINE_DEADBAND         (0.5f)

/*
 * 视觉有效超时阈值，单位毫秒
 * 1. 超过该时间没有收到新的有效视觉数据，就认为本次视觉失效。
 * 2. 失效后会触发清积分、舵机回中、双轮停车。
 */
#define NAV_LINE_VALID_TIMEOUT_MS (200U)

/*
 * 大偏移差速辅助进入阈值
 * 1. abs(offset) 超过该值时，除了舵机修正，还会叠加左右轮差速。
 * 2. 调小后更早介入，调大后更晚介入。
 */
#define NAV_LINE_DIFF_ENTER_ABS   (20.0f)

/*
 * 大偏移差速辅助退出阈值
 * 1. abs(offset) 回落到该值以下时，退出差速辅助。
 * 2. 必须小于进入阈值，用来形成滞回。
 */
#define NAV_LINE_DIFF_EXIT_ABS    (10.0f)

/*
 * 差速辅助比例系数
 * 1. 把视觉偏移量换算成附加的左右轮 RPM 修正量。
 * 2. 调大后大偏移回正更猛，调小后更柔和。
 */
#define NAV_LINE_DIFF_KP          (0.6f)

/*
 * 差速辅助最大附加 RPM
 * 1. 限制差速辅助最多能在左右轮原目标速度上叠加多少 RPM。
 * 2. 调大后极限修正更强，但也更容易让底盘动作过猛。
 */
#define NAV_LINE_DIFF_MAX_RPM     (0.0f)	// 30

/* ================= 结构体与对外变量 ================= */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float error;
    float prev_error;
    float integral;
    float output;
    float max_output;
    float max_integral;
} Nav_PID_t;

extern Nav_PID_t g_pid_avoid;
extern Nav_PID_t g_pid_line;

extern volatile float g_nav_line_offset;
extern volatile uint8_t g_nav_line_valid;
extern volatile uint8_t g_nav_badge_enable;
extern volatile int g_nav_target_left_speed;
extern volatile int g_nav_target_right_speed;
/* 导航调试观测量：
 * 1. g_nav_debug_servo_angle：当前导航控制最终下发的舵机角度（已过方向反转与限幅）。
 * 2. g_nav_debug_line_pid_output：循迹 PID 原始输出（未做舵机方向反转）。
 * 3. g_nav_debug_diff_rpm：大偏移差速辅助当前叠加量（RPM，带正负号）。
 */
extern volatile float g_nav_debug_servo_angle;
extern volatile float g_nav_debug_line_pid_output;
extern volatile float g_nav_debug_diff_rpm;

/*
 * 兼容旧命名
 * 1. 保留旧名字，避免外部代码立刻因为改名而编译失败。
 * 2. 新代码统一使用 g_nav_line_* 和 g_pid_line。
 */
#define g_camera_error g_nav_line_offset
#define g_camera_valid g_nav_line_valid
#define g_pid_lane     g_pid_line

/*
 * 接口说明
 * 1. Navigation_LineTrack_Update():
 *    输入视觉偏移量、视觉有效位、左轮目标 RPM、右轮目标 RPM。
 *    只更新缓存和状态，不直接控制底盘。
 * 2. Navigation_SetBadgeEnable():
 *    enable=0 表示允许雷达避障接管。
 *    enable!=0 表示臂章阶段屏蔽雷达，避免机械臂动作造成雷达误触发。
 * 3. Navigation_Run_Logic():
 *    GPT 10ms 周期唯一执行入口，统一输出舵机和轮速。
 */
void Navigation_Init(void);
void Navigation_LineTrack_Update(float vision_offset, uint8_t vision_valid, int left_target_rpm, int right_target_rpm);
void Navigation_SetBadgeEnable(uint8_t enable);
void Navigation_Run_Logic(int target_speed);
uint8_t Navigation_FetchAvoidExitEvent(void);
void Navigation_ClearAvoidExitEvent(void);
void Navigation_TestFeedDemo(void); //测试代码假装视觉输入，周期性更新 g_nav_line_offset 和 g_nav_line_valid
void Navigation_Debug_Print(void);

#endif
