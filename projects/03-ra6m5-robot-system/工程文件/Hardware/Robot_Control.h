#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

#include <stdint.h>

#include "Vision_Uart9_Comm.h"

/*
 * Robot_Control 模块说明
 *
 * 1. 模块定位
 *    整车控制协调层，职责包括：
 *    - 主循环中的慢任务协调：视觉数据消费、机械臂模式切换、抓取/放置流程推进。
 *    - 10ms 周期中的快任务协调：编码器测速后的底盘执行入口。
 *    - 跨模块状态缓存：缓存最近一次视觉帧、夹爪请求、机械臂目标坐标、当前控制模式。
 *
 * 2. 与其他模块的边界
 *    - Vision_Uart9_Comm：只负责 UART9 的收帧和解帧，本模块负责解释这些字段的业务含义。
 *    - Navigation：只负责循迹控制和底盘输出，本模块决定何时把视觉数据送给 Navigation。
 *    - Mechanical_hand_control：只负责机械臂和夹爪实际动作，本模块负责何时调用这些动作。
 *    - Calibration：只负责坐标转换，本模块决定什么时候把视觉坐标送进去转换。
 *
 * 3. 使用方式
 *    - 在系统初始化阶段调用 RobotControl_Init()。
 *    - 在主循环中持续调用 RobotControl_MainLoopTask()。
 *    - 在 10ms 周期中调用 RobotControl_TimerTask_10ms()。
 *
 * 4. 当前业务规则
 *    - 普通视觉帧：u/v 分别作为左右轮目标速度，yaw 用于循迹偏差。
 *    - 9999,9999,9999,9999：进入机械臂模式并停车。
 *    - u,v,depth,9988：抓取目标帧。
 *    - u,v,depth,8899：放置目标帧。
 *
 */

/*
 * 初始化控制协调层。
 *
 * 作用：
 * - 清空内部上下文。
 * - 恢复默认模式为“循迹模式”。
 * - 初始化夹爪软件状态和最近一次结果缓存。
 *
 * 调用时机：
 * - 上电初始化阶段调用一次。
 *
 * 注意：
 * - 该函数只初始化协调层状态，不会主动驱动底盘或机械臂。
 */
void RobotControl_Init(void);

/*
 * 主循环任务入口。
 *
 * 作用：
 * - 协调主循环中的慢任务。
 * - 依次完成雷达数据解析、视觉组帧解析、视觉业务处理、机械臂流程推进、夹爪请求执行。
 *
 * 调用时机：
 * - 放在 hal_entry.c 的 while(1) 主循环中持续调用。
 *
 * 当前内部处理顺序：
 * 1. Lidar_Process_Data()
 * 2. VisionUart9_Process()
 * 3. RobotControl_ProcessVisionInput()
 * 4. RobotControl_ProcessArmTarget()
 * 5. RobotControl_ProcessGripperRequest()
 *
 * 维护建议：
 * - 新增慢任务时，优先考虑放在这个入口中统一调度。
 * - 不要把阻塞型机械臂动作直接塞进定时器中断，应继续放在主循环侧推进。
 */
void RobotControl_MainLoopTask(void);

/*
 * 10ms 周期任务入口。
 *
 * 作用：
 * - 处理快周期控制逻辑。
 * - 更新编码器速度，并执行一次底盘控制输出。
 *
 * 调用时机：
 * - 由 GPT 的 10ms 周期调度调用。
 *
 * 注意：
 * - 该函数面向“快任务”，不要把视觉解帧、机械臂阻塞动作这类慢逻辑加到这里。
 * - 后续若要改变底盘控制节拍，应同时检查 GPT 周期和这里的测速周期参数是否一致。
 */
void RobotControl_TimerTask_10ms(void);

/*
 * 设置是否允许视觉循迹数据接管 Navigation。
 *
 * 参数：
 * - enable：非 0 表示允许视觉数据继续送入 Navigation，0 表示禁止。
 *
 * 作用：
 * - 该开关只影响“视觉输入是否继续喂给 Navigation”。
 * - 不会直接改变小车是否 RUN/STOP，真正的底盘运行状态仍由 g_car_state 决定。
 *
 * 典型用途：
 * - 进入机械臂模式时关闭。
 * - 恢复循迹模式时重新打开。
 */
void RobotControl_SetLineTrackEnable(uint8_t enable);

/*
 * 读取当前视觉循迹使能标志。
 *
 * 返回值：
 * - 0：当前不允许视觉数据更新 Navigation。
 * - 非 0：当前允许视觉数据更新 Navigation。
 *
 * 典型用途：
 * - 菜单调试页面读取当前模式。
 * - 上层逻辑判断协调层当前是否屏蔽了循迹输入。
 */
uint8_t RobotControl_GetLineTrackEnable(void);

/*
 * 更新协调层缓存的最新视觉帧。
 *
 * 参数：
 * - frame：外部提供的一帧完整视觉数据。
 *
 * 返回值：
 * - VISION_UART9_OK：缓存成功。
 * - VISION_UART9_ERR_PARAM：输入参数为空。
 *
 * 说明：
 * - 该函数只负责“复制并缓存”视觉帧，不会直接驱动底盘或机械臂。
 * - 当前主要由 RobotControl_ProcessVisionInput() 内部调用。
 */
int RobotControl_UpdateVisionFrame(const vision_uart9_frame_t *frame);

/*
 * 读取协调层缓存的最近一帧视觉数据。
 *
 * 参数：
 * - frame：用于接收输出数据的缓冲区。
 *
 * 返回值：
 * - VISION_UART9_OK：读取成功。
 * - VISION_UART9_ERR_PARAM：输出指针为空。
 * - VISION_UART9_ERR_NOT_READY：当前还没有缓存过有效视觉帧。
 *
 * 典型用途：
 * - 菜单页或调试代码读取最近视觉数据。
 * - 其他上层逻辑想复用协调层缓存，而不是直接访问 Vision_Uart9_Comm。
 */
int RobotControl_GetLastVisionFrame(vision_uart9_frame_t *frame);

/*
 * 请求夹爪张开。
 *
 * 作用：
 * - 只登记一个“打开夹爪”的请求，不会在当前函数内立即执行动作。
 * - 真正执行在主循环中的 RobotControl_ProcessGripperRequest()。
 *
 * 典型用途：
 * - 菜单手动测试夹爪。
 * - 后续视觉/状态机逻辑触发张爪。
 */
void RobotControl_RequestGripperOpen(void);

/*
 * 请求夹爪闭合。
 *
 * 作用：
 * - 只登记一个“关闭夹爪”的请求，不会在当前函数内立即执行动作。
 * - 真正执行在主循环中的 RobotControl_ProcessGripperRequest()。
 */
void RobotControl_RequestGripperClose(void);

/*
 * 请求夹爪切换开合状态。
 *
 * 作用：
 * - 若当前软件状态认为夹爪已闭合，则后续执行张开。
 * - 若当前软件状态认为夹爪已张开，则后续执行闭合。
 *
 * 注意：
 * - 该函数依赖 gripper_is_closed 这个软件状态。
 * - 如果机械臂被外部设备手动操作导致软件状态失真，需要先修正内部状态再使用。
 */
void RobotControl_RequestGripperToggle(void);

/*
 * 获取最近一次夹爪动作执行结果。
 *
 * 返回值：
 * - 来自 Mechanical_hand_control 模块的返回码。
 *
 * 典型用途：
 * - 菜单页查看最近一次夹爪动作是否成功。
 * - 上层状态机决定是否进入错误处理。
 */
int RobotControl_GetLastGripperResult(void);
int RobotControl_GetLastArmTargetResult(void);
int RobotControl_GetArmStateRaw(void);
uint8_t RobotControl_IsArmWaiting(void);
uint32_t RobotControl_GetArmWaitRemainingMs(void);

/*
 * 循迹联调测试函数。
 *
 * 参数：
 * - u、v、depth、yaw：用于构造一帧假的视觉数据。
 * - target_speed：本次测试想强制使用的目标速度。
 *
 * 返回值：
 * - VISION_UART9_OK：测试流程执行成功。
 * - 其他值：视觉测试注入失败。
 *
 * 作用：
 * - 在没有真实视觉硬件时，伪造一帧视觉数据。
 * - 依次调用 RobotControl_MainLoopTask() 和 RobotControl_TimerTask_10ms()，
 *   走通“接收 -> 解析 -> 更新 Navigation -> 执行底盘输出”整条链路。
 *
 * 维护建议：
 * - 该函数主要用于联调，不建议作为正式业务逻辑入口。
 */
int RobotControl_TestSimulateLineTrackOnce(float u, float v, float depth, float yaw, int target_speed);

#endif
