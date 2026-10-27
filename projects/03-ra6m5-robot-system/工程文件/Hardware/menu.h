#ifndef MENU_H_
#define MENU_H_
#include "hal_data.h"
#include "UART.h"
#include "SysTick.h"
#include "lcd_init.h"
#include "lcd.h"
#include "Hardware_IIC.h"
#include "GPT.h"
#include "Wheel_control.h"
#include "Key.h"
#include "Navigation.h"
#include "Mechanical_hand_control.h"
#include "Calibration.h"
#include "Vision_Uart9_Comm.h"
#include "Robot_Control.h"
// ==========================================
// 菜单模块说明
// 1. Menu_Draw() 负责根据当前状态绘制对应界面。
// 2. Menu_Key_Process() 负责统一分发按键与编码器事件。
// 3. 各个菜单页面尽量遵循“静态内容首次绘制，动态内容按需刷新”的思路，
//    这样可以减少整屏重绘，提高界面响应速度。
// 4. 机械手调试页是在原菜单框架上扩展出来的一个独立页面，
//    既支持末端坐标调试，也支持单舵机独立调试。
// ==========================================

// 菜单状态枚举
typedef enum {
    MENU_MAIN = 0,  // 主菜单
    MENU_PID_EDIT,  // PID 调节模式
    MENU_RUNNING,   // 运行监视模式
    MENU_SPEDD_SET, // 单独速度设置和监视画面
    MENU_NAV_EDIT,  // 导航避障 PID 及速度调节模式
    MENU_MECH_DEBUG // 机械手调试界面
} MenuState_t;

// 速度pid 数据结构体
typedef struct {
    double kp;
    double ki;
    double kd;
    uint32_t magic; // 校验标记
} PID_Data_t;
// 导航pid
typedef struct {
    double nav_kp;
    double nav_ki;
    double nav_kd;
    uint32_t magic; // 校验标记 (0xEEA)
} Nav_PID_Data_t;

// 循迹PID（独立于避障PID，单独保存）
typedef struct {
    double line_kp;
    double line_ki;
    double line_kd;
    uint32_t magic; // 校验标记 (0xEEB)
} Nav_Line_PID_Data_t;

// 速度设置结构体
typedef struct {
    int L_speed;
    int R_speed;
    short angle;
    int elect;
} Speed_Data_t;

// ==========================================
// 机械手调试界面数据结构
// 1. x / y / z / time_ms:
//    用于末端坐标调试，对应 MechanicalHand_MoveTo()。
// 2. servo_index / servo_pwm:
//    用于单舵机独立调试，对应 MechanicalHand_SendSingleServo()。
// 3. servo_positions:
//    用于在菜单中显示当前软件缓存的 S0~S5 位置。
//    注意这里显示的是“最近一次成功下发的目标 PWM”，不是传感器真实反馈。
// 4. elect:
//    表示当前菜单光标选中的可编辑项。
// 5. last_result:
//    保存最近一次机械手接口调用返回值，用于界面显示 RET。
// 6. last_clamped:
//    表示最近一次结构化命令是否触发了软件限位裁剪。
// ==========================================
typedef struct {
    float x;
    float y;
    float z;
    uint16_t time_ms;
    uint8_t servo_index;
    uint16_t servo_pwm;
    int8_t elect;
    int last_result;
    bool gripper_opened;
    mech_hand_servo_positions_t servo_positions;
    bool last_clamped;
} Mech_Debug_Data_t;

extern MenuState_t g_state;
extern int8_t g_cursor;
extern int8_t g_pid_item;
extern bool g_need_refresh;
extern PID_Data_t menu_pid_data;
extern Nav_PID_Data_t menu_nav_pid_data; // 导航PID全局变量
extern Nav_Line_PID_Data_t menu_line_pid_data; // 循迹PID全局变量
extern I2CDev *ptEepromDev;              // 引用外部的 I2C 设备指针

// ==========================================
// 对外函数说明
// 1. Menu_Draw():
//    根据当前 g_state 绘制菜单页面。
// 2. Menu_Key_Process():
//    接收按键驱动层上传的事件，完成菜单切换、参数修改和动作触发。
// 3. Load/Save_PID_From_EEPROM():
//    负责速度环 PID 参数的掉电保存与恢复。
// 4. Load/Save_Nav_PID_From_EEPROM():
//    负责导航页两套 PID（避障 PID + 循迹 PID）的掉电保存与恢复。
// ==========================================
void Menu_Draw(void);
void Menu_Key_Process(key_event_t event);
void Load_PID_From_EEPROM(void);
void Save_PID_To_EEPROM(void);
// 导航 PID 读写
void Load_Nav_PID_From_EEPROM(void);
void Save_Nav_PID_To_EEPROM(void);

#endif
