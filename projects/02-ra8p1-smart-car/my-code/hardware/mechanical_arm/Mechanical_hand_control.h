#ifndef MECHANICAL_HAND_CONTROL_H_
#define MECHANICAL_HAND_CONTROL_H_

#include "hal_data.h"
#include "hardware/mechanical_arm/Bus_servo.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  机械臂动作层返回码
 *
 * @note   负数为参数、串口、协议等通用错误；正数 1~7 为逆运动学求解错误。
 */
typedef enum e_mech_hand_status {
    MECH_HAND_OK             = 0,
    MECH_HAND_ERR_PARAM      = -1,
    MECH_HAND_ERR_UART_OPEN  = -2,
    MECH_HAND_ERR_UART_WRITE = -3,
    MECH_HAND_ERR_NOT_READY  = -4,
    MECH_HAND_ERR_UART_CLOSE = -5,
    MECH_HAND_ERR_FORMAT     = -6,
    MECH_HAND_ERR_TIMEOUT    = -7,
    MECH_HAND_ERR_RESPONSE   = -8,
    MECH_HAND_ERR_UART_READ  = -9,

    MECH_HAND_IK_ERR_1 = 1,
    MECH_HAND_IK_ERR_2 = 2,
    MECH_HAND_IK_ERR_3 = 3,
    MECH_HAND_IK_ERR_4 = 4,
    MECH_HAND_IK_ERR_5 = 5,
    MECH_HAND_IK_ERR_6 = 6,
    MECH_HAND_IK_ERR_7 = 7
} mech_hand_status_t;

/**
 * @brief  四个主关节的目标 PWM
 */
typedef struct st_mech_hand_joint_pwm {
    uint16_t servo0_pwm; /* 底座旋转舵机 PWM */
    uint16_t servo1_pwm; /* 大臂舵机 PWM */
    uint16_t servo2_pwm; /* 小臂舵机 PWM */
    uint16_t servo3_pwm; /* 腕部舵机 PWM */
} mech_hand_joint_pwm_t;

/**
 * @brief  6 路舵机最近一次下发的目标 PWM 缓存
 *
 * @note   该缓存不是舵机真实反馈；真实位置需要调用 mechanical_hand.read_servo_position()。
 */
typedef struct st_mech_hand_servo_positions {
    uint16_t servo_pwm[6];
} mech_hand_servo_positions_t;

/**
 * @brief  机械臂舵机数量
 */
#define MECH_HAND_SERVO_COUNT (6U)

/**
 * @brief  单路舵机真实反馈缓存
 *
 * @note   位置、温度和电压来自总线舵机读取结果，不是最近一次下发目标。
 */
typedef struct st_mech_hand_servo_feedback {
    /* true 表示本路最近一次位置和遥测读取成功。 */
    bool valid;

    /* 舵机真实位置 PWM。 */
    uint16_t position_pwm;

    /* 舵机温度，单位摄氏度。 */
    int16_t temperature_c;

    /* 舵机电压原始值。 */
    uint8_t voltage_raw;

    /* 本路最近一次读取返回码。 */
    int last_error;

    /* 本路反馈更新序号。 */
    uint32_t sequence;
} mech_hand_servo_feedback_t;

/**
 * @brief  6 路舵机独立软件限位，避免调试命令直接撞机械限位
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
#define MECH_HAND_SERVO5_PWM_MIN (1400U)
#define MECH_HAND_SERVO5_PWM_MAX (1900U)

/**
 * @brief  菜单调试建议范围，不代表机械臂真实可达范围
 */
#define MECH_HAND_MENU_X_MIN (-250.0f)
#define MECH_HAND_MENU_X_MAX (200.0f)
#define MECH_HAND_MENU_Y_MIN (0.0f)
#define MECH_HAND_MENU_Y_MAX (300.0f)
#define MECH_HAND_MENU_Z_MIN (-100.0f)
#define MECH_HAND_MENU_Z_MAX (200.0f)

/**
 * @brief  默认 Home 目标，单位 mm，仍需调用 home() 才会动作
 */
#define MECH_HAND_HOME_X (0.0f)
#define MECH_HAND_HOME_Y (100.0f)
#define MECH_HAND_HOME_Z (60.0f)

/**
 * @brief  旧机械模型末端参考点到夹爪中心 TCP 的偏移，单位 mm
 *
 * @note   偏移定义在工具坐标系内，move_tcp_to 系列接口会按 yaw/alpha 转到基座坐标。
 * @note   默认全 0，保证 TCP 接口和旧参考点接口行为一致。
 */
#define MECH_HAND_TOOL_REF_OFFSET_X_MM (0.0f)
#define MECH_HAND_TOOL_REF_OFFSET_Y_MM (0.0f)
#define MECH_HAND_TOOL_REF_OFFSET_Z_MM (0.0f)

/**
 * @brief  机械臂对象式 API
 *
 * @note   应用层只通过 mechanical_hand 对象调用本模块。
 * @note   init() 只初始化 UART4 和内部状态，不会自动执行 home()，避免上电误动作。
 */
typedef struct st_mechanical_hand_api {
    /**
     * @brief  初始化机械臂动作层和底层总线舵机通信
     *
     * @return uint8_t  1 表示成功，0 表示失败
     */
    uint8_t (*init)(void);

    /**
     * @brief  反初始化机械臂动作层并关闭底层 UART4
     *
     * @return void
     */
    void (*deinit)(void);

    /**
     * @brief  按旧机械模型参考点移动，内部自动扫描 alpha
     *
     * @param x        基座坐标 x，单位 mm
     * @param y        基座坐标 y，单位 mm，要求 y >= 0
     * @param z        基座坐标 z，单位 mm
     * @param time_ms  动作时间，单位 ms
     * @return int     MECH_HAND_OK 表示命令成功
     */
    int (*move_to)(float x, float y, float z, uint16_t time_ms);

    /**
     * @brief  按旧机械模型参考点移动，使用调用者指定的 alpha
     *
     * @param x          基座坐标 x，单位 mm
     * @param y          基座坐标 y，单位 mm，要求 y >= 0
     * @param z          基座坐标 z，单位 mm
     * @param alpha_deg  末端俯仰角，0 为水平，负值向下
     * @param time_ms    动作时间，单位 ms
     * @return int       MECH_HAND_OK 表示命令成功
     */
    int (*move_to_with_alpha)(float x, float y, float z, float alpha_deg, uint16_t time_ms);

    /**
     * @brief  按夹爪中心 TCP 目标点移动，内部自动扫描 alpha
     *
     * @param x        TCP 目标基座坐标 x，单位 mm
     * @param y        TCP 目标基座坐标 y，单位 mm，要求 y >= 0
     * @param z        TCP 目标基座坐标 z，单位 mm
     * @param time_ms  动作时间，单位 ms
     * @return int     MECH_HAND_OK 表示命令成功
     */
    int (*move_tcp_to)(float x, float y, float z, uint16_t time_ms);

    /**
     * @brief  按夹爪中心 TCP 目标点移动，使用调用者指定的 alpha
     *
     * @param x          TCP 目标基座坐标 x，单位 mm
     * @param y          TCP 目标基座坐标 y，单位 mm，要求 y >= 0
     * @param z          TCP 目标基座坐标 z，单位 mm
     * @param alpha_deg  末端俯仰角，0 为水平，负值向下
     * @param time_ms    动作时间，单位 ms
     * @return int       MECH_HAND_OK 表示命令成功
     */
    int (*move_tcp_to_with_alpha)(float x, float y, float z, float alpha_deg, uint16_t time_ms);

    /**
     * @brief  发送单个舵机调试命令，会经过软件限位
     *
     * @param index    舵机序号，范围 0~5
     * @param pwm      目标 PWM
     * @param time_ms  动作时间，单位 ms
     * @return int     MECH_HAND_OK 表示命令成功
     */
    int (*send_single_servo)(uint8_t index, uint16_t pwm, uint16_t time_ms);

    /**
     * @brief  同步发送四个主关节舵机调试命令
     *
     * @param pwm0     底座舵机目标 PWM
     * @param pwm1     大臂舵机目标 PWM
     * @param pwm2     小臂舵机目标 PWM
     * @param pwm3     腕部舵机目标 PWM
     * @param time_ms  动作时间，单位 ms
     * @return int     MECH_HAND_OK 表示命令成功
     */
    int (*send_multi_servo)(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t time_ms);

    /**
     * @brief  发送原始调试命令，无法识别的命令会绕过高层限位
     *
     * @param cmd   完整 ASCII 调试命令
     * @return int  MECH_HAND_OK 表示命令成功
     */
    int (*send_raw_debug)(const char *cmd);

    /**
     * @brief  控制夹爪旋转舵机到指定 PWM
     *
     * @param pwm      目标 PWM
     * @param time_ms  动作时间，单位 ms
     * @return int     MECH_HAND_OK 表示命令成功
     */
    int (*gripper_rotate)(uint16_t pwm, uint16_t time_ms);

    /**
     * @brief  控制夹爪旋转舵机回中
     *
     * @param time_ms  动作时间，单位 ms
     * @return int     MECH_HAND_OK 表示命令成功
     */
    int (*gripper_rotate_center)(uint16_t time_ms);

    /**
     * @brief  打开夹爪
     *
     * @param time_ms  动作时间，单位 ms
     * @return int     MECH_HAND_OK 表示命令成功
     */
    int (*gripper_open)(uint16_t time_ms);

    /**
     * @brief  闭合夹爪
     *
     * @param time_ms  动作时间，单位 ms
     * @return int     MECH_HAND_OK 表示命令成功
     */
    int (*gripper_close)(uint16_t time_ms);

    /**
     * @brief  广播停止所有舵机动作
     *
     * @return int  MECH_HAND_OK 表示命令成功
     */
    int (*stop_all)(void);

    /**
     * @brief  停止指定舵机动作
     *
     * @param index  舵机序号，范围 0~5
     * @return int   MECH_HAND_OK 表示命令成功
     */
    int (*stop_servo)(uint8_t index);

    /**
     * @brief  移动到默认 Home 坐标
     *
     * @param time_ms  动作时间，单位 ms
     * @return int     MECH_HAND_OK 表示命令成功
     */
    int (*home)(uint16_t time_ms);

    /**
     * @brief  读取指定舵机真实位置 PWM
     *
     * @param index  舵机序号，范围 0~5
     * @param pwm    真实位置 PWM 输出
     * @return int   MECH_HAND_OK 表示读取成功
     */
    int (*read_servo_position)(uint8_t index, uint16_t *pwm);

    /**
     * @brief  读取指定舵机温度和电压原始值
     *
     * @param index      舵机序号，范围 0~5
     * @param telemetry  温度和电压原始值输出
     * @return int       MECH_HAND_OK 表示读取成功
     */
    int (*read_servo_telemetry)(uint8_t index, bus_servo_telemetry_t *telemetry);

    /**
     * @brief  读取指定舵机工作模式
     *
     * @param index  舵机序号，范围 0~5
     * @param mode   工作模式输出
     * @return int   MECH_HAND_OK 表示读取成功
     */
    int (*read_servo_mode)(uint8_t index, uint8_t *mode);

    /**
     * @brief  设置指定舵机工作模式
     *
     * @param index  舵机序号，范围 0~5
     * @param mode   工作模式，参考 bus_servo_mode_t
     * @return int   MECH_HAND_OK 表示设置成功
     */
    int (*set_servo_mode)(uint8_t index, uint8_t mode);

    /**
     * @brief  修改指定舵机 ID，必须只连接一个目标舵机后再调用
     *
     * @param index   当前舵机序号，范围 0~5
     * @param new_id  新舵机 ID，不能为广播 ID
     * @return int    MECH_HAND_OK 表示设置成功
     */
    int (*set_servo_id)(uint8_t index, uint8_t new_id);

    /**
     * @brief  释放指定舵机扭力，机械臂可能下落
     *
     * @param index  舵机序号，范围 0~5
     * @return int   MECH_HAND_OK 表示命令成功
     */
    int (*release_servo_torque)(uint8_t index);

    /**
     * @brief  恢复指定舵机扭力
     *
     * @param index  舵机序号，范围 0~5
     * @return int   MECH_HAND_OK 表示命令成功
     */
    int (*restore_servo_torque)(uint8_t index);

    /**
     * @brief  设置指定舵机当前位置为上电启动位置
     *
     * @param index  舵机序号，范围 0~5
     * @return int   MECH_HAND_OK 表示命令成功
     */
    int (*set_servo_current_as_startup)(uint8_t index);

    /**
     * @brief  设置指定舵机上电启动模式
     *
     * @param index  舵机序号，范围 0~5
     * @param mode   上电启动模式，参考 bus_servo_startup_mode_t
     * @return int   MECH_HAND_OK 表示设置成功
     */
    int (*set_servo_startup_mode)(uint8_t index, uint8_t mode);

    /**
     * @brief  读取指定舵机上电启动模式
     *
     * @param index  舵机序号，范围 0~5
     * @param mode   上电启动模式输出
     * @return int   MECH_HAND_OK 表示读取成功
     */
    int (*read_servo_startup_mode)(uint8_t index, uint8_t *mode);

    /**
     * @brief  设置指定舵机当前位置为中值
     *
     * @param index  舵机序号，范围 0~5
     * @return int   MECH_HAND_OK 表示命令成功
     */
    int (*correct_servo_middle)(uint8_t index);

    /**
     * @brief  设置指定舵机当前位置为最小值
     *
     * @param index  舵机序号，范围 0~5
     * @return int   MECH_HAND_OK 表示命令成功
     */
    int (*set_servo_current_as_min)(uint8_t index);

    /**
     * @brief  设置指定舵机当前位置为最大值
     *
     * @param index  舵机序号，范围 0~5
     * @return int   MECH_HAND_OK 表示命令成功
     */
    int (*set_servo_current_as_max)(uint8_t index);

    /**
     * @brief  恢复出厂但保留 ID
     *
     * @param index  舵机序号，范围 0~5
     * @return int   MECH_HAND_OK 表示命令成功
     */
    int (*factory_reset_keep_id)(uint8_t index);

    /**
     * @brief  完全恢复出厂，ID 会恢复为 000，多舵机在线时禁止使用
     *
     * @param index  舵机序号，范围 0~5
     * @return int   MECH_HAND_OK 表示命令成功
     */
    int (*factory_reset_all)(uint8_t index);

    /**
     * @brief  获取最近一次错误码
     *
     * @return int  最近一次错误码
     */
    int (*get_last_error)(void);

    /**
     * @brief  获取底层总线最近一次发送的原始命令
     *
     * @return const char *  最近一次命令字符串
     */
    const char *(*get_last_command)(void);

    /**
     * @brief  获取底层总线最近一次收到的回包
     *
     * @return const char *  最近一次回包字符串
     */
    const char *(*get_last_response)(void);

    /**
     * @brief  获取最近一次下发的 6 路目标 PWM 缓存
     *
     * @param positions  PWM 缓存输出，可传入 NULL
     * @return void
     */
    void (*get_servo_positions)(mech_hand_servo_positions_t *positions);

    /**
     * @brief  查询最近一次结构化命令是否被软件限位裁剪
     *
     * @return bool  true 表示最近一次结构化命令发生过限位裁剪
     */
    bool (*was_last_command_clamped)(void);
} mechanical_hand_api_t;

extern const mechanical_hand_api_t mechanical_hand;

#endif /* MECHANICAL_HAND_CONTROL_H_ */
