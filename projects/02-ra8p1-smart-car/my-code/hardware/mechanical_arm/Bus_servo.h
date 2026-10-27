#ifndef BUS_SERVO_H_
#define BUS_SERVO_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  总线舵机 ID 约定，255 为广播 ID
 */
#define BUS_SERVO_ID_MAX        (254U)
#define BUS_SERVO_ID_BROADCAST  (255U)

/**
 * @brief  总线舵机位置和动作时间协议边界
 */
#define BUS_SERVO_PWM_MIN       (500U)
#define BUS_SERVO_PWM_MAX       (2500U)
#define BUS_SERVO_TIME_MAX      (9999U)

/**
 * @brief  UART4 收发等待参数，单位 ms；响应以 '!' 作为结束符
 */
#define BUS_SERVO_TX_TIMEOUT_MS       (200U)
#define BUS_SERVO_RX_TIMEOUT_MS       (200U)
#define BUS_SERVO_RESPONSE_SIZE       (64U)
#define BUS_SERVO_LAST_CMD_SIZE       (160U)
#define BUS_SERVO_MULTI_MAX_COUNT     (16U)

/**
 * @brief  总线舵机模块返回码
 */
typedef enum e_bus_servo_status
{
    BUS_SERVO_OK             = 0,
    BUS_SERVO_ERR_PARAM      = -1,
    BUS_SERVO_ERR_UART_OPEN  = -2,
    BUS_SERVO_ERR_UART_WRITE = -3,
    BUS_SERVO_ERR_NOT_READY  = -4,
    BUS_SERVO_ERR_UART_CLOSE = -5,
    BUS_SERVO_ERR_FORMAT     = -6,
    BUS_SERVO_ERR_TIMEOUT    = -7,
    BUS_SERVO_ERR_RESPONSE   = -8,
    BUS_SERVO_ERR_UART_READ  = -9
} bus_servo_status_t;

/**
 * @brief  总线舵机工作模式，数值直接对应舵机指令表
 */
typedef enum e_bus_servo_mode
{
    BUS_SERVO_MODE_270_CW           = 1U,
    BUS_SERVO_MODE_270_CCW          = 2U,
    BUS_SERVO_MODE_180_CW           = 3U,
    BUS_SERVO_MODE_180_CCW          = 4U,
    BUS_SERVO_MODE_MOTOR_CIRCLE_CW  = 5U,
    BUS_SERVO_MODE_MOTOR_CIRCLE_CCW = 6U,
    BUS_SERVO_MODE_MOTOR_TIME_CW    = 7U,
    BUS_SERVO_MODE_MOTOR_TIME_CCW   = 8U
} bus_servo_mode_t;

/**
 * @brief  总线舵机上电动作模式，数值直接对应舵机指令表
 */
typedef enum e_bus_servo_startup_mode
{
    BUS_SERVO_STARTUP_MODE_TO_START_POS = 1U,
    BUS_SERVO_STARTUP_MODE_HOLD_CURRENT = 2U,
    BUS_SERVO_STARTUP_MODE_FREE         = 3U
} bus_servo_startup_mode_t;

/**
 * @brief  单个舵机移动命令，用于组合多舵机同步发送
 */
typedef struct st_bus_servo_move
{
    uint8_t id;          /* 舵机 ID，组合发送不允许使用广播 ID */
    uint16_t pwm;        /* 目标位置，范围 BUS_SERVO_PWM_MIN~BUS_SERVO_PWM_MAX */
    uint16_t time_ms;    /* 动作时间，单位 ms */
} bus_servo_move_t;

/**
 * @brief  舵机温度和电压原始状态
 */
typedef struct st_bus_servo_telemetry
{
    uint8_t id;              /* 回包中的舵机 ID */
    int16_t temperature_c;   /* 温度，单位摄氏度 */
    uint8_t voltage_raw;     /* 电压原始值，未换算成实际电压 */
} bus_servo_telemetry_t;

/**
 * @brief  总线舵机对象式 API
 *
 * @note   应用层只通过 bus_servo 对象调用本模块；旧函数名不再对外暴露。
 * @note   本模块不提供波特率修改接口，避免 UART4 与舵机波特率不一致后失联。
 */
typedef struct st_bus_servo_api
{
    /**
     * @brief  打开 UART4 并初始化总线舵机通信状态
     *
     * @return int  BUS_SERVO_OK 表示成功
     */
    int (*init)(void);

    /**
     * @brief  关闭 UART4 并清理总线舵机通信状态
     *
     * @return void
     */
    void (*deinit)(void);

    /**
     * @brief  查询 UART4 是否已经由本模块打开
     *
     * @return bool  true 表示已初始化
     */
    bool (*is_initialized)(void);

    /**
     * @brief  控制单个舵机移动到指定 PWM
     *
     * @param id       舵机 ID，可为广播 ID
     * @param pwm      目标 PWM
     * @param time_ms  动作时间，单位 ms
     * @return int     BUS_SERVO_OK 表示发送成功
     */
    int (*move)(uint8_t id, uint16_t pwm, uint16_t time_ms);

    /**
     * @brief  一次发送多个舵机移动命令
     *
     * @param moves  移动命令数组
     * @param count  命令数量，不能超过 BUS_SERVO_MULTI_MAX_COUNT
     * @return int   BUS_SERVO_OK 表示发送成功
     */
    int (*move_multi)(const bus_servo_move_t *moves, uint8_t count);

    /**
     * @brief  发送原始调试命令
     *
     * @param cmd            完整 ASCII 命令
     * @param response       回包缓冲区，无需回包时传入 NULL
     * @param response_size  回包缓冲区大小
     * @param timeout_ms     接收超时，传入 0 时使用默认超时
     * @return int           BUS_SERVO_OK 表示成功
     */
    int (*send_raw)(const char *cmd, char *response, uint32_t response_size, uint32_t timeout_ms);

    /**
     * @brief  读取舵机固件版本字符串
     *
     * @param id            舵机 ID，广播读取只允许单舵机在线时使用
     * @param version       版本字符串输出缓冲区
     * @param version_size  版本字符串缓冲区大小
     * @return int          BUS_SERVO_OK 表示读取成功
     */
    int (*read_version)(uint8_t id, char *version, uint32_t version_size);

    /**
     * @brief  读取舵机 ID
     *
     * @param id       舵机 ID，广播读取只允许单舵机在线时使用
     * @param read_id  读取到的舵机 ID
     * @return int     BUS_SERVO_OK 表示读取成功
     */
    int (*read_id)(uint8_t id, uint8_t *read_id);

    /**
     * @brief  修改舵机 ID，必须只连接一个舵机后再调用
     *
     * @param id      当前舵机 ID
     * @param new_id  新舵机 ID，不能为广播 ID
     * @return int    BUS_SERVO_OK 表示设置成功
     */
    int (*set_id)(uint8_t id, uint8_t new_id);

    /**
     * @brief  释放舵机扭力，机械臂可能下落，调用前必须托住结构
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*release_torque)(uint8_t id);

    /**
     * @brief  恢复舵机扭力
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*restore_torque)(uint8_t id);

    /**
     * @brief  读取舵机工作模式
     *
     * @param id    舵机 ID，广播读取只允许单舵机在线时使用
     * @param mode  工作模式输出
     * @return int  BUS_SERVO_OK 表示读取成功
     */
    int (*read_mode)(uint8_t id, uint8_t *mode);

    /**
     * @brief  设置舵机工作模式
     *
     * @param id    舵机 ID，可为广播 ID
     * @param mode  工作模式，参考 bus_servo_mode_t
     * @return int  BUS_SERVO_OK 表示设置成功
     */
    int (*set_mode)(uint8_t id, uint8_t mode);

    /**
     * @brief  读取舵机当前位置 PWM
     *
     * @param id   舵机 ID，广播读取只允许单舵机在线时使用
     * @param pwm  当前位置 PWM 输出
     * @return int BUS_SERVO_OK 表示读取成功
     */
    int (*read_position)(uint8_t id, uint16_t *pwm);

    /**
     * @brief  暂停当前动作
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*pause)(uint8_t id);

    /**
     * @brief  继续被暂停的动作
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*continue_motion)(uint8_t id);

    /**
     * @brief  停止指定舵机动作
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*stop)(uint8_t id);

    /**
     * @brief  广播停止所有舵机动作
     *
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*stop_all)(void);

    /**
     * @brief  设置当前机械位置为中值
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*correct_middle)(uint8_t id);

    /**
     * @brief  设置当前位置为上电启动位置
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*set_current_as_startup)(uint8_t id);

    /**
     * @brief  设置上电启动模式
     *
     * @param id    舵机 ID，可为广播 ID
     * @param mode  上电启动模式，参考 bus_servo_startup_mode_t
     * @return int  BUS_SERVO_OK 表示设置成功
     */
    int (*set_startup_mode)(uint8_t id, uint8_t mode);

    /**
     * @brief  读取上电启动模式
     *
     * @param id    舵机 ID，广播读取只允许单舵机在线时使用
     * @param mode  上电启动模式输出
     * @return int  BUS_SERVO_OK 表示读取成功
     */
    int (*read_startup_mode)(uint8_t id, uint8_t *mode);

    /**
     * @brief  设置当前位置为最小值
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*set_current_as_min)(uint8_t id);

    /**
     * @brief  设置当前位置为最大值
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*set_current_as_max)(uint8_t id);

    /**
     * @brief  恢复出厂但保留 ID
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*factory_reset_keep_id)(uint8_t id);

    /**
     * @brief  完全恢复出厂，ID 会恢复为 000，多舵机在线时禁止使用
     *
     * @param id    舵机 ID，可为广播 ID
     * @return int  BUS_SERVO_OK 表示命令成功
     */
    int (*factory_reset_all)(uint8_t id);

    /**
     * @brief  读取舵机温度和电压原始值
     *
     * @param id         舵机 ID，广播读取只允许单舵机在线时使用
     * @param telemetry  温度和电压原始值输出
     * @return int       BUS_SERVO_OK 表示读取成功
     */
    int (*read_temperature_voltage)(uint8_t id, bus_servo_telemetry_t *telemetry);

    /**
     * @brief  获取最近一次发送的原始命令
     *
     * @return const char *  最近一次命令字符串
     */
    const char *(*get_last_command)(void);

    /**
     * @brief  获取最近一次收到的回包
     *
     * @return const char *  最近一次回包字符串
     */
    const char *(*get_last_response)(void);

    /**
     * @brief  获取最近一次错误码
     *
     * @return int  最近一次错误码
     */
    int (*get_last_error)(void);
} bus_servo_api_t;

extern const bus_servo_api_t bus_servo;

#endif /* BUS_SERVO_H_ */
