#ifndef HARDWARE_MECHANICAL_ARM_MECHANICAL_ARM_FEEDBACK_H_
#define HARDWARE_MECHANICAL_ARM_MECHANICAL_ARM_FEEDBACK_H_

#include "hal_data.h"
#include "hardware/mechanical_arm/Mechanical_hand_control.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  机械臂舵机反馈快照
 */
typedef struct st_mechanical_arm_feedback_snapshot
{
    /* true 表示机械臂通信已初始化。 */
    bool initialized;

    /* 下一次将要轮询的舵机序号。 */
    uint8_t polling_index;

    /* 最近一次反馈任务错误码。 */
    int last_error;

    /* 全局反馈更新序号。 */
    uint32_t sequence;

    /* 6 路舵机反馈缓存。 */
    mech_hand_servo_feedback_t servo[MECH_HAND_SERVO_COUNT];
} mechanical_arm_feedback_snapshot_t;

/**
 * @brief  机械臂舵机反馈任务对象式 API
 */
typedef struct st_mechanical_arm_feedback_api
{
    /**
     * @brief  初始化机械臂反馈缓存并尝试打开机械臂通信
     *
     * @return fsp_err_t FSP_SUCCESS 表示机械臂通信已初始化
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  按 100ms 周期轮询一路舵机反馈
     *
     * @return void
     */
    void (*process_100ms_tick)(void);

    /**
     * @brief  请求优先刷新指定舵机反馈
     *
     * @param index 舵机序号，范围 0~5
     * @return void
     */
    void (*request_refresh)(uint8_t index);

    /**
     * @brief  请求优先刷新全部舵机反馈
     *
     * @return void
     */
    void (*request_refresh_all)(void);

    /**
     * @brief  读取当前反馈快照
     *
     * @param p_snapshot 快照输出指针
     * @return void
     */
    void (*read_snapshot)(mechanical_arm_feedback_snapshot_t *p_snapshot);

    /**
     * @brief  获取机械臂 UART 总线互斥锁
     *
     * @param timeout_ms 等待超时时间，单位 ms
     * @return bool true 表示获取成功
     */
    bool (*lock_bus)(uint32_t timeout_ms);

    /**
     * @brief  释放机械臂 UART 总线互斥锁
     *
     * @return void
     */
    void (*unlock_bus)(void);
} mechanical_arm_feedback_api_t;

extern const mechanical_arm_feedback_api_t mechanical_arm_feedback;

#endif /* HARDWARE_MECHANICAL_ARM_MECHANICAL_ARM_FEEDBACK_H_ */
