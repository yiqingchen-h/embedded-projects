#ifndef FOC_APP_ENCODER_READOUT_H
#define FOC_APP_ENCODER_READOUT_H
#include <stdbool.h>
#include <stdint.h>

/* MT6835单圈原始码值；读数不使用电角度或电机极对数换算。 */
#define ENCODER_COUNTS_PER_TURN 2097152U
/* 观测样本超过20ms不再用于显示有效角度或转速。 */
#define ENCODER_STALE_MS 20U

/* 通信、磁场和速度资格分别保留，避免弱磁场被误报为通信失败。 */
typedef enum {
    ENCODER_STARTING, ENCODER_NO_DEVICE, ENCODER_IO_ERROR,
    ENCODER_CRC_ERROR, ENCODER_WEAK_FIELD, ENCODER_SENSOR_FAULT,
    ENCODER_OK, ENCODER_STALE
} encoder_state_t;

typedef struct {
    /* 最近一次读数的状态。 */
    encoder_state_t state;
    /* 最近CRC正确的21位原始机械角，即使磁场告警也保留供诊断。 */
    uint32_t raw;
    /* 原始机械角对应的毫度，范围0～359999。 */
    uint32_t angle_mdeg;
    /* 20ms角度增量窗的机械RPM，原始角增加为正；不参与电机控制。 */
    int32_t rpm;
    /* 最近CRC正确样本的芯片状态字。 */
    uint8_t status;
    /* 通信和磁场均有效时才允许显示角度。 */
    bool angle_valid;
    /* 连续有效样本形成速度窗口后为true。 */
    bool speed_valid;
    /* 最近CRC正确样本距当前的毫秒数，无样本时为UINT32_MAX。 */
    uint32_t age_ms;
    /* CRC正确样本数、IO失败数及CRC失败数。 */
    uint32_t samples, io_errors, crc_errors;
} encoder_snapshot_t;

typedef struct {
    /* 观测状态只由主循环持有，没有ISR共享缓存。 */
    encoder_snapshot_t value;
    /* 调用方提供的时间戳计数频率，单位Hz。 */
    uint32_t tick_hz;
    /* 最近CRC正确样本时间、最近有效角度及时间戳。 */
    uint32_t sample_ms, previous_raw, previous_tick, previous_ms;
    /* 当前速度窗口的有符号角增量与计数器增量。 */
    int32_t window_delta;
    uint32_t window_ticks;
    /* 无效读数或过长间隔后，需要重新建立相邻样本基准。 */
    bool previous_valid;
} encoder_readout_t;

/** @brief 初始化只读角度与转速观测器。
 * @param readout 状态。 @param tick_hz 时间戳计数频率，必须大于等于1000Hz。 */
void encoder_readout_init(encoder_readout_t *readout, uint32_t tick_hz);
/** @brief 输入一个CRC正确的角度样本，磁场或状态告警使读数无效。
 * @param readout 状态。 @param raw 21位机械角。 @param status 芯片低三位状态。
 * @param now_ms 当前毫秒。 @param tick 本次采样的自由运行计数器值，允许无符号回绕。 */
void encoder_readout_sample(encoder_readout_t *readout, uint32_t raw, uint8_t status, uint32_t now_ms, uint32_t tick);
/** @brief 通信失败时立即撤销角度与转速有效性，禁止跨错误计算速度。
 * @param readout 状态。 @param error NO_DEVICE、IO_ERROR或CRC_ERROR。 */
void encoder_readout_error(encoder_readout_t *readout, encoder_state_t error);
/** @brief 获取带新鲜度检查的快照，不把陈旧读数继续标有效。
 * @param readout 状态。 @param now_ms 当前毫秒。 @return 快照。 */
encoder_snapshot_t encoder_readout_snapshot(const encoder_readout_t *readout, uint32_t now_ms);
/** @brief 状态对应的稳定协议名称。 @param state 状态。 @return 大写ASCII名称。 */
const char *encoder_state_name(encoder_state_t state);
#endif
