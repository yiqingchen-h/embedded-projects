#ifndef FOC_CONTROL_ENCODER_MOTION_H
#define FOC_CONTROL_ENCODER_MOTION_H
#include <stdint.h>
#include <stdbool.h>

/* 循环保留最近5.12秒，主机持续分批读取，不限制运行时长。 */
#define ENCODER_MOTION_CAPACITY 256U
/* 每次串口返回六个原始点，完整响应可放入原512字节协议缓冲。 */
#define ENCODER_MOTION_BATCH 6U

typedef struct {
    /* 自首帧以来真实DWT累计tick的低/高32位，支持跨25秒硬件回绕。 */
    uint32_t time_lo, time_hi;
    /* 按校准方向展开的机械原码累计位置低/高32位，二补码；没有滤波。 */
    uint32_t position_lo, position_hi;
    /* 原始未加电零点的机械角码，以及本20ms内1kHz原始位移最小/最大值。 */
    uint32_t raw;
    int32_t min_delta, max_delta;
} encoder_motion_sample_t;

typedef struct {
    /* 只有1kHz生产者更新，主循环批读时由板卡提供短临界区。 */
    uint32_t total, tick_hz, last_tick, last_raw, phase, gaps;
    /* 硬件原计时器与机械角先展开再记录，不以控制目标或估算速度代替。 */
    uint64_t elapsed;
    int64_t position;
    int32_t direction, min_delta, max_delta;
    bool initialized;
    /* 单生产者循环缓存；total作为发布进度，读者自行检查被覆盖的序号。 */
    encoder_motion_sample_t sample[ENCODER_MOTION_CAPACITY];
} encoder_motion_t;

/** @brief 停机启动前初始化独立测量，不修改任何控制/保护状态。
 * @param state 测量实例。 @param direction 已验证方向±1。 @param tick_hz 时间戳频率Hz。 */
void encoder_motion_init(encoder_motion_t *state, int32_t direction, uint32_t tick_hz);
/** @brief 1kHz输入已由原反馈保护校验的原始机械角，首点和每20次更新发布原记录。
 * @param state 测量实例。 @param raw 未滤波机械原码。 @param tick 该原始角的真实采样时间戳。 */
void encoder_motion_update(encoder_motion_t *state, uint32_t raw, uint32_t tick);
/** @brief 在调用者短临界区内读取循环缓存，返回真实可用起点，不补造丢失记录。
 * @param state 实例。 @param first 请求序号，返回实际起点。 @param out 六点缓冲。
 * @return 实际返回点数，最新序号之后返回零。 */
uint32_t encoder_motion_read(const encoder_motion_t *state, uint32_t *first, encoder_motion_sample_t *out);
#endif
