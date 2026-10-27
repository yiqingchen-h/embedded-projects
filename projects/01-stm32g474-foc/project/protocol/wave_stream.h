#ifndef FOC_WAVE_STREAM_H
#define FOC_WAVE_STREAM_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 仅用于观测的固定容量队列；不能参与控制或保护。 */
#define WAVE_RING_CAPACITY 256U
#define WAVE_BLOCK_SAMPLES 32U
#define WAVE_PACKET_CAPACITY 512U
#define WAVE_CLOCK_HZ 170000000U
#define WAVE_FLAG_TEST 1U
#define WAVE_FLAG_CLIPPED 2U
#define WAVE_FLAG_VALID 4U

typedef struct {
    /* 同一主样本的编号和170MHz采样时标，允许自然回绕。 */
    uint32_t sequence, ticks;
    /* SOA/SOB同时保持，ISC保留独立保持点含义；VDDA单位mV。 */
    uint16_t raw[3], vdda_mv;
    /* 本次已有dq实际值和参考值，单位A；ISR只复制。 */
    float id_a, iq_a, reference_d_a, reference_q_a;
} wave_sample_t;

typedef struct {
    /* 本会话采集、封包、队列溢出、主动取消与编码截幅数量。 */
    uint32_t captured, sent, dropped, cancelled, clipped;
    /* 本会话已生成数据包数量、最大积压和最大复制耗时。 */
    uint32_t packets, backlog_max, capture_max_cycles;
} wave_stats_t;

/** @brief 复位易失遥测配置，须在开启中断之前调用。 */
void wave_stream_init(void);
/** @brief 设置5/10kHz；仅应用层确认停机且输出OFF后调用。
 * @param rate 5000或10000。 @return 合法且未使能时true。 */
bool wave_stream_set_rate(uint32_t rate);
/** @brief 获取当前易失点率。 @return 点/秒。 */
uint32_t wave_stream_rate(void);
/** @brief 开关观测并清理未发送旧队列，不触碰电机。
 * @param enabled true允许当前或下次运行采集。 */
void wave_stream_enable(bool enabled);
/** @brief 查询是否已选择二进制观测。 @return 已使能。 */
bool wave_stream_enabled(void);
/** @brief 在输出开始前建立新会话；主循环调用。
 * @param zero 本轮三个原始零点。 @param units CSA换算系数1或2。
 * @param test true为明确标记的通信测试源。 */
void wave_stream_begin(const uint16_t zero[3], uint32_t units, bool test);
/** @brief 中断可调用的非阻塞采样复制；不换算、封包或等待。
 * @param sample 本次完整主样本。 */
void wave_stream_capture(const wave_sample_t *sample);
/** @brief 停止采样，允许后台发送剩余数据与结束统计；可在关断ISR调用。 */
void wave_stream_end(void);
/** @brief 主循环发布独立的低速状态快照，不伪造高频状态。
 * @param ticks 捕获时的同域时标。 @param rpm 实际RPM。
 * @param target_milli 目标RPM乘1000。 @param bus_mv 母线mV。
 * @param run 是否运行。 @param stage 本次状态0..6。 @param valid 实际转速有效。 */
void wave_stream_status(uint32_t ticks, float rpm, int32_t target_milli,
    uint16_t bus_mv, bool run, uint8_t stage, bool valid);
/** @brief 后台取得一个COBS包；只在发送器可接受完整块时调用。
 * @param out 至少512字节缓冲。 @param size 容量。 @return 包长度，0为暂时无完整块。 */
size_t wave_stream_next(uint8_t *out, size_t size);
/** @brief 查询本会话观测统计。 @return 近似一致的独立计数。 */
wave_stats_t wave_stream_stats(void);
/** @brief 更新诊断复制耗时，不影响任何控制条件。 @param cycles 本次DWT差。 */
void wave_stream_capture_timing(uint32_t cycles);
/** @brief 停机启动通信测试源，调用者须确认功率关闭与静止。
 * @param now_tick 当前DWT。 @param now_ms 当前毫秒。 @param duration_ms 1..60000ms。 */
void wave_stream_test_start(uint32_t now_tick, uint32_t now_ms, uint32_t duration_ms);
/** @brief 主循环有界推进通信测试，绝不启动ADC或功率。
 * @param now_tick 当前DWT。 @param now_ms 当前毫秒。 */
void wave_stream_test_service(uint32_t now_tick, uint32_t now_ms);
/** @brief 查询通信测试是否仍在产样。 @return 测试活动。 */
bool wave_stream_testing(void);
#endif
