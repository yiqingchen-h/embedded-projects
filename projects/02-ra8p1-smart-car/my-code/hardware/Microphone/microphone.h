#ifndef HARDWARE_MICROPHONE_MICROPHONE_H_
#define HARDWARE_MICROPHONE_MICROPHONE_H_

#include "hal_data.h"

#include <stdint.h>

/* PDM 配置目标采样率为 16000Hz，FSP 计算后的实际采样率约为 16129Hz。 */
#define MICROPHONE_TARGET_SAMPLE_RATE_HZ    (16000U)
#define MICROPHONE_SAMPLE_RATE_HZ           (16129U)

/* 每帧 160 个采样点，约 10ms，适合实时语音解析任务按帧消费。 */
#define MICROPHONE_FRAME_SAMPLES            (160U)

/* 默认使用 SDRAM 保存最近约 30 秒的原始 PCM 数据，可在工程宏中覆盖。 */
#ifndef MICROPHONE_BUFFER_SECONDS
#define MICROPHONE_BUFFER_SECONDS           (30U)
#endif

#define MICROPHONE_BUFFER_FRAME_COUNT       ((MICROPHONE_SAMPLE_RATE_HZ * MICROPHONE_BUFFER_SECONDS) / MICROPHONE_FRAME_SAMPLES)
#define MICROPHONE_USABLE_FRAME_COUNT       (MICROPHONE_BUFFER_FRAME_COUNT - 1U)
#define MICROPHONE_BUFFER_SAMPLES           (MICROPHONE_BUFFER_FRAME_COUNT * MICROPHONE_FRAME_SAMPLES)
#define MICROPHONE_USABLE_SAMPLES           (MICROPHONE_USABLE_FRAME_COUNT * MICROPHONE_FRAME_SAMPLES)

typedef struct st_microphone_frame_info
{
    uint32_t sequence;
    uint32_t sample_rate_hz;
    uint32_t samples;
    uint32_t overrun_count;
    uint32_t error_count;
} microphone_frame_info_t;

typedef struct st_microphone_level
{
    uint32_t sequence;
    uint16_t peak_abs;
    uint16_t mean_abs;
    uint32_t overrun_count;
    uint32_t error_count;
} microphone_level_t;

typedef struct st_microphone_buffer_info
{
    uint32_t sample_rate_hz;
    uint32_t frame_samples;
    uint32_t buffer_frames;
    uint32_t usable_frames;
    uint32_t usable_samples;
    uint32_t usable_ms;
    uint32_t available_frames;
    uint32_t overrun_count;
    uint32_t error_count;
} microphone_buffer_info_t;

typedef struct st_microphone_api
{
    /**
     * @brief  初始化 PDM 麦克风和内部 SDRAM 环形缓冲
     *
     * @return fsp_err_t FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  开始连续采集
     *
     * @return fsp_err_t FSP_SUCCESS 表示已开始采集
     */
    fsp_err_t (*start)(void);

    /**
     * @brief  停止连续采集
     *
     * @return fsp_err_t FSP_SUCCESS 表示停止请求已发出
     */
    fsp_err_t (*stop)(void);

    /**
     * @brief  查询顺序读取接口当前可读帧数
     *
     * @param p_frames  可读帧数输出
     * @return fsp_err_t FSP_SUCCESS 表示查询成功
     */
    fsp_err_t (*available)(uint32_t *p_frames);

    /**
     * @brief  顺序读取下一帧 PCM 数据
     *
     * @param p_dest      输出缓冲，至少 MICROPHONE_FRAME_SAMPLES 个 int16_t
     * @param max_samples 输出缓冲容量，单位采样点
     * @param timeout_ms  无数据时等待时间，0 表示不等待
     * @param p_info      帧信息输出，可传入 NULL
     * @return fsp_err_t  FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*frame_read)(int16_t *p_dest,
                            uint32_t max_samples,
                            uint32_t timeout_ms,
                            microphone_frame_info_t *p_info);

    /**
     * @brief  读取最近一段 PCM 数据，不移动顺序读取游标
     *
     * @param p_dest          输出缓冲
     * @param samples         希望读取的采样点数，超过可用长度时自动截断为最新可用长度
     * @param p_actual_samples 实际读取采样点数输出
     * @param p_info          最新帧信息输出，可传入 NULL
     * @return fsp_err_t      FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*latest_read)(int16_t *p_dest,
                             uint32_t samples,
                             uint32_t *p_actual_samples,
                             microphone_frame_info_t *p_info);

    /**
     * @brief  获取最近一帧的声强统计
     *
     * @param p_level  声强统计输出
     * @return fsp_err_t FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*level_get)(microphone_level_t *p_level);

    /**
     * @brief  获取缓冲区状态
     *
     * @param p_info  缓冲区状态输出
     * @return fsp_err_t FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*buffer_info)(microphone_buffer_info_t *p_info);
} microphone_api_t;

extern const microphone_api_t microphone;

#endif /* HARDWARE_MICROPHONE_MICROPHONE_H_ */
