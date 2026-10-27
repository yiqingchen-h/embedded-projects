#ifndef HARDWARE_MAX98357A_MAX98357A_H_
#define HARDWARE_MAX98357A_MAX98357A_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

#define MAX98357A_SAMPLE_RATE_HZ        (16000U)

typedef struct st_max98357a_api
{
    /**
     * @brief  初始化 SSI1 I2S 输出
     *
     * @return fsp_err_t FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  播放 16bit 单声道 PCM 数据
     *
     * @param p_samples    PCM 数据指针，采样率固定为 MAX98357A_SAMPLE_RATE_HZ
     * @param sample_count PCM 采样点数量
     * @param timeout_ms   每个 I2S 分块等待回调的超时时间
     * @return fsp_err_t   FSP_SUCCESS 表示播放完成
     */
    fsp_err_t (*write_pcm16_mono)(const int16_t *p_samples,
                                  uint32_t sample_count,
                                  uint32_t timeout_ms);

    /**
     * @brief  播放测试音
     *
     * @param frequency_hz   频率，单位 Hz
     * @param duration_ms    时长，单位 ms
     * @param volume_percent 音量百分比，范围 0~100
     * @return fsp_err_t     FSP_SUCCESS 表示播放完成
     */
    fsp_err_t (*play_tone)(uint32_t frequency_hz,
                           uint32_t duration_ms,
                           uint8_t volume_percent);

    /**
     * @brief  停止当前 I2S 输出
     *
     * @param timeout_ms 等待 SSI 进入空闲的超时时间
     * @return fsp_err_t FSP_SUCCESS 表示停止完成
     */
    fsp_err_t (*stop)(uint32_t timeout_ms);

    /**
     * @brief  查询功放驱动是否正在播放
     *
     * @return true 正在播放，false 空闲
     */
    bool (*is_busy)(void);
} max98357a_api_t;

extern const max98357a_api_t max98357a;

void g_i2s1_callback(i2s_callback_args_t *p_args);

#endif /* HARDWARE_MAX98357A_MAX98357A_H_ */
