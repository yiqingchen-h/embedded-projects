#ifndef SOFTWARE_VOICE_DATASET_CAPTURE_H_
#define SOFTWARE_VOICE_DATASET_CAPTURE_H_

#include "hal_data.h"

#include <stdint.h>

/* 默认语音数据集标签，对应 my-code/ai/labels.txt 中的 wake。 */
#define VOICE_DATASET_DEFAULT_LABEL      "wake"

/* 语音数据集标签数量，必须与 my-code/ai/labels.txt 行数一致。 */
#define VOICE_DATASET_LABEL_COUNT       (14U)

/* 语音数据集标签缓冲长度，包含字符串结尾。 */
#define VOICE_DATASET_LABEL_LENGTH       (32U)

/* 默认录音时长，单位秒。 */
#define VOICE_DATASET_RECORD_SECONDS     (3U)

typedef enum e_voice_dataset_state
{
    /* 等待用户触发录音。 */
    VOICE_DATASET_STATE_IDLE = 0,

    /* 正在从 PDM 麦克风采集 PCM。 */
    VOICE_DATASET_STATE_RECORDING,

    /* 正在按 MIC_DATA 协议发送到上位机。 */
    VOICE_DATASET_STATE_SENDING,

    /* 最近一次录制和发送已完成。 */
    VOICE_DATASET_STATE_DONE,

    /* 最近一次录制或发送失败。 */
    VOICE_DATASET_STATE_ERROR,
} voice_dataset_state_t;

typedef struct st_voice_dataset_status
{
    /* 当前状态。 */
    voice_dataset_state_t state;

    /* 当前阶段进度，范围 0~100。 */
    uint8_t progress;

    /* 最近一次错误码，成功时为 FSP_SUCCESS。 */
    fsp_err_t last_error;

    /* 实际发送的采样率，单位 Hz。 */
    uint32_t sample_rate_hz;

    /* 实际发送的采样点数量。 */
    uint32_t samples;

    /* 当前样本标签。 */
    char label[VOICE_DATASET_LABEL_LENGTH];
} voice_dataset_status_t;

typedef struct st_voice_dataset_api
{
    /**
     * @brief  初始化语音数据集采集后台任务
     *
     * @return fsp_err_t FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  请求开始一次录音并自动发送到上位机
     *
     * @param label 样本标签，传入 NULL 时使用 VOICE_DATASET_DEFAULT_LABEL
     * @return fsp_err_t FSP_SUCCESS 表示请求已提交
     */
    fsp_err_t (*start)(const char *label);

    /**
     * @brief  读取当前采集发送状态
     *
     * @param p_status 状态输出指针
     * @return void
     */
    void (*status_get)(voice_dataset_status_t *p_status);
} voice_dataset_api_t;

/**
 * @brief  获取语音数据集标签名
 *
 * @param index 标签索引
 * @return const char* 标签字符串，索引越界时返回默认标签
 */
const char *voice_dataset_label_name(uint8_t index);

/**
 * @brief  获取语音数据集标签数量
 *
 * @return uint8_t 标签数量
 */
uint8_t voice_dataset_label_count(void);

extern const voice_dataset_api_t voice_dataset;

#endif /* SOFTWARE_VOICE_DATASET_CAPTURE_H_ */
