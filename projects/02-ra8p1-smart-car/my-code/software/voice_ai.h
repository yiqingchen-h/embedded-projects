#ifndef SOFTWARE_VOICE_AI_H_
#define SOFTWARE_VOICE_AI_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

/* KWS 模型标签数量，必须与 my-code/ai/labels.txt 保持一致。 */
#define VOICE_AI_LABEL_COUNT            (14U)

/* 每次打印和缓存的 TopK 数量。 */
#define VOICE_AI_TOPK_COUNT             (3U)

/* 每次语音触发最多评估的候选窗口数量。 */
#define VOICE_AI_MAX_CANDIDATES         (5U)

typedef enum e_voice_ai_state
{
    VOICE_AI_STATE_STOPPED = 0,
    VOICE_AI_STATE_STARTING,
    VOICE_AI_STATE_RUNNING,
    VOICE_AI_STATE_PAUSED,
    VOICE_AI_STATE_ERROR,
} voice_ai_state_t;

typedef enum e_voice_ai_label
{
    VOICE_AI_LABEL_WAKE = 0,
    VOICE_AI_LABEL_START,
    VOICE_AI_LABEL_STOP,
    VOICE_AI_LABEL_SELECT_CUBE,
    VOICE_AI_LABEL_SELECT_HIPPO,
    VOICE_AI_LABEL_SELECT_CAT,
    VOICE_AI_LABEL_SELECT_STONE_PILLAR,
    VOICE_AI_LABEL_SELECT_CAPYBARA,
    VOICE_AI_LABEL_SELECT_CORN,
    VOICE_AI_LABEL_SELECT_JIAOTAILANG,
    VOICE_AI_LABEL_SELECT_DOG,
    VOICE_AI_LABEL_SELECT_TNT,
    VOICE_AI_LABEL_UNKNOWN,
    VOICE_AI_LABEL_SILENCE,
    VOICE_AI_LABEL_ENUM_COUNT,
} voice_ai_label_t;

typedef struct st_voice_ai_result
{
    uint32_t sequence;
    uint32_t tick_ms;
    uint32_t cycles;
    uint32_t inference_count;
    uint32_t error_count;
    uint8_t top_index;
    float top_score;
    uint8_t top_indices[VOICE_AI_TOPK_COUNT];
    float top_scores[VOICE_AI_TOPK_COUNT];
    float scores[VOICE_AI_LABEL_COUNT];
    /* 本次语音触发实际评估的候选窗口数量。 */
    uint32_t candidate_count;
    /* 每个候选窗口的 Top1 标签索引。 */
    uint8_t candidate_top_indices[VOICE_AI_MAX_CANDIDATES];
    /* 每个候选窗口的 Top1 分数。 */
    float candidate_top_scores[VOICE_AI_MAX_CANDIDATES];
    voice_ai_state_t state;
} voice_ai_result_t;

typedef struct st_voice_ai_api
{
    /**
     * @brief  初始化 NPU 和语音 AI 后台任务
     *
     * @return fsp_err_t FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  执行一次语音 AI 后台处理
     *
     * @return fsp_err_t FSP_SUCCESS 表示处理完成，FSP_ERR_BUFFER_EMPTY 表示本次没有有效语音
     */
    fsp_err_t (*process)(void);

    /**
     * @brief  启用或暂停语音 AI 后台解析
     *
     * @param enabled true 表示启用，false 表示暂停并释放麦克风
     * @return fsp_err_t FSP_SUCCESS 表示状态切换完成
     */
    fsp_err_t (*set_enabled)(bool enabled);

    /**
     * @brief  读取最近一次推理结果
     *
     * @param p_result 结果输出
     * @return fsp_err_t FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*latest_read)(voice_ai_result_t *p_result);

    /**
     * @brief  查询是否存在尚未消费的新结果
     *
     * @return bool true 表示存在新结果
     */
    bool (*has_new)(void);

    /**
     * @brief  清除新结果标志
     *
     * @return void
     */
    void (*clear_new)(void);

    /**
     * @brief  获取标签名称
     *
     * @param index 标签索引
     * @return const char* 标签名，索引越界时返回 unknown
     */
    const char *(*label_name)(uint8_t index);
} voice_ai_api_t;

extern const voice_ai_api_t voice_ai;

#endif /* SOFTWARE_VOICE_AI_H_ */
