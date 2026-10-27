#include "software/voice_dataset_capture.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "hardware/MAX98357A/max98357a.h"
#include "hardware/Microphone/microphone.h"
#include "software/SDRAM/SDRAM.h"
#include "software/UART.h"
#include "software/voice_ai.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* 录音帧数按整帧向上取整，避免最后一帧不足 160 点。 */
#define VOICE_DATASET_RECORD_FRAMES      (((MICROPHONE_SAMPLE_RATE_HZ * VOICE_DATASET_RECORD_SECONDS) + \
                                           MICROPHONE_FRAME_SAMPLES - 1U) / MICROPHONE_FRAME_SAMPLES)

/* 单次发送的 PCM16 采样点数。 */
#define VOICE_DATASET_RECORD_SAMPLES     (VOICE_DATASET_RECORD_FRAMES * MICROPHONE_FRAME_SAMPLES)

/* 等待麦克风一帧数据的超时时间，单位 ms。 */
#define VOICE_DATASET_FRAME_TIMEOUT_MS   (100U)

/* 后台任务栈大小，单位 word。 */
#define VOICE_DATASET_TASK_STACK_SIZE    (1024U)

/* 后台任务优先级，与按键任务同级，保证按下后能及时推进录音状态。 */
#define VOICE_DATASET_TASK_PRIORITY      (6U)

/* 停止功放输出等待超时时间，单位 ms。 */
#define VOICE_DATASET_AMP_STOP_TIMEOUT_MS (200U)

/* 录音 PCM 缓冲放在 SDRAM，避免占用片上 SRAM。 */
SDRAM_SECTION_NOINIT static int16_t g_voice_dataset_pcm[VOICE_DATASET_RECORD_SAMPLES];

/* 语音数据集标签表，顺序必须与 my-code/ai/labels.txt 完全一致。 */
static const char * const g_voice_dataset_labels[VOICE_DATASET_LABEL_COUNT] =
{
    "wake",
    "start",
    "stop",
    "select_cube",
    "select_hippo",
    "select_cat",
    "select_stone_pillar",
    "select_capybara",
    "select_corn",
    "select_jiaotailang",
    "select_dog",
    "select_tnt",
    "unknown",
    "silence",
};

/* 后台任务句柄。 */
static TaskHandle_t g_voice_dataset_task_handle = NULL;

/* 开始录音请求信号量存储。 */
static StaticSemaphore_t g_voice_dataset_start_sem_storage;

/* 开始录音请求信号量。 */
static SemaphoreHandle_t g_voice_dataset_start_sem = NULL;

/* 模块是否已经初始化。 */
static bool g_voice_dataset_initialized = false;

/* 当前状态缓存，供 UI 周期读取。 */
static voice_dataset_status_t g_voice_dataset_status =
{
    .state = VOICE_DATASET_STATE_IDLE,
    .progress = 0U,
    .last_error = FSP_SUCCESS,
    .sample_rate_hz = MICROPHONE_SAMPLE_RATE_HZ,
    .samples = VOICE_DATASET_RECORD_SAMPLES,
    .label = VOICE_DATASET_DEFAULT_LABEL,
};

/**
 * @brief  获取语音数据集标签名
 *
 * @param index 标签索引
 * @return const char* 标签字符串，索引越界时返回默认标签
 */
const char *voice_dataset_label_name(uint8_t index)
{
    if (index >= VOICE_DATASET_LABEL_COUNT)
    {
        return VOICE_DATASET_DEFAULT_LABEL;
    }

    return g_voice_dataset_labels[index];
}

/**
 * @brief  获取语音数据集标签数量
 *
 * @return uint8_t 标签数量
 */
uint8_t voice_dataset_label_count(void)
{
    return VOICE_DATASET_LABEL_COUNT;
}

/**
 * @brief  设置当前状态
 *
 * @param state      新状态
 * @param progress   进度百分比
 * @param last_error 最近一次错误码
 * @return void
 */
static void voice_dataset_set_state(voice_dataset_state_t state, uint8_t progress, fsp_err_t last_error)
{
    if (progress > 100U)
    {
        progress = 100U;
    }

    taskENTER_CRITICAL();
    g_voice_dataset_status.state = state;
    g_voice_dataset_status.progress = progress;
    g_voice_dataset_status.last_error = last_error;
    g_voice_dataset_status.sample_rate_hz = MICROPHONE_SAMPLE_RATE_HZ;
    g_voice_dataset_status.samples = VOICE_DATASET_RECORD_SAMPLES;
    taskEXIT_CRITICAL();
}

/**
 * @brief  安全复制当前样本标签
 *
 * @param label 标签字符串，传入 NULL 时使用默认标签
 * @return void
 */
static void voice_dataset_set_label(const char *label)
{
    const char *p_label = label;

    if (NULL == p_label)
    {
        p_label = VOICE_DATASET_DEFAULT_LABEL;
    }

    taskENTER_CRITICAL();
    (void)strncpy(g_voice_dataset_status.label, p_label, VOICE_DATASET_LABEL_LENGTH - 1U);
    g_voice_dataset_status.label[VOICE_DATASET_LABEL_LENGTH - 1U] = '\0';
    taskEXIT_CRITICAL();
}

/**
 * @brief  复制当前样本标签
 *
 * @param p_dest 输出缓冲
 * @param length 输出缓冲长度
 * @return void
 */
static void voice_dataset_copy_label(char *p_dest, uint32_t length)
{
    if ((NULL == p_dest) || (0U == length))
    {
        return;
    }

    taskENTER_CRITICAL();
    (void)strncpy(p_dest, g_voice_dataset_status.label, length - 1U);
    p_dest[length - 1U] = '\0';
    taskEXIT_CRITICAL();
}

/**
 * @brief  采集固定时长 PCM 到 SDRAM 缓冲
 *
 * @return fsp_err_t FSP_SUCCESS 表示录音完成
 */
static fsp_err_t voice_dataset_record_pcm(void)
{
    fsp_err_t err = FSP_SUCCESS;
    uint32_t frame = 0U;
    uint32_t offset = 0U;
    uint8_t progress = 0U;

    err = microphone.init();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = microphone.start();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    for (frame = 0U; frame < VOICE_DATASET_RECORD_FRAMES; frame++)
    {
        offset = frame * MICROPHONE_FRAME_SAMPLES;
        err = microphone.frame_read(&g_voice_dataset_pcm[offset],
                                    MICROPHONE_FRAME_SAMPLES,
                                    VOICE_DATASET_FRAME_TIMEOUT_MS,
                                    NULL);
        if (FSP_SUCCESS != err)
        {
            (void)microphone.stop();
            return err;
        }

        progress = (uint8_t)(((frame + 1U) * 100U) / VOICE_DATASET_RECORD_FRAMES);
        voice_dataset_set_state(VOICE_DATASET_STATE_RECORDING, progress, FSP_SUCCESS);
    }

    err = microphone.stop();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  停止功放输出
 *
 * @return void
 */
static void voice_dataset_stop_amp(void)
{
    if (max98357a.is_busy())
    {
        (void)max98357a.stop(VOICE_DATASET_AMP_STOP_TIMEOUT_MS);
    }
}

/**
 * @brief  执行一次录音并发送
 *
 * @return void
 */
static void voice_dataset_run_once(void)
{
    fsp_err_t err = FSP_SUCCESS;
    char label[VOICE_DATASET_LABEL_LENGTH] = {0};

    voice_dataset_copy_label(label, sizeof(label));
    voice_dataset_set_state(VOICE_DATASET_STATE_RECORDING, 0U, FSP_SUCCESS);

    err = voice_dataset_record_pcm();
    if (FSP_SUCCESS != err)
    {
        voice_dataset_set_state(VOICE_DATASET_STATE_ERROR, 0U, err);
        return;
    }

    voice_dataset_set_state(VOICE_DATASET_STATE_SENDING, 0U, FSP_SUCCESS);
    err = uart.write_pcm16_packet(label,
                                  g_voice_dataset_pcm,
                                  VOICE_DATASET_RECORD_SAMPLES,
                                  MICROPHONE_SAMPLE_RATE_HZ);
    voice_dataset_stop_amp();
    if (FSP_SUCCESS != err)
    {
        voice_dataset_set_state(VOICE_DATASET_STATE_ERROR, 100U, err);
        return;
    }

    voice_dataset_set_state(VOICE_DATASET_STATE_DONE, 100U, FSP_SUCCESS);
}

/**
 * @brief  语音数据集后台任务入口
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void voice_dataset_task_entry(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        if (pdTRUE == xSemaphoreTake(g_voice_dataset_start_sem, portMAX_DELAY))
        {
            voice_dataset_run_once();
        }
    }
}

/**
 * @brief  初始化语音数据集采集后台任务
 *
 * @return fsp_err_t FSP_SUCCESS 表示初始化成功
 */
static fsp_err_t voice_dataset_init_impl(void)
{
    BaseType_t ok = pdPASS;

    if (g_voice_dataset_initialized)
    {
        return FSP_SUCCESS;
    }

    g_voice_dataset_start_sem = xSemaphoreCreateBinaryStatic(&g_voice_dataset_start_sem_storage);
    if (NULL == g_voice_dataset_start_sem)
    {
        return FSP_ERR_INTERNAL;
    }

    ok = xTaskCreate(voice_dataset_task_entry,
                     "voice_dataset",
                     VOICE_DATASET_TASK_STACK_SIZE,
                     NULL,
                     VOICE_DATASET_TASK_PRIORITY,
                     &g_voice_dataset_task_handle);
    if (pdPASS != ok)
    {
        return FSP_ERR_INTERNAL;
    }

    g_voice_dataset_initialized = true;
    voice_dataset_set_state(VOICE_DATASET_STATE_IDLE, 0U, FSP_SUCCESS);

    return FSP_SUCCESS;
}

/**
 * @brief  请求开始一次录音并自动发送到上位机
 *
 * @param label 样本标签
 * @return fsp_err_t FSP_SUCCESS 表示请求已提交
 */
static fsp_err_t voice_dataset_start_impl(const char *label)
{
    voice_dataset_state_t state = VOICE_DATASET_STATE_IDLE;
    fsp_err_t err = FSP_SUCCESS;

    err = voice_dataset_init_impl();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    taskENTER_CRITICAL();
    state = g_voice_dataset_status.state;
    taskEXIT_CRITICAL();

    if ((VOICE_DATASET_STATE_RECORDING == state) || (VOICE_DATASET_STATE_SENDING == state))
    {
        return FSP_ERR_IN_USE;
    }

    err = voice_ai.set_enabled(false);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    voice_dataset_set_label(label);
    voice_dataset_set_state(VOICE_DATASET_STATE_RECORDING, 0U, FSP_SUCCESS);
    (void)xSemaphoreTake(g_voice_dataset_start_sem, 0U);
    if (pdTRUE != xSemaphoreGive(g_voice_dataset_start_sem))
    {
        voice_dataset_set_state(VOICE_DATASET_STATE_ERROR, 0U, FSP_ERR_INTERNAL);
        return FSP_ERR_INTERNAL;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  读取当前采集发送状态
 *
 * @param p_status 状态输出指针
 * @return void
 */
static void voice_dataset_status_get_impl(voice_dataset_status_t *p_status)
{
    if (NULL == p_status)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_status = g_voice_dataset_status;
    taskEXIT_CRITICAL();
}

const voice_dataset_api_t voice_dataset =
{
    .init = voice_dataset_init_impl,
    .start = voice_dataset_start_impl,
    .status_get = voice_dataset_status_get_impl,
};
