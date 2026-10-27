#include "hardware/Microphone/microphone.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "software/SDRAM/SDRAM.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>

#if (MICROPHONE_BUFFER_FRAME_COUNT < 2U)
#error "MICROPHONE_BUFFER_SECONDS too small"
#endif

/* PDM DMA 每帧写 160 个 32bit FIFO 数据，放在 SDRAM non-cache 区避免缓存一致性问题。 */
SDRAM_SECTION_NOINIT_NOCACHE static volatile uint32_t g_microphone_raw_frames[MICROPHONE_BUFFER_FRAME_COUNT][MICROPHONE_FRAME_SAMPLES];

static StaticSemaphore_t g_microphone_frame_sem_storage;
static SemaphoreHandle_t g_microphone_frame_sem = NULL;

static volatile bool g_microphone_initialized = false;
static volatile bool g_microphone_started = false;
static volatile uint32_t g_microphone_write_frame = 0U;
static volatile uint32_t g_microphone_read_frame = 0U;
static volatile uint32_t g_microphone_overrun_count = 0U;
static volatile uint32_t g_microphone_error_count = 0U;

/* PCM 软件增益，Q8 定点格式。256 = 1.0x，4096 = 16.0x。可在工程宏中覆盖。 */
#ifndef MICROPHONE_PCM_GAIN_Q8
#define MICROPHONE_PCM_GAIN_Q8    (8192)
#endif

/* PCM 原始数据提取模式，用于实测 PDM FIFO 有效位段。 */
#ifndef MICROPHONE_PCM_EXTRACT_MODE
#define MICROPHONE_PCM_EXTRACT_MODE     (1)
#endif

/**
 * @brief  将 int32 饱和到 int16，防止增益溢出后削顶回绕
 *
 * @param value 待饱和的 int32 值
 * @return int16_t 饱和后的值
 */
static int16_t microphone_saturate_i16_from_i32(int32_t value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }

    if (value < INT16_MIN)
    {
        return INT16_MIN;
    }

    return (int16_t)value;
}

/**
 * @brief  从 PDM FIFO 32bit 数据中提取 16bit PCM
 *
 * @param raw PDM FIFO 原始 32bit 数据
 * @return int16_t 提取后的 PCM 样本
 */
static int16_t microphone_extract_pcm16(uint32_t raw)
{
#if (MICROPHONE_PCM_EXTRACT_MODE == 1)
    return (int16_t)(raw & 0xFFFFU);
#else
#error "Unsupported MICROPHONE_PCM_EXTRACT_MODE"
#endif
}

/**
 * @brief  将 PDM FIFO 32bit 数据转换为 16bit PCM
 *
 * 当前实测 MICROPHONE_PCM_EXTRACT_MODE 为 1，使用 raw[15:0] 作为 16bit 有符号 PCM。
 * 转换后统一经过 MICROPHONE_PCM_GAIN_Q8 软件增益，保证所有读取路径都获得放大后的 PCM。
 */
static int16_t microphone_raw_to_pcm16(uint32_t raw)
{
    int16_t pcm = microphone_extract_pcm16(raw);
    int32_t amplified = ((int32_t)pcm * (int32_t)MICROPHONE_PCM_GAIN_Q8) / 256;

    return microphone_saturate_i16_from_i32(amplified);
}

/**
 * @brief  计算 int16_t 的绝对值，保留 -32768 的完整幅度
 */
static uint16_t microphone_abs_i16(int16_t value)
{
    if (value >= 0)
    {
        return (uint16_t)value;
    }

    if (INT16_MIN == value)
    {
        return 32768U;
    }

    return (uint16_t)(-value);
}

static uint32_t microphone_min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static uint32_t microphone_available_frames_unlocked(void)
{
    uint32_t produced = g_microphone_write_frame - g_microphone_read_frame;

    return microphone_min_u32(produced, MICROPHONE_USABLE_FRAME_COUNT);
}

static void microphone_fill_frame_info(microphone_frame_info_t *p_info, uint32_t sequence, uint32_t samples)
{
    if (NULL != p_info)
    {
        p_info->sequence = sequence;
        p_info->sample_rate_hz = MICROPHONE_SAMPLE_RATE_HZ;
        p_info->samples = samples;
        p_info->overrun_count = g_microphone_overrun_count;
        p_info->error_count = g_microphone_error_count;
    }
}

static void microphone_copy_frame(uint32_t frame_sequence, int16_t *p_dest)
{
    uint32_t frame_index = frame_sequence % MICROPHONE_BUFFER_FRAME_COUNT;
    volatile uint32_t *p_src = g_microphone_raw_frames[frame_index];

    for (uint32_t i = 0U; i < MICROPHONE_FRAME_SAMPLES; i++)
    {
        p_dest[i] = microphone_raw_to_pcm16(p_src[i]);
    }
}

static int16_t microphone_read_sample_by_index(uint64_t sample_index)
{
    uint32_t frame_index = (uint32_t)((sample_index / MICROPHONE_FRAME_SAMPLES) % MICROPHONE_BUFFER_FRAME_COUNT);
    uint32_t sample_offset = (uint32_t)(sample_index % MICROPHONE_FRAME_SAMPLES);

    return microphone_raw_to_pcm16(g_microphone_raw_frames[frame_index][sample_offset]);
}

static fsp_err_t microphone_init_impl(void)
{
    fsp_err_t err;

    if (g_microphone_initialized)
    {
        return FSP_SUCCESS;
    }

    if (NULL == g_microphone_frame_sem)
    {
        g_microphone_frame_sem = xSemaphoreCreateCountingStatic((UBaseType_t)MICROPHONE_USABLE_FRAME_COUNT,
                                                                0U,
                                                                &g_microphone_frame_sem_storage);
        if (NULL == g_microphone_frame_sem)
        {
            return FSP_ERR_INTERNAL;
        }
    }

    g_microphone_write_frame = 0U;
    g_microphone_read_frame = 0U;
    g_microphone_overrun_count = 0U;
    g_microphone_error_count = 0U;
    g_microphone_started = false;
    (void)xQueueReset(g_microphone_frame_sem);

    err = g_pdm0.p_api->open(g_pdm0.p_ctrl, g_pdm0.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    g_microphone_initialized = true;
    return FSP_SUCCESS;
}

static fsp_err_t microphone_start_impl(void)
{
    fsp_err_t err;

    if (!g_microphone_initialized)
    {
        err = microphone_init_impl();
        if (FSP_SUCCESS != err)
        {
            return err;
        }
    }

    if (g_microphone_started)
    {
        return FSP_SUCCESS;
    }

    taskENTER_CRITICAL();
    g_microphone_write_frame = 0U;
    g_microphone_read_frame = 0U;
    g_microphone_overrun_count = 0U;
    g_microphone_error_count = 0U;
    taskEXIT_CRITICAL();
    (void)xQueueReset(g_microphone_frame_sem);

    g_microphone_started = true;
    err = g_pdm0.p_api->start(g_pdm0.p_ctrl,
                              (void *)g_microphone_raw_frames,
                              sizeof(g_microphone_raw_frames),
                              MICROPHONE_FRAME_SAMPLES);
    if (FSP_SUCCESS != err)
    {
        g_microphone_started = false;
        return err;
    }

    return FSP_SUCCESS;
}

static fsp_err_t microphone_stop_impl(void)
{
    fsp_err_t err;

    if (!g_microphone_initialized)
    {
        return FSP_ERR_NOT_OPEN;
    }

    if (!g_microphone_started)
    {
        return FSP_SUCCESS;
    }

    g_microphone_started = false;
    err = g_pdm0.p_api->stop(g_pdm0.p_ctrl);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return FSP_SUCCESS;
}

static fsp_err_t microphone_available_impl(uint32_t *p_frames)
{
    if (NULL == p_frames)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    taskENTER_CRITICAL();
    *p_frames = microphone_available_frames_unlocked();
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;
}

static fsp_err_t microphone_frame_read_impl(int16_t *p_dest,
                                            uint32_t max_samples,
                                            uint32_t timeout_ms,
                                            microphone_frame_info_t *p_info)
{
    uint32_t available_frames;
    uint32_t frame_sequence;

    if (NULL == p_dest)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    if (max_samples < MICROPHONE_FRAME_SAMPLES)
    {
        return FSP_ERR_INVALID_SIZE;
    }

    if (!g_microphone_initialized)
    {
        return FSP_ERR_NOT_OPEN;
    }

    taskENTER_CRITICAL();
    available_frames = microphone_available_frames_unlocked();
    taskEXIT_CRITICAL();

    if (0U == available_frames)
    {
        TickType_t wait_ticks = (0U == timeout_ms) ? 0U : pdMS_TO_TICKS(timeout_ms);
        if (pdTRUE != xSemaphoreTake(g_microphone_frame_sem, wait_ticks))
        {
            return (0U == timeout_ms) ? FSP_ERR_BUFFER_EMPTY : FSP_ERR_TIMEOUT;
        }
    }
    else
    {
        (void)xSemaphoreTake(g_microphone_frame_sem, 0U);
    }

    taskENTER_CRITICAL();
    available_frames = microphone_available_frames_unlocked();
    if (0U == available_frames)
    {
        taskEXIT_CRITICAL();
        return FSP_ERR_BUFFER_EMPTY;
    }

    frame_sequence = g_microphone_read_frame;
    g_microphone_read_frame++;
    taskEXIT_CRITICAL();

    microphone_copy_frame(frame_sequence, p_dest);
    microphone_fill_frame_info(p_info, frame_sequence, MICROPHONE_FRAME_SAMPLES);

    return FSP_SUCCESS;
}

static fsp_err_t microphone_latest_read_impl(int16_t *p_dest,
                                             uint32_t samples,
                                             uint32_t *p_actual_samples,
                                             microphone_frame_info_t *p_info)
{
    uint32_t write_frame;
    uint32_t available_frames;
    uint32_t available_samples;
    uint32_t read_samples;
    uint64_t start_sample;

    if ((NULL == p_dest) || (NULL == p_actual_samples))
    {
        return FSP_ERR_INVALID_POINTER;
    }

    if (0U == samples)
    {
        *p_actual_samples = 0U;
        return FSP_ERR_INVALID_SIZE;
    }

    taskENTER_CRITICAL();
    write_frame = g_microphone_write_frame;
    available_frames = microphone_min_u32(write_frame, MICROPHONE_USABLE_FRAME_COUNT);
    taskEXIT_CRITICAL();

    if (0U == available_frames)
    {
        *p_actual_samples = 0U;
        return FSP_ERR_BUFFER_EMPTY;
    }

    available_samples = available_frames * MICROPHONE_FRAME_SAMPLES;
    read_samples = microphone_min_u32(samples, available_samples);
    start_sample = ((uint64_t)write_frame * MICROPHONE_FRAME_SAMPLES) - read_samples;

    for (uint32_t i = 0U; i < read_samples; i++)
    {
        p_dest[i] = microphone_read_sample_by_index(start_sample + i);
    }

    *p_actual_samples = read_samples;
    microphone_fill_frame_info(p_info, write_frame - 1U, read_samples);

    return FSP_SUCCESS;
}

static fsp_err_t microphone_level_get_impl(microphone_level_t *p_level)
{
    uint32_t write_frame;
    uint32_t frame_sequence;
    uint32_t frame_index;
    uint32_t sum_abs = 0U;
    uint16_t peak_abs = 0U;

    if (NULL == p_level)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    taskENTER_CRITICAL();
    write_frame = g_microphone_write_frame;
    if (0U == write_frame)
    {
        taskEXIT_CRITICAL();
        return FSP_ERR_BUFFER_EMPTY;
    }
    frame_sequence = write_frame - 1U;
    taskEXIT_CRITICAL();

    frame_index = frame_sequence % MICROPHONE_BUFFER_FRAME_COUNT;
    for (uint32_t i = 0U; i < MICROPHONE_FRAME_SAMPLES; i++)
    {
        uint16_t abs_value = microphone_abs_i16(microphone_raw_to_pcm16(g_microphone_raw_frames[frame_index][i]));
        sum_abs += abs_value;
        if (abs_value > peak_abs)
        {
            peak_abs = abs_value;
        }
    }

    p_level->sequence = frame_sequence;
    p_level->peak_abs = peak_abs;
    p_level->mean_abs = (uint16_t)(sum_abs / MICROPHONE_FRAME_SAMPLES);
    p_level->overrun_count = g_microphone_overrun_count;
    p_level->error_count = g_microphone_error_count;

    return FSP_SUCCESS;
}

static fsp_err_t microphone_buffer_info_impl(microphone_buffer_info_t *p_info)
{
    if (NULL == p_info)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    p_info->sample_rate_hz = MICROPHONE_SAMPLE_RATE_HZ;
    p_info->frame_samples = MICROPHONE_FRAME_SAMPLES;
    p_info->buffer_frames = MICROPHONE_BUFFER_FRAME_COUNT;
    p_info->usable_frames = MICROPHONE_USABLE_FRAME_COUNT;
    p_info->usable_samples = MICROPHONE_USABLE_SAMPLES;
    p_info->usable_ms = (MICROPHONE_USABLE_SAMPLES * 1000U) / MICROPHONE_SAMPLE_RATE_HZ;

    taskENTER_CRITICAL();
    p_info->available_frames = microphone_available_frames_unlocked();
    p_info->overrun_count = g_microphone_overrun_count;
    p_info->error_count = g_microphone_error_count;
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;
}

void pdm_callback(pdm_callback_args_t *p_args)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (NULL == p_args)
    {
        return;
    }

    if (PDM_EVENT_DATA == p_args->event)
    {
        if (!g_microphone_started)
        {
            return;
        }

        UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
        g_microphone_write_frame++;
        if ((g_microphone_write_frame - g_microphone_read_frame) > MICROPHONE_USABLE_FRAME_COUNT)
        {
            g_microphone_read_frame = g_microphone_write_frame - MICROPHONE_USABLE_FRAME_COUNT;
            g_microphone_overrun_count++;
        }
        taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);

        if (NULL != g_microphone_frame_sem)
        {
            (void)xSemaphoreGiveFromISR(g_microphone_frame_sem, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
    else if (PDM_EVENT_ERROR == p_args->event)
    {
        UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
        g_microphone_error_count++;
        taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
    }
    else
    {
        /* 其他事件当前不需要处理。 */
    }
}

const microphone_api_t microphone =
{
    .init = microphone_init_impl,
    .start = microphone_start_impl,
    .stop = microphone_stop_impl,
    .available = microphone_available_impl,
    .frame_read = microphone_frame_read_impl,
    .latest_read = microphone_latest_read_impl,
    .level_get = microphone_level_get_impl,
    .buffer_info = microphone_buffer_info_impl,
};
