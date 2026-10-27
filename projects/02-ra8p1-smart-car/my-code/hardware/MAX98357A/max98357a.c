#include "hardware/MAX98357A/max98357a.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stddef.h>

#define MAX98357A_TX_MONO_CHUNK_SAMPLES     (256U)
#define MAX98357A_DRAIN_SILENCE_FRAMES      (64U)
#define MAX98357A_DEFAULT_TIMEOUT_MS        (200U)
#define MAX98357A_SINE_TABLE_SIZE           (32U)
#define MAX98357A_SINE_TABLE_SHIFT          (27U)

static int16_t g_max98357a_tx_buffer[(MAX98357A_TX_MONO_CHUNK_SAMPLES + 1U) * 2U] BSP_ALIGN_VARIABLE(4);

static StaticSemaphore_t g_max98357a_tx_empty_sem_storage;
static StaticSemaphore_t g_max98357a_idle_sem_storage;
static SemaphoreHandle_t g_max98357a_tx_empty_sem = NULL;
static SemaphoreHandle_t g_max98357a_idle_sem = NULL;

static volatile bool g_max98357a_initialized = false;
static volatile bool g_max98357a_busy = false;

/* 外部音频时钟 GPT8 状态，避免重复 open/start。 */
static bool s_audio_clk_opened  = false;
static bool s_audio_clk_started = false;

/* SSI1 I2S 打开状态。 */
static bool s_i2s_opened = false;

static const int16_t g_max98357a_sine_table[MAX98357A_SINE_TABLE_SIZE] =
{
    0, 6393, 12540, 18204, 23170, 27245, 30274, 32137,
    32767, 32137, 30274, 27245, 23170, 18204, 12540, 6393,
    0, -6393, -12540, -18204, -23170, -27245, -30274, -32137,
    -32767, -32137, -30274, -27245, -23170, -18204, -12540, -6393,
};

static TickType_t max98357a_timeout_to_ticks(uint32_t timeout_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);

    if ((timeout_ms > 0U) && (0U == ticks))
    {
        ticks = 1U;
    }

    return ticks;
}

static uint32_t max98357a_min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static void max98357a_drain_semaphore(SemaphoreHandle_t sem)
{
    if (NULL != sem)
    {
        while (pdTRUE == xSemaphoreTake(sem, 0U))
        {
        }
    }
}

static fsp_err_t max98357a_wait_idle(uint32_t timeout_ms)
{
    i2s_status_t status;
    fsp_err_t err = g_i2s1.p_api->statusGet(g_i2s1.p_ctrl, &status);
    if ((FSP_SUCCESS == err) && (I2S_STATE_STOPPED == status.state))
    {
        return FSP_SUCCESS;
    }

    if (pdTRUE == xSemaphoreTake(g_max98357a_idle_sem, max98357a_timeout_to_ticks(timeout_ms)))
    {
        return FSP_SUCCESS;
    }

    err = g_i2s1.p_api->statusGet(g_i2s1.p_ctrl, &status);
    if ((FSP_SUCCESS == err) && (I2S_STATE_STOPPED == status.state))
    {
        return FSP_SUCCESS;
    }

    return FSP_ERR_TIMEOUT;
}

static fsp_err_t max98357a_wait_tx_empty(uint32_t timeout_ms)
{
    if (pdTRUE == xSemaphoreTake(g_max98357a_tx_empty_sem, max98357a_timeout_to_ticks(timeout_ms)))
    {
        return FSP_SUCCESS;
    }

    return FSP_ERR_TIMEOUT;
}

static fsp_err_t max98357a_init_impl(void)
{
    fsp_err_t err;

    if (g_max98357a_initialized)
    {
        return FSP_SUCCESS;
    }

    if (NULL == g_max98357a_tx_empty_sem)
    {
        g_max98357a_tx_empty_sem = xSemaphoreCreateBinaryStatic(&g_max98357a_tx_empty_sem_storage);
        if (NULL == g_max98357a_tx_empty_sem)
        {
            return FSP_ERR_INTERNAL;
        }
    }

    if (NULL == g_max98357a_idle_sem)
    {
        g_max98357a_idle_sem = xSemaphoreCreateBinaryStatic(&g_max98357a_idle_sem_storage);
        if (NULL == g_max98357a_idle_sem)
        {
            return FSP_ERR_INTERNAL;
        }
    }

    max98357a_drain_semaphore(g_max98357a_tx_empty_sem);
    max98357a_drain_semaphore(g_max98357a_idle_sem);

    /* 启动外部 AUDIO_CLK：GPT8B → P604 → 4.096 MHz */
    if (!s_audio_clk_opened)
    {
        err = g_timer8.p_api->open(g_timer8.p_ctrl, g_timer8.p_cfg);
        if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
        {
            return err;
        }
        s_audio_clk_opened = true;
    }

    if (!s_audio_clk_started)
    {
        err = g_timer8.p_api->start(g_timer8.p_ctrl);
        if (FSP_SUCCESS != err)
        {
            return err;
        }
        s_audio_clk_started = true;
    }

    /* 等待 AUDIO_CLK 稳定后再打开 SSI1 */
    R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MICROSECONDS);

    if (!s_i2s_opened)
    {
        err = g_i2s1.p_api->open(g_i2s1.p_ctrl, g_i2s1.p_cfg);
        if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
        {
            return err;
        }
        s_i2s_opened = true;
    }

    g_max98357a_busy = false;
    g_max98357a_initialized = true;
    return FSP_SUCCESS;
}

static fsp_err_t max98357a_begin_transfer(void)
{
    fsp_err_t err;

    if (!g_max98357a_initialized)
    {
        err = max98357a_init_impl();
        if (FSP_SUCCESS != err)
        {
            return err;
        }
    }

    taskENTER_CRITICAL();
    if (g_max98357a_busy)
    {
        taskEXIT_CRITICAL();
        return FSP_ERR_IN_USE;
    }
    g_max98357a_busy = true;
    taskEXIT_CRITICAL();

    max98357a_drain_semaphore(g_max98357a_tx_empty_sem);
    max98357a_drain_semaphore(g_max98357a_idle_sem);

    return FSP_SUCCESS;
}

static void max98357a_end_transfer(void)
{
    taskENTER_CRITICAL();
    g_max98357a_busy = false;
    taskEXIT_CRITICAL();
}

static fsp_err_t max98357a_write_prepared_stereo(uint32_t stereo_frames, uint32_t timeout_ms)
{
    uint32_t bytes = stereo_frames * 2U * (uint32_t)sizeof(int16_t);
    fsp_err_t err;

    max98357a_drain_semaphore(g_max98357a_tx_empty_sem);

    err = g_i2s1.p_api->write(g_i2s1.p_ctrl, g_max98357a_tx_buffer, bytes);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return max98357a_wait_tx_empty(timeout_ms);
}

static void max98357a_fill_silence(uint32_t stereo_frames)
{
    for (uint32_t i = 0U; i < (stereo_frames * 2U); i++)
    {
        g_max98357a_tx_buffer[i] = 0;
    }
}

static fsp_err_t max98357a_finish_transfer(uint32_t timeout_ms)
{
    fsp_err_t err;

    max98357a_fill_silence(MAX98357A_DRAIN_SILENCE_FRAMES);
    err = max98357a_write_prepared_stereo(MAX98357A_DRAIN_SILENCE_FRAMES, timeout_ms);
    if (FSP_SUCCESS != err)
    {
        (void)g_i2s1.p_api->stop(g_i2s1.p_ctrl);
        return err;
    }

    max98357a_drain_semaphore(g_max98357a_idle_sem);
    err = g_i2s1.p_api->stop(g_i2s1.p_ctrl);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return max98357a_wait_idle(timeout_ms);
}

static fsp_err_t max98357a_write_pcm16_mono_impl(const int16_t *p_samples,
                                                 uint32_t sample_count,
                                                 uint32_t timeout_ms)
{
    fsp_err_t err;
    uint32_t offset = 0U;
    uint32_t wait_ms = (timeout_ms > 0U) ? timeout_ms : MAX98357A_DEFAULT_TIMEOUT_MS;

    if (NULL == p_samples)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    if (0U == sample_count)
    {
        return FSP_ERR_INVALID_SIZE;
    }

    err = max98357a_begin_transfer();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    while (offset < sample_count)
    {
        uint32_t frames = max98357a_min_u32(sample_count - offset, MAX98357A_TX_MONO_CHUNK_SAMPLES);
        uint32_t stereo_frames = frames + (frames & 1U);

        for (uint32_t i = 0U; i < frames; i++)
        {
            int16_t sample = p_samples[offset + i];
            g_max98357a_tx_buffer[(i * 2U)] = sample;
            g_max98357a_tx_buffer[(i * 2U) + 1U] = sample;
        }

        if (stereo_frames > frames)
        {
            g_max98357a_tx_buffer[(frames * 2U)] = 0;
            g_max98357a_tx_buffer[(frames * 2U) + 1U] = 0;
        }

        err = max98357a_write_prepared_stereo(stereo_frames, wait_ms);
        if (FSP_SUCCESS != err)
        {
            (void)g_i2s1.p_api->stop(g_i2s1.p_ctrl);
            max98357a_end_transfer();
            return err;
        }

        offset += frames;
    }

    err = max98357a_finish_transfer(wait_ms);
    max98357a_end_transfer();

    return err;
}

static fsp_err_t max98357a_play_tone_impl(uint32_t frequency_hz,
                                          uint32_t duration_ms,
                                          uint8_t volume_percent)
{
    fsp_err_t err;
    uint32_t remaining_frames;
    uint32_t phase = 0U;
    uint32_t wait_ms = MAX98357A_DEFAULT_TIMEOUT_MS;
    uint32_t volume = (volume_percent > 100U) ? 100U : volume_percent;
    uint32_t phase_step;

    if ((0U == frequency_hz) || (0U == duration_ms))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    remaining_frames = (uint32_t)(((uint64_t)MAX98357A_SAMPLE_RATE_HZ * duration_ms) / 1000U);
    if (0U == remaining_frames)
    {
        return FSP_ERR_INVALID_SIZE;
    }

    phase_step = (uint32_t)(((uint64_t)frequency_hz << 32) / MAX98357A_SAMPLE_RATE_HZ);

    err = max98357a_begin_transfer();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    while (remaining_frames > 0U)
    {
        uint32_t frames = max98357a_min_u32(remaining_frames, MAX98357A_TX_MONO_CHUNK_SAMPLES);
        uint32_t stereo_frames = frames + (frames & 1U);

        for (uint32_t i = 0U; i < frames; i++)
        {
            uint32_t table_index = phase >> MAX98357A_SINE_TABLE_SHIFT;
            int32_t sample = ((int32_t)g_max98357a_sine_table[table_index] * (int32_t)volume) / 100;

            g_max98357a_tx_buffer[(i * 2U)] = (int16_t)sample;
            g_max98357a_tx_buffer[(i * 2U) + 1U] = (int16_t)sample;
            phase += phase_step;
        }

        if (stereo_frames > frames)
        {
            g_max98357a_tx_buffer[(frames * 2U)] = 0;
            g_max98357a_tx_buffer[(frames * 2U) + 1U] = 0;
        }

        err = max98357a_write_prepared_stereo(stereo_frames, wait_ms);
        if (FSP_SUCCESS != err)
        {
            (void)g_i2s1.p_api->stop(g_i2s1.p_ctrl);
            max98357a_end_transfer();
            return err;
        }

        remaining_frames -= frames;
    }

    err = max98357a_finish_transfer(wait_ms);
    max98357a_end_transfer();

    return err;
}

static fsp_err_t max98357a_stop_impl(uint32_t timeout_ms)
{
    fsp_err_t err;
    uint32_t wait_ms = (timeout_ms > 0U) ? timeout_ms : MAX98357A_DEFAULT_TIMEOUT_MS;

    if (!g_max98357a_initialized)
    {
        return FSP_ERR_NOT_OPEN;
    }

    max98357a_drain_semaphore(g_max98357a_idle_sem);
    err = g_i2s1.p_api->stop(g_i2s1.p_ctrl);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = max98357a_wait_idle(wait_ms);
    max98357a_end_transfer();

    return err;
}

static bool max98357a_is_busy_impl(void)
{
    return g_max98357a_busy;
}

void g_i2s1_callback(i2s_callback_args_t *p_args)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (NULL == p_args)
    {
        return;
    }

    if ((I2S_EVENT_TX_EMPTY == p_args->event) && (NULL != g_max98357a_tx_empty_sem))
    {
        (void)xSemaphoreGiveFromISR(g_max98357a_tx_empty_sem, &higher_priority_task_woken);
    }
    else if ((I2S_EVENT_IDLE == p_args->event) && (NULL != g_max98357a_idle_sem))
    {
        (void)xSemaphoreGiveFromISR(g_max98357a_idle_sem, &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

const max98357a_api_t max98357a =
{
    .init = max98357a_init_impl,
    .write_pcm16_mono = max98357a_write_pcm16_mono_impl,
    .play_tone = max98357a_play_tone_impl,
    .stop = max98357a_stop_impl,
    .is_busy = max98357a_is_busy_impl,
};
