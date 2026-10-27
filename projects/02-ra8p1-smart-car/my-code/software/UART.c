#include "software/UART.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* MIC_DATA 协议中 label 字段最大长度，不含字符串结尾 '\0'。 */
#define UART_PCM16_LABEL_MAX_LENGTH (31U)

/* CRC32 初始值，和 Python binascii.crc32 的最终结果保持一致。 */
#define UART_CRC32_INIT             (0xFFFFFFFFUL)

/* CRC32 反射多项式，用于逐字节计算 PCM payload 校验值。 */
#define UART_CRC32_POLY             (0xEDB88320UL)

/* SCI9 UART 是否已经打开，避免重复 open。 */
static bool g_uart_open = false;

/* UART 发送互斥锁存储，避免 printf 打断 MIC_DATA 二进制 payload。 */
static StaticSemaphore_t g_uart_tx_mutex_storage;

/* UART 发送互斥锁句柄，保护普通发送和协议包发送。 */
static SemaphoreHandle_t g_uart_tx_mutex = NULL;

/* 异步发送完成标志，由 uart_9_callback 在 TX 事件中置位。 */
static volatile bool g_uart_tx_done = false;

/* 异步发送或串口错误码，由 uart_9_callback 在错误事件中记录。 */
static volatile fsp_err_t g_uart_async_err = FSP_SUCCESS;

/* printf 行缓冲，减少单字符 printf 频繁触发 UART write。 */
static uint8_t g_uart_stdio_buffer[128];

/* printf 行缓冲当前已缓存字节数。 */
static uint32_t g_uart_stdio_count = 0U;

static fsp_err_t uart_write_impl(const uint8_t *p_data, uint32_t length);
static fsp_err_t uart_write_unlocked(const uint8_t *p_data, uint32_t length);

/**
 * @brief  确保 UART 发送互斥锁已创建
 *
 * @return fsp_err_t FSP_SUCCESS 表示互斥锁可用
 */
static fsp_err_t uart_mutex_init(void)
{
    taskENTER_CRITICAL();
    if (NULL == g_uart_tx_mutex)
    {
        g_uart_tx_mutex = xSemaphoreCreateMutexStatic(&g_uart_tx_mutex_storage);
    }
    taskEXIT_CRITICAL();

    return (NULL != g_uart_tx_mutex) ? FSP_SUCCESS : FSP_ERR_INTERNAL;
}

/**
 * @brief  获取 UART 发送互斥锁
 *
 * @return fsp_err_t FSP_SUCCESS 表示已进入发送临界区
 */
static fsp_err_t uart_lock(void)
{
    fsp_err_t err = uart_mutex_init();

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    if (taskSCHEDULER_RUNNING == xTaskGetSchedulerState())
    {
        if (pdTRUE != xSemaphoreTake(g_uart_tx_mutex, portMAX_DELAY))
        {
            return FSP_ERR_TIMEOUT;
        }
    }

    return FSP_SUCCESS;
}

/**
 * @brief  释放 UART 发送互斥锁
 *
 * @return void
 */
static void uart_unlock(void)
{
    if ((NULL != g_uart_tx_mutex) && (taskSCHEDULER_RUNNING == xTaskGetSchedulerState()))
    {
        (void)xSemaphoreGive(g_uart_tx_mutex);
    }
}

/**
 * @brief  刷新 printf 行缓冲
 *
 * @return void
 */
static void uart_stdio_flush(void)
{
    if (g_uart_stdio_count > 0U)
    {
        (void)uart_write_impl(g_uart_stdio_buffer, g_uart_stdio_count);
        g_uart_stdio_count = 0U;
    }
}

/**
 * @brief  向 printf 行缓冲追加 1 个字符
 *
 * @param ch  要追加的字符
 * @return void
 */
static void uart_stdio_put(uint8_t ch)
{
    if (g_uart_stdio_count >= sizeof(g_uart_stdio_buffer))
    {
        uart_stdio_flush();
    }

    g_uart_stdio_buffer[g_uart_stdio_count] = ch;
    g_uart_stdio_count++;
}

/**
 * @brief  初始化 RASC 生成的 SCI9 UART 实例
 *
 * @return fsp_err_t  FSP_SUCCESS 表示串口可用
 */
static fsp_err_t uart_init_impl(void)
{
    fsp_err_t err = FSP_SUCCESS;

    if (g_uart_open)
    {
        return FSP_SUCCESS;
    }

    err = uart_mutex_init();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = g_uart9.p_api->open(g_uart9.p_ctrl, g_uart9.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    g_uart_open = true;
    return FSP_SUCCESS;
}

/**
 * @brief  等待一次异步发送完成
 *
 * @return fsp_err_t  FSP_SUCCESS 表示发送完成
 */
static fsp_err_t uart_wait_tx_done(void)
{
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(UART_TX_TIMEOUT_MS);

    if (0U == timeout_ticks)
    {
        timeout_ticks = 1U;
    }

    while (!g_uart_tx_done)
    {
        if ((xTaskGetTickCount() - start_tick) >= timeout_ticks)
        {
            return FSP_ERR_TIMEOUT;
        }

        if (taskSCHEDULER_RUNNING == xTaskGetSchedulerState())
        {
            vTaskDelay(1U);
        }
        else
        {
            R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
        }
    }

    if (FSP_SUCCESS != g_uart_async_err)
    {
        return g_uart_async_err;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  发送一段数据
 *
 * @param p_data  待发送数据指针
 * @param length  待发送数据长度
 * @return fsp_err_t  FSP_SUCCESS 表示数据已发送完成
 */
static fsp_err_t uart_write_unlocked(const uint8_t *p_data, uint32_t length)
{
    fsp_err_t err = FSP_SUCCESS;

    if ((NULL == p_data) && (length > 0U))
    {
        return FSP_ERR_INVALID_POINTER;
    }

    if (0U == length)
    {
        return FSP_SUCCESS;
    }

    err = uart_init_impl();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    g_uart_tx_done = false;
    g_uart_async_err = FSP_SUCCESS;

    err = g_uart9.p_api->write(g_uart9.p_ctrl, p_data, length);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return uart_wait_tx_done();
}

/**
 * @brief  发送一段数据
 *
 * @param p_data  待发送数据指针
 * @param length  待发送数据长度
 * @return fsp_err_t  FSP_SUCCESS 表示数据已发送完成
 */
static fsp_err_t uart_write_impl(const uint8_t *p_data, uint32_t length)
{
    fsp_err_t err = uart_lock();

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = uart_write_unlocked(p_data, length);
    uart_unlock();

    return err;
}

/**
 * @brief  计算较小的 uint32_t 值
 *
 * @param a  输入 A
 * @param b  输入 B
 * @return uint32_t  较小值
 */
static uint32_t uart_min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

/**
 * @brief  分包发送一段数据
 *
 * @param p_data      待发送数据指针
 * @param length      待发送数据长度
 * @param chunk_size  单包长度，0 表示使用默认值
 * @return fsp_err_t  FSP_SUCCESS 表示全部发送完成
 */
static fsp_err_t uart_write_chunks_unlocked(const uint8_t *p_data, uint32_t length, uint32_t chunk_size)
{
    uint32_t offset = 0U;

    if ((NULL == p_data) && (length > 0U))
    {
        return FSP_ERR_INVALID_POINTER;
    }

    if (0U == length)
    {
        return FSP_SUCCESS;
    }

    if (0U == chunk_size)
    {
        chunk_size = UART_WRITE_CHUNK_DEFAULT_BYTES;
    }

    while (offset < length)
    {
        uint32_t chunk = uart_min_u32(chunk_size, length - offset);
        fsp_err_t err = uart_write_unlocked(&p_data[offset], chunk);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        offset += chunk;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  分包发送一段数据
 *
 * @param p_data      待发送数据指针
 * @param length      待发送数据长度
 * @param chunk_size  单包长度，0 表示使用默认值
 * @return fsp_err_t  FSP_SUCCESS 表示全部发送完成
 */
static fsp_err_t uart_write_chunks_impl(const uint8_t *p_data, uint32_t length, uint32_t chunk_size)
{
    fsp_err_t err = uart_lock();

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = uart_write_chunks_unlocked(p_data, length, chunk_size);
    uart_unlock();

    return err;
}

/**
 * @brief  判断标签字符是否允许进入串口协议头
 *
 * @param ch  待检查字符
 * @return bool true 表示允许
 */
static bool uart_label_char_is_valid(char ch)
{
    return (((ch >= 'a') && (ch <= 'z')) ||
            ((ch >= 'A') && (ch <= 'Z')) ||
            ((ch >= '0') && (ch <= '9')) ||
            ('_' == ch));
}

/**
 * @brief  校验 MIC_DATA 标签格式
 *
 * @param label  标签字符串
 * @return fsp_err_t FSP_SUCCESS 表示标签可用于协议头
 */
static fsp_err_t uart_validate_label(const char *label)
{
    uint32_t length = 0U;

    if (NULL == label)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    while ('\0' != label[length])
    {
        if (length >= UART_PCM16_LABEL_MAX_LENGTH)
        {
            return FSP_ERR_INVALID_SIZE;
        }

        if (!uart_label_char_is_valid(label[length]))
        {
            return FSP_ERR_INVALID_ARGUMENT;
        }

        length++;
    }

    if (0U == length)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  计算 CRC32，范围只覆盖 PCM payload
 *
 * @param p_data  输入数据
 * @param length  数据长度
 * @return uint32_t CRC32
 */
static uint32_t uart_crc32(const uint8_t *p_data, uint32_t length)
{
    uint32_t crc = UART_CRC32_INIT;

    for (uint32_t i = 0U; i < length; i++)
    {
        crc ^= p_data[i];
        for (uint32_t bit = 0U; bit < 8U; bit++)
        {
            if (0U != (crc & 1U))
            {
                crc = (crc >> 1U) ^ UART_CRC32_POLY;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc ^ UART_CRC32_INIT;
}

/**
 * @brief  按 MIC_DATA 协议发送 PCM16LE 单声道音频
 *
 * @param label           样本标签
 * @param p_pcm           PCM16 采样数据
 * @param samples         采样点数量
 * @param sample_rate_hz  采样率
 * @return fsp_err_t      FSP_SUCCESS 表示发送完成
 */
static fsp_err_t uart_write_pcm16_packet_impl(const char *label,
                                              const int16_t *p_pcm,
                                              uint32_t samples,
                                              uint32_t sample_rate_hz)
{
    char header[128];
    int header_len;
    uint32_t payload_bytes;
    uint32_t payload_crc;
    const uint8_t *p_payload;
    fsp_err_t err;

    err = uart_validate_label(label);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    if (NULL == p_pcm)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    if ((0U == samples) || (0U == sample_rate_hz))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    if (samples > (UINT32_MAX / (uint32_t)sizeof(int16_t)))
    {
        return FSP_ERR_INVALID_SIZE;
    }

    payload_bytes = samples * (uint32_t)sizeof(int16_t);
    p_payload = (const uint8_t *)p_pcm;
    payload_crc = uart_crc32(p_payload, payload_bytes);

    header_len = snprintf(header,
                          sizeof(header),
                          "MIC_DATA v=1 label=%s rate=%lu samples=%lu bytes=%lu crc=%08lX\r\n",
                          label,
                          (unsigned long)sample_rate_hz,
                          (unsigned long)samples,
                          (unsigned long)payload_bytes,
                          (unsigned long)payload_crc);
    if ((header_len <= 0) || ((uint32_t)header_len >= sizeof(header)))
    {
        return FSP_ERR_INVALID_SIZE;
    }

    err = uart_lock();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = uart_write_unlocked((const uint8_t *)header, (uint32_t)header_len);
    if (FSP_SUCCESS != err)
    {
        uart_unlock();
        return err;
    }

    err = uart_write_chunks_unlocked(p_payload, payload_bytes, UART_WRITE_CHUNK_DEFAULT_BYTES);
    if (FSP_SUCCESS != err)
    {
        uart_unlock();
        return err;
    }

    err = uart_write_unlocked((const uint8_t *)"\r\nMIC_END\r\n", 11U);
    uart_unlock();

    return err;
}

/**
 * @brief  发送 1 个字符
 *
 * @param ch  要发送的字符
 * @return fsp_err_t  FSP_SUCCESS 表示字符已发送完成
 */
static fsp_err_t uart_put_char_impl(uint8_t ch)
{
    return uart_write_impl(&ch, 1U);
}

/**
 * @brief  关闭 RASC 生成的 SCI9 UART 实例
 *
 * @return void
 */
static void uart_deinit_impl(void)
{
    if (g_uart_open)
    {
        (void)g_uart9.p_api->close(g_uart9.p_ctrl);
        g_uart_open = false;
    }
}

/**
 * @brief  SCI9 UART 回调函数
 *
 * @param p_args  UART 回调参数
 * @return void
 */
void uart_9_callback(uart_callback_args_t *p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    if ((UART_EVENT_TX_DATA_EMPTY == p_args->event) ||
        (UART_EVENT_TX_COMPLETE == p_args->event))
    {
        g_uart_tx_done = true;
    }
    else if ((UART_EVENT_ERR_PARITY == p_args->event) ||
             (UART_EVENT_ERR_FRAMING == p_args->event) ||
             (UART_EVENT_ERR_OVERFLOW == p_args->event) ||
             (UART_EVENT_BREAK_DETECT == p_args->event))
    {
        g_uart_async_err = FSP_ERR_ABORTED;
        g_uart_tx_done = true;
    }
    else
    {
        /* 其他事件本模块暂不处理。 */
    }
}

/**
 * @brief  printf 底层字符输出重定向
 *
 * @param ch        printf 传入的字符
 * @param p_stream  C 库流指针，本工程未使用
 * @return int      返回已处理的字符
 */
int fputc(int ch, FILE *p_stream)
{
    static bool last_was_cr = false;

    (void)p_stream;

    if (('\n' == ch) && (!last_was_cr))
    {
        uart_stdio_put((uint8_t)'\r');
    }

    uart_stdio_put((uint8_t)ch);

    if ('\n' == ch)
    {
        uart_stdio_flush();
    }

    last_was_cr = ('\r' == ch);
    return ch;
}

const debug_uart_api_t uart =
{
    .init = uart_init_impl,
    .put_char = uart_put_char_impl,
    .write = uart_write_impl,
    .write_chunks = uart_write_chunks_impl,
    .write_pcm16_packet = uart_write_pcm16_packet_impl,
    .deinit = uart_deinit_impl,
};
