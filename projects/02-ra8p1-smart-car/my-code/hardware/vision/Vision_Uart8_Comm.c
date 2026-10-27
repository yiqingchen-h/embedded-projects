#include "hardware/vision/Vision_Uart8_Comm.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* UART8 接收环形缓冲区大小，保留 1 字节区分满和空。 */
#define VISION_UART8_RX_BUF_SIZE       (256U)

/* 协议固定为 4 个字段。 */
#define VISION_UART8_FIELD_COUNT       (4U)

/* 单个字段固定为 8 个字符。 */
#define VISION_UART8_FIELD_WIDTH       (8U)

/* 数据区总长度，单位字节。 */
#define VISION_UART8_DATA_LEN          (VISION_UART8_FIELD_COUNT * VISION_UART8_FIELD_WIDTH)

/* 总帧长 = 起始位 1 + 数据区 32 + 校验位 2 + 截止位 1。 */
#define VISION_UART8_FRAME_LEN         (1U + VISION_UART8_DATA_LEN + 2U + 1U)

/* UART8 发送等待超时时间，单位 ms。 */
#define VISION_UART8_TX_TIMEOUT_MS     (200U)

/* 单字段可编码的最大绝对值，超过后会被钳制。 */
#define VISION_UART8_FIELD_MAX_VALUE   (9999.99f)

typedef struct st_vision_uart8_ring
{
    /**
     * @brief  后台读取位置
     */
    volatile uint16_t read_index;

    /**
     * @brief  中断写入位置
     */
    volatile uint16_t write_index;

    /**
     * @brief  UART8 RX 字节缓存
     */
    uint8_t data[VISION_UART8_RX_BUF_SIZE];
} vision_uart8_ring_t;

/* UART8 是否已经打开。 */
static bool g_vision_uart8_opened = false;

/* UART8 异步发送完成标志。 */
static volatile bool g_vision_uart8_tx_complete = false;

/* UART8 异步发送或接收错误标志。 */
static volatile bool g_vision_uart8_uart_error = false;

/* UART8 当前是否正在发送。 */
static volatile bool g_vision_uart8_tx_busy = false;

/* UART8 RX 中断写入、后台读取的环形缓存。 */
static vision_uart8_ring_t g_vision_uart8_rx_ring = {0};

/* 最近一次视觉协议帧和统计信息。 */
static vision_uart8_frame_t g_vision_uart8_latest = {0};

/* 当前正在组帧的视觉协议帧缓存。 */
static char g_vision_uart8_rx_frame[VISION_UART8_FRAME_LEN] = {0};

/* 当前视觉协议帧已收集字节数。 */
static uint8_t g_vision_uart8_rx_index = 0U;

static fsp_err_t vision_uart8_init_impl(void);
static fsp_err_t vision_uart8_process_impl(void);
static fsp_err_t vision_uart8_latest_read_impl(vision_uart8_frame_t *p_frame);
static bool vision_uart8_has_new_impl(void);
static void vision_uart8_clear_new_impl(void);
static fsp_err_t vision_uart8_send4_impl(const float *p_values, uint8_t length);
static fsp_err_t vision_uart8_test_inject_impl(float data1, float data2, float data3, float data4);

/**
 * @brief  获取当前系统时间
 *
 * @return uint32_t 当前时间，单位 ms
 */
static uint32_t vision_uart8_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/**
 * @brief  等待指定毫秒数，兼容调度器启动前后
 *
 * @param ms  等待时间，单位 ms
 * @return void
 */
static void vision_uart8_delay_ms(uint32_t ms)
{
    TickType_t delay_ticks = 0U;

    if (0U == ms)
    {
        return;
    }

    delay_ticks = pdMS_TO_TICKS(ms);
    if ((taskSCHEDULER_RUNNING == xTaskGetSchedulerState()) && (delay_ticks > 0U))
    {
        vTaskDelay(delay_ticks);
    }
    else
    {
        R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

/**
 * @brief  计算环形缓存下一个索引
 *
 * @param index  当前索引
 * @return uint16_t 下一个索引
 */
static uint16_t vision_uart8_ring_next(uint16_t index)
{
    uint16_t next_index = (uint16_t)(index + 1U);

    if (next_index >= VISION_UART8_RX_BUF_SIZE)
    {
        next_index = 0U;
    }

    return next_index;
}

/**
 * @brief  清空 UART8 RX 环形缓存
 *
 * @return void
 */
static void vision_uart8_ring_reset(void)
{
    taskENTER_CRITICAL();
    g_vision_uart8_rx_ring.read_index = 0U;
    g_vision_uart8_rx_ring.write_index = 0U;
    taskEXIT_CRITICAL();

    (void)memset(g_vision_uart8_rx_ring.data, 0, sizeof(g_vision_uart8_rx_ring.data));
}

/**
 * @brief  从中断中向 UART8 RX 环形缓存写入 1 字节
 *
 * @param data  UART8 收到的字节
 * @return void
 */
static void vision_uart8_ring_put_from_isr(uint8_t data)
{
    uint16_t write_index = g_vision_uart8_rx_ring.write_index;
    uint16_t next_write = vision_uart8_ring_next(write_index);

    if (next_write == g_vision_uart8_rx_ring.read_index)
    {
        g_vision_uart8_latest.rx_overflow_count++;
        return;
    }

    g_vision_uart8_rx_ring.data[write_index] = data;
    g_vision_uart8_rx_ring.write_index = next_write;
}

/**
 * @brief  从 UART8 RX 环形缓存读取 1 字节
 *
 * @param p_data  字节输出指针
 * @return bool   true 表示读取成功
 */
static bool vision_uart8_ring_get(uint8_t *p_data)
{
    bool has_data = false;
    uint16_t read_index = 0U;

    if (NULL == p_data)
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (g_vision_uart8_rx_ring.read_index != g_vision_uart8_rx_ring.write_index)
    {
        read_index = g_vision_uart8_rx_ring.read_index;
        *p_data = g_vision_uart8_rx_ring.data[read_index];
        g_vision_uart8_rx_ring.read_index = vision_uart8_ring_next(read_index);
        has_data = true;
    }
    taskEXIT_CRITICAL();

    return has_data;
}

/**
 * @brief  清空视觉 UART8 运行状态
 *
 * @return void
 */
static void vision_uart8_clear_runtime_state(void)
{
    vision_uart8_ring_reset();

    taskENTER_CRITICAL();
    (void)memset(&g_vision_uart8_latest, 0, sizeof(g_vision_uart8_latest));
    taskEXIT_CRITICAL();

    (void)memset(g_vision_uart8_rx_frame, 0, sizeof(g_vision_uart8_rx_frame));
    g_vision_uart8_rx_index = 0U;
    g_vision_uart8_tx_complete = false;
    g_vision_uart8_uart_error = false;
    g_vision_uart8_tx_busy = false;
}

/**
 * @brief  计算数据区 XOR 校验
 *
 * @param p_data  数据区首地址
 * @param length  数据区长度
 * @return uint8_t XOR 校验值
 */
static uint8_t vision_uart8_calc_checksum(const char *p_data, uint32_t length)
{
    uint8_t checksum = 0U;
    uint32_t i = 0U;

    if (NULL == p_data)
    {
        return 0U;
    }

    for (i = 0U; i < length; i++)
    {
        checksum ^= (uint8_t)p_data[i];
    }

    return checksum;
}

/**
 * @brief  将 4 bit 数值转换为十六进制字符
 *
 * @param nibble  低 4 bit 数值
 * @return char   十六进制字符
 */
static char vision_uart8_nibble_to_hex(uint8_t nibble)
{
    nibble &= 0x0FU;

    if (nibble < 10U)
    {
        return (char)('0' + nibble);
    }

    return (char)('A' + (nibble - 10U));
}

/**
 * @brief  将十六进制字符转换为 4 bit 数值
 *
 * @param hex  十六进制字符
 * @return int 0~15 表示成功，-1 表示非法字符
 */
static int vision_uart8_hex_to_nibble(char hex)
{
    if ((hex >= '0') && (hex <= '9'))
    {
        return (int)(hex - '0');
    }

    if ((hex >= 'A') && (hex <= 'F'))
    {
        return (int)(10 + (hex - 'A'));
    }

    if ((hex >= 'a') && (hex <= 'f'))
    {
        return (int)(10 + (hex - 'a'));
    }

    return -1;
}

/**
 * @brief  解析单个固定宽度浮点字段
 *
 * @param p_text   8 字符字段首地址
 * @param p_value  解析结果输出指针
 * @return bool    true 表示解析成功
 */
static bool vision_uart8_parse_field(const char *p_text, float *p_value)
{
    uint32_t int_part = 0U;
    uint32_t frac_part = 0U;
    float parsed_value = 0.0f;
    uint8_t i = 0U;

    if ((NULL == p_text) || (NULL == p_value))
    {
        return false;
    }

    if (('+' != p_text[0]) && ('-' != p_text[0]))
    {
        return false;
    }

    for (i = 1U; i <= 4U; i++)
    {
        if ((p_text[i] < '0') || (p_text[i] > '9'))
        {
            return false;
        }
    }

    if ('.' != p_text[5])
    {
        return false;
    }

    if (((p_text[6] < '0') || (p_text[6] > '9')) ||
        ((p_text[7] < '0') || (p_text[7] > '9')))
    {
        return false;
    }

    int_part = (uint32_t)((p_text[1] - '0') * 1000U);
    int_part += (uint32_t)((p_text[2] - '0') * 100U);
    int_part += (uint32_t)((p_text[3] - '0') * 10U);
    int_part += (uint32_t)(p_text[4] - '0');

    frac_part = (uint32_t)((p_text[6] - '0') * 10U);
    frac_part += (uint32_t)(p_text[7] - '0');

    parsed_value = (float)int_part + ((float)frac_part / 100.0f);
    if ('-' == p_text[0])
    {
        parsed_value = -parsed_value;
    }

    *p_value = parsed_value;
    return true;
}

/**
 * @brief  将浮点数格式化为固定宽度字段
 *
 * @param value       待编码浮点数
 * @param p_out_text  输出 8 字符字段缓存
 * @return void
 */
static void vision_uart8_format_field(float value, char *p_out_text)
{
    bool negative = false;
    float abs_value = 0.0f;
    uint32_t scaled = 0U;
    uint32_t int_part = 0U;
    uint32_t frac_part = 0U;

    if (NULL == p_out_text)
    {
        return;
    }

    negative = (value < 0.0f);
    abs_value = negative ? -value : value;
    if (abs_value > VISION_UART8_FIELD_MAX_VALUE)
    {
        abs_value = VISION_UART8_FIELD_MAX_VALUE;
    }

    scaled = (uint32_t)((abs_value * 100.0f) + 0.5f);
    int_part = scaled / 100U;
    frac_part = scaled % 100U;

    p_out_text[0] = negative ? '-' : '+';
    p_out_text[1] = (char)('0' + ((int_part / 1000U) % 10U));
    p_out_text[2] = (char)('0' + ((int_part / 100U) % 10U));
    p_out_text[3] = (char)('0' + ((int_part / 10U) % 10U));
    p_out_text[4] = (char)('0' + (int_part % 10U));
    p_out_text[5] = '.';
    p_out_text[6] = (char)('0' + ((frac_part / 10U) % 10U));
    p_out_text[7] = (char)('0' + (frac_part % 10U));
}

/**
 * @brief  将 4 个浮点字段组为完整视觉协议帧
 *
 * @param p_values  4 个待编码浮点字段
 * @param p_frame   输出完整协议帧缓存
 * @return fsp_err_t FSP_SUCCESS 表示组帧成功
 */
static fsp_err_t vision_uart8_build_frame(const float p_values[VISION_UART8_FIELD_COUNT], char p_frame[VISION_UART8_FRAME_LEN])
{
    uint8_t checksum = 0U;
    uint8_t i = 0U;

    if ((NULL == p_values) || (NULL == p_frame))
    {
        return FSP_ERR_INVALID_POINTER;
    }

    p_frame[0] = '@';
    for (i = 0U; i < VISION_UART8_FIELD_COUNT; i++)
    {
        vision_uart8_format_field(p_values[i], &p_frame[1U + (i * VISION_UART8_FIELD_WIDTH)]);
    }

    checksum = vision_uart8_calc_checksum(&p_frame[1], VISION_UART8_DATA_LEN);
    p_frame[1U + VISION_UART8_DATA_LEN] = vision_uart8_nibble_to_hex((uint8_t)((checksum >> 4) & 0x0FU));
    p_frame[2U + VISION_UART8_DATA_LEN] = vision_uart8_nibble_to_hex((uint8_t)(checksum & 0x0FU));
    p_frame[3U + VISION_UART8_DATA_LEN] = '#';

    return FSP_SUCCESS;
}

/**
 * @brief  等待 UART8 异步发送完成
 *
 * @param timeout_ms  超时时间，单位 ms
 * @return fsp_err_t  FSP_SUCCESS 表示发送完成
 */
static fsp_err_t vision_uart8_wait_tx_complete(uint32_t timeout_ms)
{
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    if (0U == timeout_ticks)
    {
        timeout_ticks = 1U;
    }

    while (!g_vision_uart8_tx_complete)
    {
        if (g_vision_uart8_uart_error)
        {
            return FSP_ERR_ABORTED;
        }

        if ((xTaskGetTickCount() - start_tick) >= timeout_ticks)
        {
            return FSP_ERR_TIMEOUT;
        }

        vision_uart8_delay_ms(1U);
    }

    if (g_vision_uart8_uart_error)
    {
        return FSP_ERR_ABORTED;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  发送完整视觉协议帧
 *
 * @param p_frame  完整协议帧首地址
 * @param length   完整协议帧长度
 * @return fsp_err_t FSP_SUCCESS 表示发送成功
 */
static fsp_err_t vision_uart8_send_frame(const char *p_frame, uint32_t length)
{
    fsp_err_t err = FSP_SUCCESS;

    if ((NULL == p_frame) || (0U == length))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    if (g_vision_uart8_tx_busy)
    {
        return FSP_ERR_IN_USE;
    }

    g_vision_uart8_tx_complete = false;
    g_vision_uart8_uart_error = false;
    g_vision_uart8_tx_busy = true;
    g_vision_uart8_latest.tx_busy = 1U;

    err = g_uart8.p_api->write(g_uart8.p_ctrl, (uint8_t const *)p_frame, length);
    if (FSP_SUCCESS != err)
    {
        g_vision_uart8_tx_busy = false;
        g_vision_uart8_latest.tx_busy = 0U;
        g_vision_uart8_latest.tx_error_count++;
        return err;
    }

    err = vision_uart8_wait_tx_complete(VISION_UART8_TX_TIMEOUT_MS);
    if (FSP_SUCCESS != err)
    {
        (void)g_uart8.p_api->communicationAbort(g_uart8.p_ctrl, UART_DIR_TX);
        g_vision_uart8_tx_busy = false;
        g_vision_uart8_latest.tx_busy = 0U;
        g_vision_uart8_latest.tx_error_count++;
        return err;
    }

    g_vision_uart8_latest.tx_ok_count++;
    return FSP_SUCCESS;
}

/**
 * @brief  解析一整帧视觉协议数据
 *
 * @param p_frame  完整协议帧首地址
 * @return bool    true 表示解析成功
 */
static bool vision_uart8_try_parse_frame(const char *p_frame)
{
    uint8_t checksum = 0U;
    uint8_t received_checksum = 0U;
    int high_nibble = 0;
    int low_nibble = 0;
    float parsed_values[VISION_UART8_FIELD_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0U;

    if (NULL == p_frame)
    {
        g_vision_uart8_latest.rx_format_error_count++;
        return false;
    }

    if (('@' != p_frame[0]) || ('#' != p_frame[VISION_UART8_FRAME_LEN - 1U]))
    {
        g_vision_uart8_latest.rx_format_error_count++;
        return false;
    }

    checksum = vision_uart8_calc_checksum(&p_frame[1], VISION_UART8_DATA_LEN);
    high_nibble = vision_uart8_hex_to_nibble(p_frame[1U + VISION_UART8_DATA_LEN]);
    low_nibble = vision_uart8_hex_to_nibble(p_frame[2U + VISION_UART8_DATA_LEN]);
    if ((high_nibble < 0) || (low_nibble < 0))
    {
        g_vision_uart8_latest.rx_format_error_count++;
        return false;
    }

    received_checksum = (uint8_t)(((uint8_t)high_nibble << 4) | (uint8_t)low_nibble);
    if (checksum != received_checksum)
    {
        g_vision_uart8_latest.rx_checksum_error_count++;
        return false;
    }

    for (i = 0U; i < VISION_UART8_FIELD_COUNT; i++)
    {
        if (!vision_uart8_parse_field(&p_frame[1U + (i * VISION_UART8_FIELD_WIDTH)], &parsed_values[i]))
        {
            g_vision_uart8_latest.rx_format_error_count++;
            return false;
        }
    }

    taskENTER_CRITICAL();
    g_vision_uart8_latest.data1 = parsed_values[0];
    g_vision_uart8_latest.data2 = parsed_values[1];
    g_vision_uart8_latest.data3 = parsed_values[2];
    g_vision_uart8_latest.data4 = parsed_values[3];
    g_vision_uart8_latest.frame_valid = 1U;
    g_vision_uart8_latest.new_data_flag = 1U;
    g_vision_uart8_latest.tick_ms = vision_uart8_now_ms();
    g_vision_uart8_latest.rx_ok_count++;
    taskEXIT_CRITICAL();

#if VISION_UART8_ECHO_BACK_ENABLE
    (void)vision_uart8_send4_impl(parsed_values, VISION_UART8_FIELD_COUNT);
#endif

    return true;
}

/**
 * @brief  将 1 个字节送入视觉协议组帧状态机
 *
 * @param data  UART8 输入字节
 * @return void
 */
static void vision_uart8_consume_byte(uint8_t data)
{
    if (0U == g_vision_uart8_rx_index)
    {
        if ('@' == (char)data)
        {
            g_vision_uart8_rx_frame[0] = (char)data;
            g_vision_uart8_rx_index = 1U;
        }

        return;
    }

    if ('@' == (char)data)
    {
        /* 收到新的帧头时直接重新同步，丢弃前一段失步数据。 */
        g_vision_uart8_rx_frame[0] = (char)data;
        g_vision_uart8_rx_index = 1U;
        return;
    }

    g_vision_uart8_rx_frame[g_vision_uart8_rx_index] = (char)data;
    g_vision_uart8_rx_index++;

    if (g_vision_uart8_rx_index >= VISION_UART8_FRAME_LEN)
    {
        (void)vision_uart8_try_parse_frame(g_vision_uart8_rx_frame);
        g_vision_uart8_rx_index = 0U;
    }
}

/**
 * @brief  初始化 UART8 视觉通信模块
 *
 * @return fsp_err_t FSP_SUCCESS 表示 UART8 已打开
 */
static fsp_err_t vision_uart8_init_impl(void)
{
    fsp_err_t err = FSP_SUCCESS;

    vision_uart8_clear_runtime_state();

    if (g_vision_uart8_opened)
    {
        return FSP_SUCCESS;
    }

    err = g_uart8.p_api->open(g_uart8.p_ctrl, g_uart8.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    g_vision_uart8_opened = true;
    return FSP_SUCCESS;
}

/**
 * @brief  处理 UART8 接收缓存中的视觉协议字节流
 *
 * 该接口只负责后台解析和可选回传，建议由 FreeRTOS 后台任务每 4ms 调用一次。
 * 其他任务通过 latest_read/has_new/clear_new 消费数据。
 *
 * @return fsp_err_t FSP_SUCCESS 表示处理完成
 */
static fsp_err_t vision_uart8_process_impl(void)
{
    uint8_t data = 0U;

    if (!g_vision_uart8_opened)
    {
        return FSP_ERR_NOT_OPEN;
    }

    while (vision_uart8_ring_get(&data))
    {
        vision_uart8_consume_byte(data);
    }

    return FSP_SUCCESS;
}

/**
 * @brief  读取最近一次缓存的数据和统计信息
 *
 * 该接口只读取快照，不清除 new_data_flag，消费任务读完后按需调用 clear_new。
 *
 * @param p_frame  视觉数据输出指针
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t vision_uart8_latest_read_impl(vision_uart8_frame_t *p_frame)
{
    if (NULL == p_frame)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    taskENTER_CRITICAL();
    *p_frame = g_vision_uart8_latest;
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;
}

/**
 * @brief  查询是否存在尚未被上层清除的新数据
 *
 * @return bool true 表示存在新数据
 */
static bool vision_uart8_has_new_impl(void)
{
    return (0U != g_vision_uart8_latest.new_data_flag);
}

/**
 * @brief  清除新数据标志
 *
 * @return void
 */
static void vision_uart8_clear_new_impl(void)
{
    g_vision_uart8_latest.new_data_flag = 0U;
}

/**
 * @brief  按视觉协议发送最多 4 个浮点字段
 *
 * @param p_values  待发送浮点数组
 * @param length    数组中有效字段数量
 * @return fsp_err_t FSP_SUCCESS 表示发送成功
 */
static fsp_err_t vision_uart8_send4_impl(const float *p_values, uint8_t length)
{
    float tx_values[VISION_UART8_FIELD_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    char frame[VISION_UART8_FRAME_LEN];
    fsp_err_t err = FSP_SUCCESS;
    uint8_t count = 0U;
    uint8_t i = 0U;

    if (!g_vision_uart8_opened)
    {
        return FSP_ERR_NOT_OPEN;
    }

    if ((length > 0U) && (NULL == p_values))
    {
        return FSP_ERR_INVALID_POINTER;
    }

    count = length;
    if (count > VISION_UART8_FIELD_COUNT)
    {
        count = VISION_UART8_FIELD_COUNT;
    }

    for (i = 0U; i < count; i++)
    {
        tx_values[i] = p_values[i];
    }

    err = vision_uart8_build_frame(tx_values, frame);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return vision_uart8_send_frame(frame, VISION_UART8_FRAME_LEN);
}

/**
 * @brief  注入一帧测试数据到 UART8 接收缓存
 *
 * @param data1  第 1 个测试字段
 * @param data2  第 2 个测试字段
 * @param data3  第 3 个测试字段
 * @param data4  第 4 个测试字段
 * @return fsp_err_t FSP_SUCCESS 表示注入成功
 */
static fsp_err_t vision_uart8_test_inject_impl(float data1, float data2, float data3, float data4)
{
    float values[VISION_UART8_FIELD_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    char frame[VISION_UART8_FRAME_LEN];
    fsp_err_t err = FSP_SUCCESS;
    uint8_t i = 0U;

    if (!g_vision_uart8_opened)
    {
        return FSP_ERR_NOT_OPEN;
    }

    values[0] = data1;
    values[1] = data2;
    values[2] = data3;
    values[3] = data4;

    err = vision_uart8_build_frame(values, frame);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    for (i = 0U; i < VISION_UART8_FRAME_LEN; i++)
    {
        vision_uart8_ring_put_from_isr((uint8_t)frame[i]);
    }

    return FSP_SUCCESS;
}

/* 视觉 UART8 公开 API 实例。 */
const vision_uart8_api_t vision_uart8 =
{
    .init = vision_uart8_init_impl,
    .process = vision_uart8_process_impl,
    .latest_read = vision_uart8_latest_read_impl,
    .has_new = vision_uart8_has_new_impl,
    .clear_new = vision_uart8_clear_new_impl,
    .send4 = vision_uart8_send4_impl,
    .test_inject = vision_uart8_test_inject_impl,
};

/**
 * @brief  UART8 驱动回调，只做收字节、发送状态和错误计数
 *
 * @param p_args  UART 回调参数
 * @return void
 */
void uart_8_callback(uart_callback_args_t *p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            vision_uart8_ring_put_from_isr((uint8_t)(p_args->data & 0xFFU));
            break;

        case UART_EVENT_TX_COMPLETE:
            g_vision_uart8_tx_complete = true;
            g_vision_uart8_tx_busy = false;
            g_vision_uart8_latest.tx_busy = 0U;
            break;

        case UART_EVENT_ERR_OVERFLOW:
            g_vision_uart8_latest.rx_overflow_count++;
            g_vision_uart8_uart_error = true;
            break;

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_BREAK_DETECT:
            g_vision_uart8_latest.rx_format_error_count++;
            g_vision_uart8_uart_error = true;
            break;

        case UART_EVENT_TX_DATA_EMPTY:
        case UART_EVENT_RX_COMPLETE:
        default:
            break;
    }
}
