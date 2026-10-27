#include "hardware/N10/n10_radar.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>

/* N10 帧头第 1 字节。 */
#define N10_RADAR_HEADER_0              (0xA5U)

/* N10 帧头第 2 字节。 */
#define N10_RADAR_HEADER_1              (0x5AU)

/* N10 正常点云帧总长度。 */
#define N10_RADAR_FRAME_LENGTH          (58U)

/* N10 CRC 覆盖字节数，Byte0 到 Byte56。 */
#define N10_RADAR_CRC_PAYLOAD_LENGTH    (57U)

/* N10 点云数据起始偏移。 */
#define N10_RADAR_POINT_DATA_OFFSET     (7U)

/* N10 每个点云占用字节数。 */
#define N10_RADAR_POINT_DATA_SIZE       (3U)

/* N10 结束角度高字节偏移。 */
#define N10_RADAR_END_ANGLE_OFFSET      (55U)

/* 角度原始值到度的缩放系数。 */
#define N10_RADAR_ANGLE_SCALE           (100.0f)

/* 圆周角度，单位度。 */
#define N10_RADAR_FULL_CIRCLE_DEG       (360.0f)

/* N10 雷达一圈包含的码盘时间数量。 */
#define N10_RADAR_SPEED_TICKS_PER_ROUND (24.0f)

/* 每分钟微秒数，用于 RPM 换算。 */
#define N10_RADAR_US_PER_MINUTE         (60000000.0f)

/* UART6 接收环形缓存大小，保留 1 字节区分满和空。 */
#define N10_RADAR_RX_BUFFER_SIZE        (2048U)

/* UART6 接收环形缓存索引掩码。 */
#define N10_RADAR_RX_BUFFER_MASK        (N10_RADAR_RX_BUFFER_SIZE - 1U)

typedef enum e_n10_radar_parse_state
{
    N10_RADAR_PARSE_WAIT_HEADER_0 = 0,
    N10_RADAR_PARSE_WAIT_HEADER_1,
    N10_RADAR_PARSE_WAIT_LENGTH,
    N10_RADAR_PARSE_READ_FRAME,
} n10_radar_parse_state_t;

/* UART6 是否已经打开。 */
static bool g_n10_uart_opened = false;

/* UART6 RX 中断写入、后台任务读取的环形缓存。 */
static volatile uint8_t g_n10_rx_buffer[N10_RADAR_RX_BUFFER_SIZE];

/* UART6 RX 环形缓存写入位置。 */
static volatile uint32_t g_n10_rx_head = 0U;

/* UART6 RX 环形缓存读取位置。 */
static volatile uint32_t g_n10_rx_tail = 0U;

/* UART6 RX 环形缓存溢出计数。 */
static volatile uint32_t g_n10_rx_overrun_count = 0U;

/* UART6 错误中断计数。 */
static volatile uint32_t g_n10_uart_error_count = 0U;

/* N10 当前解析状态。 */
static n10_radar_parse_state_t g_n10_parse_state = N10_RADAR_PARSE_WAIT_HEADER_0;

/* N10 当前正在收集的帧缓存。 */
static uint8_t g_n10_frame[N10_RADAR_FRAME_LENGTH];

/* N10 当前帧已收集字节数。 */
static uint32_t g_n10_frame_index = 0U;

/* N10 长度错误计数。 */
static uint32_t g_n10_length_error_count = 0U;

/* N10 CRC 错误计数。 */
static uint32_t g_n10_crc_error_count = 0U;

/* 最近一次解析成功的扫描数据。 */
static n10_radar_scan_t g_n10_latest_scan;

/* 是否已经有解析成功的扫描数据。 */
static bool g_n10_latest_scan_valid = false;

/* 成功解析帧序号。 */
static uint32_t g_n10_sequence = 0U;

/* 当前一圈正在累积的 360° 扇区容器。 */
static n10_radar_cloud_t g_n10_working_cloud;

/* 最近一次发布的完整 360° 扇区容器。 */
static n10_radar_cloud_t g_n10_latest_cloud;

/* 是否已经有发布完成的 360° 扇区容器。 */
static bool g_n10_latest_cloud_valid = false;

/* 成功发布的 360° 扇区容器序号。 */
static uint32_t g_n10_cloud_sequence = 0U;

static fsp_err_t n10_radar_init_impl(void);
static fsp_err_t n10_radar_process_impl(void);
static fsp_err_t n10_radar_latest_read_impl(n10_radar_scan_t *p_scan);
static fsp_err_t n10_radar_cloud_read_impl(n10_radar_cloud_t *p_cloud);

/**
 * @brief  计算环形缓存下一个索引
 *
 * @param index  当前索引
 * @return uint32_t 下一个索引
 */
static uint32_t n10_radar_next_index(uint32_t index)
{
    return (index + 1U) & N10_RADAR_RX_BUFFER_MASK;
}

/**
 * @brief  从大端字节读取 16 位无符号数
 *
 * @param p_data  大端字节指针
 * @return uint16_t 读取到的 16 位值
 */
static uint16_t n10_radar_read_be16(const uint8_t *p_data)
{
    return (uint16_t)(((uint16_t)p_data[0] << 8) | p_data[1]);
}

/**
 * @brief  将角度归一化到 0~360 度范围
 *
 * @param angle_deg  输入角度，单位度
 * @return float 归一化后的角度
 */
static float n10_radar_normalize_angle(float angle_deg)
{
    while (angle_deg >= N10_RADAR_FULL_CIRCLE_DEG)
    {
        angle_deg -= N10_RADAR_FULL_CIRCLE_DEG;
    }

    while (angle_deg < 0.0f)
    {
        angle_deg += N10_RADAR_FULL_CIRCLE_DEG;
    }

    return angle_deg;
}

/**
 * @brief  清空一个完整 360° 雷达容器
 *
 * @param p_cloud  待清空的雷达容器
 * @return void
 */
static void n10_radar_clear_cloud(n10_radar_cloud_t *p_cloud)
{
    uint32_t i = 0U;

    if (NULL == p_cloud)
    {
        return;
    }

    p_cloud->sequence = 0U;
    p_cloud->source_scan_sequence = 0U;
    p_cloud->rpm = 0.0f;

    for (i = 0U; i < N10_RADAR_CLOUD_SECTOR_COUNT; i++)
    {
        p_cloud->sectors[i].sample_count = 0U;
        p_cloud->sectors[i].valid_count = 0U;
        p_cloud->sectors[i].zero_count = 0U;
        p_cloud->sectors[i].min_distance_mm = N10_RADAR_CLOUD_INVALID_DISTANCE_MM;
        p_cloud->sectors[i].peak_max = 0U;
    }
}

/**
 * @brief  将单个点云点累积到 360° 扇区容器
 *
 * @param p_cloud  目标雷达容器
 * @param p_point  待累积点云点
 * @return void
 */
static void n10_radar_accumulate_point(n10_radar_cloud_t *p_cloud,
                                       const n10_radar_point_t *p_point)
{
    float angle = 0.0f;
    uint32_t sector = 0U;
    n10_radar_cloud_sector_t *p_sector = NULL;

    if ((NULL == p_cloud) || (NULL == p_point))
    {
        return;
    }

    angle = n10_radar_normalize_angle(p_point->angle_deg);
    sector = (uint32_t)(angle / (float)N10_RADAR_CLOUD_SECTOR_DEG);
    if (sector >= N10_RADAR_CLOUD_SECTOR_COUNT)
    {
        sector = N10_RADAR_CLOUD_SECTOR_COUNT - 1U;
    }

    p_sector = &p_cloud->sectors[sector];
    p_sector->sample_count++;

    if (0U == p_point->distance_mm)
    {
        p_sector->zero_count++;
        return;
    }

    p_sector->valid_count++;

    if (p_point->distance_mm < p_sector->min_distance_mm)
    {
        p_sector->min_distance_mm = p_point->distance_mm;
    }

    if (p_point->peak > p_sector->peak_max)
    {
        p_sector->peak_max = p_point->peak;
    }
}

/**
 * @brief  发布当前已累积的一圈 360° 雷达容器
 *
 * @param p_scan  触发发布的最近一帧扫描数据
 * @return void
 */
static void n10_radar_publish_cloud(const n10_radar_scan_t *p_scan)
{
    n10_radar_cloud_t cloud;

    if (NULL == p_scan)
    {
        return;
    }

    cloud = g_n10_working_cloud;
    cloud.sequence = g_n10_cloud_sequence + 1U;
    cloud.source_scan_sequence = p_scan->sequence;
    cloud.rpm = p_scan->rpm;

    taskENTER_CRITICAL();
    g_n10_cloud_sequence = cloud.sequence;
    g_n10_latest_cloud = cloud;
    g_n10_latest_cloud_valid = true;
    taskEXIT_CRITICAL();

    n10_radar_clear_cloud(&g_n10_working_cloud);
}

/**
 * @brief  将解析成功的一帧扫描数据累积到完整雷达容器
 *
 * @param p_scan  解析成功的一帧扫描数据
 * @return void
 */
static void n10_radar_accumulate_scan(const n10_radar_scan_t *p_scan)
{
    bool crosses_zero = false;
    uint32_t i = 0U;

    if (NULL == p_scan)
    {
        return;
    }

    crosses_zero = (p_scan->end_angle_deg < p_scan->start_angle_deg);
    g_n10_working_cloud.source_scan_sequence = p_scan->sequence;
    g_n10_working_cloud.rpm = p_scan->rpm;

    for (i = 0U; i < N10_RADAR_POINT_COUNT; i++)
    {
        n10_radar_accumulate_point(&g_n10_working_cloud, &p_scan->points[i]);
    }

    if (crosses_zero)
    {
        n10_radar_publish_cloud(p_scan);
    }
}

/**
 * @brief  重置 N10 帧解析状态机
 *
 * @return void
 */
static void n10_radar_reset_parser(void)
{
    g_n10_parse_state = N10_RADAR_PARSE_WAIT_HEADER_0;
    g_n10_frame_index = 0U;
}

/**
 * @brief  用当前字节重新进入帧头同步状态
 *
 * @param data  当前输入字节
 * @return void
 */
static void n10_radar_resync_from_byte(uint8_t data)
{
    if (N10_RADAR_HEADER_0 == data)
    {
        g_n10_frame[0] = data;
        g_n10_frame_index = 1U;
        g_n10_parse_state = N10_RADAR_PARSE_WAIT_HEADER_1;
    }
    else
    {
        n10_radar_reset_parser();
    }
}

/**
 * @brief  校验 N10 完整帧 CRC
 *
 * @param p_frame  完整帧数据
 * @return bool true 表示 CRC 正确
 */
static bool n10_radar_crc_is_valid(const uint8_t *p_frame)
{
    uint32_t sum = 0U;
    uint32_t i = 0U;
    uint8_t crc = 0U;

    for (i = 0U; i < N10_RADAR_CRC_PAYLOAD_LENGTH; i++)
    {
        sum += p_frame[i];
    }

    crc = (uint8_t)(sum & 0xFFU);
    return (crc == p_frame[N10_RADAR_FRAME_LENGTH - 1U]);
}

/**
 * @brief  保存解析成功的 N10 扫描数据
 *
 * @param p_scan  待保存扫描数据
 * @return void
 */
static void n10_radar_store_scan(const n10_radar_scan_t *p_scan)
{
    taskENTER_CRITICAL();
    g_n10_sequence = p_scan->sequence;
    g_n10_latest_scan = *p_scan;
    g_n10_latest_scan_valid = true;
    taskEXIT_CRITICAL();
}

/**
 * @brief  解析 N10 完整帧并更新最新扫描数据
 *
 * @param p_frame  CRC 正确的完整帧
 * @return void
 */
static void n10_radar_parse_frame(const uint8_t *p_frame)
{
    n10_radar_scan_t scan;
    uint16_t speed_us = 0U;
    uint16_t start_raw = 0U;
    uint16_t end_raw = 0U;
    float start_angle = 0.0f;
    float end_angle = 0.0f;
    float interp_end_angle = 0.0f;
    float angle_step = 0.0f;
    uint32_t i = 0U;
    uint32_t offset = 0U;

    speed_us = n10_radar_read_be16(&p_frame[3]);
    start_raw = n10_radar_read_be16(&p_frame[5]);
    end_raw = n10_radar_read_be16(&p_frame[N10_RADAR_END_ANGLE_OFFSET]);

    start_angle = (float)start_raw / N10_RADAR_ANGLE_SCALE;
    end_angle = (float)end_raw / N10_RADAR_ANGLE_SCALE;
    interp_end_angle = end_angle;

    if (interp_end_angle < start_angle)
    {
        interp_end_angle += N10_RADAR_FULL_CIRCLE_DEG;
    }

    angle_step = (interp_end_angle - start_angle) / (float)(N10_RADAR_POINT_COUNT - 1U);

    scan.sequence = g_n10_sequence + 1U;
    scan.speed_us = speed_us;
    scan.start_angle_deg = n10_radar_normalize_angle(start_angle);
    scan.end_angle_deg = n10_radar_normalize_angle(end_angle);

    if (0U == speed_us)
    {
        scan.rpm = 0.0f;
    }
    else
    {
        scan.rpm = N10_RADAR_US_PER_MINUTE / ((float)speed_us * N10_RADAR_SPEED_TICKS_PER_ROUND);
    }

    for (i = 0U; i < N10_RADAR_POINT_COUNT; i++)
    {
        offset = N10_RADAR_POINT_DATA_OFFSET + (i * N10_RADAR_POINT_DATA_SIZE);
        scan.points[i].angle_deg = n10_radar_normalize_angle(start_angle + (angle_step * (float)i));
        scan.points[i].distance_mm = n10_radar_read_be16(&p_frame[offset]);
        scan.points[i].peak = p_frame[offset + 2U];
    }

    n10_radar_store_scan(&scan);
    n10_radar_accumulate_scan(&scan);
}

/**
 * @brief  处理已收满的 N10 完整帧
 *
 * @return void
 */
static void n10_radar_handle_complete_frame(void)
{
    if (n10_radar_crc_is_valid(g_n10_frame))
    {
        n10_radar_parse_frame(g_n10_frame);
    }
    else
    {
        g_n10_crc_error_count++;
    }

    n10_radar_reset_parser();
}

/**
 * @brief  将 1 个字节送入 N10 状态机
 *
 * @param data  输入字节
 * @return void
 */
static void n10_radar_consume_byte(uint8_t data)
{
    switch (g_n10_parse_state)
    {
        case N10_RADAR_PARSE_WAIT_HEADER_0:
            n10_radar_resync_from_byte(data);
            break;

        case N10_RADAR_PARSE_WAIT_HEADER_1:
            if (N10_RADAR_HEADER_1 == data)
            {
                g_n10_frame[1] = data;
                g_n10_frame_index = 2U;
                g_n10_parse_state = N10_RADAR_PARSE_WAIT_LENGTH;
            }
            else
            {
                n10_radar_resync_from_byte(data);
            }
            break;

        case N10_RADAR_PARSE_WAIT_LENGTH:
            if (N10_RADAR_FRAME_LENGTH == data)
            {
                g_n10_frame[2] = data;
                g_n10_frame_index = 3U;
                g_n10_parse_state = N10_RADAR_PARSE_READ_FRAME;
            }
            else
            {
                g_n10_length_error_count++;
                n10_radar_resync_from_byte(data);
            }
            break;

        case N10_RADAR_PARSE_READ_FRAME:
            if (g_n10_frame_index < N10_RADAR_FRAME_LENGTH)
            {
                g_n10_frame[g_n10_frame_index] = data;
                g_n10_frame_index++;
            }
            else
            {
                n10_radar_reset_parser();
            }

            if (N10_RADAR_FRAME_LENGTH == g_n10_frame_index)
            {
                n10_radar_handle_complete_frame();
            }
            break;

        default:
            n10_radar_reset_parser();
            break;
    }
}

/**
 * @brief  向 UART6 RX 环形缓存写入 1 个字节
 *
 * @param data  UART6 收到的字节
 * @return void
 */
static void n10_radar_push_byte_from_isr(uint8_t data)
{
    uint32_t head = g_n10_rx_head;
    uint32_t next_head = n10_radar_next_index(head);

    if (next_head == g_n10_rx_tail)
    {
        g_n10_rx_overrun_count++;
        return;
    }

    g_n10_rx_buffer[head] = data;
    g_n10_rx_head = next_head;
}

/**
 * @brief  从 UART6 RX 环形缓存读取 1 个字节
 *
 * @param p_data  字节输出
 * @return bool true 表示读取到字节
 */
static bool n10_radar_pop_byte(uint8_t *p_data)
{
    bool has_data = false;
    uint32_t tail = 0U;

    if (NULL == p_data)
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (g_n10_rx_tail != g_n10_rx_head)
    {
        tail = g_n10_rx_tail;
        *p_data = g_n10_rx_buffer[tail];
        g_n10_rx_tail = n10_radar_next_index(tail);
        has_data = true;
    }
    taskEXIT_CRITICAL();

    return has_data;
}

/**
 * @brief  清空 UART6 RX 环形缓存和解析状态
 *
 * @return void
 */
static void n10_radar_clear_runtime_state(void)
{
    taskENTER_CRITICAL();
    g_n10_rx_head = 0U;
    g_n10_rx_tail = 0U;
    g_n10_rx_overrun_count = 0U;
    g_n10_uart_error_count = 0U;
    g_n10_latest_scan_valid = false;
    g_n10_latest_cloud_valid = false;
    g_n10_sequence = 0U;
    g_n10_cloud_sequence = 0U;
    taskEXIT_CRITICAL();

    g_n10_length_error_count = 0U;
    g_n10_crc_error_count = 0U;
    n10_radar_clear_cloud(&g_n10_working_cloud);
    n10_radar_clear_cloud(&g_n10_latest_cloud);
    n10_radar_reset_parser();
}

/**
 * @brief  初始化 N10 雷达 UART6 接收
 *
 * @return fsp_err_t FSP_SUCCESS 表示 UART6 已打开
 */
static fsp_err_t n10_radar_init_impl(void)
{
    fsp_err_t err = FSP_SUCCESS;

    if (g_n10_uart_opened)
    {
        return FSP_SUCCESS;
    }

    n10_radar_clear_runtime_state();

    err = g_uart6.p_api->open(g_uart6.p_ctrl, g_uart6.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    g_n10_uart_opened = true;
    return FSP_SUCCESS;
}

/**
 * @brief  处理 UART6 接收缓存中的 N10 连续字节流
 *
 * @return fsp_err_t FSP_SUCCESS 表示处理完成
 */
static fsp_err_t n10_radar_process_impl(void)
{
    uint8_t data = 0U;

    if (!g_n10_uart_opened)
    {
        return FSP_ERR_NOT_OPEN;
    }

    while (n10_radar_pop_byte(&data))
    {
        n10_radar_consume_byte(data);
    }

    return FSP_SUCCESS;
}

/**
 * @brief  读取最近一次解析成功的雷达扫描数据
 *
 * @param p_scan  雷达扫描数据输出
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t n10_radar_latest_read_impl(n10_radar_scan_t *p_scan)
{
    if (NULL == p_scan)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    taskENTER_CRITICAL();
    if (!g_n10_latest_scan_valid)
    {
        taskEXIT_CRITICAL();
        return FSP_ERR_NOT_FOUND;
    }

    *p_scan = g_n10_latest_scan;
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;
}

/**
 * @brief  读取最近一次发布的完整 360° 雷达容器
 *
 * @param p_cloud  完整雷达容器输出
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t n10_radar_cloud_read_impl(n10_radar_cloud_t *p_cloud)
{
    if (NULL == p_cloud)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    taskENTER_CRITICAL();
    if (!g_n10_latest_cloud_valid)
    {
        taskEXIT_CRITICAL();
        return FSP_ERR_NOT_FOUND;
    }

    *p_cloud = g_n10_latest_cloud;
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;
}

/**
 * @brief  UART6 驱动回调，只做接收入缓存和错误计数
 *
 * @param p_args  UART 回调参数
 * @return void
 */
void uart_6_callback(uart_callback_args_t *p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            n10_radar_push_byte_from_isr((uint8_t)(p_args->data & 0xFFU));
            break;

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        case UART_EVENT_BREAK_DETECT:
            g_n10_uart_error_count++;
            break;

        case UART_EVENT_TX_DATA_EMPTY:
        case UART_EVENT_TX_COMPLETE:
        case UART_EVENT_RX_COMPLETE:
        default:
            break;
    }
}

const n10_radar_api_t n10_radar =
{
    .init = n10_radar_init_impl,
    .process = n10_radar_process_impl,
    .latest_read = n10_radar_latest_read_impl,
    .cloud_read = n10_radar_cloud_read_impl,
};
