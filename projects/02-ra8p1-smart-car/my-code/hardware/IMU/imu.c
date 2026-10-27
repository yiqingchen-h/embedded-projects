#include "hardware/IMU/imu.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* IMU 帧头字节。 */
#define IMU_FRAME_HEADER              (0x55U)

/* IMU 单帧长度，单位字节。 */
#define IMU_FRAME_LENGTH              (11U)

/* IMU 帧校验字节索引。 */
#define IMU_FRAME_CHECKSUM_INDEX      (10U)

/* IMU 角速度帧类型。 */
#define IMU_FRAME_TYPE_GYRO           (0x52U)

/* IMU 欧拉角帧类型。 */
#define IMU_FRAME_TYPE_ANGLE          (0x53U)

/* IMU 原始有符号 16 位数据满量程。 */
#define IMU_RAW_SCALE                 (32768.0f)

/* IMU 角速度满量程，单位度每秒。 */
#define IMU_GYRO_SCALE_DPS            (2000.0f)

/* IMU 欧拉角满量程，单位度。 */
#define IMU_ANGLE_SCALE_DEG           (180.0f)

/* IMU 数据超过该时间未更新则视为无效，单位 ms。 */
#define IMU_DATA_READY_TIMEOUT_MS     (300U)

/* IMU 数据年龄最大保持值，防止长时间断线后无符号溢出。 */
#define IMU_DATA_AGE_MAX_MS           (600000U)

/* UART0 接收环形缓存大小，保留 1 字节区分满和空。 */
#define IMU_RX_BUFFER_SIZE            (512U)

/* UART0 接收环形缓存索引掩码。 */
#define IMU_RX_BUFFER_MASK            (IMU_RX_BUFFER_SIZE - 1U)

/* IMU UART0 是否已经打开。 */
static bool g_imu_uart_opened = false;

/* UART0 RX 中断写入、后台任务读取的环形缓存。 */
static volatile uint8_t g_imu_rx_buffer[IMU_RX_BUFFER_SIZE];

/* UART0 RX 环形缓存写入位置。 */
static volatile uint32_t g_imu_rx_head = 0U;

/* UART0 RX 环形缓存读取位置。 */
static volatile uint32_t g_imu_rx_tail = 0U;

/* UART0 RX 环形缓存溢出计数。 */
static volatile uint32_t g_imu_rx_overflow_count = 0U;

/* UART0 错误中断计数。 */
static volatile uint32_t g_imu_uart_error_count = 0U;

/* IMU 帧校验错误计数。 */
static uint32_t g_imu_checksum_error_count = 0U;

/* IMU 未支持帧类型计数。 */
static uint32_t g_imu_frame_error_count = 0U;

/* IMU 当前正在收集的帧缓存。 */
static uint8_t g_imu_frame[IMU_FRAME_LENGTH] = {0};

/* IMU 当前帧已收集字节数。 */
static uint32_t g_imu_frame_index = 0U;

/* 是否已经收到过欧拉角帧。 */
static bool g_imu_angle_seen = false;

/* 是否已经收到过角速度帧。 */
static bool g_imu_gyro_seen = false;

/* 最近一次 IMU 快照。 */
static imu_snapshot_t g_imu_snapshot = {0};

static fsp_err_t imu_init_impl(void);
static fsp_err_t imu_process_impl(uint32_t elapsed_ms);
static fsp_err_t imu_read_impl(imu_snapshot_t *p_snapshot);

/**
 * @brief  计算环形缓存下一个索引
 *
 * @param index 当前索引
 * @return uint32_t 下一个索引
 */
static uint32_t imu_next_index(uint32_t index)
{
    return (index + 1U) & IMU_RX_BUFFER_MASK;
}

/**
 * @brief  饱和累加毫秒计数
 *
 * @param value 当前值
 * @param add   增量
 * @return uint32_t 累加后的饱和值
 */
static uint32_t imu_age_add(uint32_t value, uint32_t add)
{
    if (value >= IMU_DATA_AGE_MAX_MS)
    {
        return IMU_DATA_AGE_MAX_MS;
    }

    if (add > (IMU_DATA_AGE_MAX_MS - value))
    {
        return IMU_DATA_AGE_MAX_MS;
    }

    return value + add;
}

/**
 * @brief  更新快照内用于显示的最大数据年龄
 *
 * @return void
 */
static void imu_update_display_age_unlocked(void)
{
    if (g_imu_angle_seen && g_imu_gyro_seen)
    {
        g_imu_snapshot.age_ms = (g_imu_snapshot.angle_age_ms > g_imu_snapshot.gyro_age_ms) ?
                                g_imu_snapshot.angle_age_ms :
                                g_imu_snapshot.gyro_age_ms;
    }
    else if (g_imu_angle_seen)
    {
        g_imu_snapshot.age_ms = g_imu_snapshot.angle_age_ms;
    }
    else if (g_imu_gyro_seen)
    {
        g_imu_snapshot.age_ms = g_imu_snapshot.gyro_age_ms;
    }
    else
    {
        g_imu_snapshot.age_ms = 0U;
    }
}

/**
 * @brief  更新 IMU 数据年龄和有效标志
 *
 * @param elapsed_ms 距离上次调用经过的时间，单位 ms
 * @return void
 */
static void imu_update_age(uint32_t elapsed_ms)
{
    if (0U == elapsed_ms)
    {
        return;
    }

    taskENTER_CRITICAL();
    if (g_imu_angle_seen)
    {
        g_imu_snapshot.angle_age_ms = imu_age_add(g_imu_snapshot.angle_age_ms, elapsed_ms);
    }

    if (g_imu_gyro_seen)
    {
        g_imu_snapshot.gyro_age_ms = imu_age_add(g_imu_snapshot.gyro_age_ms, elapsed_ms);
    }

    g_imu_snapshot.angle_valid = g_imu_angle_seen &&
                                 (g_imu_snapshot.angle_age_ms <= IMU_DATA_READY_TIMEOUT_MS);
    g_imu_snapshot.gyro_valid = g_imu_gyro_seen &&
                                (g_imu_snapshot.gyro_age_ms <= IMU_DATA_READY_TIMEOUT_MS);
    imu_update_display_age_unlocked();
    taskEXIT_CRITICAL();
}

/**
 * @brief  从小端字节读取 16 位有符号数
 *
 * @param p_data 小端字节指针
 * @return int16_t 读取到的 16 位有符号值
 */
static int16_t imu_read_i16_le(const uint8_t *p_data)
{
    int32_t raw = 0;

    if (NULL == p_data)
    {
        return 0;
    }

    raw = (int32_t)(((uint16_t)p_data[1] << 8) | p_data[0]);
    if (raw >= 32768)
    {
        raw -= 65536;
    }

    return (int16_t)raw;
}

/**
 * @brief  将角速度原始值转换为度每秒
 *
 * @param raw 角速度原始值
 * @return float 转换后的角速度，单位度每秒
 */
static float imu_raw_to_gyro_dps(int16_t raw)
{
    return ((float)raw * IMU_GYRO_SCALE_DPS) / IMU_RAW_SCALE;
}

/**
 * @brief  将欧拉角原始值转换为度
 *
 * @param raw 欧拉角原始值
 * @return float 转换后的角度，单位度
 */
static float imu_raw_to_angle_deg(int16_t raw)
{
    return ((float)raw * IMU_ANGLE_SCALE_DEG) / IMU_RAW_SCALE;
}

/**
 * @brief  计算 IMU 帧校验和
 *
 * @param p_frame IMU 帧指针
 * @return uint8_t Byte0 到 Byte9 的累加校验和
 */
static uint8_t imu_calc_checksum(const uint8_t *p_frame)
{
    uint32_t i = 0U;
    uint8_t checksum = 0U;

    if (NULL == p_frame)
    {
        return 0U;
    }

    for (i = 0U; i < IMU_FRAME_CHECKSUM_INDEX; i++)
    {
        checksum = (uint8_t)(checksum + p_frame[i]);
    }

    return checksum;
}

/**
 * @brief  发布一帧角速度数据到快照
 *
 * @param p_frame IMU 角速度帧
 * @return void
 */
static void imu_publish_gyro(const uint8_t *p_frame)
{
    float gyro_z_dps = 0.0f;

    if (NULL == p_frame)
    {
        return;
    }

    gyro_z_dps = imu_raw_to_gyro_dps(imu_read_i16_le(&p_frame[6]));

    taskENTER_CRITICAL();
    g_imu_gyro_seen = true;
    g_imu_snapshot.gyro_valid = true;
    g_imu_snapshot.gyro_z_dps = gyro_z_dps;
    g_imu_snapshot.gyro_sequence++;
    g_imu_snapshot.gyro_age_ms = 0U;
    g_imu_snapshot.last_error = FSP_SUCCESS;
    imu_update_display_age_unlocked();
    taskEXIT_CRITICAL();
}

/**
 * @brief  发布一帧欧拉角数据到快照
 *
 * @param p_frame IMU 欧拉角帧
 * @return void
 */
static void imu_publish_angle(const uint8_t *p_frame)
{
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;

    if (NULL == p_frame)
    {
        return;
    }

    roll_deg = imu_raw_to_angle_deg(imu_read_i16_le(&p_frame[2]));
    pitch_deg = imu_raw_to_angle_deg(imu_read_i16_le(&p_frame[4]));
    yaw_deg = imu_raw_to_angle_deg(imu_read_i16_le(&p_frame[6]));

    taskENTER_CRITICAL();
    g_imu_angle_seen = true;
    g_imu_snapshot.angle_valid = true;
    g_imu_snapshot.roll_deg = roll_deg;
    g_imu_snapshot.pitch_deg = pitch_deg;
    g_imu_snapshot.yaw_deg = yaw_deg;
    g_imu_snapshot.angle_sequence++;
    g_imu_snapshot.angle_age_ms = 0U;
    g_imu_snapshot.last_error = FSP_SUCCESS;
    imu_update_display_age_unlocked();
    taskEXIT_CRITICAL();
}

/**
 * @brief  处理一帧完整 IMU 数据
 *
 * @param p_frame IMU 帧指针
 * @return void
 */
static void imu_process_frame(const uint8_t *p_frame)
{
    uint8_t checksum = 0U;

    if (NULL == p_frame)
    {
        return;
    }

    checksum = imu_calc_checksum(p_frame);
    if (checksum != p_frame[IMU_FRAME_CHECKSUM_INDEX])
    {
        taskENTER_CRITICAL();
        g_imu_checksum_error_count++;
        g_imu_snapshot.last_error = FSP_ERR_INVALID_DATA;
        taskEXIT_CRITICAL();
        return;
    }

    switch (p_frame[1])
    {
        case IMU_FRAME_TYPE_GYRO:
            imu_publish_gyro(p_frame);
            break;

        case IMU_FRAME_TYPE_ANGLE:
            imu_publish_angle(p_frame);
            break;

        default:
            taskENTER_CRITICAL();
            g_imu_frame_error_count++;
            g_imu_snapshot.last_error = FSP_ERR_UNSUPPORTED;
            taskEXIT_CRITICAL();
            break;
    }
}

/**
 * @brief  消费 1 个 IMU 串口字节并尝试组帧
 *
 * @param data 串口收到的字节
 * @return void
 */
static void imu_consume_byte(uint8_t data)
{
    if (0U == g_imu_frame_index)
    {
        if (IMU_FRAME_HEADER != data)
        {
            return;
        }
    }

    g_imu_frame[g_imu_frame_index] = data;
    g_imu_frame_index++;

    if (g_imu_frame_index >= IMU_FRAME_LENGTH)
    {
        imu_process_frame(g_imu_frame);
        g_imu_frame_index = 0U;
    }
}

/**
 * @brief  从中断中向 UART0 RX 环形缓存写入 1 字节
 *
 * @param data UART0 收到的字节
 * @return void
 */
static void imu_push_byte_from_isr(uint8_t data)
{
    uint32_t head = g_imu_rx_head;
    uint32_t next_head = imu_next_index(head);

    if (next_head == g_imu_rx_tail)
    {
        g_imu_rx_overflow_count++;
        return;
    }

    g_imu_rx_buffer[head] = data;
    g_imu_rx_head = next_head;
}

/**
 * @brief  从 UART0 RX 环形缓存读取 1 个字节
 *
 * @param p_data 字节输出指针
 * @return bool true 表示读取到字节
 */
static bool imu_pop_byte(uint8_t *p_data)
{
    bool has_data = false;
    uint32_t tail = 0U;

    if (NULL == p_data)
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (g_imu_rx_tail != g_imu_rx_head)
    {
        tail = g_imu_rx_tail;
        *p_data = g_imu_rx_buffer[tail];
        g_imu_rx_tail = imu_next_index(tail);
        has_data = true;
    }
    taskEXIT_CRITICAL();

    return has_data;
}

/**
 * @brief  清空 IMU 运行状态
 *
 * @return void
 */
static void imu_clear_runtime_state(void)
{
    taskENTER_CRITICAL();
    g_imu_rx_head = 0U;
    g_imu_rx_tail = 0U;
    g_imu_rx_overflow_count = 0U;
    g_imu_uart_error_count = 0U;
    g_imu_checksum_error_count = 0U;
    g_imu_frame_error_count = 0U;
    g_imu_angle_seen = false;
    g_imu_gyro_seen = false;
    (void)memset(&g_imu_snapshot, 0, sizeof(g_imu_snapshot));
    taskEXIT_CRITICAL();

    (void)memset((void *)g_imu_rx_buffer, 0, sizeof(g_imu_rx_buffer));
    (void)memset(g_imu_frame, 0, sizeof(g_imu_frame));
    g_imu_frame_index = 0U;
}

/**
 * @brief  同步调试计数到快照
 *
 * @return void
 */
static void imu_sync_counters_unlocked(void)
{
    g_imu_snapshot.rx_overflow_count = g_imu_rx_overflow_count;
    g_imu_snapshot.uart_error_count = g_imu_uart_error_count;
    g_imu_snapshot.checksum_error_count = g_imu_checksum_error_count;
    g_imu_snapshot.frame_error_count = g_imu_frame_error_count;
}

/**
 * @brief  初始化 IMU UART0 接收
 *
 * @return fsp_err_t FSP_SUCCESS 表示 UART0 已打开
 */
static fsp_err_t imu_init_impl(void)
{
    fsp_err_t err = FSP_SUCCESS;

    if (g_imu_uart_opened)
    {
        return FSP_SUCCESS;
    }

    imu_clear_runtime_state();

    err = g_uart0.p_api->open(g_uart0.p_ctrl, g_uart0.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        taskENTER_CRITICAL();
        g_imu_snapshot.last_error = err;
        taskEXIT_CRITICAL();
        return err;
    }

    g_imu_uart_opened = true;
    return FSP_SUCCESS;
}

/**
 * @brief  处理 UART0 接收缓存中的 IMU 连续字节流
 *
 * @param elapsed_ms 距离上次调用经过的时间，单位 ms
 * @return fsp_err_t FSP_SUCCESS 表示处理完成
 */
static fsp_err_t imu_process_impl(uint32_t elapsed_ms)
{
    uint8_t data = 0U;

    if (!g_imu_uart_opened)
    {
        taskENTER_CRITICAL();
        g_imu_snapshot.last_error = FSP_ERR_NOT_OPEN;
        taskEXIT_CRITICAL();
        return FSP_ERR_NOT_OPEN;
    }

    imu_update_age(elapsed_ms);

    while (imu_pop_byte(&data))
    {
        imu_consume_byte(data);
    }

    taskENTER_CRITICAL();
    imu_sync_counters_unlocked();
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;
}

/**
 * @brief  读取最近一次 IMU 快照
 *
 * @param p_snapshot 快照输出指针
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t imu_read_impl(imu_snapshot_t *p_snapshot)
{
    if (NULL == p_snapshot)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    taskENTER_CRITICAL();
    imu_sync_counters_unlocked();
    *p_snapshot = g_imu_snapshot;
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;
}

/**
 * @brief  UART0 驱动回调，只做接收入缓存和错误计数
 *
 * @param p_args UART 回调参数
 * @return void
 */
void uart_0_callback(uart_callback_args_t *p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            imu_push_byte_from_isr((uint8_t)(p_args->data & 0xFFU));
            break;

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        case UART_EVENT_BREAK_DETECT:
            g_imu_uart_error_count++;
            break;

        case UART_EVENT_TX_DATA_EMPTY:
        case UART_EVENT_TX_COMPLETE:
        case UART_EVENT_RX_COMPLETE:
        default:
            break;
    }
}

const imu_api_t imu =
{
    .init = imu_init_impl,
    .process = imu_process_impl,
    .read = imu_read_impl,
};
