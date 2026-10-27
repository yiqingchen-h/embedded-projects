#include "hardware/mechanical_arm/Mechanical_arm_feedback.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdbool.h>
#include <string.h>

/* 机械臂未初始化时的重试周期，100ms tick 计数，10 次约 1s。 */
#define MECHANICAL_ARM_FEEDBACK_INIT_RETRY_TICKS (10U)

/* 单路舵机温度和电压读取分频，10 次位置轮询读取一次温压。 */
#define MECHANICAL_ARM_FEEDBACK_TELEMETRY_DIVIDER (10U)

/* 机械臂反馈缓存快照，由反馈任务写入，UI 任务读取。 */
static mechanical_arm_feedback_snapshot_t g_mechanical_arm_feedback_snapshot = {0};

/* 下一次轮询的舵机序号。 */
static uint8_t g_mechanical_arm_feedback_next_index = 0U;

/* 机械臂初始化重试 tick 计数。 */
static uint32_t g_mechanical_arm_feedback_retry_ticks = 0U;

/* 单路舵机温度和电压读取分频计数。 */
static uint8_t g_mechanical_arm_feedback_telemetry_ticks[MECH_HAND_SERVO_COUNT] = {0U};

/* true 表示下一轮优先读取指定舵机。 */
static bool g_mechanical_arm_feedback_priority_valid = false;

/* 优先读取的舵机序号。 */
static uint8_t g_mechanical_arm_feedback_priority_index = 0U;

/* 机械臂 UART 总线互斥锁，避免后台读反馈和 UI 动作同时访问 UART4。 */
static SemaphoreHandle_t g_mechanical_arm_feedback_bus_mutex = NULL;

/**
 * @brief  进入机械臂反馈缓存临界区
 *
 * @return void
 */
static void mechanical_arm_feedback_enter_critical(void)
{
    taskENTER_CRITICAL();
}

/**
 * @brief  退出机械臂反馈缓存临界区
 *
 * @return void
 */
static void mechanical_arm_feedback_exit_critical(void)
{
    taskEXIT_CRITICAL();
}

/**
 * @brief  获取机械臂 UART 总线互斥锁
 *
 * @param timeout_ms 等待超时时间，单位 ms
 * @return bool true 表示获取成功
 */
static bool mechanical_arm_feedback_lock_bus_impl(uint32_t timeout_ms)
{
    TickType_t timeout_ticks = 0;

    if (NULL == g_mechanical_arm_feedback_bus_mutex)
    {
        return false;
    }

    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    return (pdTRUE == xSemaphoreTake(g_mechanical_arm_feedback_bus_mutex, timeout_ticks));
}

/**
 * @brief  释放机械臂 UART 总线互斥锁
 *
 * @return void
 */
static void mechanical_arm_feedback_unlock_bus_impl(void)
{
    if (NULL != g_mechanical_arm_feedback_bus_mutex)
    {
        (void)xSemaphoreGive(g_mechanical_arm_feedback_bus_mutex);
    }
}

/**
 * @brief  标记指定舵机反馈读取失败
 *
 * @param index      舵机序号
 * @param last_error 最近一次错误码
 * @return void
 */
static void mechanical_arm_feedback_mark_invalid(uint8_t index, int last_error)
{
    uint32_t sequence = 0U;

    if (index >= MECH_HAND_SERVO_COUNT)
    {
        return;
    }

    mechanical_arm_feedback_enter_critical();
    g_mechanical_arm_feedback_snapshot.sequence++;
    sequence = g_mechanical_arm_feedback_snapshot.sequence;
    g_mechanical_arm_feedback_snapshot.polling_index = (uint8_t)((index + 1U) % MECH_HAND_SERVO_COUNT);
    g_mechanical_arm_feedback_snapshot.servo[index].valid = false;
    g_mechanical_arm_feedback_snapshot.servo[index].last_error = last_error;
    g_mechanical_arm_feedback_snapshot.servo[index].sequence = sequence;
    g_mechanical_arm_feedback_snapshot.last_error = last_error;
    mechanical_arm_feedback_exit_critical();
}

/**
 * @brief  尝试初始化机械臂通信并更新反馈模块状态
 *
 * @return fsp_err_t FSP_SUCCESS 表示初始化成功
 */
static fsp_err_t mechanical_arm_feedback_try_init_unlocked(void)
{
    int last_error = MECH_HAND_OK;
    bool initialized = false;

    initialized = (0U != mechanical_hand.init());
    last_error = initialized ? MECH_HAND_OK : mechanical_hand.get_last_error();

    mechanical_arm_feedback_enter_critical();
    g_mechanical_arm_feedback_snapshot.initialized = initialized;
    g_mechanical_arm_feedback_snapshot.last_error = last_error;
    mechanical_arm_feedback_exit_critical();

    return initialized ? FSP_SUCCESS : FSP_ERR_NOT_OPEN;
}

/**
 * @brief  初始化机械臂反馈缓存
 *
 * @return fsp_err_t FSP_SUCCESS 表示机械臂通信已初始化
 */
static fsp_err_t mechanical_arm_feedback_init_impl(void)
{
    uint32_t i = 0U;
    fsp_err_t err = FSP_SUCCESS;

    if (NULL == g_mechanical_arm_feedback_bus_mutex)
    {
        g_mechanical_arm_feedback_bus_mutex = xSemaphoreCreateMutex();
        if (NULL == g_mechanical_arm_feedback_bus_mutex)
        {
            return FSP_ERR_INVALID_POINTER;
        }
    }

    mechanical_arm_feedback_enter_critical();
    (void)memset(&g_mechanical_arm_feedback_snapshot, 0, sizeof(g_mechanical_arm_feedback_snapshot));
    for (i = 0U; i < MECH_HAND_SERVO_COUNT; i++)
    {
        g_mechanical_arm_feedback_snapshot.servo[i].last_error = MECH_HAND_ERR_NOT_READY;
        g_mechanical_arm_feedback_telemetry_ticks[i] = MECHANICAL_ARM_FEEDBACK_TELEMETRY_DIVIDER;
    }
    g_mechanical_arm_feedback_next_index = 0U;
    g_mechanical_arm_feedback_retry_ticks = 0U;
    g_mechanical_arm_feedback_priority_valid = false;
    g_mechanical_arm_feedback_priority_index = 0U;
    mechanical_arm_feedback_exit_critical();

    if (!mechanical_arm_feedback_lock_bus_impl(500U))
    {
        return FSP_ERR_NOT_OPEN;
    }
    err = mechanical_arm_feedback_try_init_unlocked();
    mechanical_arm_feedback_unlock_bus_impl();

    return err;
}

/**
 * @brief  按 100ms 周期轮询一路舵机反馈
 *
 * @return void
 */
static void mechanical_arm_feedback_process_100ms_tick_impl(void)
{
    bus_servo_telemetry_t telemetry = {0};
    uint16_t position_pwm = 0U;
    uint8_t index = 0U;
    int position_err = MECH_HAND_OK;
    int telemetry_err = MECH_HAND_OK;
    uint32_t sequence = 0U;
    bool initialized = false;
    bool read_telemetry = false;

    mechanical_arm_feedback_enter_critical();
    initialized = g_mechanical_arm_feedback_snapshot.initialized;
    mechanical_arm_feedback_exit_critical();

    if (!initialized)
    {
        g_mechanical_arm_feedback_retry_ticks++;
        if (g_mechanical_arm_feedback_retry_ticks >= MECHANICAL_ARM_FEEDBACK_INIT_RETRY_TICKS)
        {
            g_mechanical_arm_feedback_retry_ticks = 0U;
            if (mechanical_arm_feedback_lock_bus_impl(0U))
            {
                (void)mechanical_arm_feedback_try_init_unlocked();
                mechanical_arm_feedback_unlock_bus_impl();
            }
        }
        return;
    }

    if (!mechanical_arm_feedback_lock_bus_impl(0U))
    {
        return;
    }

    mechanical_arm_feedback_enter_critical();
    if (g_mechanical_arm_feedback_priority_valid)
    {
        index = g_mechanical_arm_feedback_priority_index;
        g_mechanical_arm_feedback_priority_valid = false;
    }
    else
    {
        index = g_mechanical_arm_feedback_next_index;
    }
    mechanical_arm_feedback_exit_critical();

    if (index >= MECH_HAND_SERVO_COUNT)
    {
        index = 0U;
    }

    position_err = mechanical_hand.read_servo_position(index, &position_pwm);
    if (MECH_HAND_OK != position_err)
    {
        mechanical_arm_feedback_mark_invalid(index, position_err);
        g_mechanical_arm_feedback_next_index = (uint8_t)((index + 1U) % MECH_HAND_SERVO_COUNT);
        mechanical_arm_feedback_unlock_bus_impl();
        return;
    }

    if (g_mechanical_arm_feedback_telemetry_ticks[index] >= MECHANICAL_ARM_FEEDBACK_TELEMETRY_DIVIDER)
    {
        read_telemetry = true;
    }

    if (read_telemetry)
    {
        telemetry_err = mechanical_hand.read_servo_telemetry(index, &telemetry);
        if (MECH_HAND_OK != telemetry_err)
        {
            mechanical_arm_feedback_mark_invalid(index, telemetry_err);
            g_mechanical_arm_feedback_next_index = (uint8_t)((index + 1U) % MECH_HAND_SERVO_COUNT);
            mechanical_arm_feedback_unlock_bus_impl();
            return;
        }
    }

    mechanical_arm_feedback_enter_critical();
    g_mechanical_arm_feedback_snapshot.sequence++;
    sequence = g_mechanical_arm_feedback_snapshot.sequence;
    g_mechanical_arm_feedback_snapshot.polling_index = (uint8_t)((index + 1U) % MECH_HAND_SERVO_COUNT);
    g_mechanical_arm_feedback_snapshot.last_error = MECH_HAND_OK;
    g_mechanical_arm_feedback_snapshot.servo[index].valid = true;
    g_mechanical_arm_feedback_snapshot.servo[index].position_pwm = position_pwm;
    if (read_telemetry)
    {
        g_mechanical_arm_feedback_snapshot.servo[index].temperature_c = telemetry.temperature_c;
        g_mechanical_arm_feedback_snapshot.servo[index].voltage_raw = telemetry.voltage_raw;
        g_mechanical_arm_feedback_telemetry_ticks[index] = 0U;
    }
    else
    {
        g_mechanical_arm_feedback_telemetry_ticks[index]++;
    }
    g_mechanical_arm_feedback_snapshot.servo[index].last_error = MECH_HAND_OK;
    g_mechanical_arm_feedback_snapshot.servo[index].sequence = sequence;
    mechanical_arm_feedback_exit_critical();

    g_mechanical_arm_feedback_next_index = (uint8_t)((index + 1U) % MECH_HAND_SERVO_COUNT);
    mechanical_arm_feedback_unlock_bus_impl();
}

/**
 * @brief  请求优先刷新指定舵机反馈
 *
 * @param index 舵机序号
 * @return void
 */
static void mechanical_arm_feedback_request_refresh_impl(uint8_t index)
{
    if (index >= MECH_HAND_SERVO_COUNT)
    {
        return;
    }

    mechanical_arm_feedback_enter_critical();
    g_mechanical_arm_feedback_priority_index = index;
    g_mechanical_arm_feedback_priority_valid = true;
    g_mechanical_arm_feedback_telemetry_ticks[index] = MECHANICAL_ARM_FEEDBACK_TELEMETRY_DIVIDER;
    mechanical_arm_feedback_exit_critical();
}

/**
 * @brief  请求优先刷新全部舵机反馈
 *
 * @return void
 */
static void mechanical_arm_feedback_request_refresh_all_impl(void)
{
    uint32_t i = 0U;

    mechanical_arm_feedback_enter_critical();
    for (i = 0U; i < MECH_HAND_SERVO_COUNT; i++)
    {
        g_mechanical_arm_feedback_telemetry_ticks[i] = MECHANICAL_ARM_FEEDBACK_TELEMETRY_DIVIDER;
    }
    g_mechanical_arm_feedback_next_index = 0U;
    g_mechanical_arm_feedback_priority_index = 0U;
    g_mechanical_arm_feedback_priority_valid = true;
    mechanical_arm_feedback_exit_critical();
}

/**
 * @brief  读取当前反馈快照
 *
 * @param p_snapshot 快照输出指针
 * @return void
 */
static void mechanical_arm_feedback_read_snapshot_impl(mechanical_arm_feedback_snapshot_t *p_snapshot)
{
    if (NULL == p_snapshot)
    {
        return;
    }

    mechanical_arm_feedback_enter_critical();
    *p_snapshot = g_mechanical_arm_feedback_snapshot;
    mechanical_arm_feedback_exit_critical();
}

/**
 * @brief  机械臂反馈任务对外对象
 */
const mechanical_arm_feedback_api_t mechanical_arm_feedback =
{
    .init                = mechanical_arm_feedback_init_impl,
    .process_100ms_tick  = mechanical_arm_feedback_process_100ms_tick_impl,
    .request_refresh     = mechanical_arm_feedback_request_refresh_impl,
    .request_refresh_all = mechanical_arm_feedback_request_refresh_all_impl,
    .read_snapshot       = mechanical_arm_feedback_read_snapshot_impl,
    .lock_bus            = mechanical_arm_feedback_lock_bus_impl,
    .unlock_bus          = mechanical_arm_feedback_unlock_bus_impl,
};
