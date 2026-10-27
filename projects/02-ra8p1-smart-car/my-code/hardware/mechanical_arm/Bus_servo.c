#include "hardware/mechanical_arm/Bus_servo.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define BUS_SERVO_CMD_BUFFER_SIZE       (260U)

/* UART4 回调与同步等待共享的状态；回调只置位，业务函数负责等待和清理。 */
static volatile bool g_bus_servo_uart4_opened = false;
static volatile bool g_bus_servo_uart4_tx_complete = false;
static volatile bool g_bus_servo_rx_complete = false;
static volatile bool g_bus_servo_uart_error = false;
static volatile uint32_t g_bus_servo_rx_count = 0U;

static char g_bus_servo_rx_buffer[BUS_SERVO_RESPONSE_SIZE];
static char g_bus_servo_last_response[BUS_SERVO_RESPONSE_SIZE];
static char g_bus_servo_last_cmd[BUS_SERVO_LAST_CMD_SIZE];
static int g_bus_servo_last_error = BUS_SERVO_OK;

static int bus_servo_check_id(uint8_t id, bool allow_broadcast);
static int bus_servo_check_pwm(uint16_t pwm);
static int bus_servo_check_time(uint16_t time_ms);
static int bus_servo_check_mode(uint8_t mode);
static int bus_servo_check_startup_mode(uint8_t mode);
static void bus_servo_delay_ms(uint32_t ms);
static void bus_servo_clear_rx_state(void);
static void bus_servo_record_last_command(const char *cmd);
static void bus_servo_record_last_response(const char *response);
static int bus_servo_wait_tx_complete(uint32_t timeout_ms);
static int bus_servo_wait_response(char *response, uint32_t response_size, uint32_t timeout_ms);
static int bus_servo_uart_write(const char *cmd);
static int bus_servo_send_command(const char *cmd, char *response, uint32_t response_size, uint32_t timeout_ms);
static int bus_servo_send_expect_ok(uint8_t id, const char *cmd);
static bool bus_servo_response_has_ok(const char *response);
static bool bus_servo_is_digit(char ch);
static int bus_servo_parse_leading_id(const char *response, uint8_t *id);
static int bus_servo_parse_position(const char *response, uint8_t *id, uint16_t *pwm);

/**
 * @brief  打开 UART4，允许重复调用；重复初始化不会重新触发 Home 或舵机动作
 */
static int bus_servo_init(void)
{
    fsp_err_t err;

    if (g_bus_servo_uart4_opened)
    {
        g_bus_servo_last_error = BUS_SERVO_OK;
        return BUS_SERVO_OK;
    }

    err = g_uart4.p_api->open(g_uart4.p_ctrl, g_uart4.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_UART_OPEN;
        return g_bus_servo_last_error;
    }

    g_bus_servo_uart4_opened = true;
    g_bus_servo_uart4_tx_complete = false;
    bus_servo_clear_rx_state();
    g_bus_servo_last_error = BUS_SERVO_OK;

    return BUS_SERVO_OK;
}

/**
 * @brief  关闭 UART4，主要用于模块收尾或低功耗前释放串口
 */
static void bus_servo_deinit(void)
{
    fsp_err_t err;

    if (!g_bus_servo_uart4_opened)
    {
        g_bus_servo_last_error = BUS_SERVO_OK;
        return;
    }

    err = g_uart4.p_api->close(g_uart4.p_ctrl);
    if ((FSP_SUCCESS != err) && (FSP_ERR_NOT_OPEN != err))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_UART_CLOSE;
        return;
    }

    g_bus_servo_uart4_opened = false;
    g_bus_servo_uart4_tx_complete = false;
    bus_servo_clear_rx_state();
    g_bus_servo_last_error = BUS_SERVO_OK;
}

/**
 * @brief  查询 UART4 是否已由本模块打开
 */
static bool bus_servo_is_initialized(void)
{
    return g_bus_servo_uart4_opened;
}

/**
 * @brief  拼接单舵机移动命令，格式为 #000P1500T1000!
 */
static int bus_servo_move(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    char cmd[32];
    int len;

    if ((BUS_SERVO_OK != bus_servo_check_id(id, true)) ||
        (BUS_SERVO_OK != bus_servo_check_pwm(pwm)) ||
        (BUS_SERVO_OK != bus_servo_check_time(time_ms)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uP%04uT%04u!", (unsigned int)id, (unsigned int)pwm, (unsigned int)time_ms);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_command(cmd, NULL, 0U, 0U);
    return g_bus_servo_last_error;
}

/**
 * @brief  拼接多舵机组合命令，组合发送不允许使用广播 ID
 */
static int bus_servo_move_multi(const bus_servo_move_t *moves, uint8_t count)
{
    char cmd[BUS_SERVO_CMD_BUFFER_SIZE];
    uint32_t offset = 0U;
    int len;

    if ((NULL == moves) || (0U == count) || (count > BUS_SERVO_MULTI_MAX_COUNT))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    cmd[offset] = '{';
    offset++;
    cmd[offset] = '\0';

    for (uint8_t i = 0U; i < count; i++)
    {
        if ((BUS_SERVO_OK != bus_servo_check_id(moves[i].id, false)) ||
            (BUS_SERVO_OK != bus_servo_check_pwm(moves[i].pwm)) ||
            (BUS_SERVO_OK != bus_servo_check_time(moves[i].time_ms)))
        {
            g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
            return g_bus_servo_last_error;
        }

        len = snprintf(&cmd[offset],
                       sizeof(cmd) - offset,
                       "#%03uP%04uT%04u!",
                       (unsigned int)moves[i].id,
                       (unsigned int)moves[i].pwm,
                       (unsigned int)moves[i].time_ms);
        if ((len <= 0) || ((uint32_t)len >= (sizeof(cmd) - offset)))
        {
            g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
            return g_bus_servo_last_error;
        }

        offset += (uint32_t)len;
    }

    if ((offset + 2U) > sizeof(cmd))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    cmd[offset] = '}';
    offset++;
    cmd[offset] = '\0';

    g_bus_servo_last_error = bus_servo_send_command(cmd, NULL, 0U, 0U);
    return g_bus_servo_last_error;
}

/**
 * @brief  原始调试命令入口；调用者负责确认命令不会修改波特率或造成危险动作
 */
static int bus_servo_send_raw(const char *cmd, char *response, uint32_t response_size, uint32_t timeout_ms)
{
    if ((NULL == cmd) || ('\0' == cmd[0]))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_command(cmd, response, response_size, timeout_ms);
    return g_bus_servo_last_error;
}

static int bus_servo_read_version(uint8_t id, char *version, uint32_t version_size)
{
    char cmd[16];
    char response[BUS_SERVO_RESPONSE_SIZE];
    const char *p_start;
    const char *p_end;
    uint8_t response_id;
    uint32_t copy_len;
    int len;
    int status;

    if ((NULL == version) || (0U == version_size) || (BUS_SERVO_OK != bus_servo_check_id(id, true)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPVER!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    status = bus_servo_send_command(cmd, response, sizeof(response), BUS_SERVO_RX_TIMEOUT_MS);
    if (BUS_SERVO_OK != status)
    {
        g_bus_servo_last_error = status;
        return status;
    }

    status = bus_servo_parse_leading_id(response, &response_id);
    if ((BUS_SERVO_OK != status) ||
        ((BUS_SERVO_ID_BROADCAST != id) && (response_id != id)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    p_start = strchr(response, 'P');
    p_end = strrchr(response, '!');
    if ((NULL == p_start) || (NULL == p_end) || (p_end <= p_start))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    p_start++;
    copy_len = (uint32_t)(p_end - p_start);
    if (copy_len >= version_size)
    {
        copy_len = version_size - 1U;
    }

    memcpy(version, p_start, copy_len);
    version[copy_len] = '\0';
    g_bus_servo_last_error = BUS_SERVO_OK;
    return BUS_SERVO_OK;
}

static int bus_servo_read_id(uint8_t id, uint8_t *read_id)
{
    char cmd[16];
    char response[BUS_SERVO_RESPONSE_SIZE];
    int len;
    int status;

    if ((NULL == read_id) || (BUS_SERVO_OK != bus_servo_check_id(id, true)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPID!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    status = bus_servo_send_command(cmd, response, sizeof(response), BUS_SERVO_RX_TIMEOUT_MS);
    if (BUS_SERVO_OK != status)
    {
        g_bus_servo_last_error = status;
        return status;
    }

    status = bus_servo_parse_leading_id(response, read_id);
    if (BUS_SERVO_OK != status)
    {
        g_bus_servo_last_error = status;
        return status;
    }

    if ((BUS_SERVO_ID_BROADCAST != id) && (*read_id != id))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = BUS_SERVO_OK;
    return BUS_SERVO_OK;
}

static int bus_servo_set_id(uint8_t id, uint8_t new_id)
{
    char cmd[20];
    char response[BUS_SERVO_RESPONSE_SIZE];
    uint8_t read_id;
    int len;
    int status;

    if ((BUS_SERVO_OK != bus_servo_check_id(id, true)) ||
        (BUS_SERVO_OK != bus_servo_check_id(new_id, false)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPID%03u!", (unsigned int)id, (unsigned int)new_id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    status = bus_servo_send_command(cmd, response, sizeof(response), BUS_SERVO_RX_TIMEOUT_MS);
    if (BUS_SERVO_OK != status)
    {
        g_bus_servo_last_error = status;
        return status;
    }

    status = bus_servo_parse_leading_id(response, &read_id);
    if ((BUS_SERVO_OK != status) || (read_id != new_id))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = BUS_SERVO_OK;
    return BUS_SERVO_OK;
}

static int bus_servo_release_torque(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPULK!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_restore_torque(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPULR!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_read_mode(uint8_t id, uint8_t *mode)
{
    char cmd[16];
    char response[BUS_SERVO_RESPONSE_SIZE];
    unsigned int temp_id;
    unsigned int temp_mode;
    int len;
    int status;

    if ((NULL == mode) || (BUS_SERVO_OK != bus_servo_check_id(id, true)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPMOD!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    status = bus_servo_send_command(cmd, response, sizeof(response), BUS_SERVO_RX_TIMEOUT_MS);
    if (BUS_SERVO_OK != status)
    {
        g_bus_servo_last_error = status;
        return status;
    }

    if (2 != sscanf(response, "#%3uPMOD%1u!", &temp_id, &temp_mode))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    if ((temp_id > BUS_SERVO_ID_MAX) ||
        ((BUS_SERVO_ID_BROADCAST != id) && (temp_id != id)) ||
        (BUS_SERVO_OK != bus_servo_check_mode((uint8_t)temp_mode)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    *mode = (uint8_t)temp_mode;
    g_bus_servo_last_error = BUS_SERVO_OK;
    return BUS_SERVO_OK;
}

static int bus_servo_set_mode(uint8_t id, uint8_t mode)
{
    char cmd[20];
    int len;

    if ((BUS_SERVO_OK != bus_servo_check_id(id, true)) ||
        (BUS_SERVO_OK != bus_servo_check_mode(mode)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPMOD%u!", (unsigned int)id, (unsigned int)mode);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_read_position(uint8_t id, uint16_t *pwm)
{
    char cmd[16];
    char response[BUS_SERVO_RESPONSE_SIZE];
    uint8_t response_id;
    int len;
    int status;

    if ((NULL == pwm) || (BUS_SERVO_OK != bus_servo_check_id(id, true)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPRAD!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    status = bus_servo_send_command(cmd, response, sizeof(response), BUS_SERVO_RX_TIMEOUT_MS);
    if (BUS_SERVO_OK != status)
    {
        g_bus_servo_last_error = status;
        return status;
    }

    status = bus_servo_parse_position(response, &response_id, pwm);
    if (BUS_SERVO_OK != status)
    {
        g_bus_servo_last_error = status;
        return status;
    }

    if ((BUS_SERVO_ID_BROADCAST != id) && (response_id != id))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = BUS_SERVO_OK;
    return BUS_SERVO_OK;
}

static int bus_servo_pause(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPDPT!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_continue(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPDCT!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_stop(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPDST!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_stop_all(void)
{
    return bus_servo_stop(BUS_SERVO_ID_BROADCAST);
}

static int bus_servo_correct_middle(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPSCK!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_set_current_as_startup(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPCSD!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_set_startup_mode(uint8_t id, uint8_t mode)
{
    char cmd[20];
    int len;

    if ((BUS_SERVO_OK != bus_servo_check_id(id, true)) ||
        (BUS_SERVO_OK != bus_servo_check_startup_mode(mode)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPCSM%u!", (unsigned int)id, (unsigned int)mode);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_read_startup_mode(uint8_t id, uint8_t *mode)
{
    char cmd[16];
    char response[BUS_SERVO_RESPONSE_SIZE];
    unsigned int temp_id;
    unsigned int temp_mode;
    int len;
    int status;

    if ((NULL == mode) || (BUS_SERVO_OK != bus_servo_check_id(id, true)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPCSM!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    status = bus_servo_send_command(cmd, response, sizeof(response), BUS_SERVO_RX_TIMEOUT_MS);
    if (BUS_SERVO_OK != status)
    {
        g_bus_servo_last_error = status;
        return status;
    }

    if ((2 != sscanf(response, "#%3uPCSM%1u!", &temp_id, &temp_mode)) &&
        (2 != sscanf(response, "#%3uCSM%1u!", &temp_id, &temp_mode)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    if ((temp_id > BUS_SERVO_ID_MAX) ||
        ((BUS_SERVO_ID_BROADCAST != id) && (temp_id != id)) ||
        (BUS_SERVO_OK != bus_servo_check_startup_mode((uint8_t)temp_mode)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    *mode = (uint8_t)temp_mode;
    g_bus_servo_last_error = BUS_SERVO_OK;
    return BUS_SERVO_OK;
}

static int bus_servo_set_current_as_min(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPSMI!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_set_current_as_max(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPSMX!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_factory_reset_keep_id(uint8_t id)
{
    char cmd[20];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPCLE0!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_factory_reset_all(uint8_t id)
{
    char cmd[16];
    int len;

    if (BUS_SERVO_OK != bus_servo_check_id(id, true))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPCLE!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    g_bus_servo_last_error = bus_servo_send_expect_ok(id, cmd);
    return g_bus_servo_last_error;
}

static int bus_servo_read_temperature_voltage(uint8_t id, bus_servo_telemetry_t *telemetry)
{
    char cmd[16];
    char response[BUS_SERVO_RESPONSE_SIZE];
    unsigned int temp_id;
    int temp_temperature;
    unsigned int temp_voltage;
    int len;
    int status;

    if ((NULL == telemetry) || (BUS_SERVO_OK != bus_servo_check_id(id, true)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_PARAM;
        return g_bus_servo_last_error;
    }

    len = snprintf(cmd, sizeof(cmd), "#%03uPRTV!", (unsigned int)id);
    if ((len <= 0) || ((uint32_t)len >= sizeof(cmd)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_FORMAT;
        return g_bus_servo_last_error;
    }

    status = bus_servo_send_command(cmd, response, sizeof(response), BUS_SERVO_RX_TIMEOUT_MS);
    if (BUS_SERVO_OK != status)
    {
        g_bus_servo_last_error = status;
        return status;
    }

    if (3 != sscanf(response, "#%3uT%dV%2u!", &temp_id, &temp_temperature, &temp_voltage))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    if ((temp_id > BUS_SERVO_ID_MAX) ||
        ((BUS_SERVO_ID_BROADCAST != id) && (temp_id != id)))
    {
        g_bus_servo_last_error = BUS_SERVO_ERR_RESPONSE;
        return g_bus_servo_last_error;
    }

    telemetry->id = (uint8_t)temp_id;
    telemetry->temperature_c = (int16_t)temp_temperature;
    telemetry->voltage_raw = (uint8_t)temp_voltage;
    g_bus_servo_last_error = BUS_SERVO_OK;
    return BUS_SERVO_OK;
}

static const char *bus_servo_get_last_command(void)
{
    return g_bus_servo_last_cmd;
}

static const char *bus_servo_get_last_response(void)
{
    return g_bus_servo_last_response;
}

static int bus_servo_get_last_error(void)
{
    return g_bus_servo_last_error;
}

static int bus_servo_check_id(uint8_t id, bool allow_broadcast)
{
    if (id <= BUS_SERVO_ID_MAX)
    {
        return BUS_SERVO_OK;
    }

    if ((allow_broadcast) && (BUS_SERVO_ID_BROADCAST == id))
    {
        return BUS_SERVO_OK;
    }

    return BUS_SERVO_ERR_PARAM;
}

static int bus_servo_check_pwm(uint16_t pwm)
{
    if ((pwm < BUS_SERVO_PWM_MIN) || (pwm > BUS_SERVO_PWM_MAX))
    {
        return BUS_SERVO_ERR_PARAM;
    }

    return BUS_SERVO_OK;
}

static int bus_servo_check_time(uint16_t time_ms)
{
    if (time_ms > BUS_SERVO_TIME_MAX)
    {
        return BUS_SERVO_ERR_PARAM;
    }

    return BUS_SERVO_OK;
}

static int bus_servo_check_mode(uint8_t mode)
{
    if ((mode < BUS_SERVO_MODE_270_CW) || (mode > BUS_SERVO_MODE_MOTOR_TIME_CCW))
    {
        return BUS_SERVO_ERR_PARAM;
    }

    return BUS_SERVO_OK;
}

static int bus_servo_check_startup_mode(uint8_t mode)
{
    if ((mode < BUS_SERVO_STARTUP_MODE_TO_START_POS) || (mode > BUS_SERVO_STARTUP_MODE_FREE))
    {
        return BUS_SERVO_ERR_PARAM;
    }

    return BUS_SERVO_OK;
}

static void bus_servo_delay_ms(uint32_t ms)
{
    TickType_t delay_ticks;

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

static void bus_servo_clear_rx_state(void)
{
    g_bus_servo_rx_count = 0U;
    g_bus_servo_rx_complete = false;
    g_bus_servo_uart_error = false;
    g_bus_servo_rx_buffer[0] = '\0';
}

static void bus_servo_record_last_command(const char *cmd)
{
    uint32_t i;

    if (NULL == cmd)
    {
        g_bus_servo_last_cmd[0] = '\0';
        return;
    }

    for (i = 0U; (i < (BUS_SERVO_LAST_CMD_SIZE - 1U)) && ('\0' != cmd[i]); i++)
    {
        g_bus_servo_last_cmd[i] = cmd[i];
    }

    g_bus_servo_last_cmd[i] = '\0';
}

static void bus_servo_record_last_response(const char *response)
{
    uint32_t i;

    if (NULL == response)
    {
        g_bus_servo_last_response[0] = '\0';
        return;
    }

    for (i = 0U; (i < (BUS_SERVO_RESPONSE_SIZE - 1U)) && ('\0' != response[i]); i++)
    {
        g_bus_servo_last_response[i] = response[i];
    }

    g_bus_servo_last_response[i] = '\0';
}

static int bus_servo_wait_tx_complete(uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0U;

    while (!g_bus_servo_uart4_tx_complete)
    {
        if (g_bus_servo_uart_error)
        {
            return BUS_SERVO_ERR_UART_WRITE;
        }

        if (elapsed_ms >= timeout_ms)
        {
            return BUS_SERVO_ERR_TIMEOUT;
        }

        bus_servo_delay_ms(1U);
        elapsed_ms++;
    }

    if (g_bus_servo_uart_error)
    {
        return BUS_SERVO_ERR_UART_WRITE;
    }

    g_bus_servo_uart4_tx_complete = false;
    return BUS_SERVO_OK;
}

static int bus_servo_wait_response(char *response, uint32_t response_size, uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0U;
    uint32_t copy_len;

    if ((NULL == response) || (0U == response_size))
    {
        return BUS_SERVO_ERR_PARAM;
    }

    /* RX 回调逐字符写入缓冲区，遇到 '!' 或错误时置位 g_bus_servo_rx_complete。 */
    while (!g_bus_servo_rx_complete)
    {
        if (g_bus_servo_uart_error)
        {
            return BUS_SERVO_ERR_UART_READ;
        }

        if (elapsed_ms >= timeout_ms)
        {
            response[0] = '\0';
            bus_servo_record_last_response(response);
            return BUS_SERVO_ERR_TIMEOUT;
        }

        bus_servo_delay_ms(1U);
        elapsed_ms++;
    }

    if (g_bus_servo_uart_error)
    {
        return BUS_SERVO_ERR_UART_READ;
    }

    copy_len = g_bus_servo_rx_count;
    if (copy_len >= response_size)
    {
        copy_len = response_size - 1U;
    }

    memcpy(response, g_bus_servo_rx_buffer, copy_len);
    response[copy_len] = '\0';
    bus_servo_record_last_response(response);

    return BUS_SERVO_OK;
}

static int bus_servo_uart_write(const char *cmd)
{
    fsp_err_t err;
    uint32_t len;
    int status;

    if ((NULL == cmd) || ('\0' == cmd[0]))
    {
        return BUS_SERVO_ERR_PARAM;
    }

    if (!g_bus_servo_uart4_opened)
    {
        status = bus_servo_init();
        if (BUS_SERVO_OK != status)
        {
            return status;
        }
    }

    len = (uint32_t)strlen(cmd);
    bus_servo_record_last_command(cmd);
    g_bus_servo_uart4_tx_complete = false;
    g_bus_servo_uart_error = false;

    err = g_uart4.p_api->write(g_uart4.p_ctrl, (uint8_t const *)cmd, len);
    if (FSP_SUCCESS != err)
    {
        return BUS_SERVO_ERR_UART_WRITE;
    }

    return bus_servo_wait_tx_complete(BUS_SERVO_TX_TIMEOUT_MS);
}

static int bus_servo_send_command(const char *cmd, char *response, uint32_t response_size, uint32_t timeout_ms)
{
    int status;

    /* 每条命令发送前清空 RX 状态，避免上一帧残留影响本次回包解析。 */
    bus_servo_clear_rx_state();
    bus_servo_record_last_response(NULL);

    status = bus_servo_uart_write(cmd);
    if (BUS_SERVO_OK != status)
    {
        return status;
    }

    if (NULL == response)
    {
        return BUS_SERVO_OK;
    }

    if (0U == timeout_ms)
    {
        timeout_ms = BUS_SERVO_RX_TIMEOUT_MS;
    }

    return bus_servo_wait_response(response, response_size, timeout_ms);
}

static int bus_servo_send_expect_ok(uint8_t id, const char *cmd)
{
    char response[BUS_SERVO_RESPONSE_SIZE];
    int status;

    /* 广播写命令可能有多个舵机同时回包，本模块不等待广播响应。 */
    if (BUS_SERVO_ID_BROADCAST == id)
    {
        return bus_servo_send_command(cmd, NULL, 0U, 0U);
    }

    status = bus_servo_send_command(cmd, response, sizeof(response), BUS_SERVO_RX_TIMEOUT_MS);
    if (BUS_SERVO_OK != status)
    {
        return status;
    }

    if (!bus_servo_response_has_ok(response))
    {
        return BUS_SERVO_ERR_RESPONSE;
    }

    return BUS_SERVO_OK;
}

static bool bus_servo_response_has_ok(const char *response)
{
    if (NULL == response)
    {
        return false;
    }

    return (NULL != strstr(response, "#OK"));
}

static bool bus_servo_is_digit(char ch)
{
    return (('0' <= ch) && (ch <= '9'));
}

static int bus_servo_parse_leading_id(const char *response, uint8_t *id)
{
    uint32_t value;

    if ((NULL == response) || (NULL == id) ||
        ('#' != response[0]) ||
        (!bus_servo_is_digit(response[1])) ||
        (!bus_servo_is_digit(response[2])) ||
        (!bus_servo_is_digit(response[3])))
    {
        return BUS_SERVO_ERR_RESPONSE;
    }

    value = ((uint32_t)(response[1] - '0') * 100U) +
            ((uint32_t)(response[2] - '0') * 10U) +
            (uint32_t)(response[3] - '0');
    if (value > BUS_SERVO_ID_MAX)
    {
        return BUS_SERVO_ERR_RESPONSE;
    }

    *id = (uint8_t)value;
    return BUS_SERVO_OK;
}

static int bus_servo_parse_position(const char *response, uint8_t *id, uint16_t *pwm)
{
    unsigned int temp_id;
    unsigned int temp_pwm;

    if ((NULL == response) || (NULL == id) || (NULL == pwm))
    {
        return BUS_SERVO_ERR_PARAM;
    }

    if (2 != sscanf(response, "#%3uP%4u!", &temp_id, &temp_pwm))
    {
        return BUS_SERVO_ERR_RESPONSE;
    }

    if ((temp_id > BUS_SERVO_ID_MAX) ||
        (temp_pwm < BUS_SERVO_PWM_MIN) ||
        (temp_pwm > BUS_SERVO_PWM_MAX))
    {
        return BUS_SERVO_ERR_RESPONSE;
    }

    *id = (uint8_t)temp_id;
    *pwm = (uint16_t)temp_pwm;
    return BUS_SERVO_OK;
}

/**
 * @brief  总线舵机对外对象，应用层通过 bus_servo.xxx() 调用
 */
const bus_servo_api_t bus_servo =
{
    .init                     = bus_servo_init,
    .deinit                   = bus_servo_deinit,
    .is_initialized           = bus_servo_is_initialized,
    .move                     = bus_servo_move,
    .move_multi               = bus_servo_move_multi,
    .send_raw                 = bus_servo_send_raw,
    .read_version             = bus_servo_read_version,
    .read_id                  = bus_servo_read_id,
    .set_id                   = bus_servo_set_id,
    .release_torque           = bus_servo_release_torque,
    .restore_torque           = bus_servo_restore_torque,
    .read_mode                = bus_servo_read_mode,
    .set_mode                 = bus_servo_set_mode,
    .read_position            = bus_servo_read_position,
    .pause                    = bus_servo_pause,
    .continue_motion          = bus_servo_continue,
    .stop                     = bus_servo_stop,
    .stop_all                 = bus_servo_stop_all,
    .correct_middle           = bus_servo_correct_middle,
    .set_current_as_startup   = bus_servo_set_current_as_startup,
    .set_startup_mode         = bus_servo_set_startup_mode,
    .read_startup_mode        = bus_servo_read_startup_mode,
    .set_current_as_min       = bus_servo_set_current_as_min,
    .set_current_as_max       = bus_servo_set_current_as_max,
    .factory_reset_keep_id    = bus_servo_factory_reset_keep_id,
    .factory_reset_all        = bus_servo_factory_reset_all,
    .read_temperature_voltage = bus_servo_read_temperature_voltage,
    .get_last_command         = bus_servo_get_last_command,
    .get_last_response        = bus_servo_get_last_response,
    .get_last_error           = bus_servo_get_last_error,
};

/**
 * @brief  UART4 驱动回调，只由 RA/FSP 中断回调链路调用，应用层不要直接调用
 *
 * @note   RX 按字符接收，遇到 '!' 认为一帧响应结束；错误事件会同时唤醒 TX/RX 等待。
 */
void uart_4_callback(uart_callback_args_t *p_args)
{
    char ch;
    uint32_t index;

    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
    case UART_EVENT_TX_COMPLETE:
        g_bus_servo_uart4_tx_complete = true;
        break;

    case UART_EVENT_TX_DATA_EMPTY:
        break;

    case UART_EVENT_RX_CHAR:
        ch = (char)(p_args->data & 0xFFU);
        index = g_bus_servo_rx_count;
        if (index < (BUS_SERVO_RESPONSE_SIZE - 1U))
        {
            g_bus_servo_rx_buffer[index] = ch;
            index++;
            g_bus_servo_rx_buffer[index] = '\0';
            g_bus_servo_rx_count = index;

            if ('!' == ch)
            {
                g_bus_servo_rx_complete = true;
            }
        }
        else
        {
            g_bus_servo_uart_error = true;
            g_bus_servo_rx_complete = true;
        }
        break;

    case UART_EVENT_ERR_PARITY:
    case UART_EVENT_ERR_FRAMING:
    case UART_EVENT_ERR_OVERFLOW:
    case UART_EVENT_BREAK_DETECT:
        g_bus_servo_uart_error = true;
        g_bus_servo_uart4_tx_complete = true;
        g_bus_servo_rx_complete = true;
        break;

    case UART_EVENT_RX_COMPLETE:
    default:
        break;
    }
}
