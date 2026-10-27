#include "uart1.h"
#include "uart_protocol.h"

/* U23 -> U22: C,set,run
 * U22 -> U23: S,set,cur,fan,heat,run
 */
static unsigned char g_rx_buf[32];
static unsigned char g_rx_len = 0U;
static unsigned char g_ack_ready = 0U;
static unsigned char g_cmd_ready = 0U;
static uart_command_t g_last_command;

static unsigned char UART_PROTOCOL_IsSpace(unsigned char ch)
{
    return (unsigned char)((ch == ' ') || (ch == '\t'));
}

static unsigned char UART_PROTOCOL_ParseUint(const unsigned char *str, unsigned int *value)
{
    unsigned int result;

    result = 0U;
    while (UART_PROTOCOL_IsSpace(*str))
    {
        str++;
    }

    if ((*str < '0') || (*str > '9'))
    {
        return 0U;
    }

    while ((*str >= '0') && (*str <= '9'))
    {
        result = (unsigned int)(result * 10U + (unsigned int)(*str - '0'));
        str++;
    }

    while (UART_PROTOCOL_IsSpace(*str))
    {
        str++;
    }

    *value = result;
    return (unsigned char)(*str == '\0');
}

static unsigned char UART_PROTOCOL_ParseOk(const unsigned char *frame)
{
    unsigned char i;

    i = 0U;
    while (UART_PROTOCOL_IsSpace(frame[i]))
    {
        i++;
    }

    if (((frame[i] != 'O') && (frame[i] != 'o')) ||
        ((frame[i + 1U] != 'K') && (frame[i + 1U] != 'k')))
    {
        return 0U;
    }

    i += 2U;
    while (UART_PROTOCOL_IsSpace(frame[i]))
    {
        i++;
    }

    return (unsigned char)(frame[i] == '\0');
}

static unsigned char UART_PROTOCOL_ParseCommand(const unsigned char *frame, uart_command_t *command)
{
    unsigned char i;
    unsigned char field_count;
    unsigned char local_buf[8];
    unsigned int values[2];
    unsigned int parsed_value;

    i = 0U;
    while (UART_PROTOCOL_IsSpace(frame[i]))
    {
        i++;
    }

    if ((frame[i] != 'C') && (frame[i] != 'c'))
    {
        return 0U;
    }

    i++;
    while (UART_PROTOCOL_IsSpace(frame[i]))
    {
        i++;
    }

    if (frame[i] != ',')
    {
        return 0U;
    }

    field_count = 0U;
    i++;
    while (field_count < 2U)
    {
        unsigned char j;

        j = 0U;
        while ((frame[i] != ',') && (frame[i] != '\0') && (j < (sizeof(local_buf) - 1U)))
        {
            local_buf[j++] = frame[i++];
        }
        local_buf[j] = '\0';

        if (UART_PROTOCOL_ParseUint(local_buf, &parsed_value) == 0U)
        {
            return 0U;
        }

        values[field_count++] = parsed_value;
        if (frame[i] == ',')
        {
            i++;
        }
        else
        {
            break;
        }
    }

    if (field_count != 2U)
    {
        return 0U;
    }

    command->set_temp = values[0];
    command->run_on = (unsigned char)(values[1] ? 1U : 0U);
    return 1U;
}

static void UART_PROTOCOL_SendUint(unsigned int value)
{
    unsigned char digits[5];
    unsigned char count;

    if (value == 0U)
    {
        UART1_SendByte('0');
        return;
    }

    count = 0U;
    while ((value > 0U) && (count < sizeof(digits)))
    {
        digits[count++] = (unsigned char)('0' + (value % 10U));
        value /= 10U;
    }

    while (count > 0U)
    {
        count--;
        UART1_SendByte(digits[count]);
    }
}

static void UART_PROTOCOL_FinalizeFrame(void)
{
    uart_command_t command;

    g_rx_buf[g_rx_len] = '\0';
    /* 一帧只认一种语义，先判 OK，再判控制命令。 */
    if (UART_PROTOCOL_ParseOk(g_rx_buf) != 0U)
    {
        g_ack_ready = 1U;
    }
    else if (UART_PROTOCOL_ParseCommand(g_rx_buf, &command) != 0U)
    {
        g_last_command = command;
        g_cmd_ready = 1U;
    }

    g_rx_len = 0U;
}

void UART_PROTOCOL_Init(void)
{
    g_rx_len = 0U;
    g_ack_ready = 0U;
    g_cmd_ready = 0U;
    g_last_command.set_temp = 0U;
    g_last_command.run_on = 0U;
}

void UART_PROTOCOL_ProcessByte(unsigned char rx)
{
    if ((rx == '\r') || (rx == '\n'))
    {
        if (g_rx_len > 0U)
        {
            UART_PROTOCOL_FinalizeFrame();
        }
        return;
    }

    if (g_rx_len < (sizeof(g_rx_buf) - 1U))
    {
        g_rx_buf[g_rx_len++] = rx;
    }
    else
    {
        g_rx_len = 0U;
    }
}

bit UART_PROTOCOL_CommandReady(void)
{
    return (bit)(g_cmd_ready != 0U);
}

void UART_PROTOCOL_GetCommand(uart_command_t *cmd)
{
    *cmd = g_last_command;
    g_cmd_ready = 0U;
}

unsigned char UART_PROTOCOL_TakeAck(void)
{
    unsigned char ack;

    ack = g_ack_ready;
    g_ack_ready = 0U;
    return ack;
}

void UART_PROTOCOL_SendOk(void)
{
    UART1_SendByte('O');
    UART1_SendByte('K');
    UART1_SendByte('\r');
    UART1_SendByte('\n');
}

void UART_PROTOCOL_SendStatus(unsigned int set_temp,
                              unsigned int cur_temp,
                              unsigned char fan_on,
                              unsigned char heat_on,
                              unsigned char run_on)
{
    UART1_SendByte('S');
    UART1_SendByte(',');
    UART_PROTOCOL_SendUint(set_temp);
    UART1_SendByte(',');
    UART_PROTOCOL_SendUint(cur_temp);
    UART1_SendByte(',');
    UART1_SendByte((unsigned char)(fan_on ? '1' : '0'));
    UART1_SendByte(',');
    UART1_SendByte((unsigned char)(heat_on ? '1' : '0'));
    UART1_SendByte(',');
    UART1_SendByte((unsigned char)(run_on ? '1' : '0'));
    UART1_SendByte('\r');
    UART1_SendByte('\n');
}
