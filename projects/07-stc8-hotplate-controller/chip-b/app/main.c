#include "bsp_config.h"
#include "delay.h"
#include "fan.h"
#include "key_adc.h"
#include "oled.h"
#include "sysclk.h"
#include "uart1.h"

#define APP_SET_TEMP_MIN            50U
#define APP_SET_TEMP_MAX            400U
#define APP_SET_TEMP_STEP           5U

typedef enum
{
    MENU_FOCUS_SET = 0,
    MENU_FOCUS_RUN = 1
} menu_focus_t;

typedef struct
{
    unsigned int set_temp;
    unsigned int cur_temp;
    unsigned int desired_set_temp;
    unsigned int adc_raw;
    key_id_t key;
    unsigned char fan_on;
    unsigned char heating_on;
    unsigned char run_enabled;
    unsigned char desired_run_on;
    unsigned char sync_pending;
    unsigned char link_ok;
    unsigned char status_received;
    unsigned char last_rx;
    menu_focus_t focus;
} ui_state_t;

static const unsigned char code g_set_label[] = "SET:";
static const unsigned char code g_cur_label[] = "CUR:";
static const unsigned char code g_deg_label[] = "C";
static const unsigned char code g_fan_label[] = "FAN:";
static const unsigned char code g_hot_label[] = "HOT:";
static const unsigned char code g_run_label[] = "RUN:";
static const unsigned char code g_sync_label[] = "SYNC:";
static const unsigned char code g_on_text[] = "ON ";
static const unsigned char code g_off_text[] = "OFF";
static const unsigned char code g_ok_text[] = "OK ";
static const unsigned char code g_wait_text[] = "WAIT";
static const unsigned char code g_key_none[] = "0";
static const unsigned char code g_key_1[] = "1";
static const unsigned char code g_key_2[] = "2";
static const unsigned char code g_key_3[] = "3";
static unsigned char g_rx_buf[32];
static unsigned char g_rx_len = 0U;
static unsigned char g_rx_idle_ticks = 0U;

static unsigned char App_IsSpace(unsigned char ch)
{
    return (unsigned char)((ch == ' ') || (ch == '\t'));
}

static const unsigned char code *App_KeyName(key_id_t key)
{
    if (key == KEY_ID_1)
    {
        return g_key_1;
    }

    if (key == KEY_ID_2)
    {
        return g_key_2;
    }

    if (key == KEY_ID_3)
    {
        return g_key_3;
    }

    return g_key_none;
}

static unsigned char App_ParseUint(const unsigned char *str, unsigned int *value)
{
    unsigned int result;

    result = 0U;
    while (App_IsSpace(*str))
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

    while (App_IsSpace(*str))
    {
        str++;
    }

    *value = result;
    return (unsigned char)(*str == '\0');
}

static unsigned char App_ParseOkFrame(const unsigned char *frame)
{
    unsigned char i;

    i = 0U;
    while (App_IsSpace(frame[i]))
    {
        i++;
    }

    if (((frame[i] != 'O') && (frame[i] != 'o')) ||
        ((frame[i + 1U] != 'K') && (frame[i + 1U] != 'k')))
    {
        return 0U;
    }

    i += 2U;
    while (App_IsSpace(frame[i]))
    {
        i++;
    }

    return (unsigned char)(frame[i] == '\0');
}

static unsigned char App_ParseStatusFrame(const unsigned char *frame, ui_state_t *state)
{
    unsigned char i;
    unsigned char field_count;
    unsigned char local_buf[8];
    unsigned int parsed_value;
    unsigned int field_values[5];

    i = 0U;
    while (App_IsSpace(frame[i]))
    {
        i++;
    }

    if ((frame[i] != 'S') && (frame[i] != 's'))
    {
        return 0U;
    }

    i++;
    while (App_IsSpace(frame[i]))
    {
        i++;
    }

    if (frame[i] != ',')
    {
        return 0U;
    }

    field_count = 0U;
    i++;
    while (field_count < 5U)
    {
        unsigned char j;

        j = 0U;
        while ((frame[i] != ',') && (frame[i] != '\0') && (j < (sizeof(local_buf) - 1U)))
        {
            local_buf[j++] = frame[i++];
        }
        local_buf[j] = '\0';

        if (!App_ParseUint(local_buf, &parsed_value))
        {
            return 0U;
        }

        field_values[field_count++] = parsed_value;
        if (frame[i] == ',')
        {
            i++;
        }
        else
        {
            break;
        }
    }

    if (field_count < 4U)
    {
        return 0U;
    }

    state->set_temp = field_values[0];
    state->cur_temp = field_values[1];
    state->fan_on = (unsigned char)(field_values[2] ? 1U : 0U);
    state->heating_on = (unsigned char)(field_values[3] ? 1U : 0U);
    if (field_count >= 5U)
    {
        state->run_enabled = (unsigned char)(field_values[4] ? 1U : 0U);
    }
    else
    {
        state->run_enabled = state->heating_on;
    }

    return 1U;
}

static void App_SendCrlf(void)
{
    UART1_SendByte('\r');
    UART1_SendByte('\n');
}

static void App_SendOk(void)
{
    UART1_SendByte('O');
    UART1_SendByte('K');
    App_SendCrlf();
}

static void App_SendUint(unsigned int value)
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

static void App_SendCommand(const ui_state_t *state)
{
    UART1_SendByte('C');
    UART1_SendByte(',');
    App_SendUint(state->desired_set_temp);
    UART1_SendByte(',');
    UART1_SendByte((unsigned char)(state->desired_run_on ? '1' : '0'));
    App_SendCrlf();
}

static void App_RequestSync(ui_state_t *state)
{
    state->sync_pending = 1U;
    state->link_ok = 0U;
    /* 本地参数一变就先发一帧，避免界面和 U22 脱节。 */
    App_SendCommand(state);
}

static void UI_Init(void)
{
    OLED_Clear();
    OLED_ShowCodeString(6U, 0U, g_set_label);
    OLED_ShowCodeString(0U, 1U, g_cur_label);
    OLED_ShowCodeString(0U, 2U, g_fan_label);
    OLED_ShowCodeString(60U, 2U, g_hot_label);
    OLED_ShowCodeString(0U, 3U, g_run_label);
    OLED_ShowCodeString(60U, 3U, g_sync_label);
}

static void UI_UpdateDynamic(const ui_state_t *state)
{
    static unsigned int last_desired_set_temp = 0xFFFFU;
    static unsigned int last_cur_temp = 0xFFFFU;
    static unsigned char last_fan_on = 0xFFU;
    static unsigned char last_heating_on = 0xFFU;
    static unsigned char last_desired_run_on = 0xFFU;
    static unsigned char last_sync_pending = 0xFFU;
    static menu_focus_t last_focus = (menu_focus_t)0xFFU;

    if (state->focus != last_focus)
    {
        OLED_ShowChar(0U, 0U, (char)(state->focus == MENU_FOCUS_SET ? '>' : ' '));
        OLED_ShowChar(54U, 2U, (char)(state->focus == MENU_FOCUS_RUN ? '>' : ' '));
        last_focus = state->focus;
    }

    if (state->desired_set_temp != last_desired_set_temp)
    {
        OLED_ClearArea(30U, 0U, 30U);
        OLED_ShowNum(30U, 0U, state->desired_set_temp, 3U);
        OLED_ShowCodeString(48U, 0U, g_deg_label);
        last_desired_set_temp = state->desired_set_temp;
    }

    if (state->cur_temp != last_cur_temp)
    {
        OLED_ClearArea(24U, 1U, 30U);
        OLED_ShowNum(24U, 1U, state->cur_temp, 3U);
        OLED_ShowCodeString(42U, 1U, g_deg_label);
        last_cur_temp = state->cur_temp;
    }

    if (state->fan_on != last_fan_on)
    {
        OLED_ClearArea(24U, 2U, 24U);
        OLED_ShowCodeString(24U, 2U, state->fan_on ? g_on_text : g_off_text);
        last_fan_on = state->fan_on;
    }

    if (state->heating_on != last_heating_on)
    {
        OLED_ClearArea(84U, 2U, 24U);
        OLED_ShowCodeString(84U, 2U, state->heating_on ? g_on_text : g_off_text);
        last_heating_on = state->heating_on;
    }

    if (state->desired_run_on != last_desired_run_on)
    {
        OLED_ClearArea(24U, 3U, 24U);
        OLED_ShowCodeString(24U, 3U, state->desired_run_on ? g_on_text : g_off_text);
        last_desired_run_on = state->desired_run_on;
    }

    if (state->sync_pending != last_sync_pending)
    {
        OLED_ClearArea(90U, 3U, 24U);
        OLED_ShowCodeString(90U, 3U, state->sync_pending ? g_wait_text : g_ok_text);
        last_sync_pending = state->sync_pending;
    }
}

static void App_HandleStatusUpdate(ui_state_t *state)
{
    state->status_received = 1U;
    state->link_ok = 1U;

    if (state->sync_pending)
    {
        if ((state->desired_set_temp == state->set_temp) &&
            (state->desired_run_on == state->run_enabled))
        {
            state->sync_pending = 0U;
        }
    }
    else
    {
        state->desired_set_temp = state->set_temp;
        state->desired_run_on = state->run_enabled;
    }

    if (state->fan_on)
    {
        FAN_On();
    }
    else
    {
        FAN_Off();
    }
}

static void App_HandleIncomingFrame(unsigned char *rx_buf, unsigned char rx_len, ui_state_t *state)
{
#if BSP_UART1_RAW_ECHO_ENABLE
    (void)rx_buf;
    (void)rx_len;
    (void)state;
#else
    rx_buf[rx_len] = '\0';

    if (App_ParseOkFrame(rx_buf))
    {
        state->link_ok = 1U;
        return;
    }

    if (App_ParseStatusFrame(rx_buf, state))
    {
        App_HandleStatusUpdate(state);
        App_SendOk();
    }
#endif
}

static void App_HandleIncomingByte(unsigned char rx, ui_state_t *state)
{
    state->last_rx = rx;
    g_rx_idle_ticks = 0U;

#if BSP_UART1_RAW_ECHO_ENABLE
    UART1_SendByte(rx);
#endif

    if ((rx == '\r') || (rx == '\n'))
    {
        if (g_rx_len > 0U)
        {
            App_HandleIncomingFrame(g_rx_buf, g_rx_len, state);
            g_rx_len = 0U;
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

static void App_PollIncomingFrameTimeout(ui_state_t *state)
{
    if (g_rx_len == 0U)
    {
        return;
    }

    if (g_rx_idle_ticks < 3U)
    {
        g_rx_idle_ticks++;
        return;
    }

    App_HandleIncomingFrame(g_rx_buf, g_rx_len, state);
    g_rx_len = 0U;
    g_rx_idle_ticks = 0U;
}

static unsigned int App_ClampSetTemp(unsigned int value)
{
    if (value < APP_SET_TEMP_MIN)
    {
        return APP_SET_TEMP_MIN;
    }

    if (value > APP_SET_TEMP_MAX)
    {
        return APP_SET_TEMP_MAX;
    }

    return value;
}

static void App_HandleKeyAction(ui_state_t *state, key_id_t key)
{
    unsigned int new_set_temp;
    unsigned char new_run_on;

    if (key == KEY_ID_3)
    {
        state->focus = (menu_focus_t)(state->focus == MENU_FOCUS_SET ? MENU_FOCUS_RUN : MENU_FOCUS_SET);
        return;
    }

    if (state->focus == MENU_FOCUS_SET)
    {
        new_set_temp = state->desired_set_temp;
        if (key == KEY_ID_1)
        {
            if (new_set_temp > APP_SET_TEMP_MIN)
            {
                if (new_set_temp > APP_SET_TEMP_STEP)
                {
                    new_set_temp = (unsigned int)(new_set_temp - APP_SET_TEMP_STEP);
                }
                new_set_temp = App_ClampSetTemp(new_set_temp);
            }
        }
        else if (key == KEY_ID_2)
        {
            new_set_temp = App_ClampSetTemp((unsigned int)(new_set_temp + APP_SET_TEMP_STEP));
        }

        if (new_set_temp != state->desired_set_temp)
        {
            state->desired_set_temp = new_set_temp;
            App_RequestSync(state);
        }
        return;
    }

    new_run_on = state->desired_run_on;
    if (key == KEY_ID_1)
    {
        new_run_on = 0U;
    }
    else if (key == KEY_ID_2)
    {
        new_run_on = 1U;
    }

    if (new_run_on != state->desired_run_on)
    {
        state->desired_run_on = new_run_on;
        App_RequestSync(state);
    }
}

void main(void)
{
    ui_state_t ui_state;
    key_id_t sampled_key;
    key_id_t last_sampled_key;
    key_id_t stable_key;
    key_id_t last_reported_key;
    unsigned char stable_count;
    unsigned char sync_retry_ticks;
    unsigned char heartbeat_ticks;

    SYSCLK_Init();
    UART1_Init(BSP_UART1_BAUD);
    FAN_Init();
    KEY_ADC_Init();
    OLED_Init();

    ui_state.set_temp = 230U;
    ui_state.cur_temp = 0U;
    ui_state.desired_set_temp = 230U;
    ui_state.adc_raw = 0U;
    ui_state.key = KEY_ID_NONE;
    ui_state.fan_on = 0U;
    ui_state.heating_on = 0U;
    ui_state.run_enabled = 0U;
    ui_state.desired_run_on = 0U;
    ui_state.sync_pending = 0U;
    ui_state.link_ok = 0U;
    ui_state.status_received = 0U;
    ui_state.last_rx = 0U;
    ui_state.focus = MENU_FOCUS_SET;

    stable_key = KEY_ID_NONE;
    last_sampled_key = KEY_ID_NONE;
    last_reported_key = KEY_ID_NONE;
    stable_count = 0U;
    sync_retry_ticks = 0U;
    heartbeat_ticks = 0U;

    UI_Init();
    UI_UpdateDynamic(&ui_state);

    while (1)
    {
        ui_state.adc_raw = KEY_ADC_ReadRaw();
        sampled_key = KEY_ADC_Decode(ui_state.adc_raw);

        if (sampled_key == last_sampled_key)
        {
            if (stable_count < 1U)
            {
                stable_count++;
            }
        }
        else
        {
            last_sampled_key = sampled_key;
            stable_count = 0U;
        }

        if ((stable_count >= 1U) && (sampled_key != stable_key))
        {
            stable_key = sampled_key;
            ui_state.key = stable_key;

            if (stable_key == KEY_ID_NONE)
            {
                last_reported_key = KEY_ID_NONE;
            }
            else if (stable_key != last_reported_key)
            {
                last_reported_key = stable_key;
                App_HandleKeyAction(&ui_state, stable_key);
            }
        }

        while (UART1_ByteReady())
        {
            App_HandleIncomingByte(UART1_ReadByte(), &ui_state);
        }

        App_PollIncomingFrameTimeout(&ui_state);
        if (ui_state.sync_pending != 0U)
        {
            /* 等待确认时加快重发。 */
            if (sync_retry_ticks < 20U)
            {
                sync_retry_ticks++;
            }
            else
            {
                sync_retry_ticks = 0U;
                App_SendCommand(&ui_state);
            }
        }
        else
        {
            sync_retry_ticks = 0U;
            /* 已同步后继续发送数据，避免 U22 因超时自动停止加热。 */
            if (heartbeat_ticks < 50U)
            {
                heartbeat_ticks++;
            }
            else
            {
                heartbeat_ticks = 0U;
                App_SendCommand(&ui_state);
            }
        }
        UI_UpdateDynamic(&ui_state);

        DelayMs(10U);
    }
}
