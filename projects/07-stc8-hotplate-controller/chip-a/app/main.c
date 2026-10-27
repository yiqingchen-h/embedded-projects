#include "bsp_config.h"
#include "heater_ctrl.h"
#include "ntc_adc.h"
#include "pid_ctrl.h"
#include "sysclk.h"
#include "timebase.h"
#include "uart1.h"
#include "uart_protocol.h"

typedef struct
{
    unsigned int set_temp_c;
    unsigned int cur_temp_c;
    signed int cur_temp_c10;
    unsigned int adc_raw;
    unsigned char fan_on;
    unsigned char heat_on;
    unsigned char requested_run;
    unsigned char actual_run;
    unsigned char link_alive;
    unsigned char sensor_fault;
    unsigned char overtemp_fault;
    unsigned char power_percent;
    ntc_range_t range;
} app_state_t;

/* 现场标定点：前一列是传感器换算值，后一列是外部测温修正后的真实值 */
static const signed int code g_temp_cal_sensor_c10[] = {315, 911, 1341, 1849, 2704, 3000};
static const signed int code g_temp_cal_actual_c10[] = {240, 500, 1050, 1450, 2030, 2185};

static signed int App_InterpolateTemp(signed int x,
                                      signed int x0,
                                      signed int y0,
                                      signed int x1,
                                      signed int y1)
{
    long numerator;
    long denominator;
    long delta;

    if (x1 == x0)
    {
        return y0;
    }

    numerator = (long)(x - x0) * (long)(y1 - y0);
    denominator = (long)(x1 - x0);
    if (numerator >= 0L)
    {
        delta = (numerator + (denominator / 2L)) / denominator;
    }
    else
    {
        delta = (numerator - (denominator / 2L)) / denominator;
    }

    return (signed int)((long)y0 + delta);
}

static signed int App_ApplyTempCalibration(signed int sensor_temp_c10)
{
    unsigned char i;
    unsigned char last_index;

    last_index = (unsigned char)((sizeof(g_temp_cal_sensor_c10) / sizeof(g_temp_cal_sensor_c10[0])) - 1U);

    if (sensor_temp_c10 <= g_temp_cal_sensor_c10[0])
    {
        return App_InterpolateTemp(sensor_temp_c10,
                                   g_temp_cal_sensor_c10[0],
                                   g_temp_cal_actual_c10[0],
                                   g_temp_cal_sensor_c10[1],
                                   g_temp_cal_actual_c10[1]);
    }

    for (i = 0U; i < last_index; i++)
    {
        if (sensor_temp_c10 <= g_temp_cal_sensor_c10[i + 1U])
        {
            return App_InterpolateTemp(sensor_temp_c10,
                                       g_temp_cal_sensor_c10[i],
                                       g_temp_cal_actual_c10[i],
                                       g_temp_cal_sensor_c10[i + 1U],
                                       g_temp_cal_actual_c10[i + 1U]);
        }
    }

    return App_InterpolateTemp(sensor_temp_c10,
                               g_temp_cal_sensor_c10[last_index - 1U],
                               g_temp_cal_actual_c10[last_index - 1U],
                               g_temp_cal_sensor_c10[last_index],
                               g_temp_cal_actual_c10[last_index]);
}

static unsigned int App_ClampSetTemp(unsigned int set_temp_c)
{
    if (set_temp_c < BSP_SET_TEMP_MIN_C)
    {
        return BSP_SET_TEMP_MIN_C;
    }

    if (set_temp_c > BSP_SET_TEMP_MAX_C)
    {
        return BSP_SET_TEMP_MAX_C;
    }

    return set_temp_c;
}

static unsigned int App_TempToDisplayC(signed int temp_c10)
{
    if (temp_c10 <= 0)
    {
        return 0U;
    }

    return (unsigned int)((temp_c10 + 5) / 10);
}

static unsigned char App_UpdateFan(app_state_t *state)
{
    unsigned char prev_fan;

    prev_fan = state->fan_on;
    /* 加热时不主动开风扇，停热后按温度回差散热。 */
    if (state->actual_run != 0U)
    {
        state->fan_on = 0U;
    }
    else if (state->fan_on != 0U)
    {
        if (state->cur_temp_c10 <= (signed int)(BSP_FAN_OFF_TEMP_C * 10U))
        {
            state->fan_on = 0U;
        }
    }
    else if (state->cur_temp_c10 >= (signed int)(BSP_FAN_ON_TEMP_C * 10U))
    {
        state->fan_on = 1U;
    }

    return (unsigned char)(prev_fan != state->fan_on);
}

static unsigned char App_UpdateRunState(app_state_t *state)
{
    unsigned char next_run;
    unsigned char changed;

    next_run = (unsigned char)((state->requested_run != 0U) &&
                               (state->link_alive != 0U) &&
                               (state->sensor_fault == 0U) &&
                               (state->overtemp_fault == 0U));

    changed = (unsigned char)(next_run != state->actual_run);
    if (next_run != state->actual_run)
    {
        state->actual_run = next_run;
        HEATERCTRL_SetRunEnabled(next_run);
    }

    state->heat_on = HEATERCTRL_IsHeating();
    return changed;
}

void main(void)
{
    app_state_t state;
    pid_ctrl_t pid;
    unsigned int now_ms;
    unsigned int last_sample_ms;
    unsigned int last_pid_ms;
    unsigned int last_status_ms;
    unsigned int last_comm_ms;
    unsigned char status_dirty;

    SYSCLK_Init();
    UART1_Init(BSP_UART1_BAUD);
    TIMEBASE_Init();
    UART_PROTOCOL_Init();
    HEATERCTRL_Init();
    NTCADC_Init();
    PIDCTRL_Init(&pid, BSP_PID_KP_X100, BSP_PID_KI_X100, BSP_PID_KD_X100);

    now_ms = TIMEBASE_GetMs();

    state.set_temp_c = BSP_SET_TEMP_DEFAULT_C;
    state.cur_temp_c = 0U;
    state.cur_temp_c10 = 0;
    state.adc_raw = 0U;
    state.fan_on = 0U;
    state.heat_on = 0U;
    state.requested_run = 0U;
    state.actual_run = 0U;
    state.link_alive = 0U;
    state.sensor_fault = 0U;
    state.overtemp_fault = 0U;
    state.power_percent = 0U;
    state.range = NTCADC_GetRange();

    last_sample_ms = now_ms;
    last_pid_ms = now_ms;
    last_status_ms = (unsigned int)(now_ms - BSP_STATUS_PERIOD_MS);
    last_comm_ms = now_ms;
    status_dirty = 1U;

    while (1)
    {
        now_ms = TIMEBASE_GetMs();

        /* 串口收命令不阻塞主循环 */
        while (UART1_ByteReady())
        {
            UART_PROTOCOL_ProcessByte(UART1_ReadByte());
        }

        if (UART_PROTOCOL_TakeAck() != 0U)
        {
            state.link_alive = 1U;
            last_comm_ms = now_ms;
        }

        if (UART_PROTOCOL_CommandReady())
        {
            uart_command_t command;

            UART_PROTOCOL_GetCommand(&command);
            state.set_temp_c = App_ClampSetTemp(command.set_temp);
            state.requested_run = command.run_on;
            state.link_alive = 1U;
            last_comm_ms = now_ms;
            UART_PROTOCOL_SendOk();
            status_dirty = 1U;
        }

        if ((state.link_alive != 0U) &&
            ((unsigned int)(now_ms - last_comm_ms) >= BSP_COMM_TIMEOUT_MS))
        {
            state.link_alive = 0U;
            state.requested_run = 0U;
            status_dirty = 1U;
        }

        if ((unsigned int)(now_ms - last_sample_ms) >= BSP_ADC_SAMPLE_PERIOD_MS)
        {
            unsigned char prev_sensor_fault;
            unsigned char prev_overtemp_fault;
            ntc_range_t prev_range;
            signed int sensor_temp_c10;

            last_sample_ms = now_ms;
            prev_sensor_fault = state.sensor_fault;
            prev_overtemp_fault = state.overtemp_fault;
            prev_range = state.range;

            if (NTCADC_Task50ms() != 0U)
            {
                state.adc_raw = NTCADC_GetRaw();
                state.range = NTCADC_GetRange();
                state.sensor_fault = NTCADC_IsFault();

                if (state.sensor_fault == 0U)
                {
                    sensor_temp_c10 = NTCADC_GetTempC10();
                    /* 控温和显示都使用同一份修正后的温度 */
                    state.cur_temp_c10 = App_ApplyTempCalibration(sensor_temp_c10);
                    state.cur_temp_c = App_TempToDisplayC(state.cur_temp_c10);
                }
                else
                {
                    state.requested_run = 0U;
                    state.cur_temp_c10 = 0;
                    state.cur_temp_c = 0U;
                }

                if (state.cur_temp_c10 >= (signed int)(BSP_OVERTEMP_C * 10U))
                {
                    state.overtemp_fault = 1U;
                    state.requested_run = 0U;
                }
                else if ((state.overtemp_fault != 0U) &&
                         (state.cur_temp_c10 <= (signed int)(BSP_SET_TEMP_MAX_C * 10U)))
                {
                    state.overtemp_fault = 0U;
                }
            }

            if ((prev_sensor_fault != state.sensor_fault) ||
                (prev_overtemp_fault != state.overtemp_fault) ||
                (prev_range != state.range))
            {
                status_dirty = 1U;
            }
        }

        if (App_UpdateRunState(&state) != 0U)
        {
            status_dirty = 1U;
        }

        if ((unsigned int)(now_ms - last_pid_ms) >= BSP_PID_PERIOD_MS)
        {
            unsigned char prev_heat_on;

            last_pid_ms = now_ms;
            prev_heat_on = state.heat_on;

            if (state.actual_run != 0U)
            {
                /* PID 输出的是功率百分比，后面再换成 1s 窗口里的导通段数。 */
                state.power_percent = PIDCTRL_Compute(&pid,
                                                      (signed int)(state.set_temp_c * 10U),
                                                      state.cur_temp_c10);
            }
            else
            {
                state.power_percent = 0U;
                PIDCTRL_Reset(&pid);
            }

            HEATERCTRL_Update100ms(state.power_percent);
            state.heat_on = HEATERCTRL_IsHeating();
            if (prev_heat_on != state.heat_on)
            {
                status_dirty = 1U;
            }
        }

        if (App_UpdateFan(&state) != 0U)
        {
            status_dirty = 1U;
        }

        if (status_dirty != 0U)
        {
            UART_PROTOCOL_SendStatus(state.set_temp_c,
                                     state.cur_temp_c,
                                     state.fan_on,
                                     state.heat_on,
                                     state.actual_run);
            last_status_ms = now_ms;
            status_dirty = 0U;
        }
        else if ((unsigned int)(now_ms - last_status_ms) >= BSP_STATUS_PERIOD_MS)
        {
            UART_PROTOCOL_SendStatus(state.set_temp_c,
                                     state.cur_temp_c,
                                     state.fan_on,
                                     state.heat_on,
                                     state.actual_run);
            last_status_ms = now_ms;
        }
    }
}
