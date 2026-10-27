#include "bsp_config.h"
#include "heater_ctrl.h"
#include "stc8g1k08a.h"

static unsigned char g_run_enabled = 0U;
static unsigned char g_heat_on = 0U;
static unsigned char g_window_slot = 0U;

static void HEATERCTRL_SetOutput(unsigned char on)
{
    P54 = (on != 0U) ? 1 : 0;
    g_heat_on = (on != 0U) ? 1U : 0U;
}

void HEATERCTRL_Init(void)
{
    P5M1 &= (unsigned char)(~0x10U);
    P5M0 |= 0x10U;

    g_run_enabled = 0U;
    g_heat_on = 0U;
    g_window_slot = 0U;
    HEATERCTRL_SetOutput(0U);
}

void HEATERCTRL_SetRunEnabled(unsigned char enabled)
{
    if (enabled == 0U)
    {
        g_run_enabled = 0U;
        g_window_slot = 0U;
        HEATERCTRL_SetOutput(0U);
        return;
    }

    if (g_run_enabled == 0U)
    {
        g_window_slot = 0U;
    }
    g_run_enabled = 1U;
}

void HEATERCTRL_Update100ms(unsigned char power_percent)
{
    unsigned char active_slots;

    /* 1 秒窗口里的时间比例控制。 */
    if ((g_run_enabled == 0U) || (power_percent == 0U))
    {
        g_window_slot = 0U;
        HEATERCTRL_SetOutput(0U);
        return;
    }

    if (power_percent >= 100U)
    {
        active_slots = BSP_CONTROL_WINDOW_SLOTS;
    }
    else
    {
        active_slots = (unsigned char)((power_percent + 9U) / 10U);
    }

    if (active_slots > BSP_CONTROL_WINDOW_SLOTS)
    {
        active_slots = BSP_CONTROL_WINDOW_SLOTS;
    }

    HEATERCTRL_SetOutput((unsigned char)(g_window_slot < active_slots));
    g_window_slot++;
    if (g_window_slot >= BSP_CONTROL_WINDOW_SLOTS)
    {
        g_window_slot = 0U;
    }
}

unsigned char HEATERCTRL_IsHeating(void)
{
    return g_heat_on;
}
