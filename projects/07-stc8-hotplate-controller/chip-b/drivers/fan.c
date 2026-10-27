#include "stc8g1k08a.h"
#include "fan.h"

static bit g_fan_on = 0;

void FAN_Init(void)
{
    P5M1 &= (unsigned char)(~0x10U);
    P5M0 |= 0x10U;
    P54 = 0;
    g_fan_on = 0;
}

void FAN_On(void)
{
    P54 = 1;
    g_fan_on = 1;
}

void FAN_Off(void)
{
    P54 = 0;
    g_fan_on = 0;
}

void FAN_Toggle(void)
{
    if (g_fan_on)
    {
        FAN_Off();
    }
    else
    {
        FAN_On();
    }
}

bit FAN_IsOn(void)
{
    return g_fan_on;
}
