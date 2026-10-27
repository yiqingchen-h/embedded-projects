#include "bsp_config.h"
#include "delay.h"

void DelayUs(unsigned int us)
{
    while (us--)
    {
        unsigned char i;
        for (i = 0U; i < 8U; i++)
        {
        }
    }
}

void DelayMs(unsigned int ms)
{
    while (ms--)
    {
        DelayUs(1000U);
    }
}
