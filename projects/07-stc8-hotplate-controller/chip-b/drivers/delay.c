#include "bsp_config.h"
#include "delay.h"

void DelayUs(unsigned int us)
{
    while (us--)
    {
        unsigned char i;

        /*
         * Rough busy-wait delay based on a 24MHz clock.
         * This is only for the UART template heartbeat.
         */
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
