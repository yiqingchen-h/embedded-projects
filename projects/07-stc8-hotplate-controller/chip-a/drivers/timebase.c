#include "bsp_config.h"
#include "stc8g1k08a.h"
#include "timebase.h"

static volatile unsigned int g_timebase_ms = 0U;

void TIMEBASE_Init(void)
{
    unsigned int reload;

    AUXR |= 0x80;      /* Timer0 in 1T mode */
    TMOD &= 0xF0;      /* Timer0 mode 0: 16-bit auto reload on STC8 */

    reload = 65536U - (unsigned int)(BSP_FOSC_HZ / 1000UL);
    TH0 = (unsigned char)(reload >> 8);
    TL0 = (unsigned char)reload;

    ET0 = 1;
    TR0 = 1;
    EA = 1;
}

unsigned int TIMEBASE_GetMs(void)
{
    unsigned int now_ms;
    bit ea_backup;

    ea_backup = EA;
    EA = 0;
    now_ms = g_timebase_ms;
    EA = ea_backup;
    return now_ms;
}

void TIMEBASE_ISR(void) interrupt 1
{
    g_timebase_ms++;
}
