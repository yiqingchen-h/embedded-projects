#include "stc8g1k08a.h"
#include "sysclk.h"

void SYSCLK_Init(void)
{
    /*
     * STC-ISP is expected to configure the internal IRC to 24MHz.
     * Firmware only clears the divider so UART math matches BSP_FOSC_HZ.
     */
    CLKDIV = 0x00;
}
