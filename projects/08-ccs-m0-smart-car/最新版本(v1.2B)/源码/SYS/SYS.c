#include "SYS.h"

void delay_ms(unsigned int ms)
{   
    while(ms--)
    {
        // CPUCLK_FREQ -- 当前主频在"ti_msp_dl_config.h"中定义
        delay_cycles(CPUCLK_FREQ / 1000);
    }
}


void delay_us(unsigned int us)
{   
    while(us--)
    {
        // CPUCLK_FREQ -- 当前主频在"ti_msp_dl_config.h"中定义
        delay_cycles(CPUCLK_FREQ / 1000000);
    }
}

void Electromagnets_ON(void)
{
    DL_GPIO_setPins(Electromagnets_IO_PORT,Electromagnets_IO_PB17_PIN);
}

void Electromagnets_OFF(void)
{
    DL_GPIO_clearPins(Electromagnets_IO_PORT,Electromagnets_IO_PB17_PIN);
}


