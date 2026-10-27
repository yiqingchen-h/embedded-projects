#include <stdio.h>

#include "bsp_config.h"
#include "stc8g1k08a.h"
#include "uart1.h"

static volatile unsigned char g_uart1_rx_buf[64];
static volatile unsigned char g_uart1_rx_head = 0U;
static volatile unsigned char g_uart1_rx_tail = 0U;
static volatile unsigned char g_uart1_tx_done = 1U;

void UART1_Init(unsigned long baud)
{
    unsigned int reload;

    /* UART1 uses the default pins P3.0(RXD) / P3.1(TXD). */
    P_SW1 &= 0x3F;
    /* Keep RXD as input and drive TXD actively to avoid a floating TX line. */
    P3M1 |= 0x01;
    P3M0 &= (unsigned char)~0x01;
    P3M1 &= (unsigned char)~0x02;
    P3M0 |= 0x02;

    SCON = 0x50;
    AUXR |= 0x40;      /* Timer1 in 1T mode */
    TMOD &= 0x0F;      /* Timer1 mode 0: 16-bit auto reload on STC8 */

    reload = 65536U - (unsigned int)(BSP_FOSC_HZ / baud / 4UL);
    TH1 = (unsigned char)(reload >> 8);
    TL1 = (unsigned char)reload;

    TR1 = 1;
    TI = 0;
    RI = 0;
    g_uart1_tx_done = 1U;
    ES = 1;
    EA = 1;
}

void UART1_SendByte(unsigned char dat)
{
    TI = 0;
    g_uart1_tx_done = 0U;
    SBUF = dat;
    while (g_uart1_tx_done == 0U)
    {
    }
}

void UART1_SendString(const char *str)
{
    while (*str != '\0')
    {
        UART1_SendByte((unsigned char)*str);
        str++;
    }
}

bit UART1_ByteReady(void)
{
    return (bit)(g_uart1_rx_head != g_uart1_rx_tail);
}

unsigned char UART1_ReadByte(void)
{
    unsigned char dat;

    while (g_uart1_rx_head == g_uart1_rx_tail)
    {
    }

    dat = g_uart1_rx_buf[g_uart1_rx_tail];
    g_uart1_rx_tail = (unsigned char)((g_uart1_rx_tail + 1U) % sizeof(g_uart1_rx_buf));
    return dat;
}

char putchar(char c)
{
    UART1_SendByte((unsigned char)c);
    return c;
}

void UART1_ISR(void) interrupt 4
{
    if (RI)
    {
        unsigned char next_head;

        RI = 0;
        next_head = (unsigned char)((g_uart1_rx_head + 1U) % sizeof(g_uart1_rx_buf));
        if (next_head != g_uart1_rx_tail)
        {
            g_uart1_rx_buf[g_uart1_rx_head] = SBUF;
            g_uart1_rx_head = next_head;
        }
        else
        {
            (void)SBUF;
        }
    }

    if (TI)
    {
        g_uart1_tx_done = 1U;
        TI = 0;
    }
}
