#include "board_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 仅模拟被测收发函数直接使用的USART寄存器。 */
static struct { uint32_t ISR, ICR, CR1, RDR, TDR; } uart;
#define USART3 (&uart)
#define USART_ISR_ORE (1U << 3)
#define USART_ISR_FE (1U << 1)
#define USART_ISR_NE (1U << 2)
#define USART_ISR_PE (1U << 0)
#define USART_ISR_RXNE_RXFNE (1U << 5)
#define USART_ISR_TXE_TXFNF (1U << 7)
#define USART_ICR_ORECF USART_ISR_ORE
#define USART_ICR_FECF USART_ISR_FE
#define USART_ICR_NECF USART_ISR_NE
#define USART_ICR_PECF USART_ISR_PE
#define USART_CR1_TXEIE_TXFNFIE (1U << 7)
#define UART_IT_TXE USART_CR1_TXEIE_TXFNFIE
#define __HAL_UART_ENABLE_IT(h, bit) ((void)(h), USART3->CR1 |= (bit))
#define __HAL_UART_DISABLE_IT(h, bit) ((void)(h), USART3->CR1 &= ~(bit))
#define __DMB() ((void)0)
/* 与实际板卡函数同名的状态，函数从固件源码提取。 */
static bool g_uart_ready = true, stopped;
static int g_uart;
static volatile board_inputs_t g_inputs;
static volatile uint8_t g_rx[256], g_rx_head, g_rx_tail;
static char g_async_tx[128];
static volatile uint16_t g_async_count, g_async_position;
/* 断言计数。 */
static unsigned checks;

/** @brief 模拟即时关断。 */
void board_trip(void) { stopped = true; }
/** @brief 断言并计数。 @param value 条件。 @param line 行号。 */
static void check(bool value, unsigned line)
{
    ++checks;
    if (!value) { fprintf(stderr, "UART failed line %u\n", line); exit(1); }
}
#define CHECK(v) check((v), __LINE__)
#include "uart_under_test.inc"

/** @brief 验证短响应与RX/急停共用中断时互不覆盖。 @return 退出码。 */
int main(void)
{
    CHECK(board_uart_try_write("LOAD STATE=OUTPUT\r\n"));
    CHECK(!board_uart_try_write("overwrite"));
    char output[128] = {0};
    unsigned length = 0U;
    while (g_async_count != 0U)
    {
        uart.ISR = USART_ISR_TXE_TXFNF;
        board_uart_irq();
        output[length++] = (char)uart.TDR;
    }
    CHECK(strcmp(output, "LOAD STATE=OUTPUT\r\n") == 0);
    CHECK((uart.CR1 & USART_CR1_TXEIE_TXFNFIE) == 0U);
    CHECK(!board_uart_try_write(NULL) && !board_uart_try_write(""));
    char large[129]; memset(large, 'X', 128U); large[128] = '\0';
    CHECK(!board_uart_try_write(large));
    g_uart_ready = false; CHECK(!board_uart_try_write("X")); g_uart_ready = true;
    CHECK(board_uart_try_write("OK"));
    uart.ISR = USART_ISR_TXE_TXFNF | USART_ISR_RXNE_RXFNE; uart.RDR = '!';
    board_uart_irq();
    CHECK(stopped && g_inputs.rx_errors == 1U && uart.TDR == 'O');
    char received;
    CHECK(board_uart_get(&received) && received == '!');
    CHECK(!board_uart_get(&received) && !board_uart_get(NULL));
    stopped = false; uart.ISR = USART_ISR_RXNE_RXFNE | USART_ISR_ORE; uart.RDR = 'R';
    board_uart_irq();
    CHECK(stopped && g_inputs.rx_errors == 2U && !board_uart_get(&received));
    g_rx_tail = 0U; g_rx_head = 255U; stopped = false;
    uart.ISR = USART_ISR_RXNE_RXFNE; uart.RDR = '!'; board_uart_irq();
    CHECK(stopped && g_inputs.rx_errors == 4U);
    printf("UART load/stop: %u checks passed\n", checks);
    return 0;
}
