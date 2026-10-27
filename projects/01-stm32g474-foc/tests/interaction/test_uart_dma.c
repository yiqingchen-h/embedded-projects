#include "stm32g4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

USART_TypeDef test_uart;
DMA_Channel_TypeDef test_dma4,test_dma5;
DMA_TypeDef test_dma;
DMAMUX_Channel_TypeDef test_mux3,test_mux4;
DWT_Type test_dwt;
struct test_gpio test_gpio_c;
static uint32_t now_ms;
static unsigned checks;
#define CHECK(value) do { ++checks; assert(value); } while(0)
/** @brief 配置GPIO但不操作硬件。 @param gpio 端口。 @param init 配置。 */
void HAL_GPIO_Init(void *gpio,GPIO_InitTypeDef *init) { (void)gpio; (void)init; }
/** @brief 配置UART并模拟空闲发送线。 @param handle 配置。 @return 成功。 */
int HAL_UART_Init(UART_HandleTypeDef *handle)
{ test_uart.BRR=(170000000U+handle->Init.BaudRate/2U)/handle->Init.BaudRate; test_uart.ISR|=USART_ISR_TC; return HAL_OK; }
/** @brief FIFO替身。 @param handle 句柄。 @return 成功。 */
int HAL_UARTEx_EnableFifoMode(UART_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
/** @brief 验证UART及两个DMA中断保持低优先级。 @param irq 中断。 @param pre 优先级。 @param sub 子优先级。 */
void HAL_NVIC_SetPriority(unsigned irq,unsigned pre,unsigned sub)
{ CHECK(irq>=3U && irq<=5U && pre==14U && sub==0U); }
/** @brief 中断使能替身。 @param irq 中断。 */
void HAL_NVIC_EnableIRQ(unsigned irq) { (void)irq; }
/** @brief 返回测试毫秒时钟。 @return 毫秒。 */
uint32_t HAL_GetTick(void) { return now_ms; }

/* 编译实际板卡实现，直接观察固定DMA缓冲的所有权。 */
#include "../../project/board/board_io.c"

/** @brief 模拟完整TX DMA和线路最终空闲。 */
static void finish_tx(void)
{
    test_dma.ISR=DMA_ISR_TCIF4; DMA1_Channel4_IRQHandler(); test_dma.ISR=0U;
    test_uart.ISR|=USART_ISR_TC;
}
/** @brief 只模拟DMA写入，绝不逐字节调用UART IRQ。 @param bytes 数据。 @param size 长度。 */
static void receive(const char *bytes,size_t size)
{
    for(size_t n=0;n<size;++n)
    {
        unsigned offset=512U-test_dma5.CNDTR;
        g_uart_rx[offset]=(uint8_t)bytes[n];
        --test_dma5.CNDTR;
        if(test_dma5.CNDTR==0U)
        {
            test_dma5.CNDTR=512U; test_dma.ISR=DMA_ISR_TCIF5;
            DMA1_Channel5_IRQHandler(); test_dma.ISR=0U;
        }
    }
}
/** @brief 验证实际DMA路由、命令优先和缓冲不可被覆盖。 */
static void routing_and_priority(void)
{
    now_ms=0U; CHECK(board_io_init());
    CHECK(test_mux3.CCR==29U && test_mux4.CCR==28U);
    CHECK(test_dma5.CNDTR==512U && (test_dma5.CCR&DMA_CCR_CIRC)!=0U);
    CHECK(board_uart_baud()==921600U && board_uart_wave_ready());
    uint8_t *active=board_uart_wave_buffer(); memset(active,0xA5,100U);
    CHECK(board_uart_wave_submit(100U));
    CHECK(test_dma4.CNDTR==100U && !board_uart_wave_ready());
    CHECK(board_uart_wave_buffer()!=active && active[0]==0xA5U);
    CHECK(serial_buffer_write("OK PING\r\n",9U)); board_uart_kick();
    CHECK(test_dma4.CNDTR==100U);
    finish_tx(); CHECK(!board_uart_wave_ready());
    board_uart_service(); CHECK(test_dma4.CNDTR==9U);
    CHECK(memcmp(g_uart_tx[g_uart_tx_slot^1U],"OK PING\r\n",9U)==0);
    finish_tx(); CHECK(board_uart_wave_ready());
    CHECK(board_uart_stats().tx==109U);
}
/** @brief 验证旧ACK物理发完后100ms切速、PING确认和2秒回退。 */
static void baud_handshake(void)
{
    CHECK(board_uart_request_baud(2000000U)); CHECK(board_uart_baud_pending());
    CHECK(!board_uart_request_baud(3000000U));
    CHECK(serial_buffer_write("ACK\r\n",5U)); board_uart_kick();
    now_ms=200U; board_uart_service(); CHECK(board_uart_baud()==921600U);
    finish_tx(); board_uart_service();
    now_ms=299U; board_uart_service(); CHECK(board_uart_baud()==921600U);
    now_ms=300U; board_uart_service(); CHECK(board_uart_baud()==2000000U && test_uart.BRR==85U);
    board_uart_confirm_baud(); CHECK(!board_uart_baud_pending());
    CHECK(board_uart_request_baud(3000000U)); board_uart_service();
    now_ms=400U; board_uart_service(); CHECK(board_uart_baud()==3000000U);
    now_ms=2400U; board_uart_service(); CHECK(board_uart_baud()==921600U && !board_uart_baud_pending());
    CHECK(!board_uart_request_baud(115200U));
}
/** @brief 验证DMA接收满环与溢出后可信LF恢复，不接受损坏命令。 */
static void rx_wrap_and_overflow(void)
{
    CHECK(board_io_init());
    receive("VERSION\r\n",9U); board_uart_service();
    uint16_t token; unsigned count=0U;
    while(serial_buffer_read(&token)) { CHECK(token<256U); ++count; }
    CHECK(count==9U && board_uart_stats().rx==9U);
    char noise[1025]; memset(noise,'x',sizeof(noise)); noise[1024]='\n';
    receive(noise,sizeof(noise));
    for(unsigned n=0;n<9U;++n) { board_uart_service(); }
    CHECK(board_uart_stats().rx_overruns==1U);
    CHECK(serial_buffer_read(&token) && token==0x10AU);
    CHECK(!serial_buffer_read(&token));
    receive("PING\n",5U); board_uart_service();
    CHECK(serial_buffer_read(&token) && token=='P');
    test_dma.ISR=DMA_ISR_TEIF5; DMA1_Channel5_IRQHandler(); test_dma.ISR=0U;
    board_uart_service(); CHECK(board_uart_stats().dma_errors==1U && test_dma5.CNDTR==512U);
}
/** @brief 错误位图只记录硬件原因，不改变已有损坏行恢复和计数行为。 */
static void error_flags_diagnostics(void)
{
    CHECK(board_io_init());
    test_uart.ISR=USART_ISR_NE|USART_ISR_FE;USART3_IRQHandler();
    CHECK(board_uart_stats().errors==1U && board_uart_stats().error_flags==(USART_ISR_NE|USART_ISR_FE));
    test_uart.ISR=USART_ISR_ORE|USART_ISR_PE;USART3_IRQHandler();
    CHECK(board_uart_stats().errors==2U && board_uart_stats().error_flags==15U);
}

/** @brief 运行实际UART驱动状态机回归。 @return 成功0。 */
int main(void)
{
    routing_and_priority(); baud_handshake(); rx_wrap_and_overflow();
    error_flags_diagnostics();
    printf("actual UART DMA: %u checks passed\n",checks); return 0;
}
