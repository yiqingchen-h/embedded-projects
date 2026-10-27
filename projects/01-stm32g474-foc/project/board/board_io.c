#include "board_io.h"
#include "serial_buffer.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* UART独占DMA通道4/5；ADC通道1、编码器通道2/3保持原占用。 */
static UART_HandleTypeDef g_board_uart;
static volatile board_uart_stats_t g_board_uart_stats;
static uint8_t g_uart_tx[2][512], g_uart_rx[512];
static volatile bool g_uart_tx_active, g_uart_rx_restart;
static volatile uint32_t g_uart_rx_wraps;
static uint32_t g_uart_rx_consumed;
static uint16_t g_uart_tx_size;
static unsigned g_uart_tx_slot;
/* 0正常、1等待旧TX、2延迟100ms、3等待新速率PING。 */
static unsigned g_uart_baud_phase;
static uint32_t g_uart_baud, g_uart_baud_requested, g_uart_baud_deadline;

/** @brief 停机配置连续RX DMA，不等待数据。 */
static void uart_rx_start(void)
{
    DMA1_Channel5->CCR=0U; DMA1->IFCR=DMA_IFCR_CGIF5;
    g_uart_rx_wraps=0U; g_uart_rx_consumed=0U;
    DMAMUX1_Channel4->CCR=DMA_REQUEST_USART3_RX;
    DMA1_Channel5->CPAR=(uint32_t)&USART3->RDR;
    DMA1_Channel5->CMAR=(uint32_t)g_uart_rx; DMA1_Channel5->CNDTR=sizeof(g_uart_rx);
    DMA1_Channel5->CCR=DMA_CCR_MINC|DMA_CCR_CIRC|DMA_CCR_TCIE|DMA_CCR_TEIE|DMA_CCR_EN;
    SET_BIT(USART3->CR3,USART_CR3_DMAR);
}
/** @brief 无发送时重配UART，沿用170MHz和8N1/FIFO。
 * @param baud 请求速率。 @return HAL配置成功。 */
static bool uart_configure(uint32_t baud)
{
    CLEAR_BIT(USART3->CR3,USART_CR3_DMAR|USART_CR3_DMAT); DMA1_Channel5->CCR=0U;
    g_board_uart.Init.BaudRate=baud;
    if(HAL_UART_Init(&g_board_uart)!=HAL_OK || HAL_UARTEx_EnableFifoMode(&g_board_uart)!=HAL_OK)
    { return false; }
    USART3->ICR=USART_ICR_IDLECF|USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NECF|USART_ICR_PECF;
    __HAL_UART_ENABLE_IT(&g_board_uart,UART_IT_IDLE); __HAL_UART_ENABLE_IT(&g_board_uart,UART_IT_ERR);
    uart_rx_start(); g_uart_baud=baud; return true;
}
/** @brief 见board_io.h。 */
bool board_io_init(void)
{
    GPIO_InitTypeDef gpio={0}; serial_buffer_init();
    __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE(); __HAL_RCC_DMAMUX1_CLK_ENABLE();
    gpio.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9; gpio.Mode=GPIO_MODE_INPUT; gpio.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC,&gpio);
    gpio.Pin=GPIO_PIN_10|GPIO_PIN_11; gpio.Mode=GPIO_MODE_AF_PP; gpio.Pull=GPIO_PULLUP;
    gpio.Speed=GPIO_SPEED_FREQ_HIGH; gpio.Alternate=GPIO_AF7_USART3; HAL_GPIO_Init(GPIOB,&gpio);
    g_board_uart.Instance=USART3;
    g_board_uart.Init.WordLength=UART_WORDLENGTH_8B; g_board_uart.Init.StopBits=UART_STOPBITS_1;
    g_board_uart.Init.Parity=UART_PARITY_NONE; g_board_uart.Init.Mode=UART_MODE_TX_RX;
    g_board_uart.Init.HwFlowCtl=UART_HWCONTROL_NONE; g_board_uart.Init.OverSampling=UART_OVERSAMPLING_16;
    g_board_uart.Init.OneBitSampling=UART_ONE_BIT_SAMPLE_DISABLE;
    g_board_uart.Init.ClockPrescaler=UART_PRESCALER_DIV1;
    memset((void *)&g_board_uart_stats,0,sizeof(g_board_uart_stats));
    g_uart_tx_active=false; g_uart_rx_restart=false; g_uart_tx_slot=0U; g_uart_baud_phase=0U;
    DMA1_Channel4->CCR=0U; DMA1->IFCR=DMA_IFCR_CGIF4;
    DMAMUX1_Channel3->CCR=DMA_REQUEST_USART3_TX;
    HAL_NVIC_SetPriority(USART3_IRQn,14U,0U); HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn,14U,0U); HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn,14U,0U); HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    return uart_configure(BOARD_UART_BAUD_RATE);
}
/** @brief 见board_io.h。 */
uint8_t board_switch_raw(void) { return (uint8_t)((GPIOC->IDR>>7U)&7U); }
/** @brief 见board_io.h。 */
uint32_t board_millis(void) { return HAL_GetTick(); }
/** @brief 见board_io.h。 */
board_uart_stats_t board_uart_stats(void) { return g_board_uart_stats; }
/** @brief 见board_io.h。 */
uint32_t board_uart_baud(void) { return g_uart_baud; }
/** @brief 见board_io.h。 */
uint32_t board_uart_ticks(void) { return DWT->CYCCNT; }
/** @brief 见board_io.h。 */
bool board_uart_baud_pending(void) { return g_uart_baud_phase!=0U; }
/** @brief 见board_io.h。 */
bool board_uart_request_baud(uint32_t baud)
{
    if(g_uart_baud_phase!=0U || (baud!=921600U && baud!=1000000U && baud!=2000000U && baud!=3000000U))
    { return false; }
    if(baud!=g_uart_baud) { g_uart_baud_requested=baud; g_uart_baud_phase=1U; }
    return true;
}
/** @brief 见board_io.h。 */
void board_uart_confirm_baud(void)
{ if(g_uart_baud_phase==3U) { g_uart_baud_phase=0U; } }
/** @brief 读取DMA已发布字节数，TC待处理时先让中断记账。
 * @param produced 单调字节数。 @return 快照有效。 */
static bool uart_rx_position(uint32_t *produced)
{
    uint32_t before=g_uart_rx_wraps, remaining=DMA1_Channel5->CNDTR, flags=DMA1->ISR;
    if(before!=g_uart_rx_wraps || (flags&DMA_ISR_TCIF5)!=0U || remaining==0U) { return false; }
    *produced=before*512U+512U-remaining; return true;
}
/** @brief 主循环启动完整TX块。 @param size 长度。 */
static void uart_tx_start(size_t size)
{
    DMA1_Channel4->CCR=0U; DMA1->IFCR=DMA_IFCR_CGIF4;
    DMA1_Channel4->CPAR=(uint32_t)&USART3->TDR;
    DMA1_Channel4->CMAR=(uint32_t)g_uart_tx[g_uart_tx_slot]; DMA1_Channel4->CNDTR=(uint32_t)size;
    g_uart_tx_size=(uint16_t)size; g_uart_tx_active=true; __DMB();
    DMA1_Channel4->CCR=DMA_CCR_MINC|DMA_CCR_DIR|DMA_CCR_TCIE|DMA_CCR_TEIE|DMA_CCR_EN;
    SET_BIT(USART3->CR3,USART_CR3_DMAT); g_uart_tx_slot^=1U;
}
/** @brief 见board_io.h。 */
void board_uart_kick(void)
{
    if(g_uart_tx_active) { return; }
    size_t size=0U;
    while(size<sizeof(g_uart_tx[0]) && serial_buffer_transmit(&g_uart_tx[g_uart_tx_slot][size])) { ++size; }
    if(size!=0U) { uart_tx_start(size); }
}
/** @brief 见board_io.h。 */
bool board_uart_wave_ready(void)
{ return !g_uart_tx_active && g_uart_baud_phase==0U && serial_buffer_tx_free()==SERIAL_TX_CAPACITY-1U; }
/** @brief 见board_io.h。 */
uint8_t *board_uart_wave_buffer(void) { return g_uart_tx[g_uart_tx_slot]; }
/** @brief 见board_io.h。 */
bool board_uart_wave_submit(size_t size)
{
    if(size==0U || size>sizeof(g_uart_tx[0]) || !board_uart_wave_ready()) { return false; }
    uart_tx_start(size); return true;
}
/** @brief 见board_io.h。 */
void board_uart_service(void)
{
    if(g_uart_rx_restart) { g_uart_rx_restart=false; serial_buffer_error(); uart_rx_start(); }
    uint32_t produced;
    if(uart_rx_position(&produced))
    {
        if(produced-g_uart_rx_consumed>sizeof(g_uart_rx))
        {
            ++g_board_uart_stats.rx_overruns; serial_buffer_error();
            g_uart_rx_consumed=produced-(uint32_t)sizeof(g_uart_rx);
        }
        /* 每轮至多64字节，命令解析保留在应用层。 */
        for(unsigned n=0;n<64U && g_uart_rx_consumed!=produced;++n)
        {
            uint8_t byte=g_uart_rx[g_uart_rx_consumed&511U]; ++g_uart_rx_consumed;
            ++g_board_uart_stats.rx; serial_buffer_receive(byte,false);
        }
    }
    board_uart_kick();
    uint32_t now=HAL_GetTick();
    bool idle=!g_uart_tx_active && serial_buffer_tx_free()==SERIAL_TX_CAPACITY-1U && (USART3->ISR&USART_ISR_TC)!=0U;
    if(g_uart_baud_phase==1U && idle) { g_uart_baud_phase=2U; g_uart_baud_deadline=now+100U; }
    else if(g_uart_baud_phase==2U && idle && (int32_t)(now-g_uart_baud_deadline)>=0)
    {
        if(!uart_configure(g_uart_baud_requested))
        { (void)uart_configure(BOARD_UART_BAUD_RATE); g_uart_baud_phase=0U; ++g_board_uart_stats.errors; }
        else { g_uart_baud_phase=3U; g_uart_baud_deadline=now+2000U; }
    }
    else if(g_uart_baud_phase==3U && idle && (int32_t)(now-g_uart_baud_deadline)>=0)
    { (void)uart_configure(BOARD_UART_BAUD_RATE); g_uart_baud_phase=0U; }
}
/** @brief UART只处理中断边界和错误，不逐字节发送或解析。 */
void USART3_IRQHandler(void)
{
    uint32_t started=DWT->CYCCNT, flags=USART3->ISR;
    if((flags&(USART_ISR_ORE|USART_ISR_FE|USART_ISR_NE|USART_ISR_PE))!=0U)
    {
        ++g_board_uart_stats.errors;
        g_board_uart_stats.error_flags|=flags&(USART_ISR_ORE|USART_ISR_FE|USART_ISR_NE|USART_ISR_PE);
        serial_buffer_error();
        USART3->ICR=USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NECF|USART_ICR_PECF;
    }
    if((flags&USART_ISR_IDLE)!=0U) { USART3->ICR=USART_ICR_IDLECF; }
    uint32_t elapsed=DWT->CYCCNT-started;
    if(elapsed>g_board_uart_stats.irq_max_cycles) { g_board_uart_stats.irq_max_cycles=elapsed; }
}
/** @brief 完整TX块结束后交还缓冲，不在中断里启动下一块。 */
void DMA1_Channel4_IRQHandler(void)
{
    uint32_t flags=DMA1->ISR; DMA1->IFCR=DMA_IFCR_CGIF4;
    if((flags&(DMA_ISR_TCIF4|DMA_ISR_TEIF4))==0U) { return; }
    DMA1_Channel4->CCR=0U; CLEAR_BIT(USART3->CR3,USART_CR3_DMAT);
    if((flags&DMA_ISR_TEIF4)!=0U) { ++g_board_uart_stats.dma_errors; }
    else { g_board_uart_stats.tx+=g_uart_tx_size; }
    __DMB(); g_uart_tx_active=false;
}
/** @brief RX DMA只累计整环与错误，字节由主循环搬运。 */
void DMA1_Channel5_IRQHandler(void)
{
    uint32_t flags=DMA1->ISR; DMA1->IFCR=DMA_IFCR_CGIF5;
    if((flags&DMA_ISR_TCIF5)!=0U) { ++g_uart_rx_wraps; }
    if((flags&DMA_ISR_TEIF5)!=0U) { ++g_board_uart_stats.dma_errors; g_uart_rx_restart=true; }
}
