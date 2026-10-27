#ifndef FOC_UART_TEST_HAL_H
#define FOC_UART_TEST_HAL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 仅定义本板UART代码实际使用的寄存器，测试不访问真实地址。 */
typedef struct { volatile uint32_t CR1,CR3,ISR,ICR,BRR,TDR,RDR; } USART_TypeDef;
typedef struct { volatile uint32_t CCR,CNDTR,CPAR,CMAR; } DMA_Channel_TypeDef;
typedef struct { volatile uint32_t ISR,IFCR; } DMA_TypeDef;
typedef struct { volatile uint32_t CCR; } DMAMUX_Channel_TypeDef;
typedef struct { volatile uint32_t CYCCNT; } DWT_Type;
typedef struct { uint32_t Pin,Mode,Pull,Speed,Alternate; } GPIO_InitTypeDef;
typedef struct {
    USART_TypeDef *Instance;
    struct { uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling,
        OneBitSampling,ClockPrescaler; } Init;
} UART_HandleTypeDef;
extern USART_TypeDef test_uart;
extern DMA_Channel_TypeDef test_dma4,test_dma5;
extern DMA_TypeDef test_dma;
extern DMAMUX_Channel_TypeDef test_mux3,test_mux4;
extern DWT_Type test_dwt;
#define USART3 (&test_uart)
#define DMA1_Channel4 (&test_dma4)
#define DMA1_Channel5 (&test_dma5)
#define DMA1 (&test_dma)
#define DMAMUX1_Channel3 (&test_mux3)
#define DMAMUX1_Channel4 (&test_mux4)
#define DWT (&test_dwt)
#define GPIOB ((void *)2)
#define GPIOC (&test_gpio_c)
extern struct test_gpio { uint32_t IDR; } test_gpio_c;
#define GPIO_PIN_7 (1U<<7)
#define GPIO_PIN_8 (1U<<8)
#define GPIO_PIN_9 (1U<<9)
#define GPIO_PIN_10 (1U<<10)
#define GPIO_PIN_11 (1U<<11)
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_AF_PP 2U
#define GPIO_NOPULL 0U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_HIGH 3U
#define GPIO_AF7_USART3 7U
#define UART_WORDLENGTH_8B 0U
#define UART_STOPBITS_1 0U
#define UART_PARITY_NONE 0U
#define UART_MODE_TX_RX 3U
#define UART_HWCONTROL_NONE 0U
#define UART_OVERSAMPLING_16 0U
#define UART_ONE_BIT_SAMPLE_DISABLE 0U
#define UART_PRESCALER_DIV1 0U
#define HAL_OK 0
#define UART_IT_IDLE 16U
#define UART_IT_ERR 1U
#define USART3_IRQn 3U
#define DMA1_Channel4_IRQn 4U
#define DMA1_Channel5_IRQn 5U
#define DMA_REQUEST_USART3_RX 28U
#define DMA_REQUEST_USART3_TX 29U
#define DMA_CCR_EN 1U
#define DMA_CCR_TCIE 2U
#define DMA_CCR_TEIE 8U
#define DMA_CCR_DIR 16U
#define DMA_CCR_CIRC 32U
#define DMA_CCR_MINC 128U
#define DMA_IFCR_CGIF4 (1U<<12)
#define DMA_IFCR_CGIF5 (1U<<16)
#define DMA_ISR_TCIF4 (1U<<13)
#define DMA_ISR_TEIF4 (1U<<15)
#define DMA_ISR_TCIF5 (1U<<17)
#define DMA_ISR_TEIF5 (1U<<19)
#define USART_CR3_DMAR 64U
#define USART_CR3_DMAT 128U
#define USART_ISR_PE 1U
#define USART_ISR_FE 2U
#define USART_ISR_NE 4U
#define USART_ISR_ORE 8U
#define USART_ISR_IDLE 16U
#define USART_ISR_TC 64U
#define USART_ICR_PECF 1U
#define USART_ICR_FECF 2U
#define USART_ICR_NECF 4U
#define USART_ICR_ORECF 8U
#define USART_ICR_IDLECF 16U
#define SET_BIT(reg,bits) ((reg)|=(bits))
#define CLEAR_BIT(reg,bits) ((reg)&=~(bits))
#define __DMB() __asm volatile ("" ::: "memory")
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMAMUX1_CLK_ENABLE() ((void)0)
#define __HAL_UART_ENABLE_IT(handle,bits) ((handle)->Instance->CR1|=(bits))
/** @brief GPIO替身，只检查正确板卡适配路径。 @param gpio 端口。 @param init 配置。 */
void HAL_GPIO_Init(void *gpio,GPIO_InitTypeDef *init);
/** @brief UART配置替身。 @param handle 句柄。 @return HAL_OK。 */
int HAL_UART_Init(UART_HandleTypeDef *handle);
/** @brief FIFO替身。 @param handle 句柄。 @return HAL_OK。 */
int HAL_UARTEx_EnableFifoMode(UART_HandleTypeDef *handle);
/** @brief IRQ优先级替身。 @param irq 中断。 @param pre 主优先级。 @param sub 子优先级。 */
void HAL_NVIC_SetPriority(unsigned irq,unsigned pre,unsigned sub);
/** @brief IRQ使能替身。 @param irq 中断。 */
void HAL_NVIC_EnableIRQ(unsigned irq);
/** @brief 模拟毫秒时钟。 @return 毫秒。 */
uint32_t HAL_GetTick(void);
#endif
