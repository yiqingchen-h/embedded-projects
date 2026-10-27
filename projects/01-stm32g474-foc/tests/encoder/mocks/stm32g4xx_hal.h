#ifndef FOC_TEST_ENCODER_HAL_H
#define FOC_TEST_ENCODER_HAL_H
#include <stdint.h>
/* 只替换寄存器存储与HAL入口，正式SPI/DMA状态机由测试直接包含。 */
typedef struct { volatile uint32_t CR1,CR2,SR,DR; } SPI_TypeDef;
typedef struct { volatile uint32_t BSRR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CYCCNT,CTRL; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
typedef struct { volatile uint32_t CCR,CNDTR,CPAR,CMAR; } DMA_Channel_TypeDef;
typedef struct { volatile uint32_t ISR,IFCR; } DMA_TypeDef;
typedef struct { volatile uint32_t CCR; } DMAMUX_Channel_TypeDef;
typedef struct { uint32_t Pin,Mode,Pull,Speed,Alternate; } GPIO_InitTypeDef;
extern SPI_TypeDef test_spi;
extern GPIO_TypeDef test_gpioa,test_gpiob,test_gpioc,test_gpiod;
extern DWT_Type test_dwt;
extern CoreDebug_Type test_debug;
extern DMA_TypeDef test_dma;
extern DMA_Channel_TypeDef test_rx,test_tx;
extern DMAMUX_Channel_TypeDef test_rx_mux,test_tx_mux;
extern uint32_t SystemCoreClock,test_ms,test_primask;
#define SPI1 (&test_spi)
#define GPIOA (&test_gpioa)
#define GPIOB (&test_gpiob)
#define GPIOC (&test_gpioc)
#define GPIOD (&test_gpiod)
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1U<<24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define DMA1 (&test_dma)
#define DMA1_Channel2 (&test_rx)
#define DMA1_Channel3 (&test_tx)
#define DMAMUX1_Channel1 (&test_rx_mux)
#define DMAMUX1_Channel2 (&test_tx_mux)
#define SPI_CR1_MSTR (1U<<2)
#define SPI_CR1_SSM (1U<<9)
#define SPI_CR1_SSI (1U<<8)
#define SPI_CR1_CPOL (1U<<1)
#define SPI_CR1_CPHA 1U
#define SPI_CR1_BR_2 (1U<<5)
#define SPI_CR1_BR_0 (1U<<3)
#define SPI_CR1_SPE (1U<<6)
#define SPI_CR2_DS_0 (1U<<8)
#define SPI_CR2_DS_1 (1U<<9)
#define SPI_CR2_DS_2 (1U<<10)
#define SPI_CR2_FRXTH (1U<<12)
#define SPI_CR2_RXDMAEN 1U
#define SPI_CR2_TXDMAEN 2U
#define SPI_SR_TXE 2U
#define SPI_SR_RXNE 1U
#define SPI_SR_BSY (1U<<7)
#define SPI_SR_OVR (1U<<6)
#define SPI_SR_MODF (1U<<5)
#define SPI_SR_FRE (1U<<8)
#define DMA_CCR_EN 1U
#define DMA_CCR_DIR (1U<<4)
#define DMA_CCR_MINC (1U<<7)
#define DMA_IFCR_CGIF2 (1U<<4)
#define DMA_IFCR_CGIF3 (1U<<8)
#define DMA_ISR_TEIF2 (1U<<7)
#define DMA_ISR_TEIF3 (1U<<11)
#define DMA_ISR_TCIF2 (1U<<5)
#define DMA_ISR_TCIF3 (1U<<9)
#define DMA_REQUEST_SPI1_RX 10U
#define DMA_REQUEST_SPI1_TX 11U
#define GPIO_PIN_2 (1U<<2)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_5 (1U<<5)
#define GPIO_PIN_6 (1U<<6)
#define GPIO_PIN_7 (1U<<7)
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_MODE_INPUT 0U
#define GPIO_NOPULL 0U
#define GPIO_PULLUP 1U
#define GPIO_PULLDOWN 2U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_SPEED_FREQ_HIGH 2U
#define GPIO_AF5_SPI1 5U
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMAMUX1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI1_FORCE_RESET() (test_spi.SR=0U)
#define __HAL_RCC_SPI1_RELEASE_RESET() ((void)0)
#define __get_PRIMASK() (test_primask)
#define __disable_irq() (test_primask=1U)
#define __set_PRIMASK(value) (test_primask=(value))
#define __NOP() (++test_dwt.CYCCNT)
/** @brief 模拟毫秒时钟。 @return 当前时间。 */
static inline uint32_t HAL_GetTick(void) { return test_ms; }
/** @brief GPIO复用配置在主机不驱动引脚。 @param gpio 端口。 @param config 配置。 */
static inline void HAL_GPIO_Init(GPIO_TypeDef *gpio,GPIO_InitTypeDef *config) { (void)gpio;(void)config; }
#endif
