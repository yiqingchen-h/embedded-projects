#include "mocks/stm32g4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
SPI_TypeDef test_spi;
GPIO_TypeDef test_gpioa,test_gpiob,test_gpioc,test_gpiod;
DWT_Type test_dwt;
CoreDebug_Type test_debug;
DMA_TypeDef test_dma;
DMA_Channel_TypeDef test_rx,test_tx;
DMAMUX_Channel_TypeDef test_rx_mux,test_tx_mux;
uint32_t SystemCoreClock=170000000U,test_ms,test_primask;
/* 包含正式板卡实现；只替换寄存器，不复制采集逻辑。 */
#include "../../project/board/board_encoder.c"
static unsigned g_checks;
#define CHECK(value) do { ++g_checks; assert(value); } while(0)

/** @brief 每次推进真实50us峰顶时间。 @return 控制反馈。 */
static encoder_six_feedback_t peak(void)
{
    test_dwt.CYCCNT+=8500U;
    return board_encoder_control_tick();
}

/** @brief 注入一个完整DMA接收帧。 @param raw 机械码。 @param status 芯片状态。 @param crc_ok CRC完整性。 */
static void received(uint32_t raw,uint8_t status,bool crc_ok)
{
    uint8_t data[3]={(uint8_t)(raw>>13U),(uint8_t)(raw>>5U),(uint8_t)((raw<<3U)|status)};
    for(unsigned i=0;i<3;++i) { g_encoder_control_rx[i+2U]=data[i]; }
    g_encoder_control_rx[5]=mt6835_crc(data)^(crc_ok?0U:1U);
    test_rx.CNDTR=test_tx.CNDTR=0U;
    test_spi.SR=0U;
    test_dma.ISR=DMA_ISR_TCIF2|DMA_ISR_TCIF3;
}

/** @brief 进入下一次DMA启动，检查片选建立和完整6字节事务。 */
static void launch(void)
{
    test_dma.ISR=0U;
    encoder_six_feedback_t before=peak();
    CHECK(g_encoder_control_phase==1U && test_gpioc.BSRR==(GPIO_PIN_4<<16U));
    CHECK((test_spi.CR2&(SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN))==0U);
    (void)peak();
    CHECK(g_encoder_control_phase==2U && test_rx.CNDTR==6U && test_tx.CNDTR==6U);
    CHECK(test_rx_mux.CCR==10U && test_tx_mux.CCR==11U);
    CHECK(g_encoder_control_feedback.sequence==before.sequence);
}

/** @brief 验证CRC/磁场/不完整DMA、保持时间和回收，不调用主循环也能继续采集。 @return 成功0。 */
int main(void)
{
    board_encoder_init();CHECK(!board_encoder_control_begin());
    g_encoder_probed=true;
    CHECK(board_encoder_control_begin());CHECK(!board_encoder_control_begin());
    launch();
    received(2097100U,0U,true);
    encoder_six_feedback_t value=peak();
    CHECK(value.valid && value.raw==2097100U && value.sequence==1U);
    CHECK(test_gpioc.BSRR==(GPIO_PIN_4<<16U));
    (void)peak();CHECK(test_gpioc.BSRR==GPIO_PIN_4);
    for(unsigned i=0;i<100U;++i)
    {
        launch();received(i*100U,0U,true);value=peak();
        CHECK(value.valid && value.raw==i*100U && value.sequence==i+2U);
        CHECK(test_dwt.CYCCNT-value.tick==8500U);
        (void)peak();
    }
    launch();received(12345U,0U,false);value=peak();CHECK(!value.valid && value.sequence==102U);(void)peak();
    launch();received(12345U,2U,true);value=peak();CHECK(!value.valid && g_encoder_control_status==2U);(void)peak();
    launch();received(12345U,0U,true);test_rx.CNDTR=1U;
    uint32_t before=g_encoder_control_feedback.sequence;
    value=peak();CHECK(value.sequence==before);
    for(unsigned i=0;i<3U;++i) { value=peak(); }
    CHECK(!value.valid && value.sequence==before+1U);(void)peak();
    launch();test_dma.ISR=DMA_ISR_TEIF2;value=peak();CHECK(!value.valid);(void)peak();
    board_encoder_control_end();
    CHECK(!g_encoder_control_enabled && test_gpioc.BSRR==GPIO_PIN_4);
    CHECK(test_rx.CCR==0U && test_tx.CCR==0U && (test_spi.CR2&3U)==0U);
    CHECK(!board_encoder_control_tick().valid);
    printf("encoder DMA actual board state machine: %u checks passed\n",g_checks);
    return 0;
}
