/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_uart_rxi_isr, /* SCI7 RXI (Receive data full) */
            [1] = sci_uart_txi_isr, /* SCI7 TXI (Transmit data empty) */
            [2] = sci_uart_tei_isr, /* SCI7 TEI (Transmit end) */
            [3] = sci_uart_eri_isr, /* SCI7 ERI (Receive error) */
            [4] = sci_uart_rxi_isr, /* SCI5 RXI (Receive data full) */
            [5] = sci_uart_txi_isr, /* SCI5 TXI (Transmit data empty) */
            [6] = sci_uart_tei_isr, /* SCI5 TEI (Transmit end) */
            [7] = sci_uart_eri_isr, /* SCI5 ERI (Receive error) */
            [8] = sci_uart_txi_isr, /* SCI4 TXI (Transmit data empty) */
            [9] = sci_uart_tei_isr, /* SCI4 TEI (Transmit end) */
            [10] = gpt_counter_overflow_isr, /* GPT3 COUNTER OVERFLOW (Overflow) */
            [11] = gpt_counter_overflow_isr, /* GPT2 COUNTER OVERFLOW (Overflow) */
            [12] = gpt_counter_overflow_isr, /* GPT6 COUNTER OVERFLOW (Overflow) */
            [13] = r_icu_isr, /* ICU IRQ8 (External pin interrupt 8) */
            [14] = r_icu_isr, /* ICU IRQ9 (External pin interrupt 9) */
            [15] = spi_tei_isr, /* SPI1 TEI (Transmission complete event) */
            [16] = spi_eri_isr, /* SPI1 ERI (Error) */
            [17] = dmac_int_isr, /* DMAC0 INT (DMAC0 transfer end) */
            [18] = dmac_int_isr, /* DMAC1 INT (DMAC1 transfer end) */
            [19] = sci_uart_rxi_isr, /* SCI4 RXI (Receive data full) */
            [20] = sci_uart_eri_isr, /* SCI4 ERI (Receive error) */
            [21] = sci_uart_rxi_isr, /* SCI9 RXI (Receive data full) */
            [22] = sci_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [23] = sci_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [24] = sci_uart_eri_isr, /* SCI9 ERI (Receive error) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_SCI7_RXI,GROUP0), /* SCI7 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_SCI7_TXI,GROUP1), /* SCI7 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_SCI7_TEI,GROUP2), /* SCI7 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_SCI7_ERI,GROUP3), /* SCI7 ERI (Receive error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_SCI5_RXI,GROUP4), /* SCI5 RXI (Receive data full) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TXI,GROUP5), /* SCI5 TXI (Transmit data empty) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TEI,GROUP6), /* SCI5 TEI (Transmit end) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI5_ERI,GROUP7), /* SCI5 ERI (Receive error) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TXI,GROUP0), /* SCI4 TXI (Transmit data empty) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TEI,GROUP1), /* SCI4 TEI (Transmit end) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_GPT3_COUNTER_OVERFLOW,GROUP2), /* GPT3 COUNTER OVERFLOW (Overflow) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_GPT2_COUNTER_OVERFLOW,GROUP3), /* GPT2 COUNTER OVERFLOW (Overflow) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_GPT6_COUNTER_OVERFLOW,GROUP4), /* GPT6 COUNTER OVERFLOW (Overflow) */
            [13] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ8,GROUP5), /* ICU IRQ8 (External pin interrupt 8) */
            [14] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ9,GROUP6), /* ICU IRQ9 (External pin interrupt 9) */
            [15] = BSP_PRV_VECT_ENUM(EVENT_SPI1_TEI,GROUP7), /* SPI1 TEI (Transmission complete event) */
            [16] = BSP_PRV_VECT_ENUM(EVENT_SPI1_ERI,GROUP0), /* SPI1 ERI (Error) */
            [17] = BSP_PRV_VECT_ENUM(EVENT_DMAC0_INT,GROUP1), /* DMAC0 INT (DMAC0 transfer end) */
            [18] = BSP_PRV_VECT_ENUM(EVENT_DMAC1_INT,GROUP2), /* DMAC1 INT (DMAC1 transfer end) */
            [19] = BSP_PRV_VECT_ENUM(EVENT_SCI4_RXI,GROUP3), /* SCI4 RXI (Receive data full) */
            [20] = BSP_PRV_VECT_ENUM(EVENT_SCI4_ERI,GROUP4), /* SCI4 ERI (Receive error) */
            [21] = BSP_PRV_VECT_ENUM(EVENT_SCI9_RXI,GROUP5), /* SCI9 RXI (Receive data full) */
            [22] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TXI,GROUP6), /* SCI9 TXI (Transmit data empty) */
            [23] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TEI,GROUP7), /* SCI9 TEI (Transmit end) */
            [24] = BSP_PRV_VECT_ENUM(EVENT_SCI9_ERI,GROUP0), /* SCI9 ERI (Receive error) */
        };
        #endif
        #endif