/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = gpt_counter_overflow_isr, /* GPT0 COUNTER OVERFLOW (Overflow) */
            [1] = gpt_counter_overflow_isr, /* GPT2 COUNTER OVERFLOW (Overflow) */
            [2] = gpt_counter_overflow_isr, /* GPT11 COUNTER OVERFLOW (Overflow) */
            [3] = gpt_counter_overflow_isr, /* GPT12 COUNTER OVERFLOW (Overflow) */
            [4] = sci_b_uart_txi_isr, /* SCI6 TXI (Transmit data empty) */
            [5] = sci_b_uart_tei_isr, /* SCI6 TEI (Transmit end) */
            [6] = ssi_txi_isr, /* SSI1 TXI (Receive data full/Transmit data empty) */
            [7] = ssi_int_isr, /* SSI1 INT (Error interrupt) */
            [8] = pdm_err_isr, /* PDM ERR0 (Error detection interrupt channel 0) */
            [9] = dmac_int_isr, /* DMAC2 INT (DMAC2 transfer end) */
            [10] = sci_b_spi_rxi_isr, /* SCI5 RXI (Receive data full) */
            [11] = sci_b_spi_txi_isr, /* SCI5 TXI (Transmit data empty) */
            [12] = sci_b_spi_tei_isr, /* SCI5 TEI (Transmit end) */
            [13] = sci_b_spi_eri_isr, /* SCI5 ERI (Receive error) */
            [14] = glcdc_line_detect_isr, /* GLCDC LINE DETECT (Specified line) */
            [15] = mipi_dsi_seq0_isr, /* MIPIDSI SEQ0 (Sequence operation channel 0 interrupt) */
            [16] = mipi_dsi_seq1_isr, /* MIPIDSI SEQ1 (Sequence operation channel 1 interrupt) */
            [17] = mipi_dsi_vin1_isr, /* MIPIDSI VIN1 (Video-Input operation channel1 interrupt) */
            [18] = mipi_dsi_rcv_isr, /* MIPIDSI RCV (DSI packet receive interrupt) */
            [19] = mipi_dsi_ferr_isr, /* MIPIDSI FERR (DSI fatal error interrupt) */
            [20] = mipi_dsi_ppi_isr, /* MIPIDSI PPI (DSI D-PHY PPI interrupt) */
            [21] = sci_b_uart_rxi_isr, /* SCI9 RXI (Receive data full) */
            [22] = sci_b_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [23] = sci_b_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [24] = sci_b_uart_eri_isr, /* SCI9 ERI (Receive error) */
            [25] = gpt_counter_overflow_isr, /* GPT4 COUNTER OVERFLOW (Overflow) */
            [26] = gpt_counter_overflow_isr, /* GPT7 COUNTER OVERFLOW (Overflow) */
            [27] = sci_b_spi_tei_isr, /* SCI2 TEI (Transmit end) */
            [28] = sci_b_spi_eri_isr, /* SCI2 ERI (Receive error) */
            [29] = dmac_int_isr, /* DMAC0 INT (DMAC0 transfer end) */
            [30] = dmac_int_isr, /* DMAC1 INT (DMAC1 transfer end) */
            [31] = sci_b_uart_rxi_isr, /* SCI4 RXI (Receive data full) */
            [32] = sci_b_uart_txi_isr, /* SCI4 TXI (Transmit data empty) */
            [33] = sci_b_uart_tei_isr, /* SCI4 TEI (Transmit end) */
            [34] = sci_b_uart_eri_isr, /* SCI4 ERI (Receive error) */
            [35] = sci_b_uart_rxi_isr, /* SCI3 RXI (Receive data full) */
            [36] = sci_b_uart_txi_isr, /* SCI3 TXI (Transmit data empty) */
            [37] = sci_b_uart_tei_isr, /* SCI3 TEI (Transmit end) */
            [38] = sci_b_uart_eri_isr, /* SCI3 ERI (Receive error) */
            [39] = sci_b_uart_rxi_isr, /* SCI6 RXI (Receive data full) */
            [40] = sci_b_uart_eri_isr, /* SCI6 ERI (Receive error) */
            [41] = sci_b_uart_rxi_isr, /* SCI8 RXI (Receive data full) */
            [42] = sci_b_uart_txi_isr, /* SCI8 TXI (Transmit data empty) */
            [43] = sci_b_uart_tei_isr, /* SCI8 TEI (Transmit end) */
            [44] = sci_b_uart_eri_isr, /* SCI8 ERI (Receive error) */
            [45] = rm_ethosu_isr, /* NPU IRQ (NPU IRQ) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_GPT0_COUNTER_OVERFLOW,GROUP0), /* GPT0 COUNTER OVERFLOW (Overflow) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_GPT2_COUNTER_OVERFLOW,GROUP1), /* GPT2 COUNTER OVERFLOW (Overflow) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_GPT11_COUNTER_OVERFLOW,GROUP2), /* GPT11 COUNTER OVERFLOW (Overflow) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_GPT12_COUNTER_OVERFLOW,GROUP3), /* GPT12 COUNTER OVERFLOW (Overflow) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_SCI6_TXI,GROUP4), /* SCI6 TXI (Transmit data empty) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI6_TEI,GROUP5), /* SCI6 TEI (Transmit end) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SSI1_TXI,GROUP6), /* SSI1 TXI (Receive data full/Transmit data empty) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SSI1_INT,GROUP7), /* SSI1 INT (Error interrupt) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_PDM_ERR0,GROUP0), /* PDM ERR0 (Error detection interrupt channel 0) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_DMAC2_INT,GROUP1), /* DMAC2 INT (DMAC2 transfer end) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_SCI5_RXI,GROUP2), /* SCI5 RXI (Receive data full) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TXI,GROUP3), /* SCI5 TXI (Transmit data empty) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TEI,GROUP4), /* SCI5 TEI (Transmit end) */
            [13] = BSP_PRV_VECT_ENUM(EVENT_SCI5_ERI,GROUP5), /* SCI5 ERI (Receive error) */
            [14] = BSP_PRV_VECT_ENUM(EVENT_GLCDC_LINE_DETECT,GROUP6), /* GLCDC LINE DETECT (Specified line) */
            [15] = BSP_PRV_VECT_ENUM(EVENT_MIPIDSI_SEQ0,GROUP7), /* MIPIDSI SEQ0 (Sequence operation channel 0 interrupt) */
            [16] = BSP_PRV_VECT_ENUM(EVENT_MIPIDSI_SEQ1,GROUP0), /* MIPIDSI SEQ1 (Sequence operation channel 1 interrupt) */
            [17] = BSP_PRV_VECT_ENUM(EVENT_MIPIDSI_VIN1,GROUP1), /* MIPIDSI VIN1 (Video-Input operation channel1 interrupt) */
            [18] = BSP_PRV_VECT_ENUM(EVENT_MIPIDSI_RCV,GROUP2), /* MIPIDSI RCV (DSI packet receive interrupt) */
            [19] = BSP_PRV_VECT_ENUM(EVENT_MIPIDSI_FERR,GROUP3), /* MIPIDSI FERR (DSI fatal error interrupt) */
            [20] = BSP_PRV_VECT_ENUM(EVENT_MIPIDSI_PPI,GROUP4), /* MIPIDSI PPI (DSI D-PHY PPI interrupt) */
            [21] = BSP_PRV_VECT_ENUM(EVENT_SCI9_RXI,GROUP5), /* SCI9 RXI (Receive data full) */
            [22] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TXI,GROUP6), /* SCI9 TXI (Transmit data empty) */
            [23] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TEI,GROUP7), /* SCI9 TEI (Transmit end) */
            [24] = BSP_PRV_VECT_ENUM(EVENT_SCI9_ERI,GROUP0), /* SCI9 ERI (Receive error) */
            [25] = BSP_PRV_VECT_ENUM(EVENT_GPT4_COUNTER_OVERFLOW,GROUP1), /* GPT4 COUNTER OVERFLOW (Overflow) */
            [26] = BSP_PRV_VECT_ENUM(EVENT_GPT7_COUNTER_OVERFLOW,GROUP2), /* GPT7 COUNTER OVERFLOW (Overflow) */
            [27] = BSP_PRV_VECT_ENUM(EVENT_SCI2_TEI,GROUP3), /* SCI2 TEI (Transmit end) */
            [28] = BSP_PRV_VECT_ENUM(EVENT_SCI2_ERI,GROUP4), /* SCI2 ERI (Receive error) */
            [29] = BSP_PRV_VECT_ENUM(EVENT_DMAC0_INT,GROUP5), /* DMAC0 INT (DMAC0 transfer end) */
            [30] = BSP_PRV_VECT_ENUM(EVENT_DMAC1_INT,GROUP6), /* DMAC1 INT (DMAC1 transfer end) */
            [31] = BSP_PRV_VECT_ENUM(EVENT_SCI4_RXI,GROUP7), /* SCI4 RXI (Receive data full) */
            [32] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TXI,FIXED), /* SCI4 TXI (Transmit data empty) */
            [33] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TEI,FIXED), /* SCI4 TEI (Transmit end) */
            [34] = BSP_PRV_VECT_ENUM(EVENT_SCI4_ERI,FIXED), /* SCI4 ERI (Receive error) */
            [35] = BSP_PRV_VECT_ENUM(EVENT_SCI3_RXI,FIXED), /* SCI3 RXI (Receive data full) */
            [36] = BSP_PRV_VECT_ENUM(EVENT_SCI3_TXI,FIXED), /* SCI3 TXI (Transmit data empty) */
            [37] = BSP_PRV_VECT_ENUM(EVENT_SCI3_TEI,FIXED), /* SCI3 TEI (Transmit end) */
            [38] = BSP_PRV_VECT_ENUM(EVENT_SCI3_ERI,FIXED), /* SCI3 ERI (Receive error) */
            [39] = BSP_PRV_VECT_ENUM(EVENT_SCI6_RXI,FIXED), /* SCI6 RXI (Receive data full) */
            [40] = BSP_PRV_VECT_ENUM(EVENT_SCI6_ERI,FIXED), /* SCI6 ERI (Receive error) */
            [41] = BSP_PRV_VECT_ENUM(EVENT_SCI8_RXI,FIXED), /* SCI8 RXI (Receive data full) */
            [42] = BSP_PRV_VECT_ENUM(EVENT_SCI8_TXI,FIXED), /* SCI8 TXI (Transmit data empty) */
            [43] = BSP_PRV_VECT_ENUM(EVENT_SCI8_TEI,FIXED), /* SCI8 TEI (Transmit end) */
            [44] = BSP_PRV_VECT_ENUM(EVENT_SCI8_ERI,FIXED), /* SCI8 ERI (Receive error) */
            [45] = BSP_PRV_VECT_ENUM(EVENT_NPU_IRQ,FIXED), /* NPU IRQ (NPU IRQ) */
        };
        #endif
        #endif