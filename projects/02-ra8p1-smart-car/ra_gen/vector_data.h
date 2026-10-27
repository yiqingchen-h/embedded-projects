/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
        #ifdef __cplusplus
        extern "C" {
        #endif
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (46)
        #endif
        /* ISR prototypes */
        void gpt_counter_overflow_isr(void);
        void sci_b_uart_txi_isr(void);
        void sci_b_uart_tei_isr(void);
        void ssi_txi_isr(void);
        void ssi_int_isr(void);
        void pdm_err_isr(void);
        void dmac_int_isr(void);
        void sci_b_spi_rxi_isr(void);
        void sci_b_spi_txi_isr(void);
        void sci_b_spi_tei_isr(void);
        void sci_b_spi_eri_isr(void);
        void glcdc_line_detect_isr(void);
        void mipi_dsi_seq0_isr(void);
        void mipi_dsi_seq1_isr(void);
        void mipi_dsi_vin1_isr(void);
        void mipi_dsi_rcv_isr(void);
        void mipi_dsi_ferr_isr(void);
        void mipi_dsi_ppi_isr(void);
        void sci_b_uart_rxi_isr(void);
        void sci_b_uart_eri_isr(void);
        void rm_ethosu_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW ((IRQn_Type) 0) /* GPT0 COUNTER OVERFLOW (Overflow) */
        #define GPT0_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 0) /* GPT0 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_GPT2_COUNTER_OVERFLOW ((IRQn_Type) 1) /* GPT2 COUNTER OVERFLOW (Overflow) */
        #define GPT2_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 1) /* GPT2 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_GPT11_COUNTER_OVERFLOW ((IRQn_Type) 2) /* GPT11 COUNTER OVERFLOW (Overflow) */
        #define GPT11_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 2) /* GPT11 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_GPT12_COUNTER_OVERFLOW ((IRQn_Type) 3) /* GPT12 COUNTER OVERFLOW (Overflow) */
        #define GPT12_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 3) /* GPT12 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_SCI6_TXI ((IRQn_Type) 4) /* SCI6 TXI (Transmit data empty) */
        #define SCI6_TXI_IRQn          ((IRQn_Type) 4) /* SCI6 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI6_TEI ((IRQn_Type) 5) /* SCI6 TEI (Transmit end) */
        #define SCI6_TEI_IRQn          ((IRQn_Type) 5) /* SCI6 TEI (Transmit end) */
        #define VECTOR_NUMBER_SSI1_TXI ((IRQn_Type) 6) /* SSI1 TXI (Receive data full/Transmit data empty) */
        #define SSI1_TXI_IRQn          ((IRQn_Type) 6) /* SSI1 TXI (Receive data full/Transmit data empty) */
        #define VECTOR_NUMBER_SSI1_INT ((IRQn_Type) 7) /* SSI1 INT (Error interrupt) */
        #define SSI1_INT_IRQn          ((IRQn_Type) 7) /* SSI1 INT (Error interrupt) */
        #define VECTOR_NUMBER_PDM_ERR0 ((IRQn_Type) 8) /* PDM ERR0 (Error detection interrupt channel 0) */
        #define PDM_ERR0_IRQn          ((IRQn_Type) 8) /* PDM ERR0 (Error detection interrupt channel 0) */
        #define VECTOR_NUMBER_DMAC2_INT ((IRQn_Type) 9) /* DMAC2 INT (DMAC2 transfer end) */
        #define DMAC2_INT_IRQn          ((IRQn_Type) 9) /* DMAC2 INT (DMAC2 transfer end) */
        #define VECTOR_NUMBER_SCI5_RXI ((IRQn_Type) 10) /* SCI5 RXI (Receive data full) */
        #define SCI5_RXI_IRQn          ((IRQn_Type) 10) /* SCI5 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI5_TXI ((IRQn_Type) 11) /* SCI5 TXI (Transmit data empty) */
        #define SCI5_TXI_IRQn          ((IRQn_Type) 11) /* SCI5 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI5_TEI ((IRQn_Type) 12) /* SCI5 TEI (Transmit end) */
        #define SCI5_TEI_IRQn          ((IRQn_Type) 12) /* SCI5 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI5_ERI ((IRQn_Type) 13) /* SCI5 ERI (Receive error) */
        #define SCI5_ERI_IRQn          ((IRQn_Type) 13) /* SCI5 ERI (Receive error) */
        #define VECTOR_NUMBER_GLCDC_LINE_DETECT ((IRQn_Type) 14) /* GLCDC LINE DETECT (Specified line) */
        #define GLCDC_LINE_DETECT_IRQn          ((IRQn_Type) 14) /* GLCDC LINE DETECT (Specified line) */
        #define VECTOR_NUMBER_MIPIDSI_SEQ0 ((IRQn_Type) 15) /* MIPIDSI SEQ0 (Sequence operation channel 0 interrupt) */
        #define MIPIDSI_SEQ0_IRQn          ((IRQn_Type) 15) /* MIPIDSI SEQ0 (Sequence operation channel 0 interrupt) */
        #define VECTOR_NUMBER_MIPIDSI_SEQ1 ((IRQn_Type) 16) /* MIPIDSI SEQ1 (Sequence operation channel 1 interrupt) */
        #define MIPIDSI_SEQ1_IRQn          ((IRQn_Type) 16) /* MIPIDSI SEQ1 (Sequence operation channel 1 interrupt) */
        #define VECTOR_NUMBER_MIPIDSI_VIN1 ((IRQn_Type) 17) /* MIPIDSI VIN1 (Video-Input operation channel1 interrupt) */
        #define MIPIDSI_VIN1_IRQn          ((IRQn_Type) 17) /* MIPIDSI VIN1 (Video-Input operation channel1 interrupt) */
        #define VECTOR_NUMBER_MIPIDSI_RCV ((IRQn_Type) 18) /* MIPIDSI RCV (DSI packet receive interrupt) */
        #define MIPIDSI_RCV_IRQn          ((IRQn_Type) 18) /* MIPIDSI RCV (DSI packet receive interrupt) */
        #define VECTOR_NUMBER_MIPIDSI_FERR ((IRQn_Type) 19) /* MIPIDSI FERR (DSI fatal error interrupt) */
        #define MIPIDSI_FERR_IRQn          ((IRQn_Type) 19) /* MIPIDSI FERR (DSI fatal error interrupt) */
        #define VECTOR_NUMBER_MIPIDSI_PPI ((IRQn_Type) 20) /* MIPIDSI PPI (DSI D-PHY PPI interrupt) */
        #define MIPIDSI_PPI_IRQn          ((IRQn_Type) 20) /* MIPIDSI PPI (DSI D-PHY PPI interrupt) */
        #define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 21) /* SCI9 RXI (Receive data full) */
        #define SCI9_RXI_IRQn          ((IRQn_Type) 21) /* SCI9 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 22) /* SCI9 TXI (Transmit data empty) */
        #define SCI9_TXI_IRQn          ((IRQn_Type) 22) /* SCI9 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 23) /* SCI9 TEI (Transmit end) */
        #define SCI9_TEI_IRQn          ((IRQn_Type) 23) /* SCI9 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 24) /* SCI9 ERI (Receive error) */
        #define SCI9_ERI_IRQn          ((IRQn_Type) 24) /* SCI9 ERI (Receive error) */
        #define VECTOR_NUMBER_GPT4_COUNTER_OVERFLOW ((IRQn_Type) 25) /* GPT4 COUNTER OVERFLOW (Overflow) */
        #define GPT4_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 25) /* GPT4 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_GPT7_COUNTER_OVERFLOW ((IRQn_Type) 26) /* GPT7 COUNTER OVERFLOW (Overflow) */
        #define GPT7_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 26) /* GPT7 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_SCI2_TEI ((IRQn_Type) 27) /* SCI2 TEI (Transmit end) */
        #define SCI2_TEI_IRQn          ((IRQn_Type) 27) /* SCI2 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI2_ERI ((IRQn_Type) 28) /* SCI2 ERI (Receive error) */
        #define SCI2_ERI_IRQn          ((IRQn_Type) 28) /* SCI2 ERI (Receive error) */
        #define VECTOR_NUMBER_DMAC0_INT ((IRQn_Type) 29) /* DMAC0 INT (DMAC0 transfer end) */
        #define DMAC0_INT_IRQn          ((IRQn_Type) 29) /* DMAC0 INT (DMAC0 transfer end) */
        #define VECTOR_NUMBER_DMAC1_INT ((IRQn_Type) 30) /* DMAC1 INT (DMAC1 transfer end) */
        #define DMAC1_INT_IRQn          ((IRQn_Type) 30) /* DMAC1 INT (DMAC1 transfer end) */
        #define VECTOR_NUMBER_SCI4_RXI ((IRQn_Type) 31) /* SCI4 RXI (Receive data full) */
        #define SCI4_RXI_IRQn          ((IRQn_Type) 31) /* SCI4 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI4_TXI ((IRQn_Type) 32) /* SCI4 TXI (Transmit data empty) */
        #define SCI4_TXI_IRQn          ((IRQn_Type) 32) /* SCI4 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI4_TEI ((IRQn_Type) 33) /* SCI4 TEI (Transmit end) */
        #define SCI4_TEI_IRQn          ((IRQn_Type) 33) /* SCI4 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI4_ERI ((IRQn_Type) 34) /* SCI4 ERI (Receive error) */
        #define SCI4_ERI_IRQn          ((IRQn_Type) 34) /* SCI4 ERI (Receive error) */
        #define VECTOR_NUMBER_SCI3_RXI ((IRQn_Type) 35) /* SCI3 RXI (Receive data full) */
        #define SCI3_RXI_IRQn          ((IRQn_Type) 35) /* SCI3 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI3_TXI ((IRQn_Type) 36) /* SCI3 TXI (Transmit data empty) */
        #define SCI3_TXI_IRQn          ((IRQn_Type) 36) /* SCI3 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI3_TEI ((IRQn_Type) 37) /* SCI3 TEI (Transmit end) */
        #define SCI3_TEI_IRQn          ((IRQn_Type) 37) /* SCI3 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI3_ERI ((IRQn_Type) 38) /* SCI3 ERI (Receive error) */
        #define SCI3_ERI_IRQn          ((IRQn_Type) 38) /* SCI3 ERI (Receive error) */
        #define VECTOR_NUMBER_SCI6_RXI ((IRQn_Type) 39) /* SCI6 RXI (Receive data full) */
        #define SCI6_RXI_IRQn          ((IRQn_Type) 39) /* SCI6 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI6_ERI ((IRQn_Type) 40) /* SCI6 ERI (Receive error) */
        #define SCI6_ERI_IRQn          ((IRQn_Type) 40) /* SCI6 ERI (Receive error) */
        #define VECTOR_NUMBER_SCI8_RXI ((IRQn_Type) 41) /* SCI8 RXI (Receive data full) */
        #define SCI8_RXI_IRQn          ((IRQn_Type) 41) /* SCI8 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI8_TXI ((IRQn_Type) 42) /* SCI8 TXI (Transmit data empty) */
        #define SCI8_TXI_IRQn          ((IRQn_Type) 42) /* SCI8 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI8_TEI ((IRQn_Type) 43) /* SCI8 TEI (Transmit end) */
        #define SCI8_TEI_IRQn          ((IRQn_Type) 43) /* SCI8 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI8_ERI ((IRQn_Type) 44) /* SCI8 ERI (Receive error) */
        #define SCI8_ERI_IRQn          ((IRQn_Type) 44) /* SCI8 ERI (Receive error) */
        #define VECTOR_NUMBER_NPU_IRQ ((IRQn_Type) 45) /* NPU IRQ (NPU IRQ) */
        #define NPU_IRQ_IRQn          ((IRQn_Type) 45) /* NPU IRQ (NPU IRQ) */
        /* The number of entries required for the ICU vector table. */
        #define BSP_ICU_VECTOR_NUM_ENTRIES (46)

        #ifdef __cplusplus
        }
        #endif
        #endif /* VECTOR_DATA_H */