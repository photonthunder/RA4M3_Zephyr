/* generated vector source file - do not edit */
#include <bsp_api.h>
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = iic_master_rxi_isr, /* IIC0 RXI (Receive data full) */
            [1] = iic_master_txi_isr, /* IIC0 TXI (Transmit data empty) */
            [2] = iic_master_tei_isr, /* IIC0 TEI (Transmit end) */
            [3] = iic_master_eri_isr, /* IIC0 ERI (Transfer error) */
            [4] = sci_spi_rxi_isr, /* SCI0 RXI (Receive data full) */
            [5] = sci_spi_txi_isr, /* SCI0 TXI (Transmit data empty) */
            [6] = sci_spi_tei_isr, /* SCI0 TEI (Transmit end) */
            [7] = sci_spi_eri_isr, /* SCI0 ERI (Receive error) */
            [8] = sci_spi_rxi_isr, /* SCI3 RXI (Received data full) */
            [9] = sci_spi_txi_isr, /* SCI3 TXI (Transmit data empty) */
            [10] = sci_spi_tei_isr, /* SCI3 TEI (Transmit end) */
            [11] = sci_spi_eri_isr, /* SCI3 ERI (Receive error) */
            [12] = sci_uart_rxi_isr, /* SCI1 RXI (Received data full) */
            [13] = sci_uart_txi_isr, /* SCI1 TXI (Transmit data empty) */
            [14] = sci_uart_tei_isr, /* SCI1 TEI (Transmit end) */
            [15] = sci_uart_eri_isr, /* SCI1 ERI (Receive error) */
            [16] = sci_uart_rxi_isr, /* SCI2 RXI (Received data full) */
            [17] = sci_uart_txi_isr, /* SCI2 TXI (Transmit data empty) */
            [18] = sci_uart_tei_isr, /* SCI2 TEI (Transmit end) */
            [19] = sci_uart_eri_isr, /* SCI2 ERI (Receive error) */
            [20] = sci_uart_rxi_isr, /* SCI4 RXI (Received data full) */
            [21] = sci_uart_txi_isr, /* SCI4 TXI (Transmit data empty) */
            [22] = sci_uart_tei_isr, /* SCI4 TEI (Transmit end) */
            [23] = sci_uart_eri_isr, /* SCI4 ERI (Receive error) */
            [24] = sci_uart_rxi_isr, /* SCI9 RXI (Received data full) */
            [25] = sci_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [26] = sci_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [27] = sci_uart_eri_isr, /* SCI9 ERI (Receive error) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_IIC0_RXI), /* IIC0 RXI (Receive data full) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_IIC0_TXI), /* IIC0 TXI (Transmit data empty) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_IIC0_TEI), /* IIC0 TEI (Transmit end) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_IIC0_ERI), /* IIC0 ERI (Transfer error) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_SCI0_RXI), /* SCI0 RXI (Receive data full) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_SCI0_TXI), /* SCI0 TXI (Transmit data empty) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_SCI0_TEI), /* SCI0 TEI (Transmit end) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_SCI0_ERI), /* SCI0 ERI (Receive error) */
            [8] = BSP_PRV_IELS_ENUM(EVENT_SCI3_RXI), /* SCI3 RXI (Received data full) */
            [9] = BSP_PRV_IELS_ENUM(EVENT_SCI3_TXI), /* SCI3 TXI (Transmit data empty) */
            [10] = BSP_PRV_IELS_ENUM(EVENT_SCI3_TEI), /* SCI3 TEI (Transmit end) */
            [11] = BSP_PRV_IELS_ENUM(EVENT_SCI3_ERI), /* SCI3 ERI (Receive error) */
            [12] = BSP_PRV_IELS_ENUM(EVENT_SCI1_RXI), /* SCI1 RXI (Received data full) */
            [13] = BSP_PRV_IELS_ENUM(EVENT_SCI1_TXI), /* SCI1 TXI (Transmit data empty) */
            [14] = BSP_PRV_IELS_ENUM(EVENT_SCI1_TEI), /* SCI1 TEI (Transmit end) */
            [15] = BSP_PRV_IELS_ENUM(EVENT_SCI1_ERI), /* SCI1 ERI (Receive error) */
            [16] = BSP_PRV_IELS_ENUM(EVENT_SCI2_RXI), /* SCI2 RXI (Received data full) */
            [17] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TXI), /* SCI2 TXI (Transmit data empty) */
            [18] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TEI), /* SCI2 TEI (Transmit end) */
            [19] = BSP_PRV_IELS_ENUM(EVENT_SCI2_ERI), /* SCI2 ERI (Receive error) */
            [20] = BSP_PRV_IELS_ENUM(EVENT_SCI4_RXI), /* SCI4 RXI (Received data full) */
            [21] = BSP_PRV_IELS_ENUM(EVENT_SCI4_TXI), /* SCI4 TXI (Transmit data empty) */
            [22] = BSP_PRV_IELS_ENUM(EVENT_SCI4_TEI), /* SCI4 TEI (Transmit end) */
            [23] = BSP_PRV_IELS_ENUM(EVENT_SCI4_ERI), /* SCI4 ERI (Receive error) */
            [24] = BSP_PRV_IELS_ENUM(EVENT_SCI9_RXI), /* SCI9 RXI (Received data full) */
            [25] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TXI), /* SCI9 TXI (Transmit data empty) */
            [26] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TEI), /* SCI9 TEI (Transmit end) */
            [27] = BSP_PRV_IELS_ENUM(EVENT_SCI9_ERI), /* SCI9 ERI (Receive error) */
        };
        #endif
