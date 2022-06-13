/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#include <bsp_arm_exceptions.h>
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (28)
#endif
/* ISR prototypes */
void iic_master_rxi_isr(void);
void iic_master_txi_isr(void);
void iic_master_tei_isr(void);
void iic_master_eri_isr(void);
void sci_spi_rxi_isr(void);
void sci_spi_txi_isr(void);
void sci_spi_tei_isr(void);
void sci_spi_eri_isr(void);
void sci_uart_rxi_isr(void);
void sci_uart_txi_isr(void);
void sci_uart_tei_isr(void);
void sci_uart_eri_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_IIC0_RXI ((IRQn_Type) 0) /* IIC0 RXI (Receive data full) */
#define IIC0_RXI_IRQn          ((IRQn_Type) 0) /* IIC0 RXI (Receive data full) */
#define VECTOR_NUMBER_IIC0_TXI ((IRQn_Type) 1) /* IIC0 TXI (Transmit data empty) */
#define IIC0_TXI_IRQn          ((IRQn_Type) 1) /* IIC0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_IIC0_TEI ((IRQn_Type) 2) /* IIC0 TEI (Transmit end) */
#define IIC0_TEI_IRQn          ((IRQn_Type) 2) /* IIC0 TEI (Transmit end) */
#define VECTOR_NUMBER_IIC0_ERI ((IRQn_Type) 3) /* IIC0 ERI (Transfer error) */
#define IIC0_ERI_IRQn          ((IRQn_Type) 3) /* IIC0 ERI (Transfer error) */
#define VECTOR_NUMBER_SCI0_RXI ((IRQn_Type) 4) /* SCI0 RXI (Receive data full) */
#define SCI0_RXI_IRQn          ((IRQn_Type) 4) /* SCI0 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI0_TXI ((IRQn_Type) 5) /* SCI0 TXI (Transmit data empty) */
#define SCI0_TXI_IRQn          ((IRQn_Type) 5) /* SCI0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI0_TEI ((IRQn_Type) 6) /* SCI0 TEI (Transmit end) */
#define SCI0_TEI_IRQn          ((IRQn_Type) 6) /* SCI0 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI0_ERI ((IRQn_Type) 7) /* SCI0 ERI (Receive error) */
#define SCI0_ERI_IRQn          ((IRQn_Type) 7) /* SCI0 ERI (Receive error) */
#define VECTOR_NUMBER_SCI3_RXI ((IRQn_Type) 8) /* SCI3 RXI (Received data full) */
#define SCI3_RXI_IRQn          ((IRQn_Type) 8) /* SCI3 RXI (Received data full) */
#define VECTOR_NUMBER_SCI3_TXI ((IRQn_Type) 9) /* SCI3 TXI (Transmit data empty) */
#define SCI3_TXI_IRQn          ((IRQn_Type) 9) /* SCI3 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI3_TEI ((IRQn_Type) 10) /* SCI3 TEI (Transmit end) */
#define SCI3_TEI_IRQn          ((IRQn_Type) 10) /* SCI3 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI3_ERI ((IRQn_Type) 11) /* SCI3 ERI (Receive error) */
#define SCI3_ERI_IRQn          ((IRQn_Type) 11) /* SCI3 ERI (Receive error) */
#define VECTOR_NUMBER_SCI1_RXI ((IRQn_Type) 12) /* SCI1 RXI (Received data full) */
#define SCI1_RXI_IRQn          ((IRQn_Type) 12) /* SCI1 RXI (Received data full) */
#define VECTOR_NUMBER_SCI1_TXI ((IRQn_Type) 13) /* SCI1 TXI (Transmit data empty) */
#define SCI1_TXI_IRQn          ((IRQn_Type) 13) /* SCI1 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI1_TEI ((IRQn_Type) 14) /* SCI1 TEI (Transmit end) */
#define SCI1_TEI_IRQn          ((IRQn_Type) 14) /* SCI1 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI1_ERI ((IRQn_Type) 15) /* SCI1 ERI (Receive error) */
#define SCI1_ERI_IRQn          ((IRQn_Type) 15) /* SCI1 ERI (Receive error) */
#define VECTOR_NUMBER_SCI2_RXI ((IRQn_Type) 16) /* SCI2 RXI (Received data full) */
#define SCI2_RXI_IRQn          ((IRQn_Type) 16) /* SCI2 RXI (Received data full) */
#define VECTOR_NUMBER_SCI2_TXI ((IRQn_Type) 17) /* SCI2 TXI (Transmit data empty) */
#define SCI2_TXI_IRQn          ((IRQn_Type) 17) /* SCI2 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI2_TEI ((IRQn_Type) 18) /* SCI2 TEI (Transmit end) */
#define SCI2_TEI_IRQn          ((IRQn_Type) 18) /* SCI2 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI2_ERI ((IRQn_Type) 19) /* SCI2 ERI (Receive error) */
#define SCI2_ERI_IRQn          ((IRQn_Type) 19) /* SCI2 ERI (Receive error) */
#define VECTOR_NUMBER_SCI4_RXI ((IRQn_Type) 20) /* SCI4 RXI (Received data full) */
#define SCI4_RXI_IRQn          ((IRQn_Type) 20) /* SCI4 RXI (Received data full) */
#define VECTOR_NUMBER_SCI4_TXI ((IRQn_Type) 21) /* SCI4 TXI (Transmit data empty) */
#define SCI4_TXI_IRQn          ((IRQn_Type) 21) /* SCI4 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI4_TEI ((IRQn_Type) 22) /* SCI4 TEI (Transmit end) */
#define SCI4_TEI_IRQn          ((IRQn_Type) 22) /* SCI4 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI4_ERI ((IRQn_Type) 23) /* SCI4 ERI (Receive error) */
#define SCI4_ERI_IRQn          ((IRQn_Type) 23) /* SCI4 ERI (Receive error) */
#define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 24) /* SCI9 RXI (Received data full) */
#define SCI9_RXI_IRQn          ((IRQn_Type) 24) /* SCI9 RXI (Received data full) */
#define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 25) /* SCI9 TXI (Transmit data empty) */
#define SCI9_TXI_IRQn          ((IRQn_Type) 25) /* SCI9 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 26) /* SCI9 TEI (Transmit end) */
#define SCI9_TEI_IRQn          ((IRQn_Type) 26) /* SCI9 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 27) /* SCI9 ERI (Receive error) */
#define SCI9_ERI_IRQn          ((IRQn_Type) 27) /* SCI9 ERI (Receive error) */
#endif /* VECTOR_DATA_H */

