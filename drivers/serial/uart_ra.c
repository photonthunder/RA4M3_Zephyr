/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra_uart

#include <device.h>
#include <errno.h>
#include <init.h>
#include <sys/__assert.h>
#include <soc.h>
#include <soc_ioport.h>
#include <renesas_ra_dt.h>
#include <drivers/uart.h>
#include <drivers/dma.h>
#include <string.h>
#include <r_ioport_api.h>


#define SCI_SMR_CKS_VALUE_MASK                  (0x03U) ///< CKS: 2 bits
#define SCI_UART_SCMR_DEFAULT_VALUE             (0xF2U)
#define SCI_UART_BRR_DEFAULT_VALUE              (0xFFU)
#define SCI_UART_MDDR_DEFAULT_VALUE             (0xFFU)
#define SCI_UART_FCR_DEFAULT_VALUE              (0xF800)
#define SCI_UART_DCCR_DEFAULT_VALUE             (0x40U)

#define SPTR_SPB2D_BIT                          (1U)
#define SPTR_OUTPUT_ENABLE_MASK                 (0x04U)

/* Device constant configuration parameters */
struct uart_ra_dev_cfg {
	R_SCI0_Type *regs;
	uint8_t instance_idx;
	uint32_t baudrate;
#if CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
	//uart_irq_config_func_t	irq_config_func;
#endif
	uint32_t num_pins;
	struct soc_port_pin pins[];
};

/* Device run time data */
struct uart_ra_dev_data {
	struct uart_config config_cache;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;
	void *cb_data;
#endif
};

#if CONFIG_UART_INTERRUPT_DRIVEN

#define RCI_EVENT_NUM  (DT_INST_IRQ_BY_IDX(n, 0, irq))
#define TXI_EVENT_NUM  (DT_INST_IRQ_BY_IDX(n, 1, irq))
#define TEI_EVENT_NUM  (DT_INST_IRQ_BY_IDX(n, 2, irq))
#define ERI_EVENT_NUM  (DT_INST_IRQ_BY_IDX(n, 3, irq))

#endif

#define DEV_CFG(dev) ((const struct uart_ra_dev_cfg *const)(dev)->config)
#define DEV_DATA(dev) ((struct uart_ra_dev_data * const)(dev)->data)

static int baudrate_set(R_SCI0_Type *const uart, uint32_t baudrate)
{
	//_ASSERT(baudrate,"baud rate has to be bigger than 0");
	//_ASSERT(uart->SCR != 0, "RE, RIE and TE must be off" );

	uint8_t brrValue;

	/* Assumes SEMR bits BGDM, ABCS and ABCS E are 0 */
	/* and that SMR CKS settings are 0 or PCLK (no divisor) */
	uart->SMR_b.CKS = (uint8_t) (SCI_SMR_CKS_VALUE_MASK & 0);
	brrValue = (uint8_t) (RA4M3_CPU_CLOCK_FREQ_HZ / (32 * baudrate)) - 1;
	uart->BRR = brrValue;
	uart->MDDR = SCI_UART_MDDR_DEFAULT_VALUE;
	//LOG_DBG("BRR = %d", brrValue);
	return 0;
}

static int uart_ra_init(const struct device *dev)
{
	const struct uart_ra_dev_cfg *const cfg = DEV_CFG(dev);
	R_SCI0_Type *const uart = cfg->regs;

    /* Initialize registers as defined in section 34.3.7 "SCI Initialization in Asynchronous Mode" */
	/* Page 868 of the RA4M3 datasheet R01UH0893EJ0120 Rev.1.20 */
    uart->SCR   = 0U;
	uart->FCR_b.FM = 0;
	uart->SIMR1 = 0U;
	uart->SIMR2 = 0U;
    uart->SIMR3 = 0U;
	uart->SPMR = 0U;
	uart->SNFR = 0U;
	/* If fifo support is disabled and the current channel supports fifo make sure it's disabled. */
    if (BSP_FEATURE_SCI_UART_FIFO_CHANNELS & (1U << cfg->instance_idx))
    {
        uart->FCR = SCI_UART_FCR_DEFAULT_VALUE;
    }
	/* Configure parity and stop bits. */
    // uint32_t smr  = (((uint32_t) p_cfg->parity << 4U) | ((uint32_t) p_cfg->stop_bits << 3U));
	uart->SMR = 0U;
    uart->SCMR = SCI_UART_SCMR_DEFAULT_VALUE;
	/* Detect Falling Edge */
	uart->SEMR = 0x80;
	uart->SSR = 0;
	uart->CDR = 0;
	uart->DCCR = 0;
	/* Set the default level of the TX pin to 1. */
    uart->SPTR = (uint8_t) (1U << SPTR_SPB2D_BIT) | SPTR_OUTPUT_ENABLE_MASK;
	uart->ACTR = 0;

	baudrate_set(uart, cfg->baudrate);

	soc_ioport_list_configure(cfg->pins, cfg->num_pins);

	/* Enable Receive and Transmit */
	uart->SCR_b.RE = 1;
	uart->SCR_b.TE = 1;


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int uart_ra_poll_in(const struct device *dev, unsigned char *c)
{
	R_SCI0_Type *const uart = DEV_CFG(dev)->regs;

	if (uart->SSR_b.RDRF != 0) {
		return -EBUSY;
	}

	/* got a character */
	*c = (unsigned char)uart->RDR;
	return 0;
}

static void uart_ra_poll_out(const struct device *dev, unsigned char c)
{
	R_SCI0_Type *const uart = DEV_CFG(dev)->regs;

	/* Wait for transmitter to be ready */
	while (uart->SSR_b.TEND == 0) {
	}

	/* send a character */
	uart->TDR = c;
}

static int uart_ra_err_check(const struct device *dev)
{
	R_SCI0_Type * const uart = DEV_CFG(dev)->regs;
	int errors = 0;

	if (uart->SSR_b.ORER) {
		errors |= UART_ERROR_OVERRUN;
		uart->SSR_b.ORER = 0;
	}

	if (uart->SSR_b.PER) {
		errors |= UART_ERROR_PARITY;
		uart->SSR_b.PER = 0;
	}

	if (uart->SSR_b.FER) {
		errors |= UART_ERROR_FRAMING;
		uart->SSR_b.FER = 0;
	}

	return errors;
}

#if CONFIG_UART_INTERRUPT_DRIVEN
static void uart_ra_isr(const struct device *dev)
{
	struct uart_ra_dev_data *const dev_data = DEV_DATA(dev);

	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}
}
#endif

#if CONFIG_UART_INTERRUPT_DRIVEN

static int uart_ra_fifo_fill(const struct device *dev,
			       const uint8_t *tx_data, int len)
{
	R_SCI0_Type *uart = DEV_CFG(dev)->regs;


	if (uart->SSR_b.TEND && len >= 1) {
		uart->TDR = tx_data[0];
		return 1;
	} else {
		return 0;
	}
}

static int uart_ra_fifo_read(const struct device *dev, uint8_t *rx_data,
			      const int size)
{
	R_SCI0_Type *uart = DEV_CFG(dev)->regs;

	if (uart->SSR_b.RDRF) {
		uint8_t ch = uart->RDR;

		if (size >= 1) {
			*rx_data = ch;
			return 1;
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

static void uart_ra_irq_tx_complete_disable(const struct device *dev)
{
	R_SCI0_Type * const uart = DEV_CFG(dev)->regs;

	uart->SCR_b.TIEE = 0;

}

static void uart_ra_irq_tx_complete_enable(const struct device *dev)
{
	R_SCI0_Type * const uart = DEV_CFG(dev)->regs;

	uart->SCR_b.TIEE = 1;

}

static void uart_ra_irq_tx_enable(const struct device *dev)
{
	R_SCI0_Type * const uart = DEV_CFG(dev)->regs;

	uart->SCR_b.TIE = 1;
}

static void uart_ra_irq_tx_disable(const struct device *dev)
{
	R_SCI0_Type * const uart = DEV_CFG(dev)->regs;

	uart->SCR_b.TIE = 0;
}

static int uart_ra_irq_tx_ready(const struct device *dev)
{
	R_SCI0_Type * const uart = DEV_CFG(dev)->regs;

	return (uart->SSR_b.TEND != 0) && (art->SCR_b.TIE != 0);
}

static int uart_ra_irq_tx_complete(const struct device *dev)
{
	R_SCI0_Type *const uart = DEV_CFG(dev)->regs;

	return (uart->SSR_b.TEND != 0) && (art->SCR_b.TIEE != 0);
}

static void uart_ra_irq_rx_enable(const struct device *dev)
{
	R_SCI0_Type * const uart = DEV_CFG(dev)->regs;

	uart->SCR_b.RIE = 1;
}

static void uart_ra_irq_rx_disable(const struct device *dev)
{
	R_SCI0_Type * const uart = DEV_CFG(dev)->regs;

	uart->SCR_b.RIE = 0;
}

static int uart_ra_irq_rx_ready(const struct device *dev)
{
	R_SCI0_Type * const uart = DEV_CFG(dev)->regs;

	return (uart->SSR_b.RDRF != 0) && (art->SCR_b.RIE != 0);
}

static int uart_ra_irq_is_pending(const struct device *dev)
{
	return (R_ICU->IELSR_b[RCI_EVENT_NUM].IR || R_ICU->IELSR_b[TXI_EVENT_NUM].IR ||
			R_ICU->IELSR_b[TEI_EVENT_NUM].IR);
}

static int uart_ra_irq_update(const struct device *dev)
{
	/* Clear sticky interrupts */
	R_ICU->IELSR_b[RCI_EVENT_NUM].IR = 0;
	R_ICU->IELSR_b[TXI_EVENT_NUM].IR = 0;
	R_ICU->IELSR_b[TEI_EVENT_NUM].IR = 0;
	R_ICU->IELSR_b[ERI_EVENT_NUM].IR = 0;
	return 1;
}

static void uart_ra_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_ra_dev_data *const dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
}
#endif

static const struct uart_driver_api uart_ra_driver_api = {
	.poll_in = uart_ra_poll_in,
	.poll_out = uart_ra_poll_out,
	.err_check = uart_ra_err_check,
#if CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_ra_fifo_fill,
	.fifo_read = uart_ra_fifo_read,
	.irq_tx_complete_disable = uart_ra_irq_tx_complete_disable,
	.irq_tx_complete_enable = uart_ra_irq_tx_complete_enable,
	.irq_tx_enable = uart_ra_irq_tx_enable,
	.irq_tx_disable = uart_ra_irq_tx_disable,
	.irq_tx_ready = uart_ra_irq_tx_ready,
	.irq_tx_complete = uart_ra_irq_tx_complete,
	.irq_rx_enable = uart_ra_irq_rx_enable,
	.irq_rx_disable = uart_ra_irq_rx_disable,
	.irq_rx_ready = uart_ra_irq_rx_ready,
	.irq_is_pending = uart_ra_irq_is_pending,
	.irq_update = uart_ra_irq_update,
	.irq_callback_set = uart_ra_irq_callback_set,
#endif
};

#if CONFIG_UART_INTERRUPT_DRIVEN

#define RA_UART_IRQ_CONNECT(n, m)					\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq),		\
			    DT_INST_IRQ_BY_IDX(n, m, priority),		\
			    uart_ra_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));		\
	} while (0)

#define UART_RA_IRQ_HANDLER_DECL(n)					\
	static void uart_ra_irq_config_##n(const struct device *dev)
#define UART_RA_IRQ_HANDLER_FUNC(n)					\
	.irq_config_func = uart_ra_irq_config_##n,

#if DT_INST_IRQ_HAS_IDX(0, 3)
#define UART_RA_IRQ_HANDLER(n)					\
static void uart_ra_irq_config_##n(const struct device *dev)		\
{									\
	RA_UART_IRQ_CONNECT(n, 0);					\
	RA_UART_IRQ_CONNECT(n, 1);					\
	RA_UART_IRQ_CONNECT(n, 2);					\
	RA_UART_IRQ_CONNECT(n, 3);					\
}
#else
#define UART_RA_IRQ_HANDLER(n)					\
static void uart_ra_irq_config_##n(const struct device *dev)		\
{									\
	RA_UART_IRQ_CONNECT(n, 0);					\
}
#endif
#else
#define UART_RA_IRQ_HANDLER_DECL(n)
#define UART_RA_IRQ_HANDLER_FUNC(n)
#define UART_RA_IRQ_HANDLER(n)
#endif

#define UART_RA_DMA_CHANNELS(n)

#define UART_RA_CONFIG_DEFN(n)									\
static const struct uart_ra_dev_cfg uart_ra_config_##n = {		\
	.regs = (R_SCI0_Type *)DT_INST_REG_ADDR(n),					\
	.instance_idx = DT_INST_PROP(n, instance_idx), 				\
	.baudrate = DT_INST_PROP(n, current_speed),					\
	.num_pins = RENESAS_RA_DT_INST_NUM_PINS(n),					\
	.pins = RENESAS_RA_DT_INST_PINS(n),							\
	UART_RA_IRQ_HANDLER_FUNC(n)									\
	UART_RA_DMA_CHANNELS(n)										\
}

#define UART_RA_DEVICE_INIT(n)						\
static struct uart_ra_dev_data uart_ra_data_##n;	\
UART_RA_IRQ_HANDLER_DECL(n);						\
UART_RA_CONFIG_DEFN(n);								\
DEVICE_DT_INST_DEFINE(n, uart_ra_init, NULL,		\
		    &uart_ra_data_##n,						\
		    &uart_ra_config_##n, PRE_KERNEL_1,		\
		    CONFIG_SERIAL_INIT_PRIORITY,			\
		    &uart_ra_driver_api);					\
UART_RA_IRQ_HANDLER(n) 

DT_INST_FOREACH_STATUS_OKAY(UART_RA_DEVICE_INIT)