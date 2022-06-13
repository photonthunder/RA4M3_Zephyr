/*
 * Copyright (c) 2020 MXT Creation
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra_sci

/** @file
 * @brief UART driver for Renesas RA MCU family.
 *
 * Note:
 * - Error handling is not implemented.
 * - The driver works only in polling mode, interrupt mode is not implemented.
 */

#include <errno.h>
#include <sys/__assert.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <drivers/uart.h>
#include <soc_ioport.h>
#if (CONFIG_RENESAS_BSP_API)
#include <bsp_api.h>
#else
#include <bsp_feature.h>
#include <renesas.h>
#include <fsp_features.h>
#include <bsp_common.h>
#include <bsp_module_stop.h>
#endif

/** UART Data bit length definition */
typedef enum e_uart_data_bits
{
    UART_DATA_BITS_8,                  ///< Data bits 8-bit
    UART_DATA_BITS_7,                  ///< Data bits 7-bit
    UART_DATA_BITS_9                   ///< Data bits 9-bit
} uart_data_bits_t;

/** UART Parity definition */
typedef enum e_uart_parity
{
    UART_PARITY_OFF  = 0U,             ///< No parity
    UART_PARITY_EVEN = 2U,             ///< Even parity
    UART_PARITY_ODD  = 3U,             ///< Odd parity
} uart_parity_t;

/** UART Stop bits definition */
typedef enum e_uart_stop_bits
{
    UART_STOP_BITS_1 = 0U,             ///< Stop bit 1-bit
    UART_STOP_BITS_2 = 1U,             ///< Stop bits 2-bit
} uart_stop_bits_t;

/** Noise filter setting definition */
typedef enum e_noise_cancel_lvl
{
    NOISE_CANCEL_LVL1,                 /**< Noise filter level 1(weak) */
    NOISE_CANCEL_LVL2,                 /**< Noise filter level 2 */
    NOISE_CANCEL_LVL3,                 /**< Noise filter level 3 */
    NOISE_CANCEL_LVL4                  /**< Noise filter level 4(strong) */
} noise_cancel_lvl_t;

/** CTS/RTS function of the SSn pin. */
typedef enum e_sci_uart_ctsrts_config
{
    SCI_UART_CTSRTS_RTS_OUTPUT = 0x0,  ///< Disable CTS function (RTS output function is enabled)
    SCI_UART_CTSRTS_CTS_INPUT  = 0x1,  ///< Enable CTS function
} sci_uart_ctsrts_config_t;

/** Asynchronous Start Bit Edge Detection configuration. */
typedef enum e_sci_uart_start_bit_detect
{
    SCI_UART_START_BIT_LOW_LEVEL    = 0x0, ///< Detect low level on RXDn pin as start bit
    SCI_UART_START_BIT_FALLING_EDGE = 0x1, ///< Detect falling level on RXDn pin as start bit
} sci_uart_start_bit_detect_t;

/** Noise cancellation configuration. */
typedef enum e_sci_uart_noise_cancellation
{
    SCI_UART_NOISE_CANCELLATION_DISABLE = 0x0, ///< Disable noise cancellation
    SCI_UART_NOISE_CANCELLATION_ENABLE  = 0x1, ///< Enable noise cancellation
} sci_uart_noise_cancellation_t;

/** Enumeration for SCI clock source */
typedef enum e_sci_clk_src
{
    SCI_UART_CLOCK_INT,                      ///< Use internal clock for baud generation
    SCI_UART_CLOCK_INT_WITH_BAUDRATE_OUTPUT, ///< Use internal clock for baud generation and output on SCK
    SCI_UART_CLOCK_EXT8X,                    ///< Use external clock 8x baud rate
    SCI_UART_CLOCK_EXT16X                    ///< Use external clock 16x baud rate
} sci_clk_src_t;

/** SCI SCR register bit masks */
#define SCI_SCR_TEIE_MASK                       (0x04U) ///< Transmit End Interrupt Enable
#define SCI_SCR_RE_MASK                         (0x10U) ///< Receive Enable
#define SCI_SCR_TE_MASK                         (0x20U) ///< Transmit Enable
#define SCI_SCR_RIE_MASK                        (0x40U) ///< Receive Interrupt Enable
#define SCI_SCR_TIE_MASK                        (0x80U) ///< Transmit Interrupt Enable

#define SCI_UART_SCMR_DEFAULT_VALUE             (0xF2U)
#define SCI_UART_BRR_DEFAULT_VALUE              (0xFFU)
#define SCI_UART_MDDR_DEFAULT_VALUE             (0xFFU)
#define SCI_UART_FCR_DEFAULT_VALUE              (0xF800)
#define SCI_UART_DCCR_DEFAULT_VALUE             (0x40U)

#define FRDR_TDAT_MASK_9BITS                    (0x01FFU)
#define SPTR_SPB2D_BIT                          (1U)
#define SPTR_OUTPUT_ENABLE_MASK                 (0x04U)
#define SCI_UART_SPMR_CTSE_OFFSET               (1U)

/** SCI SEMR register bit offsets */
#define SCI_UART_SEMR_BRME_OFFSET               (2U)
#define SCI_UART_SEMR_ABCSE_OFFSET              (3U)
#define SCI_UART_SEMR_ABCS_OFFSET               (4U)
#define SCI_UART_SEMR_BGDM_OFFSET               (6U)
#define SCI_UART_SEMR_BAUD_SETTING_MASK         ((1U << SCI_UART_SEMR_BRME_OFFSET) |  \
                                                 (1U << SCI_UART_SEMR_ABCSE_OFFSET) | \
                                                 (1U << SCI_UART_SEMR_ABCS_OFFSET) | (1U << SCI_UART_SEMR_BGDM_OFFSET))

/** SCI SMR register bit masks */
#define SCI_SMR_CKS_VALUE_MASK                  (0x03U) ///< CKS: 2 bits

/* Device constant configuration parameters */
struct uart_ra_dev_cfg {
	R_SCI0_Type *regs;
	uint8_t sci_instance;
	struct soc_ioport_pin pin_rx;
	struct soc_ioport_pin pin_tx;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t	irq_config_func;
#endif
};

/* Device run time data */
struct uart_ra_dev_data {
	uint32_t baud_rate;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t irq_cb;	/* Interrupt Callback */
	void *irq_cb_data;	/* Interrupt Callback Arg */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#define DEV_CFG(dev) \
	((const struct uart_ra_dev_cfg *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct uart_ra_dev_data *const)(dev)->data)


static int baudrate_set(R_SCI0_Type *const uart, uint32_t baudrate,
			uint32_t mck_freq_hz);


static int uart_ra_init(const struct device *dev)
{

	/*TODO - implement dts uart advanced settings */
	uart_data_bits_t data_bits = UART_DATA_BITS_8;
	uart_parity_t parity = UART_PARITY_OFF;
	uart_stop_bits_t stop_bits = UART_STOP_BITS_1;
	sci_uart_ctsrts_config_t ctsrts_en = SCI_UART_CTSRTS_RTS_OUTPUT;
	sci_uart_start_bit_detect_t rx_edge_start = SCI_UART_START_BIT_LOW_LEVEL;
	sci_uart_noise_cancellation_t noise_cancel = SCI_UART_NOISE_CANCELLATION_DISABLE;
	sci_clk_src_t clock = SCI_UART_CLOCK_INT;

	const struct uart_ra_dev_cfg *const cfg = DEV_CFG(dev);
	struct uart_ra_dev_data *const dev_data = DEV_DATA(dev);
	R_SCI0_Type *const uart = cfg->regs;

	/* Connect pins to the peripheral */
	soc_ioport_configure(&cfg->pin_rx);
	soc_ioport_configure(&cfg->pin_tx);

	/* Enable the SCI channel and reset the registers to their initial state. */
	R_BSP_MODULE_START(FSP_IP_SCI, cfg->sci_instance);

	 /* Initialize registers as defined in section 34.3.7 "SCI Initialization in Asynchronous Mode" in the RA6M3 manual
	 * R01UH0886EJ0100 or the relevant section for the MCU being used. */
	cfg->regs->SCR   = 0U;
	cfg->regs->SSR   = 0U;
	cfg->regs->SIMR1 = 0U;
	cfg->regs->SIMR2 = 0U;
	cfg->regs->SIMR3 = 0U;
	cfg->regs->CDR   = 0U;

	/* Check if the channel supports address matching */
	if (BSP_FEATURE_SCI_ADDRESS_MATCH_CHANNELS & (1U << cfg->sci_instance))
	{
		cfg->regs->DCCR = SCI_UART_DCCR_DEFAULT_VALUE;
	}

    /* Set the default level of the TX pin to 1. */
	cfg->regs->SPTR = (uint8_t) (1U << SPTR_SPB2D_BIT) | SPTR_OUTPUT_ENABLE_MASK;

    /* Set the UART configuration settings provided in ::uart_cfg_t and ::sci_uart_extended_cfg_t. */
#ifdef SCI_UART_CFG_FIFO_SUPPORT

    /* Configure FIFO related registers. */
    r_sci_uart_fifo_cfg(p_ctrl);
#else

    /* If fifo support is disabled and the current channel supports fifo make sure it's disabled. */
    if (BSP_FEATURE_SCI_UART_FIFO_CHANNELS & (1U << cfg->sci_instance))
    {
    	cfg->regs->FCR = SCI_UART_FCR_DEFAULT_VALUE;
    }
#endif

    /* Configure parity and stop bits. */
    uint32_t smr  = (((uint32_t) parity << 4U) | ((uint32_t) stop_bits << 3U));
    uint32_t scmr = SCI_UART_SCMR_DEFAULT_VALUE;

    /* Configure data size. */
    if (UART_DATA_BITS_7 == data_bits)
    {
        /* Set the SMR.CHR bit & SCMR.CHR1 bit as selected (Character Length)
         *  Character Length
         *  (CHR1,CHR)
         *  (1, 1) Transmit/receive in 7-bit data length*3
         */
        smr |= (1U << 6);
    }
    else if (UART_DATA_BITS_9 == data_bits)
    {
        /* Set the SMR.CHR bit & SCMR.CHR1 bit as selected (Character Length)
         *  Character Length
         *  (CHR1,CHR)
         *  (0, 0) Transmit/receive in 9-bit data length
         */
        scmr &= ~(1U << 4);
    }
    else
    {
        /* Do nothing.  Default is 8-bit mode. */
    }

    /* Write to the SMR register. */
    cfg->regs->SMR = (uint8_t) smr;

    /* Write to the SCMR register. */
    cfg->regs->SCMR = (uint8_t) scmr;

   // sci_uart_extended_cfg_t * p_extend = (sci_uart_extended_cfg_t *) p_cfg->p_extend;

    /* Configure CTS flow control if CTS/RTS flow control is enabled. */
    cfg->regs->SPMR = (uint8_t) (ctsrts_en << SCI_UART_SPMR_CTSE_OFFSET);

    uint32_t semr = 0;

    /* Starts reception on falling edge of RXD if enabled in extension (otherwise reception starts at low level
     * of RXD). */
    semr |= (rx_edge_start & 1U) << 7;

    /* Enables the noise cancellation, fixed to the minimum level, if enabled in the extension. */
    semr |= (noise_cancel & 1U) << 5;

    cfg->regs->SEMR = (uint8_t) semr;

    cfg->regs->SNFR = NOISE_CANCEL_LVL1;

    baudrate_set(uart, dev_data->baud_rate, RA4W1_CPU_CLOCK_FREQ_HZ);

    uint32_t scr = ((uint8_t) clock) & 0x3U;

#if (CONFIG_SCI_UART_CFG_RX_ENABLE)

    /* If reception is enabled at build time, enable reception. */
    /* NOTE: Transmitter and its interrupt are enabled in R_SCI_UART_Write(). */
    scr |= SCI_SCR_RE_MASK;
    R_BSP_IrqEnable(p_ctrl->p_cfg->rxi_irq);
    R_BSP_IrqEnable(p_ctrl->p_cfg->eri_irq);

    scr |= SCI_SCR_RIE_MASK;
#endif

#if (CONFIG_SCI_UART_CFG_TX_ENABLE)
   // R_BSP_IrqEnable(p_ctrl->p_cfg->txi_irq);
    //R_BSP_IrqEnable(p_ctrl->p_cfg->tei_irq);
    scr |= SCI_SCR_TE_MASK;
#endif
    cfg->regs->SCR = (uint8_t) scr;

#if CONFIG_SCI_UART_CFG_FLOW_CONTROL_SUPPORT
/*TODO implement uart flow control*/
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int uart_ra_poll_in(const struct device *dev, unsigned char *c)
{
	R_SCI0_Type *const uart = DEV_CFG(dev)->regs;

	if (!(uart->SSR_FIFO_b.DR)) {
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
	while (!(uart->SSR_FIFO_b.TEND)) {
	}

	/* send a character */
	uart->TDR = (uint32_t)c;
}

static int uart_ra_err_check(const struct device *dev)
{
	volatile R_SCI0_Type * const uart = DEV_CFG(dev)->regs;
	int errors = 0;

	if (uart->SSR_FIFO_b.ORER) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (uart->SSR_FIFO_b.PER) {
		errors |= UART_ERROR_PARITY;
	}

	if (uart->SSR_FIFO_b.FER) {
		errors |= UART_ERROR_FRAMING;
	}

	return errors;
}

static int baudrate_set(R_SCI0_Type *const uart, uint32_t baudrate,
			uint32_t mck_freq_hz)
{
	uint8_t brrValue;

	__ASSERT(baudrate,
		 "baud rate has to be bigger than 0");

	brrValue = (uint8_t)(mck_freq_hz / (16 * baudrate)) - 1;

	if (brrValue > 0xFF) {
		return -EINVAL;
	}

    /* Set BRR register value. */
	uart->BRR = brrValue;

    /* Set clock source for the on-chip baud rate generator. */
	uart->SMR_b.CKS = (uint8_t) (SCI_SMR_CKS_VALUE_MASK & 0);

    /* Set MDDR register value. */
	uart->MDDR = 255;

    /* Set clock divisor settings. */
	uart->SEMR = (uint8_t) ((uart->SEMR & ~(SCI_UART_SEMR_BAUD_SETTING_MASK)) |
                                 ((1U << SCI_UART_SEMR_BGDM_OFFSET) & SCI_UART_SEMR_BAUD_SETTING_MASK));

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_ra_fifo_fill(const struct device *dev,
			      const uint8_t *tx_data,
			      int size)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	/* Wait for transmitter to be ready. */
	while ((uart->UART_SR & UART_SR_TXRDY) == 0) {
	}

	uart->UART_THR = *tx_data;

	return 1;
}

static int uart_ra_fifo_read(const struct device *dev, uint8_t *rx_data,
			      const int size)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;
	int bytes_read;

	bytes_read = 0;

	while (bytes_read < size) {
		if (uart->UART_SR & UART_SR_RXRDY) {
			rx_data[bytes_read] = uart->UART_RHR;
			bytes_read++;
		} else {
			break;
		}
	}

	return bytes_read;
}

static void uart_ra_irq_tx_enable(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	uart->UART_IER = UART_IER_TXRDY;
}

static void uart_ra_irq_tx_disable(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	uart->UART_IDR = UART_IDR_TXRDY;
}

static int uart_ra_irq_tx_ready(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	return (uart->UART_SR & UART_SR_TXRDY);
}

static void uart_ra_irq_rx_enable(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	uart->UART_IER = UART_IER_RXRDY;
}

static void uart_ra_irq_rx_disable(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	uart->UART_IDR = UART_IDR_RXRDY;
}

static int uart_ra_irq_tx_complete(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	return !(uart->UART_SR & UART_SR_TXRDY);
}

static int uart_ra_irq_rx_ready(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	return (uart->UART_SR & UART_SR_RXRDY);
}

static void uart_ra_irq_err_enable(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	uart->UART_IER = UART_IER_OVRE | UART_IER_FRAME | UART_IER_PARE;
}

static void uart_ra_irq_err_disable(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	uart->UART_IDR = UART_IDR_OVRE | UART_IDR_FRAME | UART_IDR_PARE;
}

static int uart_ra_irq_is_pending(const struct device *dev)
{
	volatile Uart * const uart = DEV_CFG(dev)->regs;

	return (uart->UART_IMR & (UART_IMR_TXRDY | UART_IMR_RXRDY)) &
		(uart->UART_SR & (UART_SR_TXRDY | UART_SR_RXRDY));
}

static int uart_ra_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

static void uart_ra_irq_callback_set(const struct device *dev,
				      uart_irq_callback_user_data_t cb,
				      void *cb_data)
{
	struct uart_ra_dev_data *const dev_data = DEV_DATA(dev);

	dev_data->irq_cb = cb;
	dev_data->irq_cb_data = cb_data;
}

static void uart_ra_isr(const struct device *dev)
{
	struct uart_ra_dev_data *const dev_data = DEV_DATA(dev);

	if (dev_data->irq_cb) {
		dev_data->irq_cb(dev, dev_data->irq_cb_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_ra_driver_api = {
	.poll_in = uart_ra_poll_in,
	.poll_out = uart_ra_poll_out,
	.err_check = uart_ra_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_ra_fifo_fill,
	.fifo_read = uart_ra_fifo_read,
	.irq_tx_enable = uart_ra_irq_tx_enable,
	.irq_tx_disable = uart_ra_irq_tx_disable,
	.irq_tx_ready = uart_ra_irq_tx_ready,
	.irq_rx_enable = uart_ra_irq_rx_enable,
	.irq_rx_disable = uart_ra_irq_rx_disable,
	.irq_tx_complete = uart_ra_irq_tx_complete,
	.irq_rx_ready = uart_ra_irq_rx_ready,
	.irq_err_enable = uart_ra_irq_err_enable,
	.irq_err_disable = uart_ra_irq_err_disable,
	.irq_is_pending = uart_ra_irq_is_pending,
	.irq_update = uart_ra_irq_update,
	.irq_callback_set = uart_ra_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
};

#define UART_RA_DECLARE_CFG(n, IRQ_FUNC_INIT)				\
	static const struct uart_ra_dev_cfg uart##n##_ra_config = {	\
		.regs = (R_SCI0_Type *)DT_INST_REG_ADDR(n),			\
		.sci_instance = DT_INST_PROP(n, instance_idx),								\
		.pin_rx.pfsBase = (R_PFS_PORT_Type *)DT_REG_ADDR_BY_IDX(DT_PHANDLE(DT_INST_PHANDLE_BY_IDX(n, pinctrl_0, 0), renesas_pins), 1),		\
		.pin_rx.pin = DT_PHA(DT_INST_PHANDLE_BY_IDX(n, pinctrl_0, 0), renesas_pins, pin), \
		.pin_rx.value = DT_PHA(DT_INST_PHANDLE_BY_IDX(n, pinctrl_0, 0), renesas_pins, peripheral),						\
		.pin_tx.pfsBase = (R_PFS_PORT_Type *)DT_REG_ADDR_BY_IDX(DT_PHANDLE(DT_INST_PHANDLE_BY_IDX(n, pinctrl_0, 1), renesas_pins), 1),		\
		.pin_tx.pin = DT_PHA(DT_INST_PHANDLE_BY_IDX(n, pinctrl_0, 1), renesas_pins, pin), \
		.pin_tx.value = DT_PHA(DT_INST_PHANDLE_BY_IDX(n, pinctrl_0, 1), renesas_pins, peripheral),						\
									\
		IRQ_FUNC_INIT						\
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_RA_CONFIG_FUNC(n)						\
	static void uart##n##_ra_irq_config_func(const struct device *port)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    uart_ra_isr,				\
			    DEVICE_GET(uart##n##_ra), 0);		\
		irq_enable(DT_INST_IRQN(n));				\
	}
#define UART_RA_IRQ_CFG_FUNC_INIT(n)					\
	.irq_config_func = uart##n##_ra_irq_config_func
#define UART_RA_INIT_CFG(n)						\
	UART_RA_DECLARE_CFG(n, UART_RA_IRQ_CFG_FUNC_INIT(n))
#else
#define UART_RA_CONFIG_FUNC(n)
#define UART_RA_IRQ_CFG_FUNC_INIT
#define UART_RA_INIT_CFG(n)						\
	UART_RA_DECLARE_CFG(n, UART_RA_IRQ_CFG_FUNC_INIT)
#endif

#define UART_RA_INIT(n)						\
	static struct uart_ra_dev_data uart##n##_ra_data = {		\
		.baud_rate = DT_INST_PROP(n, current_speed),		\
	};								\
									\
	static const struct uart_ra_dev_cfg uart##n##_ra_config;	\
									\
	DEVICE_AND_API_INIT(uart##n##_ra, DT_INST_LABEL(n),		\
			    &uart_ra_init, &uart##n##_ra_data,	\
			    &uart##n##_ra_config, PRE_KERNEL_1,	\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			    &uart_ra_driver_api);			\
									\
	UART_RA_CONFIG_FUNC(n)						\
									\
	UART_RA_INIT_CFG(n);

DT_INST_FOREACH_STATUS_OKAY(UART_RA_INIT)
