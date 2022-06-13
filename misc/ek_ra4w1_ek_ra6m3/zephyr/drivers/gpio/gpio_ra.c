/*
 * Copyright (c) 2020, MXT Creation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra_gpio

#include <errno.h>
#include <device.h>
#include <drivers/gpio.h>
#include <soc.h>
#include <sys/util.h>

#include <dt-bindings/pinctrl/ra-pinctrl.h>

#if (CONFIG_RENESAS_BSP_API)
#include <bsp_api.h>
#else
#include <bsp_feature.h>
#include <renesas.h>
#include <bsp_io.h>
#endif

#include "gpio_utils.h"

struct ra_gpio_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	R_PORT0_Type *portBase;
	R_PFS_PORT_Type *pfsBase;
};

struct ra_gpio_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
};

static int ra_gpio_configure(const struct device *port, gpio_pin_t pin,
			      gpio_flags_t flags)
{
	const struct ra_gpio_config *config = port->config;
	R_PORT0_Type *portBase = config->portBase;

	if (((flags & GPIO_INPUT) != 0U) && ((flags & GPIO_OUTPUT) != 0U)) {
		return -ENOTSUP;
	}

	if ((flags & (GPIO_SINGLE_ENDED
		      | GPIO_PULL_UP
		      | GPIO_PULL_DOWN)) != 0U) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_OUTPUT) != 0U) {
		/* Set output pin initial value */
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			WRITE_BIT(portBase->PODR, pin, BSP_IO_LEVEL_LOW);
		} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			WRITE_BIT(portBase->PODR, pin, BSP_IO_LEVEL_HIGH);
		}



		/* Set pin as output */
		WRITE_BIT(portBase->PDR, pin, BSP_IO_DIRECTION_OUTPUT);
	} else {
		/* Set pin as input */
		WRITE_BIT(portBase->PDR, pin, BSP_IO_DIRECTION_INPUT);
	}

	return 0;
}

static int ra_gpio_port_get_raw(const struct device *port, uint32_t *value)
{
	const struct ra_gpio_config *config = port->config;
	R_PORT0_Type *portBase = config->portBase;

	*value = portBase->PIDR;

	return 0;
}

static int ra_gpio_port_set_masked_raw(const struct device *port,
					gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	const struct ra_gpio_config *config = port->config;
	R_PORT0_Type *portBase = config->portBase;

	portBase->PODR = ((portBase->PIDR & ~mask) | (value & mask));

	return 0;
}

static int ra_gpio_port_set_bits_raw(const struct device *port,
				      gpio_port_pins_t pins)
{
	const struct ra_gpio_config *config = port->config;
	R_PORT0_Type *portBase = config->portBase;

	portBase->PODR = (portBase->PIDR | pins);

	return 0;
}

static int ra_gpio_port_clear_bits_raw(const struct device *port,
					gpio_port_pins_t pins)
{
	const struct ra_gpio_config *config = port->config;
	R_PORT0_Type *portBase = config->portBase;

	portBase->PODR = (portBase->PIDR & ~pins);

	return 0;
}

static int ra_gpio_port_toggle_bits(const struct device *port,
				     gpio_port_pins_t pins)
{
	const struct ra_gpio_config *config = port->config;
	R_PORT0_Type *portBase = config->portBase;

	portBase->PODR = (portBase->PIDR ^ pins);

	return 0;
}

static int ra_gpio_pin_interrupt_configure(const struct device *port,
					    gpio_pin_t pin,
					    enum gpio_int_mode mode,
					    enum gpio_int_trig trig)
{
	/*TODO - gpio pin interrupt implementation */
#if 0
	const struct ra_gpio_config *config = port->config;
	R_PFS_PORT_Type *pfsBase = config->pfsBase;

	const struct soc_ioport_pin pinInterrupt;
	pinInterrupt.pfsBase = pfsBase;
	pinInterrupt.pin = (uint32_t)pin;
	pinInterrupt.value = ((uint32_t) IOPORT_CFG_IRQ_ENABLE | (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT);

	soc_ioport_configure(pinInterrupt);


#endif
	return 0;
}

static int ra_gpio_manage_callback(const struct device *port,
				    struct gpio_callback *cb, bool set)
{
	struct ra_gpio_data *data = port->data;

	return gpio_manage_callback(&data->callbacks, cb, set);
}

static void ra_gpio_port_isr(const struct device *port)
{
	/*TODO - gpio pin interrupt implementation */
#if 0
	const struct ra_gpio_config *config = port->config;
	struct ra_gpio_data *data = port->data;
	uint32_t int_status;

	int_status = config->portBase->ISR;

	config->portBase->ISR = int_status;

	gpio_fire_callbacks(&data->callbacks, port, int_status);
#endif
}

static const struct gpio_driver_api ra_gpio_driver_api = {
	.pin_configure = ra_gpio_configure,
	.port_get_raw = ra_gpio_port_get_raw,
	.port_set_masked_raw = ra_gpio_port_set_masked_raw,
	.port_set_bits_raw = ra_gpio_port_set_bits_raw,
	.port_clear_bits_raw = ra_gpio_port_clear_bits_raw,
	.port_toggle_bits = ra_gpio_port_toggle_bits,
	.pin_interrupt_configure = ra_gpio_pin_interrupt_configure,
	.manage_callback = ra_gpio_manage_callback,
};

#define GPIO_RA_INIT(n)						\
	static int ra_gpio_##n##_init(const struct device *port);	\
									\
	static const struct ra_gpio_config ra_gpio_##n##_config = {	\
		.common = {						\
			.port_pin_mask =				\
				GPIO_PORT_PIN_MASK_FROM_DT_INST(n),	\
		},							\
		.portBase = (R_PORT0_Type *)DT_INST_REG_ADDR(n),		\
		.pfsBase = (R_PFS_PORT_Type *)DT_INST_REG_ADDR_BY_IDX(0, 1),			\
	};								\
									\
	static struct ra_gpio_data ra_gpio_##n##_data;		\
									\
	DEVICE_AND_API_INIT(ra_gpio_##n, DT_INST_LABEL(n),		\
			    ra_gpio_##n##_init,			\
			    &ra_gpio_##n##_data,			\
			    &ra_gpio_##n##_config,			\
			    POST_KERNEL,				\
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
			    &ra_gpio_driver_api);			\
									\
	static int ra_gpio_##n##_init(const struct device *port)	\
	{								\
		/* IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq),		\
			    DT_INST_IRQ_BY_IDX(n, 0, priority),		\
			    ra_gpio_port_isr,				\
			    DEVICE_GET(ra_gpio_##n), 0);		\
									\
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));		\
									\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 1, irq),		\
			    DT_INST_IRQ_BY_IDX(n, 1, priority),		\
			    ra_gpio_port_isr,				\
			    DEVICE_GET(ra_gpio_##n), 0);		\
									\
		irq_enable(DT_INST_IRQ_BY_IDX(n, 1, irq));		\
								*/	\
		return 0;						\
	}

DT_INST_FOREACH_STATUS_OKAY(GPIO_RA_INIT)
