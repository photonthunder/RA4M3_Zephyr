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
#include <soc_ioport.h>

#include <dt-bindings/pinctrl/ra-pinctrl.h>

#include "gpio_utils.h"

struct gpio_ra_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	R_PORT0_Type *regs;
	uint32_t port;
};

struct gpio_ra_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	const struct device *dev;
};

#define DEV_CFG(dev) \
	((const struct gpio_ra_config *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct gpio_ra_data *const)(dev)->data)

static int ra_configure(const struct device *dev, gpio_pin_t pin,
			      gpio_flags_t flags)
{
	const struct gpio_ra_config *config = DEV_CFG(dev);
	struct soc_port_pin socPort;
	socPort.pin = (config->port << 8 | pin);
	socPort.pin_cfg = 0;

	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		return -ENOTSUP;
	}

	/* Supports disconnected, input, output */
	if ((flags & GPIO_INPUT) != 0) {
		socPort.pin_cfg = IOPORT_CFG_PORT_DIRECTION_INPUT;
	}
	if ((flags & GPIO_OUTPUT) != 0) {
		/* Output is incompatible with pull */
		if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
			return -ENOTSUP;
		}
		socPort.pin_cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT;
	} else {
		/* Not output, may be input */
		socPort.pin_cfg = IOPORT_CFG_PORT_DIRECTION_INPUT;
		/* Pull down not supported */
		if ((flags & GPIO_PULL_DOWN) != 0) {
			return -ENOTSUP;
		}

		/* Pull configuration is supported if not output */
		if ((flags & GPIO_PULL_UP) != 0) {
			socPort.pin_cfg |= IOPORT_CFG_PORT_DIRECTION_INPUT;
		}
	}

	/* Write the now-built pin configuration */
	soc_ioport_configure(&socPort);

	return 0;
}

static int gpio_ra_port_get_raw(const struct device *dev,
				  gpio_port_value_t *value)
{
	const struct gpio_ra_config *config = DEV_CFG(dev);
	bsp_io_port_pin_t portPin;
	uint8_t pinValue = 0;
	for (int i = 0; i < 16; i++)
	{
		portPin = ((config->port << 8) | i);
		pinValue |= (soc_ioport_pinread(portPin) << i);
	}

	return 0;
}

static int gpio_ra_port_set_masked_raw(const struct device *dev,
					 gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_ra_config *config = DEV_CFG(dev);
	bsp_io_port_pin_t portPin;
	uint8_t pinValue;
	for (int i = 0; i < 16; i++)
	{
		if ((mask & (0x00000001 << i)) != 0)
		{
			portPin = ((config->port << 8) | i);
			pinValue = value & (0x00000001 << i);
			soc_ioport_pinwrite(portPin, pinValue);
		}
	}

	return 0;
}

static int gpio_ra_port_set_bits_raw(const struct device *dev,
				       gpio_port_pins_t pins)
{
	const struct gpio_ra_config *config = DEV_CFG(dev);
	bsp_io_port_pin_t portPin;
	for (int i = 0; i < 16; i++)
	{
		if ((pins & (0x00000001 << i)) != 0)
		{
			portPin = ((config->port << 8) | i);
			soc_ioport_pinwrite(portPin, 1);
		}
	}

	return 0;
}

static int gpio_ra_port_clear_bits_raw(const struct device *dev,
					 gpio_port_pins_t pins)
{
	const struct gpio_ra_config *config = DEV_CFG(dev);
	bsp_io_port_pin_t portPin;
	for (int i = 0; i < 16; i++)
	{
		if ((pins & (0x00000001 << i)) != 0)
		{
			portPin = ((config->port << 8) | i);
			soc_ioport_pinwrite(portPin, 0);
		}
	}

	return 0;
}

static int gpio_ra_port_toggle_bits(const struct device *dev,
				      gpio_port_pins_t pins)
{
	const struct gpio_ra_config *config = DEV_CFG(dev);
	bsp_io_port_pin_t portPin;
	for (int i = 0; i < 16; i++)
	{
		if ((pins & (0x00000001 << i)) != 0)
		{
			portPin = (config->port << 8 | i);
			soc_ioport_pintoggle(portPin);
		}
	}

	return 0;
}


static int gpio_ra_init(const struct device *dev) { return 0; }

static const struct gpio_driver_api gpio_ra_api = {
	.pin_configure = ra_configure,
	.port_get_raw = gpio_ra_port_get_raw,
	.port_set_masked_raw = gpio_ra_port_set_masked_raw,
	.port_set_bits_raw = gpio_ra_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ra_port_clear_bits_raw,
	.port_toggle_bits = gpio_ra_port_toggle_bits,
};

#define GPIO_RA_INIT(n)					    						\
	static const struct gpio_ra_config gpio_ra_config_##n = {	   	\
		.common = {					    							\
			.port_pin_mask =			   	 						\
				GPIO_PORT_PIN_MASK_FROM_DT_INST(n), 				\
		},						    								\
		.regs = (R_PORT0_Type *)DT_INST_REG_ADDR(n), 				\
		.port = DT_INST_PROP(n, port) 								\
	};							    								\
	static struct gpio_ra_data gpio_ra_data_##n;	    			\
								    								\
	DEVICE_DT_INST_DEFINE(n,				    					\
			      gpio_ra_init,			    						\
			      NULL,				    							\
			      &gpio_ra_data_##n,		   	 					\
			      &gpio_ra_config_##n,		    					\
			      POST_KERNEL,			    						\
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,  			\
			      &gpio_ra_api		    							\
			      );


DT_INST_FOREACH_STATUS_OKAY(GPIO_RA_INIT)
