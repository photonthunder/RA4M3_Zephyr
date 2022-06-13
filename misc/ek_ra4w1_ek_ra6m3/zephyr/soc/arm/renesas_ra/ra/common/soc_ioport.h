/*
 * soc_ioport.h
 *
 *  Created on: Nov 18, 2020
 *      Author: zoltan.ianosi.bp
 */

#ifndef SOC_ARM_RENESAS_RA_RA_RA4W1_SOC_IOPORT_H_
#define SOC_ARM_RENESAS_RA_RA_RA4W1_SOC_IOPORT_H_

#include <kernel.h>
#include <zephyr/types.h>
#include <bsp_feature.h>
#include <renesas.h>

struct soc_ioport_pin {
	R_PFS_PORT_Type *pfsBase;
	uint32_t pin;
	uint32_t value;
};

/**
 * @brief Configure IO port pin.
 *
 * Configure one pin.
 * Example scenarios:
 * - configure pin as input with debounce filter enabled.
 * - connect pin to a peripheral and enable pull-up.
 * - configure pin as open drain output.
 *
 * @param pin  pin's configuration data such as pfs register, pin, pin attributes values.
 */
void soc_ioport_configure (const struct soc_ioport_pin *pin);

#endif /* SOC_ARM_RENESAS_RA_RA_RA4W1_SOC_IOPORT_H_ */
