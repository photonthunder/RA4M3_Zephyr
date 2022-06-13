/*
 * soc_ioport.h
 */

#ifndef SOC_ARM_RENESAS_RA_RA_RA4W1_SOC_IOPORT_H_
#define SOC_ARM_RENESAS_RA_RA_RA4W1_SOC_IOPORT_H_

#include <kernel.h>
#include <zephyr/types.h>
#include <bsp_feature.h>
#include <vector_data.h>
#include <renesas.h>
#include <r_ioport.h>

#define SOC_SHIFT_PORT	16

// typedef struct st_ioport_pin_cfg
// {
//     uint32_t          pin_cfg;         ///< Pin PFS configuration - Use ioport_cfg_options_t parameters to configure
//     bsp_io_port_pin_t pin;             ///< Pin identifier
// } ioport_pin_cfg_t;

struct soc_port_pin {
	R_SCI0_Type *regs;   /** pointer to registers of the I/O Pin Controller */
	bsp_io_port_pin_t pin;    /** pin number */
	uint32_t pin_cfg;    /** pin flags/attributes */
};

void soc_ioport_configure (const struct soc_port_pin *pin);
void soc_ioport_list_configure(const struct soc_port_pin pins[], unsigned int size);
uint32_t soc_ioport_pinread (bsp_io_port_pin_t pin);
void soc_ioport_pinwrite (bsp_io_port_pin_t pin, uint8_t level);
void soc_ioport_pintoggle (bsp_io_port_pin_t pin);

#endif /* SOC_ARM_RENESAS_RA_RA_RA4W1_SOC_IOPORT_H_ */
