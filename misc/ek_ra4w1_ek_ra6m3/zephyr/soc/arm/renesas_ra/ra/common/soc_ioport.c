/*
 * soc_ioport.c
 *
 *  Created on: Nov 19, 2020
 *      Author: zoltan.ianosi.bp
 */

#include "soc_ioport.h"
#include <dt-bindings/pinctrl/ra-pinctrl.h>
#include <bsp_api.h>
#include <renesas.h>
#include <bsp_io.h>


/*******************************************************************************************************************//**
 * Writes to the specified pin's PFS register
 *
 * @param[in]    pfsBase    Pin function select register port base address
 * @param[in]    pin        Pin to write PFS data for
 * @param[in]    value      Value to be written to the PFS register
 *
 **********************************************************************************************************************/
static void soc_ioport_pfs_write (R_PFS_PORT_Type *pfsBase, uint32_t pin, uint32_t value)
{

    /* PMR bits should be cleared before specifying PSEL. Reference section "20.7 Notes on the PmnPFS Register Setting"
     * in the RA6M3 manual R01UH0886EJ0100. */
    if ((value & IOPORT_PRV_PERIPHERAL_FUNCTION) > 0)
    {
        /* Clear PMR */
    	pfsBase->PIN[pin].PmnPFS_b.PMR = 0;

        /* New config with PMR = 0 */
    	pfsBase->PIN[pin].PmnPFS = (value & ~((uint32_t) IOPORT_PRV_PERIPHERAL_FUNCTION));
    }

    /* Write configuration */
    pfsBase->PIN[pin].PmnPFS = value;

}

void soc_ioport_configure (const struct soc_ioport_pin *pin)
{
	R_BSP_PinAccessEnable();           // Protect PWPR from re-entrancy

	soc_ioport_pfs_write(pin->pfsBase, pin->pin, pin->value);

	R_BSP_PinAccessDisable();
}
