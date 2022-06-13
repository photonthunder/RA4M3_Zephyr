/*
 * soc_ioport.c
 */

#include "soc_ioport.h"
#include <dt-bindings/pinctrl/ra-pinctrl.h>
#include <bsp_api.h>
#include <renesas.h>

#define SOC_IOPORT_PRV_PORT_OFFSET            (8U)
#define SOC_IOPORT_PFS_PDR_OUTPUT         (4U)

volatile uint32_t g_protect_pfswe_counter BSP_SECTION_EARLY_INIT;

__STATIC_INLINE void socPinAccessEnable (void)
{
    /** If this is first entry then allow writing of PFS. */
    if (0 == g_protect_pfswe_counter)
    {
        R_PMISC->PWPR = 0;                               ///< Clear BOWI bit - writing to PFSWE bit enabled
        R_PMISC->PWPR = 1U << BSP_IO_PWPR_PFSWE_OFFSET;  ///< Set PFSWE bit - writing to PFS register enabled
    }

    /** Increment the protect counter */
    g_protect_pfswe_counter++;
}

__STATIC_INLINE void socPinAccessDisable (void)
{
    /** Is it safe to disable PFS register? */
    if (0 != g_protect_pfswe_counter)
    {
        /* Decrement the protect counter */
        g_protect_pfswe_counter--;
    }

    /** Is it safe to disable writing of PFS? */
    if (0 == g_protect_pfswe_counter)
    {
        R_PMISC->PWPR = 0;                              ///< Clear PFSWE bit - writing to PFS register disabled
        R_PMISC->PWPR = 1U << BSP_IO_PWPR_B0WI_OFFSET;  ///< Set BOWI bit - writing to PFSWE bit disabled
    }
}

static void soc_ioport_pfs_write (bsp_io_port_pin_t pin, uint32_t value)
{
    /* PMR bits should be cleared before specifying PSEL. Reference section "20.7 Notes on the PmnPFS Register Setting"
     * in the RA6M3 manual R01UH0886EJ0100. */
    if ((value & IOPORT_PRV_PERIPHERAL_FUNCTION) > 0)
    {
        /* Clear PMR */
        R_PFS->PORT[pin >> SOC_IOPORT_PRV_PORT_OFFSET].PIN[pin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.PMR = 0;

        /* New config with PMR = 0 */
        R_PFS->PORT[pin >> SOC_IOPORT_PRV_PORT_OFFSET].PIN[pin &
                                                       BSP_IO_PRV_8BIT_MASK].PmnPFS =
            (value & ~((uint32_t) IOPORT_PRV_PERIPHERAL_FUNCTION));
    }

    /* Write configuration */
    R_PFS->PORT[pin >> SOC_IOPORT_PRV_PORT_OFFSET].PIN[pin & BSP_IO_PRV_8BIT_MASK].PmnPFS = value;
}

void soc_ioport_configure (const struct soc_port_pin *pin)
{
	socPinAccessEnable();           // Protect PWPR from re-entrancy

    soc_ioport_pfs_write(pin->pin, pin->pin_cfg);

	socPinAccessDisable();

}

void soc_ioport_list_configure(const struct soc_port_pin pins[], unsigned int size)
{
	for (int i = 0; i < size; i++) {
		soc_ioport_configure(&pins[i]);
	}
}


uint32_t soc_ioport_pinread (bsp_io_port_pin_t pin)
{
    /* Read pin level. */
    return R_PFS->PORT[pin >> 8].PIN[pin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.PIDR;
}


void soc_ioport_pinwrite (bsp_io_port_pin_t pin, uint8_t level)
{
    socPinAccessEnable();           // Protect PWPR from re-entrancy

    /* Clear PMR, ASEL, ISEL and PODR bits. */
    uint32_t pfs_bits = R_PFS->PORT[pin >> 8].PIN[pin & BSP_IO_PRV_8BIT_MASK].PmnPFS;
    pfs_bits &= BSP_IO_PRV_PIN_WRITE_MASK;

    /* Set output level and pin direction to output. */
    uint32_t lvl = ((uint32_t) level | pfs_bits);
    R_PFS->PORT[pin >> 8].PIN[pin & BSP_IO_PRV_8BIT_MASK].PmnPFS = (SOC_IOPORT_PFS_PDR_OUTPUT | lvl);

    socPinAccessDisable();
}

void soc_ioport_pintoggle (bsp_io_port_pin_t pin)
{
    uint32_t pinState = soc_ioport_pinread(pin);
    uint8_t level = 0;
    if (pinState)
    {
        level = 1;
    }
    soc_ioport_pinwrite(pin, level);
}