/*
 * Copyright (c) 2020 MXT Creation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Renesas RA6M3MCU series initialization code
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Renesas RA6M3 series processor.
 */

#include "soc.h"

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>

#include <bsp_feature.h>
#include <fsp_features.h>
#include <renesas.h>
#include <bsp_io.h>
#include <bsp_elc.h>
#include <bsp_register_protection.h>
#include <fsp_common_api.h>
#include <bsp_common.h>
#include <bsp_module_stop.h>
#include <bsp_group_irq.h>
#include <bsp_delay.h>
#include <bsp_mcu_api.h>
#include <soc_clock.h>


uint32_t SystemCoreClock;

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/* Mask to select CP bits( 0xF00000 ) */
#define CP_MASK                                       (0xFU << 20)

/* Value to write to OAD register of MPU stack monitor to enable NMI when a stack overflow is detected. */
#define BSP_STACK_POINTER_MONITOR_NMI_ON_DETECTION    (0xA500U)

/* Key code for writing PRCR register. */
#define BSP_PRV_PRCR_KEY                              (0xA500U)
#define BSP_PRV_PRCR_PRC1_UNLOCK                      ((BSP_PRV_PRCR_KEY) | 0x2U)
#define BSP_PRV_PRCR_LOCK                             ((BSP_PRV_PRCR_KEY) | 0x0U)

#if defined(__ICCARM__)
 #define BSP_PRV_STACK_LIMIT                          ((uint32_t) __section_begin(".stack"))
 #define BSP_PRV_STACK_TOP                            ((uint32_t) __section_end(".stack"))
#elif defined(__ARMCC_VERSION)
 #define BSP_PRV_STACK_LIMIT                          ((uint32_t) &Image$$STACK$$ZI$$Base)
 #define BSP_PRV_STACK_TOP                            ((uint32_t) &Image$$STACK$$ZI$$Base + \
                                                       (uint32_t) &Image$$STACK$$ZI$$Length)
#elif defined(__GNUC__)
 #define BSP_PRV_STACK_LIMIT                          ((uint32_t) &__StackLimit)
 #define BSP_PRV_STACK_TOP                            ((uint32_t) &__StackTop)
#endif

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/* Key code for writing PRCR register. */
#define BSP_PRV_PRCR_KEY                      (0xA500U)
#define BSP_PRV_PRCR_UNLOCK                   ((BSP_PRV_PRCR_KEY) | 0x3U)
#define BSP_PRV_PRCR_LOCK                     ((BSP_PRV_PRCR_KEY) | 0x0U)

#define BSP_PRV_MAXIMUM_HOCOWTR_HSTS          ((uint8_t) 0x6U)

/* Wait state definitions for MEMWAIT. */
#define BSP_PRV_MEMWAIT_ZERO_WAIT_CYCLES      (0U)
#define BSP_PRV_MEMWAIT_TWO_WAIT_CYCLES       (1U)
#define BSP_PRV_MEMWAIT_MAX_ZERO_WAIT_FREQ    (32000000U)

/* Wait state definitions for FLDWAITR. */
#define BSP_PRV_FLDWAITR_ONE_WAIT_CYCLES      (0U)
#define BSP_PRV_FLDWAITR_TWO_WAIT_CYCLES      (1U)
#define BSP_PRV_FLDWAITR_MAX_ONE_WAIT_FREQ    (32000000U)

/* Temporary solution until R_FACI is added to renesas.h. */
#define BSP_PRV_FLDWAITR_REG_ACCESS           (*((volatile uint8_t *) (0x407EFFC4U)))

/* Wait state definitions for MCUS with SRAMWTSC and FLWT. */
#define BSP_PRV_SRAMWTSC_ZERO_WAIT_CYCLES     (0U)
#define BSP_PRV_SRAMWTSC_ONE_WAIT_CYCLES      (0xEU)
#define BSP_PRV_ROM_ZERO_WAIT_CYCLES          (0U)
#define BSP_PRV_ROM_ONE_WAIT_CYCLES           (1U)
#define BSP_PRV_ROM_TWO_WAIT_CYCLES           (2U)
#define BSP_PRV_ROM_THREE_WAIT_CYCLES         (3U)
#define BSP_PRV_SRAM_PRCR_KEY                 (0x78U)
#define BSP_PRV_SRAM_UNLOCK                   (((BSP_PRV_SRAM_PRCR_KEY) << 1) | 0x1U)
#define BSP_PRV_SRAM_LOCK                     (((BSP_PRV_SRAM_PRCR_KEY) << 1) | 0x0U)

/* Calculate value to write to MOMCR (MODRV controls main clock drive strength and MOSEL determines the source of the
 * main oscillator). */
#define BSP_PRV_MOMCR_MOSEL_BIT               (6)
#define BSP_PRV_MODRV                         ((CGC_MAINCLOCK_DRIVE << BSP_FEATURE_CGC_MODRV_SHIFT) & \
                                               BSP_FEATURE_CGC_MODRV_MASK)
#define BSP_PRV_MOSEL                         (BSP_CLOCK_CFG_MAIN_OSC_CLOCK_SOURCE << BSP_PRV_MOMCR_MOSEL_BIT)
#define BSP_PRV_MOMCR                         (BSP_PRV_MODRV | BSP_PRV_MOSEL)

/* Locations of bitfields used to configure CLKOUT. */
#define BSP_PRV_CKOCR_CKODIV_BIT              (4U)
#define BSP_PRV_CKOCR_CKOEN_BIT               (7U)

#ifdef BSP_CFG_UCK_DIV

/* If the MCU has SCKDIVCR2. */
 #if !BSP_FEATURE_BSP_HAS_USBCKDIVCR

/* Location of bitfield used to configure USB clock divider. */
  #define BSP_PRV_SCKDIVCR2_UCK_BIT    (4U)
  #define BSP_PRV_UCK_DIV              (BSP_CFG_UCK_DIV)

/* If the MCU has USBCKDIVCR. */
 #elif BSP_FEATURE_BSP_HAS_USBCKDIVCR

  #if BSP_CLOCKS_USB_CLOCK_DIV_3 == BSP_CFG_UCK_DIV
   #define BSP_PRV_UCK_DIV    (5U)
  #elif BSP_CLOCKS_USB_CLOCK_DIV_4 == BSP_CFG_UCK_DIV
   #define BSP_PRV_UCK_DIV    (2U)
  #elif BSP_CLOCKS_USB_CLOCK_DIV_5 == BSP_CFG_UCK_DIV
   #define BSP_PRV_UCK_DIV    (6U)
  #else

   #error "BSP_CFG_UCK_DIV not supported."

  #endif
 #endif
#endif

/* Calculate the value to write to SCKDIVCR. */
#define BSP_PRV_STARTUP_SCKDIVCR_ICLK_BITS      ((BSP_CFG_ICLK_DIV & 7U) << 24U)
#if BSP_FEATURE_CGC_HAS_PCLKD
 #define BSP_PRV_STARTUP_SCKDIVCR_PCLKD_BITS    (BSP_CFG_PCLKD_DIV & 0x7U)
#else
 #define BSP_PRV_STARTUP_SCKDIVCR_PCLKD_BITS    (0U)
#endif
#if BSP_FEATURE_CGC_HAS_PCLKC
 #define BSP_PRV_STARTUP_SCKDIVCR_PCLKC_BITS    ((BSP_CFG_PCLKC_DIV & 0x7U) << 4U)
#else
 #define BSP_PRV_STARTUP_SCKDIVCR_PCLKC_BITS    (0U)
#endif
#if BSP_FEATURE_CGC_HAS_PCLKB
 #define BSP_PRV_STARTUP_SCKDIVCR_PCLKB_BITS    ((BSP_CFG_PCLKB_DIV & 0x7U) << 8U)
#else
 #define BSP_PRV_STARTUP_SCKDIVCR_PCLKB_BITS    (0U)
#endif
#if BSP_FEATURE_CGC_HAS_PCLKA
 #define BSP_PRV_STARTUP_SCKDIVCR_PCLKA_BITS    ((BSP_CFG_PCLKA_DIV & 0x7U) << 12U)
#else
 #define BSP_PRV_STARTUP_SCKDIVCR_PCLKA_BITS    (0U)
#endif
#if BSP_FEATURE_CGC_HAS_BCLK
 #define BSP_PRV_STARTUP_SCKDIVCR_BCLK_BITS     ((BSP_CFG_BCLK_DIV & 0x7U) << 16U)
#elif BSP_FEATURE_CGC_SCKDIVCR_BCLK_MATCHES_PCLKB

/* Some MCUs have a requirement that bits 18-16 be set to the same value as the bits for configuring the PCLKB divisor. */
 #define BSP_PRV_STARTUP_SCKDIVCR_BCLK_BITS     ((BSP_CFG_PCLKB_DIV & 0x7U) << 16U)
#else
 #define BSP_PRV_STARTUP_SCKDIVCR_BCLK_BITS     (0U)
#endif
#if BSP_FEATURE_CGC_HAS_FCLK
 #define BSP_PRV_STARTUP_SCKDIVCR_FCLK_BITS     ((BSP_CFG_FCLK_DIV & 0x7U) << 28U)
#else
 #define BSP_PRV_STARTUP_SCKDIVCR_FCLK_BITS     (0U)
#endif
#define BSP_PRV_STARTUP_SCKDIVCR                (BSP_PRV_STARTUP_SCKDIVCR_ICLK_BITS |  \
                                                 BSP_PRV_STARTUP_SCKDIVCR_PCLKD_BITS | \
                                                 BSP_PRV_STARTUP_SCKDIVCR_PCLKC_BITS | \
                                                 BSP_PRV_STARTUP_SCKDIVCR_PCLKB_BITS | \
                                                 BSP_PRV_STARTUP_SCKDIVCR_PCLKA_BITS | \
                                                 BSP_PRV_STARTUP_SCKDIVCR_BCLK_BITS |  \
                                                 BSP_PRV_STARTUP_SCKDIVCR_FCLK_BITS)

/* The number of clocks is used to size the g_clock_freq array. */
#if BSP_PRV_PLL_SUPPORTED
 #define BSP_PRV_NUM_CLOCKS                     ((uint8_t) BSP_CLOCKS_SOURCE_CLOCK_PLL + 1U)
#else
 #define BSP_PRV_NUM_CLOCKS                     ((uint8_t) BSP_CLOCKS_SOURCE_CLOCK_SUBCLOCK + 1U)
#endif

/* Calculate PLLCCR value. */
#if BSP_PRV_PLL_SUPPORTED
 #if (1U == BSP_FEATURE_CGC_PLLCCR_TYPE)
  #if BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC == BSP_CFG_PLL_SOURCE
   #define BSP_PRV_PLSRCSEL              (0)
  #elif BSP_CLOCKS_SOURCE_CLOCK_HOCO == BSP_CFG_PLL_SOURCE
   #define BSP_PRV_PLSRCSEL              (1)
  #endif
  #define BSP_PRV_PLLCCR_PLLMUL_MASK     (0x3F) // PLLMUL in PLLCCR is 6 bits wide
  #define BSP_PRV_PLLCCR_PLLMUL_BIT      (8)    // PLLMUL in PLLCCR starts at bit 8
  #define BSP_PRV_PLLCCR_PLSRCSEL_BIT    (4)    // PLSRCSEL in PLLCCR starts at bit 4
  #define BSP_PRV_PLLCCR                 ((((BSP_CFG_PLL_MUL & BSP_PRV_PLLCCR_PLLMUL_MASK) <<   \
                                            BSP_PRV_PLLCCR_PLLMUL_BIT) |                        \
                                           (BSP_PRV_PLSRCSEL << BSP_PRV_PLLCCR_PLSRCSEL_BIT)) | \
                                          BSP_CFG_PLL_DIV)
 #endif
 #if (2U == BSP_FEATURE_CGC_PLLCCR_TYPE)
  #define BSP_PRV_PLLCCR2_PLLMUL_MASK    (0x1F) // PLLMUL in PLLCCR2 is 5 bits wide
  #define BSP_PRV_PLLCCR2_PLODIV_BIT     (6)    // PLODIV in PLLCCR2 starts at bit 6

  #define BSP_PRV_PLLCCR2_PLLMUL         (BSP_CFG_PLL_MUL >> 1)
  #define BSP_PRV_PLLCCR                 (BSP_PRV_PLLCCR2_PLLMUL & BSP_PRV_PLLCCR2_PLLMUL_MASK) | \
    (BSP_CFG_PLL_DIV << BSP_PRV_PLLCCR2_PLODIV_BIT)
 #endif
#endif

#if BSP_FEATURE_CGC_HAS_PLL2
 #if BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC == BSP_CFG_PLL2_SOURCE
  #define BSP_PRV_PL2SRCSEL                (0)
 #elif BSP_CLOCKS_SOURCE_CLOCK_HOCO == BSP_CFG_PLL2_SOURCE
  #define BSP_PRV_PL2SRCSEL                (1)
 #endif
 #define BSP_PRV_PLL2CCR                   ((BSP_CFG_PLL2_MUL << R_SYSTEM_PLL2CCR_PLL2MUL_Pos) | \
                                            (BSP_CFG_PLL2_DIV << R_SYSTEM_PLL2CCR_PL2IDIV_Pos) | \
                                            (BSP_PRV_PL2SRCSEL << R_SYSTEM_PLL2CCR_PL2SRCSEL_Pos))
#endif

/* Determine the optimal operating speed mode to apply after clock configuration based on the startup clock
 * frequency. */
#if BSP_STARTUP_ICLK_HZ <= BSP_FEATURE_CGC_LOW_SPEED_MAX_FREQ_HZ && \
    BSP_CLOCKS_SOURCE_CLOCK_PLL != BSP_CFG_CLOCK_SOURCE
 #define BSP_PRV_STARTUP_OPERATING_MODE    (BSP_PRV_OPERATING_MODE_LOW_SPEED)
#elif BSP_STARTUP_ICLK_HZ <= BSP_FEATURE_CGC_MIDDLE_SPEED_MAX_FREQ_HZ
 #define BSP_PRV_STARTUP_OPERATING_MODE    (BSP_PRV_OPERATING_MODE_MIDDLE_SPEED)
#else
 #define BSP_PRV_STARTUP_OPERATING_MODE    (BSP_PRV_OPERATING_MODE_HIGH_SPEED)
#endif

#if !BSP_CFG_SOFT_RESET_SUPPORTED

static void bsp_prv_clock_set_hard_reset (void)
{
    /* Wait states in SRAMWTSC are set after hard reset. No change required here. */

    /* Calculate the wait states for ROM */
 #if BSP_FEATURE_CGC_HAS_FLWT
  #if BSP_STARTUP_ICLK_HZ <= BSP_FEATURE_BSP_SYS_CLOCK_FREQ_ONE_ROM_WAITS

    /* Do nothing. Default setting in FLWT is correct. */
  #elif BSP_STARTUP_ICLK_HZ <= BSP_FEATURE_BSP_SYS_CLOCK_FREQ_TWO_ROM_WAITS
    R_FCACHE->FLWT = BSP_PRV_ROM_ONE_WAIT_CYCLES;
  #elif 0 == BSP_FEATURE_BSP_SYS_CLOCK_FREQ_THREE_ROM_WAITS || \
    (BSP_STARTUP_ICLK_HZ <= BSP_FEATURE_BSP_SYS_CLOCK_FREQ_THREE_ROM_WAITS)
    R_FCACHE->FLWT = BSP_PRV_ROM_TWO_WAIT_CYCLES;
  #else
    R_FCACHE->FLWT = BSP_PRV_ROM_THREE_WAIT_CYCLES;
  #endif
 #endif

 #if BSP_FEATURE_CGC_HAS_MEMWAIT
  #if BSP_STARTUP_ICLK_HZ > BSP_PRV_MEMWAIT_MAX_ZERO_WAIT_FREQ

    /* The MCU must be in high speed mode to set wait states to 2. High speed mode is the default out of reset. */
    R_SYSTEM->MEMWAIT = BSP_PRV_MEMWAIT_TWO_WAIT_CYCLES;
  #endif
 #endif

 #if BSP_FEATURE_CGC_HAS_FLDWAITR
  #if BSP_STARTUP_ICLK_HZ > BSP_PRV_FLDWAITR_MAX_ONE_WAIT_FREQ

    /* The MCU must be in high speed mode to set wait states to 2. High speed mode is the default out of reset. */
    BSP_PRV_FLDWAITR_REG_ACCESS = BSP_PRV_FLDWAITR_TWO_WAIT_CYCLES;
  #endif
 #endif

    /* In order to avoid a system clock (momentarily) higher than expected, the order of switching the clock and
     * dividers must be so that the frequency of the clock goes lower, instead of higher, before being correct. */

    /* ICLK divider at reset is lowest possible, so set dividers first. */

    /* Set the system dividers first if ICLK divisor is larger than reset value. */
 #if BSP_CFG_ICLK_DIV >= BSP_FEATURE_CGC_ICLK_DIV_RESET
    R_SYSTEM->SCKDIVCR = BSP_PRV_STARTUP_SCKDIVCR;
 #endif

    /* Set the system source clock */
    R_SYSTEM->SCKSCR = BSP_CFG_CLOCK_SOURCE;

    /* Set the system dividers after setting the system clock source if ICLK divisor is smaller than reset value. */
 #if BSP_CFG_ICLK_DIV < BSP_FEATURE_CGC_ICLK_DIV_RESET
    R_SYSTEM->SCKDIVCR = BSP_PRV_STARTUP_SCKDIVCR;
 #endif

    /* Clock is now at requested frequency. */

    /* Update the CMSIS core clock variable so that it reflects the new ICLK frequency. */
   // SystemCoreClockUpdate();

    /* Adjust the MCU specific wait state soon after the system clock is set, if the system clock frequency to be
     * set is lower than previous. */
 #if BSP_FEATURE_CGC_HAS_SRAMWTSC
  #if BSP_STARTUP_ICLK_HZ <= BSP_FEATURE_BSP_SYS_CLOCK_FREQ_NO_RAM_WAITS
   #if BSP_FEATURE_CGC_HAS_SRAMPRCR2 == 1
    R_SRAM->SRAMPRCR2 = BSP_PRV_SRAM_UNLOCK;
    R_SRAM->SRAMWTSC  = BSP_PRV_SRAMWTSC_ZERO_WAIT_CYCLES;
    R_SRAM->SRAMPRCR2 = BSP_PRV_SRAM_LOCK;
   #else
    R_SRAM->SRAMPRCR = BSP_PRV_SRAM_UNLOCK;
    R_SRAM->SRAMWTSC = BSP_PRV_SRAMWTSC_ZERO_WAIT_CYCLES;
    R_SRAM->SRAMPRCR = BSP_PRV_SRAM_LOCK;
   #endif
  #endif
 #endif

    /* ROM wait states are 0 by default.  No change required here. */
}

#endif


/** Used for holding reference counters for protection bits. */
static volatile uint16_t g_protect_counters[] =
{
    0U, 0U, 0U, 0U
};

/** Masks for setting or clearing the PRCR register. Use -1 for size because PWPR in MPC is used differently. */
static const uint16_t g_prcr_masks[] =
{
    0x0001U,                           /* PRC0. */
    0x0002U,                           /* PRC1. */
    0x0008U,                           /* PRC3. */
    0x0010U,                           /* PRC4. */
};

/*******************************************************************************************************************//**
 *        Enable register protection. Registers that are protected cannot be written to. Register protection is
 *          enabled by using the Protect Register (PRCR) and the MPC's Write-Protect Register (PWPR).
 *
 * @param[in] regs_to_protect Registers which have write protection enabled.
 **********************************************************************************************************************/

void R_BSP_RegisterProtectEnable (bsp_reg_protect_t regs_to_protect)
{
    /** Get/save the current state of interrupts */
    FSP_CRITICAL_SECTION_DEFINE;
    FSP_CRITICAL_SECTION_ENTER;

    /* Is it safe to disable write access? */
    if (0U != g_protect_counters[regs_to_protect])
    {
        /* Decrement the protect counter */
        g_protect_counters[regs_to_protect]--;
    }

    /* Is it safe to disable write access? */
    if (0U == g_protect_counters[regs_to_protect])
    {
        /** Enable protection using PRCR register. */

        /** When writing to the PRCR register the upper 8-bits must be the correct key. Set lower bits to 0 to
         * disable writes. */
        R_SYSTEM->PRCR = ((R_SYSTEM->PRCR | BSP_PRV_PRCR_KEY) & (uint16_t) (~g_prcr_masks[regs_to_protect]));
    }

    /** Restore the interrupt state */
    FSP_CRITICAL_SECTION_EXIT;
}
/*******************************************************************************************************************//**
 *        Disable register protection. Registers that are protected cannot be written to. Register protection is
 *          disabled by using the Protect Register (PRCR) and the MPC's Write-Protect Register (PWPR).
 *
 * @param[in] regs_to_unprotect Registers which have write protection disabled.
 **********************************************************************************************************************/

void R_BSP_RegisterProtectDisable (bsp_reg_protect_t regs_to_unprotect)
{
    /** Get/save the current state of interrupts */
    FSP_CRITICAL_SECTION_DEFINE;
    FSP_CRITICAL_SECTION_ENTER;

    /* If this is first entry then disable protection. */
    if (0U == g_protect_counters[regs_to_unprotect])
    {
        /** Disable protection using PRCR register. */

        /** When writing to the PRCR register the upper 8-bits must be the correct key. Set lower bits to 0 to
         * disable writes. */
        R_SYSTEM->PRCR = ((R_SYSTEM->PRCR | BSP_PRV_PRCR_KEY) | g_prcr_masks[regs_to_unprotect]);
    }

    /** Increment the protect counter */
    g_protect_counters[regs_to_unprotect]++;

    /** Restore the interrupt state */
    FSP_CRITICAL_SECTION_EXIT;
}

#if !BSP_CFG_USE_LOW_VOLTAGE_MODE
/***********************************************************************************************************************
 * Changes the operating speed in OPCCR.  Assumes the LPM registers are unlocked in PRCR and cache is off.
 *
 * @param[in]  operating_mode  Desired operating mode, must be one of the BSP_PRV_OPERATING_MODE_* macros, cannot be
 *                             BSP_PRV_OPERATING_MODE_SUBOSC_SPEED
 **********************************************************************************************************************/
static void bsp_prv_operating_mode_opccr_set (uint8_t operating_mode)
{
 #if BSP_FEATURE_CGC_HOCOSF_BEFORE_OPCCR

    /* If the desired operating mode is already set, return. */
    if (operating_mode == R_SYSTEM->OPCCR)
    {
        return;
    }

    /* On some MCUs, the HOCO must be stable before updating OPCCR.OPCM. */
    if (0U == R_SYSTEM->HOCOCR)
    {
        /* Wait for HOCO to stabilize. */
        FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->OSCSF_b.HOCOSF, 1U);
    }
 #endif

    /* Wait for transition to complete. */
    FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->OPCCR_b.OPCMTSF, 0U);

    /* Apply requested operating speed mode. */
    R_SYSTEM->OPCCR = operating_mode;

    /* Wait for transition to complete. */
    while(R_SYSTEM->OPCCR_b.OPCMTSF != 0U){/* wait */}
}

/* Changes the operating speed mode.  Assumes the LPM registers are unlocked in PRCR and cache is off.
 *
 * @param[in]  operating_mode  Desired operating mode, must be one of the BSP_PRV_OPERATING_MODE_* macros
 **********************************************************************************************************************/
#if 0
void bsp_prv_operating_mode_set (uint8_t operating_mode)
{
    if (BSP_PRV_OPERATING_MODE_SUBOSC_SPEED == operating_mode)
    {
        /* Wait for transition to complete. */
        FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->SOPCCR_b.SOPCMTSF, 0U);

        /* Set subosc speed mode. */
        R_SYSTEM->SOPCCR = 0x1U;

        /* Wait for transition to complete. */
        FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->SOPCCR_b.SOPCMTSF, 0U);
    }
    else
    {
        /* Wait for transition to complete. */
        FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->SOPCCR_b.SOPCMTSF, 0U);

        /* Exit subosc speed mode first. */
        R_SYSTEM->SOPCCR = 0U;

        /* Wait for transition to complete. Check the entire register here since it should be set to 0 at this point.
         * Checking the entire register is slightly more efficient. This will also hang the program if the LPM
         * registers are not unlocked, which can help catch programming errors. */
        FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->SOPCCR, 0U);

        bsp_prv_operating_mode_opccr_set(operating_mode);
    }
}
#endif

#endif

/**
 * @brief Setup various clocks on SoC at boot time.
 *
 * Setup Slow, Main, PLLA, Processor and Master clocks during the device boot.
 * It is assumed that the relevant registers are at their reset value.
 */
static ALWAYS_INLINE void clock_init(void)
{
	   /* Unlock CGC and LPM protection registers. */
	    R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_UNLOCK;

	#if BSP_FEATURE_BSP_FLASH_CACHE_DISABLE_OPM
	 #if !BSP_CFG_USE_LOW_VOLTAGE_MODE && BSP_FEATURE_BSP_FLASH_CACHE

	    /* Disable flash cache before modifying MEMWAIT, SOPCCR, or OPCCR. */
	    R_BSP_FlashCacheDisable();
	 #endif
	#else

	    /* Enable the flash cache and don't disable it while running from flash. On these MCUs, the flash cache does not
	     * need to be disabled when adjusting the operating power mode. */
	    R_BSP_FlashCacheEnable();
	#endif
	#if BSP_FEATURE_BSP_FLASH_PREFETCH_BUFFER

	    /* Disable the flash prefetch buffer. */
	    R_FACI_LP->PFBER = 0;
	#endif

	  // bsp_clock_freq_var_init();

	#if BSP_CFG_SOFT_RESET_SUPPORTED

	    /* Update the main oscillator drive, source, and wait states if the main oscillator is stopped.  If the main
	     * oscillator is running, the drive, source, and wait states are assumed to be already set appropriately. */
	    if (R_SYSTEM->MOSCCR)
	    {
	        /* Don't write to MOSCWTCR unless MOSTP is 1 and MOSCSF = 0. */
	        while(R_SYSTEM->OSCSF_b.MOSCSF != 0U){ /* Wait. */}

	        /* Configure main oscillator drive. */
	        R_SYSTEM->MOMCR = BSP_PRV_MOMCR;

	        /* Set the main oscillator wait time. */
	        R_SYSTEM->MOSCWTCR = (uint8_t) BSP_CLOCK_CFG_MAIN_OSC_WAIT;
	    }

	#else

	    /* Configure main oscillator drive. */
	    R_SYSTEM->MOMCR = BSP_PRV_MOMCR;

	    /* Set the main oscillator wait time. */
	    R_SYSTEM->MOSCWTCR = (uint8_t) BSP_CLOCK_CFG_MAIN_OSC_WAIT;
	#endif

	#if BSP_CLOCK_CFG_SUBCLOCK_POPULATED

	    /* If the board has a subclock, set the subclock drive and start the subclock if the subclock is stopped.  If the
	     * subclock is running, the subclock drive is assumed to be set appropriately. */
	    if (R_SYSTEM->SOSCCR)
	    {
	        /* Configure the subclock drive if the subclock is not already running. */
	        R_SYSTEM->SOMCR  = ((BSP_CLOCK_CFG_SUBCLOCK_DRIVE << BSP_FEATURE_CGC_SODRV_SHIFT) & BSP_FEATURE_CGC_SODRV_MASK);
	        R_SYSTEM->SOSCCR = 0U;
	 #if BSP_CLOCKS_SOURCE_CLOCK_SUBCLOCK == BSP_CFG_CLOCK_SOURCE

	        /* If the subclock is the system clock source, wait for it to stabilize. */
	        R_BSP_SoftwareDelay(BSP_CLOCK_CFG_SUBCLOCK_STABILIZATION_MS, BSP_DELAY_UNITS_MILLISECONDS);
	 #endif
	    }

	#else
	    R_SYSTEM->SOSCCR = 1U;
	#endif

	#if BSP_FEATURE_CGC_HAS_HOCOWTCR
	 #if BSP_FEATURE_CGC_HOCOWTCR_64MHZ_ONLY

	    /* These MCUs only require writes to HOCOWTCR if HOCO is set to 64 MHz. */
	  #if 64000000 == BSP_HOCO_HZ
	   #if BSP_CFG_USE_LOW_VOLTAGE_MODE

	    /* Wait for HOCO to stabilize before writing to HOCOWTCR. */
	    while(R_SYSTEM->OSCSF_b.HOCOSF != 1U){ /* Wait. */}
	   #else

	    /* HOCO is assumed to be stable because these MCUs also require the HOCO to be stable before changing the operating
	     * power control mode. */
	   #endif
	    R_SYSTEM->HOCOWTCR = BSP_PRV_MAXIMUM_HOCOWTR_HSTS;
	  #endif
	 #else

	    /* These MCUs require HOCOWTCR to be set to the maximum value except in snooze mode.  There is no restriction to
	     * writing this register. */
	    R_SYSTEM->HOCOWTCR = BSP_PRV_MAXIMUM_HOCOWTR_HSTS;
	 #endif
	#endif

	#if !BSP_CFG_USE_LOW_VOLTAGE_MODE
	 #if BSP_CFG_SOFT_RESET_SUPPORTED

	    /* Switch to high-speed to prevent any issues with the subsequent clock configurations. */
	    bsp_prv_operating_mode_set(BSP_PRV_OPERATING_MODE_HIGH_SPEED);
	 #elif BSP_FEATURE_CGC_LOW_VOLTAGE_MAX_FREQ_HZ > 0U

	    /* MCUs that support low voltage mode start up in low voltage mode. */
	    bsp_prv_operating_mode_opccr_set(BSP_PRV_OPERATING_MODE_HIGH_SPEED);

	  #if BSP_CLOCKS_SOURCE_CLOCK_HOCO != BSP_CFG_CLOCK_SOURCE && BSP_CLOCKS_SOURCE_CLOCK_HOCO != BSP_CFG_PLL_SOURCE

	    /* HOCO must be running during startup in low voltage mode. If HOCO is not used, turn it off after exiting low
	     * voltage mode. */
	    R_SYSTEM->HOCOCR = 1U;
	  #endif
	 #elif BSP_FEATURE_CGC_STARTUP_OPCCR_MODE != BSP_PRV_OPERATING_MODE_HIGH_SPEED

	    /* Some MCUs do not start in high speed mode. */
	    bsp_prv_operating_mode_opccr_set(BSP_PRV_OPERATING_MODE_HIGH_SPEED);
	 #endif
	#endif

	    /* If the PLL is the desired source clock, ensure the source clock is running and stable and the power mode
	     * allows PLL operation. */
	#if BSP_PRV_PLL_SUPPORTED
	 #if BSP_CLOCKS_SOURCE_CLOCK_PLL == BSP_CFG_CLOCK_SOURCE

	    /* Start PLL source clock. */
	  #if BSP_CLOCKS_SOURCE_CLOCK_HOCO == BSP_CFG_PLL_SOURCE
	    R_SYSTEM->HOCOCR = 0U;
	  #else
	    R_SYSTEM->MOSCCR = 0U;
	  #endif

	    /* Configure the PLL registers. */
	  #if 1U == BSP_FEATURE_CGC_PLLCCR_TYPE
	    R_SYSTEM->PLLCCR = (uint16_t) BSP_PRV_PLLCCR;
	  #elif 2U == BSP_FEATURE_CGC_PLLCCR_TYPE
	    R_SYSTEM->PLLCCR2 = (uint8_t) BSP_PRV_PLLCCR;
	  #endif

	  #if BSP_FEATURE_CGC_PLLCCR_WAIT_US > 0

	    /* This loop is provided to ensure at least 1 us passes between setting PLLMUL and clearing PLLSTP on some
	     * MCUs (see PLLSTP notes in Section 8.2.4 "PLL Control Register (PLLCR)" of the RA4M1 manual R01UH0887EJ0100).
	     * Five loops are needed here to ensure the most efficient path takes at least 1 us from the setting of
	     * PLLMUL to the clearing of PLLSTP. HOCO is the fastest clock we can be using here since PLL cannot be running
	     * while setting PLLCCR. */
	    // FIXME software_delay_loop(9);
	    software_delay_loop(9);
	  #endif

	    /* Verify PLL source is stable before starting PLL. */
	  #if BSP_CLOCKS_SOURCE_CLOCK_HOCO == BSP_CFG_PLL_SOURCE

	    /* Wait for HOCO to stabilize. */
	    while(R_SYSTEM->OSCSF_b.HOCOSF != 1U){ /* Wait. */}
	  #else

	    /* Wait for main oscillator to stabilize. */
	// FIXME   while(R_SYSTEM->OSCSF_b.MOSCSF != 1U){ /* Wait. */}
	    while(R_SYSTEM->OSCSF_b.MOSCSF != 1U){ /* Wait. */}
	  #endif
	 #endif
	#endif

	    /* Start source clock. */
	#if BSP_CLOCKS_SOURCE_CLOCK_HOCO == BSP_CFG_CLOCK_SOURCE
	    R_SYSTEM->HOCOCR = 0U;

	    /* Wait for HOCO to stabilize. */
	    while(R_SYSTEM->OSCSF_b.HOCOSF != 1U){ /* Wait. */}
	#elif BSP_CLOCKS_SOURCE_CLOCK_MOCO == BSP_CFG_CLOCK_SOURCE
	 #if BSP_CFG_SOFT_RESET_SUPPORTED

	    /* If the MOCO is not running, start it and wait for it to stabilize using a software delay. */
	    if (0U != R_SYSTEM->MOCOCR)
	    {
	        R_SYSTEM->MOCOCR = 0U;
	        R_BSP_SoftwareDelay(BSP_FEATURE_CGC_MOCO_STABILIZATION_MAX_US, BSP_DELAY_UNITS_MICROSECONDS);
	    }
	 #endif
	#elif BSP_CLOCKS_SOURCE_CLOCK_LOCO == BSP_CFG_CLOCK_SOURCE
	 #if BSP_CFG_SOFT_RESET_SUPPORTED

	    /* If the LOCO is not running, start it and wait for it to stabilize using a software delay. */
	    if (0U != R_SYSTEM->LOCOCR)
	    {
	        R_SYSTEM->LOCOCR = 0U;
	        R_BSP_SoftwareDelay(BSP_FEATURE_CGC_LOCO_STABILIZATION_MAX_US, BSP_DELAY_UNITS_MICROSECONDS);
	    }

	 #else
	    R_SYSTEM->LOCOCR = 0U;
	    R_BSP_SoftwareDelay(BSP_FEATURE_CGC_LOCO_STABILIZATION_MAX_US, BSP_DELAY_UNITS_MICROSECONDS);
	 #endif
	#elif BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC == BSP_CFG_CLOCK_SOURCE
	    R_SYSTEM->MOSCCR = 0U;

	    /* Wait for main oscillator to stabilize. */
	    while(R_SYSTEM->OSCSF_b.MOSCSF != 1U){ /* Wait. */}
	#elif BSP_CLOCKS_SOURCE_CLOCK_PLL == BSP_CFG_CLOCK_SOURCE
	    R_SYSTEM->PLLCR = 0U;

	    /* Wait for PLL to stabilize. */
	    while(R_SYSTEM->OSCSF_b.PLLSF != 1U){ /* Wait. */}
	#else

	    /* Do nothing. Subclock is already started and stabilized if it is populated and selected as system clock. */
	#endif

	    /* Set source clock and dividers. */
	#if BSP_CFG_SOFT_RESET_SUPPORTED
	    bsp_prv_clock_set(BSP_CFG_CLOCK_SOURCE, BSP_PRV_STARTUP_SCKDIVCR);
	#else
	    bsp_prv_clock_set_hard_reset();
	#endif

	    /* If the MCU can run in a lower power mode, apply the optimal operating speed mode. */
	#if !BSP_CFG_USE_LOW_VOLTAGE_MODE
	 #if BSP_PRV_STARTUP_OPERATING_MODE != BSP_PRV_OPERATING_MODE_HIGH_SPEED
	  #if BSP_PRV_PLL_SUPPORTED
	   #if BSP_CFG_SOFT_RESET_SUPPORTED
	    if (BSP_PRV_OPERATING_MODE_LOW_SPEED == BSP_PRV_STARTUP_OPERATING_MODE)
	    {
	        /* If the MCU has a PLL, ensure PLL is stopped and stable before entering low speed mode. */
	        R_SYSTEM->PLLCR = 1U;

	        /* Wait for PLL to stabilize. */
	        while(R_SYSTEM->OSCSF_b.PLLSF != 0U){ /* Wait. */}
	    }
	   #endif
	  #endif
	    bsp_prv_operating_mode_set(BSP_PRV_STARTUP_OPERATING_MODE);
	 #endif
	#endif

	    /* Configure BCLK if it exists on the MCU. */
	#ifdef BSP_CFG_BCLK_OUTPUT
	 #if BSP_CFG_BCLK_OUTPUT > 0U
	    R_SYSTEM->BCKCR   = BSP_CFG_BCLK_OUTPUT - 1U;
	    R_SYSTEM->EBCKOCR = 1U;
	 #else
	  #if BSP_CFG_SOFT_RESET_SUPPORTED
	    R_SYSTEM->EBCKOCR = 0U;
	  #endif
	 #endif
	#endif

	    /* Configure SDRAM clock if it exists on the MCU. */
	#ifdef BSP_CFG_SDCLK_OUTPUT
	    R_SYSTEM->SDCKOCR = BSP_CFG_SDCLK_OUTPUT;
	#endif

	    /* Configure CLKOUT. */
	#if BSP_CFG_CLKOUT_SOURCE == BSP_CLOCKS_CLOCK_DISABLED
	 #if BSP_CFG_SOFT_RESET_SUPPORTED
	    R_SYSTEM->CKOCR = 0U;
	 #endif
	#else
	    uint8_t ckocr = BSP_CFG_CLKOUT_SOURCE | (BSP_CFG_CLKOUT_DIV << BSP_PRV_CKOCR_CKODIV_BIT);
	    R_SYSTEM->CKOCR = ckocr;
	    ckocr          |= (1U << BSP_PRV_CKOCR_CKOEN_BIT);
	    R_SYSTEM->CKOCR = ckocr;
	#endif

	#if BSP_PRV_STARTUP_OPERATING_MODE != BSP_PRV_OPERATING_MODE_LOW_SPEED
	 #if BSP_FEATURE_CGC_HAS_PLL2 && BSP_CFG_PLL2_SOURCE != BSP_CLOCKS_CLOCK_DISABLED

	    /* Start PLL source clock. */
	  #if BSP_CLOCKS_SOURCE_CLOCK_HOCO == BSP_CFG_PLL2_SOURCE
	    R_SYSTEM->HOCOCR = 0U;
	  #elif BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC == BSP_CFG_PLL2_SOURCE
	    R_SYSTEM->MOSCCR = 0U;
	    while(R_SYSTEM->OSCSF_b.MOSCSF != 1U){ /* Wait. */}
	  #endif                               /* BSP_CLOCKS_SOURCE_CLOCK_HOCO == BSP_CFG_PLL2_SOURCE */

	    R_SYSTEM->PLL2CCR = BSP_PRV_PLL2CCR;

	    /* Start the PLL. */
	    R_SYSTEM->PLL2CR = 0U;

	    /* Wait for the PLL to stabilize. */
	    while(R_SYSTEM->OSCSF_b.PLL2SF != 1U){ /* Wait. */}
	 #endif                                /* BSP_FEATURE_CGC_HAS_PLL2 && BSP_CFG_PLL2_ENABLE */

	 #if BSP_CFG_UCK_SOURCE != BSP_CLOCKS_CLOCK_DISABLED

	    /* If the USB clock has a divider setting in SCKDIVCR2. */
	  #if BSP_FEATURE_BSP_HAS_USB_CLOCK_DIV && !BSP_FEATURE_BSP_HAS_USBCKDIVCR
	    R_SYSTEM->SCKDIVCR2 = BSP_PRV_UCK_DIV << BSP_PRV_SCKDIVCR2_UCK_BIT;
	  #endif                               /* BSP_FEATURE_BSP_HAS_USB_CLOCK_DIV && !BSP_FEATURE_BSP_HAS_USBCKDIVCR */

	    /* If there is a REQ bit in USBCKCR than follow sequence from section 8.2.29 in RA6M4 hardware manual R01UH0890EJ0050. */
	  #if BSP_FEATURE_BSP_HAS_USB_CLOCK_REQ

	    /* Request to change the USB Clock. */
	    R_SYSTEM->USBCKCR_b.USBCKSREQ = 1;

	    /* Wait for the clock to be stopped. */
	    while(R_SYSTEM->USBCKCR_b.USBCKSRDY != 1U){ /* Wait. */}

	    /* Write the settings. */
	    R_SYSTEM->USBCKDIVCR = BSP_PRV_UCK_DIV;

	    /* Select the USB Clock without enabling it. */
	    R_SYSTEM->USBCKCR = BSP_CFG_UCK_SOURCE | R_SYSTEM_USBCKCR_USBCKSREQ_Msk;
	  #endif                               /* BSP_FEATURE_BSP_HAS_USB_CLOCK_REQ */

	  #if BSP_FEATURE_BSP_HAS_USB_CLOCK_SEL

	    /* Some MCUs use an alternate register for selecting the USB clock source. */
	   #if BSP_FEATURE_BSP_HAS_USB_CLOCK_SEL_ALT
	    #if BSP_CLOCKS_SOURCE_CLOCK_PLL == BSP_CFG_UCK_SOURCE

	    /* Write to USBCKCR to select the PLL. */
	    R_SYSTEM->USBCKCR_ALT = 0;
	    #elif BSP_CLOCKS_SOURCE_CLOCK_HOCO == BSP_CFG_UCK_SOURCE

	    /* Write to USBCKCR to select the HOCO. */
	    R_SYSTEM->USBCKCR_ALT = 1;
	    #endif
	   #else

	    /* Select the USB Clock. */
	    R_SYSTEM->USBCKCR = BSP_CFG_UCK_SOURCE;
	   #endif
	  #endif                               /* BSP_FEATURE_BSP_HAS_USB_CLOCK_REQ */

	  #if BSP_FEATURE_BSP_HAS_USB_CLOCK_REQ

	    /* Wait for the USB Clock to be started. */
	    while(R_SYSTEM->USBCKCR_b.USBCKSRDY != 0U){ /* Wait. */}
	  #endif                               /* BSP_FEATURE_BSP_HAS_USB_CLOCK_REQ */
	 #endif                                /* BSP_CFG_USB_ENABLE */
	#endif                                 /* BSP_PRV_STARTUP_OPERATING_MODE != BSP_PRV_OPERATING_MODE_LOW_SPEED */

	    /* Set the OCTASPI clock if it exists on the MCU (See section 8.2.30 of the RA6M4 hardware manual R01UH0890EJ0050). */
	#if BSP_FEATURE_BSP_HAS_OCTASPI_CLOCK && BSP_CFG_OCTA_SOURCE != BSP_CLOCKS_CLOCK_DISABLED
	    bsp_octaclk_settings_t octaclk_settings =
	    {
	        .source_clock = (bsp_clocks_source_t) BSP_CFG_OCTA_SOURCE,
	        .divider      = (bsp_clocks_octaclk_div_t) BSP_CFG_OCTA_DIV
	    };
	    R_BSP_OctaclkUpdate(&octaclk_settings);
	#endif                                 /* BSP_FEATURE_BSP_HAS_OCTASPI_CLOCK && BSP_CFG_OCTASPI_CLOCK_ENABLE */

	    /* Lock CGC and LPM protection registers. */
	    R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_LOCK;

	#if BSP_FEATURE_BSP_FLASH_CACHE && BSP_FEATURE_BSP_FLASH_CACHE_DISABLE_OPM
	    R_BSP_FlashCacheEnable();
	#endif

	#if BSP_FEATURE_BSP_FLASH_PREFETCH_BUFFER
	    R_FACI_LP->PFBER = 1;
	#endif
}

/*******************************************************************************************************************//**
 * Disable TRNG circuit to prevent unnecessary current draw which may otherwise occur when the Crypto module
 * is not in use.
 **********************************************************************************************************************/
#if BSP_FEATURE_BSP_RESET_TRNG
static void bsp_reset_trng_circuit (void)
{
    volatile uint8_t read_port = 0U;
    FSP_PARAMETER_NOT_USED(read_port); /// Prevent compiler 'unused' warning

    /* Release register protection for low power modes (per RA2A1 User's Manual (R01UH0888EJ0100) Figure 11.13 "Example
     * of initial setting flow for an unused circuit") */
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_OM_LPC_BATT);

    /* Enable TRNG function (disable stop function) */
 #if BSP_FEATURE_BSP_HAS_SCE_ON_RA2
    R_BSP_MODULE_START(FSP_IP_TRNG, 0); ///< TRNG Module Stop needs to be started/stopped for RA2 series.
 #elif BSP_FEATURE_BSP_HAS_SCE5
    R_BSP_MODULE_START(FSP_IP_SCE, 0);  ///< TRNG Module Stop needs to be started/stopped for RA4 series.
 #else
  #error "BSP_FEATURE_BSP_RESET_TRNG is defined but not handled."
 #endif

    /* Wait for at least 3 PCLKB cycles */
    read_port = R_PFS->PORT[0].PIN[0].PmnPFS_b.PODR;
    read_port = R_PFS->PORT[0].PIN[0].PmnPFS_b.PODR;
    read_port = R_PFS->PORT[0].PIN[0].PmnPFS_b.PODR;

    /* Disable TRNG function */
 #if BSP_FEATURE_BSP_HAS_SCE_ON_RA2
    R_BSP_MODULE_STOP(FSP_IP_TRNG, 0); ///< TRNG Module Stop needs to be started/stopped for RA2 series.
 #elif BSP_FEATURE_BSP_HAS_SCE5
    R_BSP_MODULE_STOP(FSP_IP_SCE, 0);  ///< TRNG Module Stop needs to be started/stopped for RA4 series.
 #else
  #error "BSP_FEATURE_BSP_RESET_TRNG is defined but not handled."
 #endif

    /* Reapply register protection for low power modes (per RA2A1 User's Manual (R01UH0888EJ0100) Figure 11.13 "Example
     * of initial setting flow for an unused circuit") */
    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_OM_LPC_BATT);
}

#endif


/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This has to be run at the very beginning thus the init priority is set at
 * 0 (zero).
 *
 * @return 0
 */
static int renesas_ra6_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	SystemCoreClock = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;

#if __FPU_USED

    /* Enable the FPU only when it is used.
     * Code taken from Section 7.1, Cortex-M4 TRM (DDI0439C) */

    /* Set bits 20-23 (CP10 and CP11) to enable FPU. */
    SCB->CPACR = (uint32_t) CP_MASK;
#endif

/* Set the Secure/Non-Secure VTOR to the vector table address based on the build. */
#if FSP_PRIV_TZ_USE_SECURE_REGS
    SCB->VTOR = (uint32_t) &__Vectors;
#endif

#if !BSP_TZ_CFG_SKIP_INIT
 #if BSP_FEATURE_BSP_VBATT_HAS_VBTCR1_BPWSWSTP

    /* Unlock VBTCR1 register. */
    R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_PRC1_UNLOCK;

    /* The VBTCR1.BPWSWSTP must be set after reset on MCUs that have VBTCR1.BPWSWSTP. Reference section 11.2.1
     * "VBATT Control Register 1 (VBTCR1)" and Figure 11.2 "Setting flow of the VBTCR1.BPWSWSTP bit" in the RA4M1 manual
     * R01UM0007EU0110. This must be done before bsp_clock_init because LOCOCR, LOCOUTCR, SOSCCR, and SOMCR cannot
     * be accessed until VBTSR.VBTRVLD is set. */
    R_SYSTEM->VBTCR1 = 1U;
    while (R_SYSTEM->VBTSR_b.VBTRVLD != 1U) { /* Wait. */}

    /* Lock VBTCR1 register. */
    R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_LOCK;
 #endif
#endif

#if BSP_TZ_CFG_SKIP_INIT

    /* Initialize clock variables to be used with R_BSP_SoftwareDelay. */
    bsp_clock_freq_var_init();
#else

    /* Configure system clocks. */
    clock_init();

 #if BSP_FEATURE_BSP_RESET_TRNG

    /* To prevent an undesired current draw, this MCU requires a reset
     * of the TRNG circuit after the clocks are initialized */

    bsp_reset_trng_circuit();
 #endif
#endif

    /*FIXME stack limit and stack top have to be defined*/
#if !BSP_FEATURE_BSP_HAS_SP_MON

    /* Disable MSP monitoring  */
    R_MPU_SPMON->SP[0].CTL = 0;

    /* Setup NMI interrupt  */
    R_MPU_SPMON->SP[0].OAD = BSP_STACK_POINTER_MONITOR_NMI_ON_DETECTION;

    /* Setup start address  */
    R_MPU_SPMON->SP[0].SA = BSP_PRV_STACK_LIMIT;

    /* Setup end address  */
    R_MPU_SPMON->SP[0].EA = BSP_PRV_STACK_TOP;

    /* Set SPEEN bit to enable NMI on stack monitor exception. NMIER bits cannot be cleared after reset, so no need
     * to read-modify-write. */
    R_ICU->NMIER = R_ICU_NMIER_SPEEN_Msk;

    /* Enable MSP monitoring  */
    R_MPU_SPMON->SP[0].CTL = 1U;
#endif

#if BSP_FEATURE_TZ_HAS_TRUSTZONE

    /* Use CM33 stack monitor. */
    __set_MSPLIM(BSP_PRV_STACK_LIMIT);
#endif

#if BSP_CFG_C_RUNTIME_INIT

    /* Initialize C runtime environment. */
    /* Zero out BSS */
 #if defined(__ARMCC_VERSION)
    memset((uint8_t *) &Image$$BSS$$ZI$$Base, 0U, (uint32_t) &Image$$BSS$$ZI$$Length);
 #elif defined(__GNUC__)
    memset(&__bss_start__, 0U, ((uint32_t) &__bss_end__ - (uint32_t) &__bss_start__));
 #elif defined(__ICCARM__)
    memset((uint32_t *) __section_begin(".bss"), 0U, (uint32_t) __section_size(".bss"));
 #endif

    /* Copy initialized RAM data from ROM to RAM. */
 #if defined(__ARMCC_VERSION)
    memcpy((uint8_t *) &Image$$DATA$$Base, (uint8_t *) &Load$$DATA$$Base, (uint32_t) &Image$$DATA$$Length);
 #elif defined(__GNUC__)
    memcpy(&__data_start__, &__etext, ((uint32_t) &__data_end__ - (uint32_t) &__data_start__));
 #elif defined(__ICCARM__)
    memcpy((uint32_t *) __section_begin(".data"), (uint32_t *) __section_begin(".data_init"),
           (uint32_t) __section_size(".data"));

    /* Copy functions to be executed from RAM. */
  #pragma section=".code_in_ram"
  #pragma section=".code_in_ram_init"
    memcpy((uint32_t *) __section_begin(".code_in_ram"),
           (uint32_t *) __section_begin(".code_in_ram_init"),
           (uint32_t) __section_size(".code_in_ram"));

    /* Copy main thread TLS to RAM. */
  #pragma section="__DLIB_PERTHREAD_init"
  #pragma section="__DLIB_PERTHREAD"
    memcpy((uint32_t *) __section_begin("__DLIB_PERTHREAD"), (uint32_t *) __section_begin("__DLIB_PERTHREAD_init"),
           (uint32_t) __section_size("__DLIB_PERTHREAD_init"));
 #endif

    /* Initialize static constructors */
 #if defined(__ARMCC_VERSION)
    int32_t count = Image$$INIT_ARRAY$$Limit - Image$$INIT_ARRAY$$Base;
    for (int32_t i = 0; i < count; i++)
    {
        void (* p_init_func)(void) =
            (void (*)(void))((uint32_t) &Image$$INIT_ARRAY$$Base + (uint32_t) Image$$INIT_ARRAY$$Base[i]);
        p_init_func();
    }

 #elif defined(__GNUC__)
    int32_t count = __init_array_end - __init_array_start;
    for (int32_t i = 0; i < count; i++)
    {
        __init_array_start[i]();
    }

 #elif defined(__ICCARM__)
    void const * pibase = __section_begin("SHT$$PREINIT_ARRAY");
    void const * ilimit = __section_end("SHT$$INIT_ARRAY");
    __call_ctors(pibase, ilimit);
 #endif
#endif                                 // BSP_CFG_C_RUNTIME_INIT

    /* Initialize SystemCoreClock variable. */
   // SystemCoreClockUpdate();

#if !BSP_CFG_PFS_PROTECT
 #if BSP_TZ_SECURE_BUILD
    R_PMISC->PWPRS = 0;                              ///< Clear BOWI bit - writing to PFSWE bit enabled
    R_PMISC->PWPRS = 1U << BSP_IO_PWPR_PFSWE_OFFSET; ///< Set PFSWE bit - writing to PFS register enabled
 #else
    R_PMISC->PWPR = 0;                               ///< Clear BOWI bit - writing to PFSWE bit enabled
    R_PMISC->PWPR = 1U << BSP_IO_PWPR_PFSWE_OFFSET;  ///< Set PFSWE bit - writing to PFS register enabled
 #endif
#endif

    /* Ensure that the PMSAR registers are reset (Soft reset does not reset PMSAR). */
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_SAR);

    for (uint32_t i = 0; i < 9; i++)
    {
        R_PMISC->PMSAR[i].PMSAR = UINT16_MAX;
    }

    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_SAR);

#if BSP_TZ_SECURE_BUILD

    /* Initialize security features. */
    R_BSP_SecurityInit();
#endif

    irq_unlock(key);

	return 0;
}

SYS_INIT(renesas_ra6_init, PRE_KERNEL_1, 0);
