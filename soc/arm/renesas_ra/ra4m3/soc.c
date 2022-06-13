/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Renesas RA4M3 MCU series initialization code
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Renesas RA4M3 series processor.
 * A lot of this code is taken from bsp_clocks.c and only the basic configuration
 * has been kept, so look there if you need to add options.
 */

#include "soc.h"

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>
#include <bsp_feature.h>
#include <soc_ioport.h>

#if 0
/* Clock config settings */
#define BSP_CLOCK_CFG_H_
#define BSP_CFG_CLOCKS_SECURE (0)
#define BSP_CFG_CLOCKS_OVERRIDE (0)
#define BSP_CFG_XTAL_HZ (20000000) /* XTAL 20000000Hz */
#define BSP_CFG_HOCO_FREQUENCY (2) /* HOCO 20MHz */
#define BSP_CFG_PLL_SOURCE (BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC) /* PLL Src: XTAL */
#define BSP_CFG_PLL_DIV (BSP_CLOCKS_PLL_DIV_3) /* PLL Div /3 */
#define BSP_CFG_PLL_MUL BSP_CLOCKS_PLL_MUL_20_0 /* PLL Mul x20.0 */
#define BSP_CFG_PLL2_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* PLL2 Disabled */
#define BSP_CFG_PLL2_DIV (BSP_CLOCKS_PLL_DIV_2) /* PLL2 Div /2 */
#define BSP_CFG_PLL2_MUL BSP_CLOCKS_PLL_MUL_20_0 /* PLL2 Mul x20.0 */
#define BSP_CFG_CLOCK_SOURCE (BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC) /* Clock Src: XTAL */
#define BSP_CFG_CLKOUT_SOURCE (BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC) /* CLKOUT Src: XTAL */
#define BSP_CFG_UCK_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* UCLK Disabled */
#define BSP_CFG_ICLK_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_2) /* ICLK Div /2 */
#define BSP_CFG_PCLKA_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_2) /* PCLKA Div /2 */
#define BSP_CFG_PCLKB_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_4) /* PCLKB Div /4 */
#define BSP_CFG_PCLKC_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_4) /* PCLKC Div /4 */
#define BSP_CFG_PCLKD_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_2) /* PCLKD Div /2 */
#define BSP_CFG_FCLK_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_4) /* FCLK Div /4 */
#define BSP_CFG_CLKOUT_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_2) /* CLKOUT Div /2 */
#define BSP_CFG_UCK_DIV (BSP_CLOCKS_USB_CLOCK_DIV_5) /* UCLK Div /5 */
#endif

#define BSP_CFG_CLOCKS_SECURE (0)
#define BSP_CFG_CLOCKS_OVERRIDE (0)
#define BSP_CFG_XTAL_HZ (24000000) /* XTAL 24000000Hz */
#define BSP_CFG_HOCO_FREQUENCY (2) /* HOCO 20MHz */
#define BSP_CFG_PLL_SOURCE (BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC) /* PLL Src: XTAL */
#define BSP_CFG_PLL_DIV (BSP_CLOCKS_PLL_DIV_3) /* PLL Div /3 */
#define BSP_CFG_PLL_MUL BSP_CLOCKS_PLL_MUL_25_0 /* PLL Mul x25.0 */
#define BSP_CFG_PLL2_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* PLL2 Disabled */
#define BSP_CFG_PLL2_DIV (BSP_CLOCKS_PLL_DIV_2) /* PLL2 Div /2 */
#define BSP_CFG_PLL2_MUL BSP_CLOCKS_PLL_MUL_20_0 /* PLL2 Mul x20.0 */
#define BSP_CFG_CLOCK_SOURCE (BSP_CLOCKS_SOURCE_CLOCK_PLL) /* Clock Src: PLL */
#define BSP_CFG_CLKOUT_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* CLKOUT Disabled */
#define BSP_CFG_UCK_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* UCLK Disabled */
#define BSP_CFG_ICLK_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_2) /* ICLK Div /2 */
#define BSP_CFG_PCLKA_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_2) /* PCLKA Div /2 */
#define BSP_CFG_PCLKB_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_4) /* PCLKB Div /4 */
#define BSP_CFG_PCLKC_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_4) /* PCLKC Div /4 */
#define BSP_CFG_PCLKD_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_2) /* PCLKD Div /2 */
#define BSP_CFG_FCLK_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_4) /* FCLK Div /4 */
#define BSP_CFG_CLKOUT_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_1) /* CLKOUT Div /1 */
#define BSP_CFG_UCK_DIV (BSP_CLOCKS_USB_CLOCK_DIV_5) /* UCLK Div /5 */

#define CP_MASK (0xFU << 20)

#define BSP_IRQ_UINT32_MAX (0xFFFFFFFFU)
#define BSP_PRV_BITS_PER_WORD (32)

#define BSP_PRV_CKOCR_CKODIV_BIT (4U)
#define BSP_PRV_CKOCR_CKOEN_BIT (7U)

#define BSP_PRV_PRCR_KEY (0xA500U)
#define BSP_PRV_PRCR_UNLOCK ((BSP_PRV_PRCR_KEY) | 0x3U)
#define BSP_PRV_PRCR_LOCK ((BSP_PRV_PRCR_KEY) | 0x0U)
#define BSP_PRV_SRAM_PRCR_KEY (0x78U)
#define BSP_PRV_SRAM_UNLOCK (((BSP_PRV_SRAM_PRCR_KEY) << 1) | 0x1U)
#define BSP_PRV_SRAM_LOCK (((BSP_PRV_SRAM_PRCR_KEY) << 1) | 0x0U)

#define BSP_PRV_MOMCR_MOSEL_BIT (6)
#define BSP_PRV_MODRV ((CGC_MAINCLOCK_DRIVE << BSP_FEATURE_CGC_MODRV_SHIFT) & BSP_FEATURE_CGC_MODRV_MASK)
#define BSP_PRV_MOSEL (BSP_CLOCK_CFG_MAIN_OSC_CLOCK_SOURCE << BSP_PRV_MOMCR_MOSEL_BIT)
#define BSP_PRV_MOMCR (BSP_PRV_MODRV | BSP_PRV_MOSEL)

#define BSP_PRV_SRAMWTSC_WAIT_CYCLES_DISABLE (0U)
#define BSP_PRV_ROM_ONE_WAIT_CYCLES (1U)
#define BSP_PRV_NUM_CLOCKS ((uint8_t)BSP_CLOCKS_SOURCE_CLOCK_PLL2 + 1U)
#define BSP_PRV_STARTUP_SCKDIVCR_ICLK_BITS ((BSP_CFG_ICLK_DIV & 7U) << 24U)
#define BSP_PRV_STARTUP_SCKDIVCR_PCLKD_BITS (BSP_CFG_PCLKD_DIV & 0x7U)
#define BSP_PRV_STARTUP_SCKDIVCR_PCLKC_BITS ((BSP_CFG_PCLKC_DIV & 0x7U) << 4U)
#define BSP_PRV_STARTUP_SCKDIVCR_PCLKB_BITS ((BSP_CFG_PCLKB_DIV & 0x7U) << 8U)
#define BSP_PRV_STARTUP_SCKDIVCR_PCLKA_BITS ((BSP_CFG_PCLKA_DIV & 0x7U) << 12U)
#define BSP_PRV_STARTUP_SCKDIVCR_BCLK_BITS ((BSP_CFG_PCLKB_DIV & 0x7U) << 16U)
#define BSP_PRV_STARTUP_SCKDIVCR_FCLK_BITS ((BSP_CFG_FCLK_DIV & 0x7U) << 28U)

#define BSP_PRV_STARTUP_SCKDIVCR                                                                                       \
	(BSP_PRV_STARTUP_SCKDIVCR_ICLK_BITS | BSP_PRV_STARTUP_SCKDIVCR_PCLKD_BITS |                                    \
	 BSP_PRV_STARTUP_SCKDIVCR_PCLKC_BITS | BSP_PRV_STARTUP_SCKDIVCR_PCLKB_BITS |                                   \
	 BSP_PRV_STARTUP_SCKDIVCR_PCLKA_BITS | BSP_PRV_STARTUP_SCKDIVCR_BCLK_BITS |                                    \
	 BSP_PRV_STARTUP_SCKDIVCR_FCLK_BITS)

// typedef enum e_bsp_reg_protect
// {
//     /** Enables writing to the registers related to the clock generation circuit. */
//     BSP_REG_PROTECT_CGC = 0,

//     /** Enables writing to the registers related to operating modes, low power consumption, and battery backup
//      * function. */
//     BSP_REG_PROTECT_OM_LPC_BATT,

//     /** Enables writing to the registers related to the LVD: LVCMPCR, LVDLVLR, LVD1CR0, LVD1CR1, LVD1SR, LVD2CR0,
//      * LVD2CR1, LVD2SR. */
//     BSP_REG_PROTECT_LVD,

//     /** Enables writing to the registers related to the security function. */
//     BSP_REG_PROTECT_SAR,
// } bsp_reg_protect_t;

volatile uint16_t g_protect_counters[4] BSP_SECTION_EARLY_INIT;

static const uint16_t g_prcr_masks[] = {
	0x0001U, /* PRC0. */
	0x0002U, /* PRC1. */
	0x0008U, /* PRC3. */
	0x0010U, /* PRC4. */
};

/* This table is used to store the context in the ISR. */
void *gp_renesas_isr_context[BSP_ICU_VECTOR_MAX_ENTRIES];

const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_WEAK_REFERENCE = { (
	bsp_interrupt_event_t)0 };

/* This array stores the clock frequency of each system clock. This section of RAM should not be initialized by the C
 * runtime environment. This is initialized and used in bsp_clock_init, which is called before the C runtime
 * environment is initialized. */
static uint32_t g_clock_freq[BSP_PRV_NUM_CLOCKS] BSP_PLACE_IN_SECTION(BSP_SECTION_NOINIT);
/** System Clock Frequency (Core Clock) */
uint32_t SystemCoreClock BSP_SECTION_EARLY_INIT;

static void ra_bsp_registerProtectDisable(bsp_reg_protect_t regs_to_protect)
{
	/* Is it safe to disable write access? */
	if (0U != g_protect_counters[regs_to_protect]) {
		/* Decrement the protect counter */
		g_protect_counters[regs_to_protect]--;
	}

	/* Is it safe to disable write access? */
	if (0U == g_protect_counters[regs_to_protect]) {
		/** Enable protection using PRCR register. */

		/** When writing to the PRCR register the upper 8-bits must be the correct key. Set lower bits to 0 to
         * disable writes. */
		R_SYSTEM->PRCR = ((R_SYSTEM->PRCR | BSP_PRV_PRCR_KEY) & (uint16_t)(~g_prcr_masks[regs_to_protect]));
	}
}

static void ra_bsp_registerProtectEnable(bsp_reg_protect_t regs_to_protect)
{
	/* Is it safe to disable write access? */
	if (0U != g_protect_counters[regs_to_protect]) {
		/* Decrement the protect counter */
		g_protect_counters[regs_to_protect]--;
	}

	/* Is it safe to disable write access? */
	if (0U == g_protect_counters[regs_to_protect]) {
		/** Enable protection using PRCR register. */

		/** When writing to the PRCR register the upper 8-bits must be the correct key. Set lower bits to 0 to
         * disable writes. */
		R_SYSTEM->PRCR = ((R_SYSTEM->PRCR | BSP_PRV_PRCR_KEY) & (uint16_t)(~g_prcr_masks[regs_to_protect]));
	}
}

static void ra_update_system_core_clock(void)
{
	uint32_t clock_index = R_SYSTEM->SCKSCR;
	SystemCoreClock = g_clock_freq[clock_index] >> R_SYSTEM->SCKDIVCR_b.ICK;
}

static void clock_freq_var_init(void)
{
	g_clock_freq[BSP_CLOCKS_SOURCE_CLOCK_HOCO] = BSP_HOCO_HZ;
	g_clock_freq[BSP_CLOCKS_SOURCE_CLOCK_MOCO] = BSP_MOCO_FREQ_HZ;
	g_clock_freq[BSP_CLOCKS_SOURCE_CLOCK_LOCO] = BSP_LOCO_FREQ_HZ;
	g_clock_freq[BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC] = BSP_CFG_XTAL_HZ;
	g_clock_freq[BSP_CLOCKS_SOURCE_CLOCK_SUBCLOCK] = BSP_SUBCLOCK_FREQ_HZ;
	/* The PLL value will be calculated at initialization. */
	g_clock_freq[BSP_CLOCKS_SOURCE_CLOCK_PLL] = BSP_CFG_XTAL_HZ;
	ra_update_system_core_clock();
}

static void ra_clock_set_hard_reset(void)
{
	/* Calculate the wait states for ROM */
	R_FCACHE->FLWT = BSP_PRV_ROM_ONE_WAIT_CYCLES;

	/* Set the system source clock */
	R_SYSTEM->SCKSCR = BSP_CFG_CLOCK_SOURCE;

	/* Set the system dividers after setting the system clock source if ICLK divisor is smaller than reset value. */
#if BSP_CFG_ICLK_DIV < BSP_FEATURE_CGC_ICLK_DIV_RESET
	R_SYSTEM->SCKDIVCR = BSP_PRV_STARTUP_SCKDIVCR;
#endif

	/* Update the CMSIS core clock variable so that it reflects the new ICLK frequency. */
	ra_update_system_core_clock();

	/* Adjust the MCU specific wait state soon after the system clock is set, if the system clock frequency to be
     * set is lower than previous. */
	R_SRAM->SRAMPRCR2 = BSP_PRV_SRAM_UNLOCK;
	R_SRAM->SRAMWTSC = BSP_PRV_SRAMWTSC_WAIT_CYCLES_DISABLE;
	R_SRAM->SRAMPRCR2 = BSP_PRV_SRAM_LOCK;
}

static void ra_clock_init(void)
{
	/* Unlock CGC and LPM protection registers. */
	R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_UNLOCK;

	/* Enable the flash cache and don't disable it while running from flash. On these MCUs, the flash cache does not
     * need to be disabled when adjusting the operating power mode. */
	R_BSP_FlashCacheEnable();

	clock_freq_var_init();

	/* Configure main oscillator drive. */
	R_SYSTEM->MOMCR = BSP_PRV_MOMCR;

	/* Set the main oscillator wait time. */
	R_SYSTEM->MOSCWTCR = (uint8_t)BSP_CLOCK_CFG_MAIN_OSC_WAIT;

	/* If the board has a subclock, set the subclock drive and start the subclock if the subclock is stopped.  If the
     * subclock is running, the subclock drive is assumed to be set appropriately. */
	if (R_SYSTEM->SOSCCR) {
		/* Configure the subclock drive if the subclock is not already running. */
		R_SYSTEM->SOMCR =
			((BSP_CLOCK_CFG_SUBCLOCK_DRIVE << BSP_FEATURE_CGC_SODRV_SHIFT) & BSP_FEATURE_CGC_SODRV_MASK);
		R_SYSTEM->SOSCCR = 0U;
	}

	R_SYSTEM->MOSCCR = 0U;

	/* Wait for main oscillator to stabilize. */
	FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->OSCSF_b.MOSCSF, 1U);

	ra_clock_set_hard_reset();

	/* Configure CLKOUT. */
	uint8_t ckocr = BSP_CFG_CLKOUT_SOURCE | (BSP_CFG_CLKOUT_DIV << BSP_PRV_CKOCR_CKODIV_BIT);
	R_SYSTEM->CKOCR = ckocr;
	ckocr |= (1U << BSP_PRV_CKOCR_CKOEN_BIT);
	R_SYSTEM->CKOCR = ckocr;

	/* Lock CGC and LPM protection registers. */
	R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_LOCK;
}

static void ra_irq_cfg(void)
{
	/* Unprotect security registers. */
	ra_bsp_registerProtectDisable(BSP_REG_PROTECT_SAR);

	/* Set the DMAC channels to secure access. */
	R_CPSCU->ICUSARC = ~R_CPSCU_ICUSARC_SADMACn_Msk;

	/* Place all vectors in non-secure state unless they are used in the secure project. */
	uint32_t interrupt_security_state[BSP_ICU_VECTOR_MAX_ENTRIES / BSP_PRV_BITS_PER_WORD];
	memset(&interrupt_security_state, UINT8_MAX, sizeof(interrupt_security_state));

	for (uint32_t i = 0U; i < BSP_ICU_VECTOR_MAX_ENTRIES; i++) {
		if (0U != g_interrupt_event_link_select[i]) {
			/* This is a secure vector. Clear the associated bit. */
			uint32_t index = i / BSP_PRV_BITS_PER_WORD;
			uint32_t bit = i % BSP_PRV_BITS_PER_WORD;
			interrupt_security_state[index] &= ~(1U << bit);
		}
	}

	/* The Secure Attribute managed within the ARM CPU NVIC must match the security attribution of IELSEn
     * (Reference section 13.2.9 in the RA6M4 manual R01UH0890EJ0050). */
	uint32_t volatile *p_icusarg = &R_CPSCU->ICUSARG;
	for (uint32_t i = 0U; i < BSP_ICU_VECTOR_MAX_ENTRIES / BSP_PRV_BITS_PER_WORD; i++) {
		p_icusarg[i] = interrupt_security_state[i];
		NVIC->ITNS[i] = interrupt_security_state[i];
	}

	/* Protect security registers. */
	ra_bsp_registerProtectEnable(BSP_REG_PROTECT_SAR);

	for (uint32_t i = 0U; i < BSP_ICU_VECTOR_MAX_ENTRIES; i++) {
		R_ICU->IELSR[i] = (uint32_t)g_interrupt_event_link_select[i];
	}
}

#define BSP_PRV_STACK_LIMIT ((uint32_t)&__StackLimit)
#define BSP_PRV_STACK_TOP ((uint32_t)&__StackTop)

extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackLimit;
extern uint32_t __StackTop;
extern void (*__init_array_start[])(void);

extern void (*__init_array_end[])(void);

/* Vector table. */
#define BSP_SECTION_STACK BSP_UNINIT_SECTION_PREFIX ".stack"
#define BSP_SECTION_NOINIT BSP_UNINIT_SECTION_PREFIX ".noinit"
#define BSP_SECTION_FIXED_VECTORS ".fixed_vectors"
#define BSP_SECTION_APPLICATION_VECTORS ".application_vectors"
#define BSP_SECTION_ROM_REGISTERS ".rom_registers"
#define BSP_SECTION_ID_CODE ".id_code"
#define BSP_CORTEX_VECTOR_TABLE_ENTRIES (16U)
#define BSP_PLACE_IN_SECTION(x) __attribute__((section(x))) __attribute__((__used__))
#define BSP_SECTION_FIXED_VECTORS ".fixed_vectors"
#define BSP_CFG_STACK_MAIN_BYTES (0x400)
#define BSP_STACK_ALIGNMENT (8)
#define WEAK_REF_ATTRIBUTE __attribute__((weak, alias("Default_Handler")))
// void NMI_Handler(void);                // NMI has many sources and is handled by BSP
// void HardFault_Handler(void) WEAK_REF_ATTRIBUTE;
// void MemManage_Handler(void) WEAK_REF_ATTRIBUTE;
// void BusFault_Handler(void) WEAK_REF_ATTRIBUTE;
// void UsageFault_Handler(void) WEAK_REF_ATTRIBUTE;
// void SecureFault_Handler(void) WEAK_REF_ATTRIBUTE;
// void SVC_Handler(void) WEAK_REF_ATTRIBUTE;
// void DebugMon_Handler(void) WEAK_REF_ATTRIBUTE;
// void PendSV_Handler(void) WEAK_REF_ATTRIBUTE;
// void SysTick_Handler(void) WEAK_REF_ATTRIBUTE;
// static uint8_t g_main_stack[BSP_CFG_STACK_MAIN_BYTES] BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT)
// BSP_PLACE_IN_SECTION(BSP_SECTION_STACK);
// typedef void (* exc_ptr_t)(void);

// const exc_ptr_t __Vectors[BSP_CORTEX_VECTOR_TABLE_ENTRIES] BSP_PLACE_IN_SECTION(
//     BSP_SECTION_FIXED_VECTORS) =
// {
//     (exc_ptr_t) (&g_main_stack[0] + BSP_CFG_STACK_MAIN_BYTES), /*      Initial Stack Pointer     */
//     Reset_Handler,                                             /*      Reset Handler             */
//     NMI_Handler,                                               /*      NMI Handler               */
//     HardFault_Handler,                                         /*      Hard Fault Handler        */
//     MemManage_Handler,                                         /*      MPU Fault Handler         */
//     BusFault_Handler,                                          /*      Bus Fault Handler         */
//     UsageFault_Handler,                                        /*      Usage Fault Handler       */
//     SecureFault_Handler,                                       /*      Secure Fault Handler      */
//     0,                                                         /*      Reserved                  */
//     0,                                                         /*      Reserved                  */
//     0,                                                         /*      Reserved                  */
//     SVC_Handler,                                               /*      SVCall Handler            */
//     DebugMon_Handler,                                          /*      Debug Monitor Handler     */
//     0,                                                         /*      Reserved                  */
//     PendSV_Handler,                                            /*      PendSV Handler            */
//     SysTick_Handler,                                           /*      SysTick Handler           */
// };

static void setLEDS(void)
{
	struct soc_port_pin socPort;
	socPort.pin_cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH;
	socPort.pin = 415;
	soc_ioport_configure(&socPort);
	socPort.pin = 404;
	soc_ioport_configure(&socPort);
	socPort.pin = 400;
	soc_ioport_configure(&socPort);
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This has to be run at the very beginning thus the init priority is set at
 * 0 (zero).
 *
 * @return 0
 */
static int renesas_ra4_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	uint32_t key;

	key = irq_lock();

#if __FPU_USED

	/* Enable the FPU only when it is used.
     * Code taken from Section 7.1, Cortex-M4 TRM (DDI0439C) */

	/* Set bits 20-23 (CP10 and CP11) to enable FPU. */
	SCB->CPACR = (uint32_t)CP_MASK;
#endif
	/* VTOR is in undefined state out of RESET */
	//SCB->VTOR = (uint32_t) &__Vectors;

	setLEDS();

	/* Configure system clocks. */
	ra_clock_init();

	/* Use CM33 stack monitor. */
	__asm volatile("MSR msplim, %0" : : "r"(BSP_PRV_STACK_LIMIT));

	// /* Zero out BSS */
	// memset(&__bss_start__, 0U, ((uint32_t) &__bss_end__ - (uint32_t) &__bss_start__));
	// /* Copy initialized RAM data from ROM to RAM. */
	// memcpy(&__data_start__, &__etext, ((uint32_t) &__data_end__ - (uint32_t) &__data_start__));
	//  /* Initialize static constructors */
	// int32_t count = __init_array_end - __init_array_start;
	// for (int32_t i = 0; i < count; i++)
	// {
	//     __init_array_start[i]();
	// }

	/* Initialize SystemCoreClock variable. */
	ra_update_system_core_clock();

	/* Ensure that the PMSAR registers are reset (Soft reset does not reset PMSAR). */
	ra_bsp_registerProtectDisable(BSP_REG_PROTECT_SAR);

	for (uint32_t i = 0; i < 9; i++) {
		R_PMISC->PMSAR[i].PMSAR = UINT16_MAX;
	}
	ra_bsp_registerProtectEnable(BSP_REG_PROTECT_SAR);

	setLEDS();
	while (1) {
	}

	/* Initialize ELC events that will be used to trigger NVIC interrupts. */
	ra_irq_cfg();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(renesas_ra4_init, PRE_KERNEL_1, 0);
