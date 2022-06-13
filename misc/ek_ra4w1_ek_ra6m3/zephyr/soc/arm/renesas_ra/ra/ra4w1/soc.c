/*
 * Copyright (c) 2020 MXT Creation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Renesas RA4W1 MCU series initialization code
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Renesas RA4W1 series processor.
 */

#include "soc.h"

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>


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
	/* Initialize system using BSP. */
	SystemInit();

	return 0;
}

SYS_INIT(renesas_ra4_init, PRE_KERNEL_1, 0);
