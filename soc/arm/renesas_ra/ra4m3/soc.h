/*
 * Copyright (c) 2020 MXt Creation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Register access macros for the Renesas RA4 MCU.
 *
 * This file provides register access macros for the Renesas RA4 MCU, HAL
 * drivers for core peripherals as well as symbols specific to Renesas RA family.
 */

#ifndef _RENESAS_RA4M3_SOC_H_
#define _RENESAS_RA4M3_SOC_H_

/* Add include for DTS generated information */
#include <devicetree.h>
#include <bsp_feature.h>
#include <vector_data.h>
#include <renesas.h>
#include <bsp_api.h>

#define RA4M3_CPU_CLOCK_FREQ_HZ DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)

#endif /* _RENESAS_RA4M3_SOC_H_ */
