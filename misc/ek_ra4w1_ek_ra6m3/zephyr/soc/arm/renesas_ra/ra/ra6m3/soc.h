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

#ifndef _RENESAS_RA6M3_SOC_H_
#define _RENESAS_RA6M3_SOC_H_

/* Add include for DTS generated information */
#include <devicetree.h>
#include <soc_vector_table.h>
#include <bsp_feature.h>
#include <renesas.h>
#include <bsp_clock_cfg.h>
//#define __FPU_PRESENT 1

/** Determine if a C++ compiler is being used.
 * If so, ensure that standard C is used to process the API information.  */
#if 1

#ifndef _ASMLANGUAGE

#if defined(__cplusplus)
 #define FSP_CPP_HEADER    extern "C" {
 #define FSP_CPP_FOOTER    }
#else
 #define FSP_CPP_HEADER
 #define FSP_CPP_FOOTER
#endif

/** FSP Header and Footer definitions */
#define FSP_HEADER             FSP_CPP_HEADER
#define FSP_FOOTER             FSP_CPP_FOOTER

//#define DONT_USE_CMSIS_INIT
//#define DONT_USE_PREDEFINED_CORE_HANDLERS
//#define DONT_USE_PREDEFINED_PERIPHERALS_HANDLERS

#define BSP_MCU_GROUP_RA6M3 (1)
#define BSP_LOCO_HZ                 (32768)
#define BSP_MOCO_HZ                 (8000000)
#define BSP_SUB_CLOCK_HZ            (32768)
#if   BSP_CFG_HOCO_FREQUENCY == 0
#define BSP_HOCO_HZ                 (16000000)
#elif BSP_CFG_HOCO_FREQUENCY == 1
                #define BSP_HOCO_HZ                 (18000000)
            #elif BSP_CFG_HOCO_FREQUENCY == 2
                #define BSP_HOCO_HZ                 (20000000)
            #else
                #error "Invalid HOCO frequency chosen (BSP_CFG_HOCO_FREQUENCY) in bsp_clock_cfg.h"
            #endif

 #define __MPU_PRESENT             1

#if defined CONFIG_SOC_PART_NUMBER_RA6M3
#else
#error Library does not support the specified device.
#endif

#ifndef BSP_CLOCK_CFG_MAIN_OSC_POPULATED
#define BSP_CLOCK_CFG_MAIN_OSC_POPULATED (1)
#endif
#ifndef BSP_CLOCK_CFG_MAIN_OSC_WAIT
#define BSP_CLOCK_CFG_MAIN_OSC_WAIT (9)
#endif
#ifndef BSP_CLOCK_CFG_MAIN_OSC_CLOCK_SOURCE
#define BSP_CLOCK_CFG_MAIN_OSC_CLOCK_SOURCE (0)
#endif
#ifndef BSP_CLOCK_CFG_SUBCLOCK_DRIVE
#define BSP_CLOCK_CFG_SUBCLOCK_DRIVE (0)
#endif
#ifndef BSP_CLOCK_CFG_SUBCLOCK_POPULATED
#define BSP_CLOCK_CFG_SUBCLOCK_POPULATED (1)
#endif
#ifndef BSP_CLOCK_CFG_SUBCLOCK_STABILIZATION_MS
#define BSP_CLOCK_CFG_SUBCLOCK_STABILIZATION_MS 1000
#endif

#endif /* _ASMLANGUAGE */


/*******************************************************************************************************************//**
 *        This assembly language routine takes roughly 4 cycles per loop. 2 additional cycles
 *        occur when the loop exits. The 'naked' attribute  indicates that the specified function does not need
 *        prologue/epilogue sequences generated by the compiler.
 * @param[in]     loop_cnt  The number of loops to iterate.
 **********************************************************************************************************************/
static inline void software_delay_loop(__attribute__((unused)) uint32_t loop_cnt)
{
    __asm volatile ("sw_delay_loop:         \n"

#if defined(__ICCARM__) || defined(__ARMCC_VERSION)
                    "   subs r0, #1         \n"   ///< 1 cycle
#elif defined(__GNUC__)
                    "   sub r0, r0, #1      \n"   ///< 1 cycle
#endif

                    "   cmp r0, #0          \n"   ///< 1 cycle

/* CM0 and CM23 have a different instruction set */
#if defined(__CORE_CM0PLUS_H_GENERIC) || defined(__CORE_CM23_H_GENERIC)
                    "   bne sw_delay_loop   \n"   ///< 2 cycles
#else
                    "   bne.n sw_delay_loop \n"   ///< 2 cycles
#endif
                    "   bx lr               \n"); ///< 2 cycles
}
#endif /* #if 0 */

#define RA4W1_CPU_CLOCK_FREQ_HZ DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)

#endif /* _RENESAS_RA6M3_SOC_H_ */
