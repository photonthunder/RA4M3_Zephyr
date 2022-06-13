/*
 * soc_rom_reg.c
 *
 *  Created on: Dec 8, 2020
 *      Author: zoltan.ianosi.bp
 */
#include <zephyr.h>
#include <bsp_compiler_support.h>
#include <bsp_clock_cfg.h>

/** OR in the HOCO frequency setting from bsp_clock_cfg.h with the OFS1 setting from bsp_cfg.h. */
#define BSP_ROM_REG_OFS1_SETTING                                             \
    (((uint32_t) BSP_CFG_ROM_REG_OFS1 & BSP_FEATURE_BSP_OFS1_HOCOFRQ_MASK) | \
     ((uint32_t) BSP_CFG_HOCO_FREQUENCY << BSP_FEATURE_BSP_OFS1_HOCOFRQ_OFFSET))

/** Build up SECMPUAC register based on MPU settings. */
#define BSP_ROM_REG_MPU_CONTROL_SETTING                     \
    ((0xFFFFFCF0U) |                                        \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_PC0_ENABLE << 8) |     \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_PC1_ENABLE << 9) |     \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION0_ENABLE) |      \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION1_ENABLE << 1) | \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION2_ENABLE << 2) | \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION3_ENABLE << 3))

#define OFS_SEQ1 0xA001A001 | (1 << 1) | (3 << 2)
#define OFS_SEQ2 (15 << 4) | (3 << 8) | (3 << 10)
#define OFS_SEQ3 (1 << 12) | (1 << 14) | (1 << 17)
#define OFS_SEQ4 (3 << 18) |(15 << 20) | (3 << 24) | (3 << 26)
#define OFS_SEQ5 (1 << 28) | (1 << 30)
#define BSP_CFG_ROM_REG_OFS0 (OFS_SEQ1 | OFS_SEQ2 | OFS_SEQ3 | OFS_SEQ4 | OFS_SEQ5)
#define BSP_CFG_ROM_REG_OFS1 (0xFFFFFEC3 | (1 << 2) | (3 << 3) | (1 << 8))
#define BSP_CFG_USE_LOW_VOLTAGE_MODE ((0))
#define BSP_CFG_ROM_REG_MPU_PC0_ENABLE (1)
#define BSP_CFG_ROM_REG_MPU_PC0_START (0x00FFFFFC)
#define BSP_CFG_ROM_REG_MPU_PC0_END (0x00FFFFFF)
#define BSP_CFG_ROM_REG_MPU_PC1_ENABLE (1)
#define BSP_CFG_ROM_REG_MPU_PC1_START (0x00FFFFFC)
#define BSP_CFG_ROM_REG_MPU_PC1_END (0x00FFFFFF)
#define BSP_CFG_ROM_REG_MPU_REGION0_ENABLE (1)
#define BSP_CFG_ROM_REG_MPU_REGION0_START (0x00FFFFFC)
#define BSP_CFG_ROM_REG_MPU_REGION0_END (0x00FFFFFF)
#define BSP_CFG_ROM_REG_MPU_REGION1_ENABLE (1)
#define BSP_CFG_ROM_REG_MPU_REGION1_START (0x200FFFFC)
#define BSP_CFG_ROM_REG_MPU_REGION1_END (0x200FFFFF)
#define BSP_CFG_ROM_REG_MPU_REGION2_ENABLE (1)
#define BSP_CFG_ROM_REG_MPU_REGION2_START (0x407FFFFC)
#define BSP_CFG_ROM_REG_MPU_REGION2_END (0x407FFFFF)
#define BSP_CFG_ROM_REG_MPU_REGION3_ENABLE (1)
#define BSP_CFG_ROM_REG_MPU_REGION3_START (0x400DFFFC)
#define BSP_CFG_ROM_REG_MPU_REGION3_END (0x400DFFFF)

/** ROM registers defined here. Some have masks to make sure reserved bits are set appropriately. */
static const uint32_t g_bsp_rom_registers[] BSP_PLACE_IN_SECTION (BSP_SECTION_ROM_REGISTERS) =
{
    (uint32_t) BSP_CFG_ROM_REG_OFS0,
    (uint32_t) BSP_ROM_REG_OFS1_SETTING,
    ((uint32_t) BSP_CFG_ROM_REG_MPU_PC0_START & 0xFFFFFFFCU),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_PC0_END | 0x00000003U),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_PC1_START & 0xFFFFFFFCU),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_PC1_END | 0x00000003U),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION0_START & BSP_FEATURE_BSP_MPU_REGION0_MASK & 0xFFFFFFFCU),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION0_END & BSP_FEATURE_BSP_MPU_REGION0_MASK) | 0x00000003U),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION1_START & 0xFFFFFFFCU),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION1_END | 0x00000003U),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION2_START & 0x407FFFFCU) | 0x40000000U),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION2_END & 0x407FFFFCU) | 0x40000003U),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION3_START & 0x407FFFFCU) | 0x40000000U),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION3_END & 0x407FFFFCU) | 0x40000003U),
    (uint32_t) BSP_ROM_REG_MPU_CONTROL_SETTING
};
/*
BSP_DONT_REMOVE static uint8_t g_heap[0x1000] BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT) \
    BSP_PLACE_IN_SECTION(BSP_SECTION_HEAP);
*/
