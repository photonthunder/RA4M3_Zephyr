/***********************************************************************************************************************
 * Copyright [2020] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
 *
 * This software and documentation are supplied by Renesas Electronics America Inc. and may only be used with products
 * of Renesas Electronics Corp. and its affiliates ("Renesas").  No other uses are authorized.  Renesas products are
 * sold pursuant to Renesas terms and conditions of sale.  Purchasers are solely responsible for the selection and use
 * of Renesas products and Renesas assumes no liability.  No license, express or implied, to any intellectual property
 * right is granted by Renesas. This software is protected under all applicable laws, including copyright laws. Renesas
 * reserves the right to change or discontinue this software and/or this documentation. THE SOFTWARE AND DOCUMENTATION
 * IS DELIVERED TO YOU "AS IS," AND RENESAS MAKES NO REPRESENTATIONS OR WARRANTIES, AND TO THE FULLEST EXTENT
 * PERMISSIBLE UNDER APPLICABLE LAW, DISCLAIMS ALL WARRANTIES, WHETHER EXPLICITLY OR IMPLICITLY, INCLUDING WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT, WITH RESPECT TO THE SOFTWARE OR
 * DOCUMENTATION.  RENESAS SHALL HAVE NO LIABILITY ARISING OUT OF ANY SECURITY VULNERABILITY OR BREACH.  TO THE MAXIMUM
 * EXTENT PERMITTED BY LAW, IN NO EVENT WILL RENESAS BE LIABLE TO YOU IN CONNECTION WITH THE SOFTWARE OR DOCUMENTATION
 * (OR ANY PERSON OR ENTITY CLAIMING RIGHTS DERIVED FROM YOU) FOR ANY LOSS, DAMAGES, OR CLAIMS WHATSOEVER, INCLUDING,
 * WITHOUT LIMITATION, ANY DIRECT, CONSEQUENTIAL, SPECIAL, INDIRECT, PUNITIVE, OR INCIDENTAL DAMAGES; ANY LOST PROFITS,
 * OTHER ECONOMIC DAMAGE, PROPERTY DAMAGE, OR PERSONAL INJURY; AND EVEN IF RENESAS HAS BEEN ADVISED OF THE POSSIBILITY
 * OF SUCH LOSS, DAMAGES, CLAIMS OR COSTS.
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * @addtogroup LVD
 * @{
 **********************************************************************************************************************/

#ifndef R_LVD_H
#define R_LVD_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "bsp_api.h"
#include "r_lvd_cfg.h"
#include "r_lvd_api.h"

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define LVD_CODE_VERSION_MAJOR    (1U)
#define LVD_CODE_VERSION_MINOR    (1U)

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/** LVD instance control structure */
typedef struct st_lvd_instance_ctrl
{
    uint32_t          open;
    lvd_cfg_t const * p_cfg;

#if BSP_TZ_SECURE_BUILD
    bool callback_is_secure;                    // If the callback is in non-secure memory then a security state transistion is required to call p_callback (BLXNS)
#endif
    void (* p_callback)(lvd_callback_args_t *); // Pointer to callback that is called when lvd_current_state_t changes.
    lvd_callback_args_t * p_callback_memory;    // Pointer to non-secure memory that can be used to pass arguments to a callback in non-secure memory.

    /* Pointer to context to be passed into callback function */
    void const * p_context;
} lvd_instance_ctrl_t;

/**********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/

/** @cond INC_HEADER_DEFS_SEC */
/** Filled in Interface API structure for this Instance. */
extern const lvd_api_t g_lvd_on_lvd;

/** @endcond */

/***********************************************************************************************************************
 * Public APIs
 **********************************************************************************************************************/
fsp_err_t R_LVD_Open(lvd_ctrl_t * const p_api_ctrl, lvd_cfg_t const * const p_cfg);
fsp_err_t R_LVD_Close(lvd_ctrl_t * const p_api_ctrl);
fsp_err_t R_LVD_StatusGet(lvd_ctrl_t * const p_api_ctrl, lvd_status_t * p_lvd_status);
fsp_err_t R_LVD_StatusClear(lvd_ctrl_t * const p_api_ctrl);
fsp_err_t R_LVD_VersionGet(fsp_version_t * const p_version);
fsp_err_t R_LVD_CallbackSet(lvd_ctrl_t * const          p_api_ctrl,
                            void (                    * p_callback)(lvd_callback_args_t *),
                            void const * const          p_context,
                            lvd_callback_args_t * const p_callback_memory);

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif

/*******************************************************************************************************************//**
 * @} (end defgroup LVD)
 **********************************************************************************************************************/
