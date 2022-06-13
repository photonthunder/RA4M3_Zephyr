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

#include "bsp_api.h"

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "r_ospi.h"

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define OSPI_PRV_OPEN                                (0x4F535049U)
#define OSPI_PRV_SHIFT(device_0_settings, device) \
    (device_0_settings << (device * 16U))
#define OSPI_PRV_RMW(reg, device_0_settings, device) \
    ((reg & ~(uint32_t) OSPI_PRV_SHIFT(UINT16_MAX, device)) | (OSPI_PRV_SHIFT(device_0_settings, device)))
#define OSPI_PRV_RMW_MASKED(reg, mask, device_0_settings, device) \
    ((reg & ~(uint32_t) OSPI_PRV_SHIFT(mask, device)) | (OSPI_PRV_SHIFT(device_0_settings, device)))
#define OSPI_PRV_AUTOMATIC_CALIBRATION_NORMAL_END    (3U)
#define OSPI_PRV_DIRECT_COMMAND_MASK                 (3U)
#define OSPI_PRV_DIRECT_ADDR_AND_DATA_MASK           (7U)
#define OSPI_PRV_PAGE_SIZE_BYTES                     (256U)

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static fsp_err_t r_ospi_automatic_calibration_seq(ospi_instance_ctrl_t * p_instance_ctrl);
static bool      r_ospi_status_sub(ospi_instance_ctrl_t * p_instance_ctrl, uint8_t bit_pos);
static fsp_err_t r_ospi_spi_protocol_specific_settings(ospi_instance_ctrl_t * p_instance_ctrl,
                                                       spi_flash_protocol_t   spi_protocol);
static void r_ospi_wen(ospi_instance_ctrl_t * p_instance_ctrl);
static void r_ospi_direct_transfer(ospi_instance_ctrl_t              * p_instance_ctrl,
                                   spi_flash_direct_transfer_t * const p_transfer,
                                   spi_flash_direct_transfer_dir_t     direction);

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/

/** Version data structure used by error logger macro. */
static const fsp_version_t g_ospi_version =
{
    .api_version_minor  = SPI_FLASH_API_VERSION_MINOR,
    .api_version_major  = SPI_FLASH_API_VERSION_MAJOR,
    .code_version_major = OSPI_CODE_VERSION_MAJOR,
    .code_version_minor = OSPI_CODE_VERSION_MINOR
};

/*******************************************************************************************************************//**
 * @addtogroup OSPI
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/

const spi_flash_api_t g_ospi_on_spi_flash =
{
    .open           = R_OSPI_Open,
    .directWrite    = R_OSPI_DirectWrite,
    .directRead     = R_OSPI_DirectRead,
    .directTransfer = R_OSPI_DirectTransfer,
    .spiProtocolSet = R_OSPI_SpiProtocolSet,
    .write          = R_OSPI_Write,
    .erase          = R_OSPI_Erase,
    .statusGet      = R_OSPI_StatusGet,
    .xipEnter       = R_OSPI_XipEnter,
    .xipExit        = R_OSPI_XipExit,
    .bankSet        = R_OSPI_BankSet,
    .close          = R_OSPI_Close,
    .versionGet     = R_OSPI_VersionGet,
};

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * Open the OSPI driver module. After the driver is open, the OSPI can be accessed like internal flash memory.
 *
 * Implements @ref spi_flash_api_t::open.
 *
 * Example:
 * @snippet r_ospi_example.c R_OSPI_Open
 *
 * @retval FSP_SUCCESS              Configuration was successful.
 * @retval FSP_ERR_ASSERTION        The parameter p_ctrl or p_cfg is NULL.
 * @retval FSP_ERR_ALREADY_OPEN     Driver has already been opened with the same p_ctrl.
 * @retval FSP_ERR_CALIBRATE_FAILED Failed to perform auto-calibrate.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_Open (spi_flash_ctrl_t * p_ctrl, spi_flash_cfg_t const * const p_cfg)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;

#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ASSERT(NULL != p_cfg);
    FSP_ASSERT(NULL != p_cfg->p_extend);
    FSP_ERROR_RETURN(OSPI_PRV_OPEN != p_instance_ctrl->open, FSP_ERR_ALREADY_OPEN);
#endif

    /* Enable clock to the OSPI block */
    R_BSP_MODULE_START(FSP_IP_OSPI, 0U);
    ospi_extended_cfg_t * p_cfg_extend = (ospi_extended_cfg_t *) p_cfg->p_extend;

    /* Initialize control block. */
    p_instance_ctrl->p_cfg        = p_cfg;
    p_instance_ctrl->spi_protocol = p_cfg->spi_protocol;
    p_instance_ctrl->channel      = p_cfg_extend->channel;

    /* Perform OSPI initial setup as described in hardware manual (see Section 34.3.6.1
     * 'Initial Settings' of the RA6M4 manual R01UH0890EJ0100). */

    /* Set the device type as OctaFlash and storage capacity */
    R_OSPI->DSR[p_instance_ctrl->channel] = p_cfg_extend->memory_size & R_OSPI_DSR_DVSZ_Msk;

    /* DCSTR and DRCSTR values are designed to not change at the time of changing the SPI protocol.
     * Minimum latencies are same for SPI and SOPI.
     * In case the SPI protocol is intended to be changed to DOPI, these should be configured with latencies required for DOPI.
     */
    ospi_timing_setting_t const * p_timing = p_cfg_extend->p_timing_settings;
    R_OSPI->DCSTR = (uint32_t) (p_timing->cs_pulldown_lead << R_OSPI_DCSTR_DVSELLO_Pos) |
                    (uint32_t) (p_timing->cs_pullup_lag << R_OSPI_DCSTR_DVSELHI_Pos) |
                    (uint32_t) (p_timing->command_to_command_interval <<
                                R_OSPI_DCSTR_DVSELCMD_Pos);
    p_timing       = p_cfg_extend->p_mem_mapped_read_timing_settings;
    R_OSPI->DRCSTR = OSPI_PRV_RMW(R_OSPI->DRCSTR,
                                  ((uint32_t) (p_cfg_extend->single_continuous_mode_read_idle_time <<
                                               R_OSPI_DRCSTR_CTRW0_Pos) |
                                   (uint32_t) (p_timing->cs_pullup_lag << R_OSPI_DRCSTR_DVRDHI0_Pos) |
                                   (uint32_t) (p_timing->cs_pulldown_lead << R_OSPI_DRCSTR_DVRDLO0_Pos) |
                                   (uint32_t) (p_timing->command_to_command_interval << R_OSPI_DRCSTR_DVRDCMD0_Pos)),
                                  p_instance_ctrl->channel);

    p_timing = p_cfg_extend->p_mem_mapped_write_timing_settings;

    /* Always keep single continuous write mode enabled with appropriate settings */
    R_OSPI->DWCSTR = OSPI_PRV_RMW(R_OSPI->DWCSTR,
                                  ((uint32_t) (1U << R_OSPI_DWCSTR_CTW0_Pos) |
                                   (uint32_t) (p_cfg_extend->single_continuous_mode_write_idle_time <<
                                               R_OSPI_DWCSTR_CTWW0_Pos) |
                                   (uint32_t) (p_timing->cs_pullup_lag << R_OSPI_DWCSTR_DVWHI0_Pos) |
                                   (uint32_t) (p_timing->cs_pulldown_lead << R_OSPI_DWCSTR_DVWLO0_Pos) |
                                   (uint32_t) (p_timing->command_to_command_interval << R_OSPI_DWCSTR_DVWCMD0_Pos)),
                                  p_instance_ctrl->channel);

    /* Max = 256 bytes, i.e., Page size */
    R_OSPI->DWSCTSR = OSPI_PRV_SHIFT(OSPI_PRV_PAGE_SIZE_BYTES << R_OSPI_DWSCTSR_CTSN0_Pos, p_instance_ctrl->channel);

    /* Read back to ensure value has been written */
    FSP_HARDWARE_REGISTER_WAIT(R_OSPI->DWSCTSR,
                               OSPI_PRV_SHIFT(OSPI_PRV_PAGE_SIZE_BYTES << R_OSPI_DWSCTSR_CTSN0_Pos,
    p_instance_ctrl->channel));

    /* Setup SPI protocol specific registers */
    fsp_err_t ret = r_ospi_spi_protocol_specific_settings(p_instance_ctrl, p_cfg->spi_protocol);
    if (FSP_SUCCESS == ret)
    {
        p_instance_ctrl->open = OSPI_PRV_OPEN;
    }

    return ret;
}

/*******************************************************************************************************************//**
 * Writes raw data directly to the OctaFlash. API not supported. Use R_OSPI_DirectTransfer
 *
 * Implements @ref spi_flash_api_t::directWrite.
 *
 * @retval FSP_ERR_UNSUPPORTED         API not supported by OSPI.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_DirectWrite (spi_flash_ctrl_t    * p_ctrl,
                              uint8_t const * const p_src,
                              uint32_t const        bytes,
                              bool const            read_after_write)
{
    FSP_PARAMETER_NOT_USED(p_ctrl);
    FSP_PARAMETER_NOT_USED(p_src);
    FSP_PARAMETER_NOT_USED(bytes);
    FSP_PARAMETER_NOT_USED(read_after_write);

    return FSP_ERR_UNSUPPORTED;
}

/*******************************************************************************************************************//**
 * Reads raw data directly from the OctaFlash. API not supported. Use R_OSPI_DirectTransfer.
 *
 * Implements @ref spi_flash_api_t::directRead.
 *
 * @retval FSP_ERR_UNSUPPORTED         API not supported by OSPI.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_DirectRead (spi_flash_ctrl_t * p_ctrl, uint8_t * const p_dest, uint32_t const bytes)
{
    FSP_PARAMETER_NOT_USED(p_ctrl);
    FSP_PARAMETER_NOT_USED(p_dest);
    FSP_PARAMETER_NOT_USED(bytes);

    return FSP_ERR_UNSUPPORTED;
}

/*******************************************************************************************************************//**
 * Read/Write raw data directly with the OctaFlash.
 *
 * Implements @ref spi_flash_api_t::directTransfer.
 *
 * Example:
 * @snippet r_ospi_example.c R_OSPI_DirectTransfer
 *
 * @retval FSP_SUCCESS                 The flash was programmed successfully.
 * @retval FSP_ERR_ASSERTION           A required pointer is NULL.
 * @retval FSP_ERR_NOT_OPEN            Driver is not opened.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_DirectTransfer (spi_flash_ctrl_t                  * p_ctrl,
                                 spi_flash_direct_transfer_t * const p_transfer,
                                 spi_flash_direct_transfer_dir_t     direction)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;
#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ASSERT(NULL != p_transfer);
    FSP_ASSERT(0 != p_transfer->command_length);
    FSP_ERROR_RETURN(OSPI_PRV_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif
    r_ospi_direct_transfer(p_instance_ctrl, p_transfer, direction);

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * Enters Single Continuous Read/Write mode.
 *
 * Implements @ref spi_flash_api_t::xipEnter.
 *
 * @retval FSP_SUCCESS                 The flash was programmed successfully.
 * @retval FSP_ERR_ASSERTION           A required pointer is NULL.
 * @retval FSP_ERR_NOT_OPEN            Driver is not opened.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_XipEnter (spi_flash_ctrl_t * p_ctrl)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;
#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ERROR_RETURN(OSPI_PRV_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    /* Single continuous read mode (CTR0/1) is enabled. Other values are set during Open */
    R_OSPI->DRCSTR =
        OSPI_PRV_RMW_MASKED(R_OSPI->DRCSTR,
                            R_OSPI_DRCSTR_CTR0_Msk,
                            ((uint32_t) (1U << R_OSPI_DRCSTR_CTR0_Pos)),
                            p_instance_ctrl->channel);

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * Exits XIP (execute in place) mode.
 *
 * Implements @ref spi_flash_api_t::xipExit.
 *
 * @retval FSP_SUCCESS                 The flash was programmed successfully.
 * @retval FSP_ERR_ASSERTION           A required pointer is NULL.
 * @retval FSP_ERR_NOT_OPEN            Driver is not opened.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_XipExit (spi_flash_ctrl_t * p_ctrl)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;
#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ERROR_RETURN(OSPI_PRV_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    /* Single continuous read mode (CTR0/1) is disabled. Other values are set during Open */
    R_OSPI->DRCSTR = OSPI_PRV_RMW_MASKED(R_OSPI->DRCSTR, R_OSPI_DRCSTR_CTR0_Msk, 0U, p_instance_ctrl->channel);

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * Program a page of data to the flash.
 *
 * Implements @ref spi_flash_api_t::write.
 *
 * Example:
 * @snippet r_ospi_example.c R_OSPI_Write
 *
 * @retval FSP_SUCCESS                 The flash was programmed successfully.
 * @retval FSP_ERR_ASSERTION           p_instance_ctrl, p_dest or p_src is NULL, or byte_count crosses a page boundary.
 * @retval FSP_ERR_NOT_OPEN            Driver is not opened.
 * @retval FSP_ERR_DEVICE_BUSY         Another Write/Erase transaction is in progress.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_Write (spi_flash_ctrl_t    * p_ctrl,
                        uint8_t const * const p_src,
                        uint8_t * const       p_dest,
                        uint32_t              byte_count)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;
#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ASSERT(NULL != p_src);
    FSP_ASSERT(NULL != p_dest);
    FSP_ERROR_RETURN(OSPI_PRV_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
    FSP_ASSERT(OSPI_PRV_PAGE_SIZE_BYTES >= byte_count);
    FSP_ASSERT(0 != byte_count);
#endif

    FSP_ERROR_RETURN(false == r_ospi_status_sub(p_instance_ctrl, p_instance_ctrl->p_cfg->write_status_bit),
                     FSP_ERR_DEVICE_BUSY);

    r_ospi_wen(p_instance_ctrl);

    uint32_t i = 0;

    /* Perform entire write opetation keeping the same access width to remain in single continuous write mode (see Section 34.5.1, point#3
     * 'Single continuous write operation' of the RA6M4 manual R01UH0890EJ0100).
     *
     * A change in access width will reissue the write command only when the access width change happens at a 128-bit boundary.
     * This is not handled by the code code below.
     * Moreover, a blocking wait (until WIP = 0) needs to be introduced in order to change the access width within this API.
     * Below is a conservative approach to perform the entire transfer with a fixed access width.
     * Also, memcpy should not be used here to take advantage of larger access widths.
     *
     * *//* Word access */
    if (0 == byte_count % 4U)
    {
        uint32_t * p_word_aligned_dest = (uint32_t *) p_dest;
        uint32_t * p_word_aligned_src  = (uint32_t *) p_src;
        for (i = 0; i < byte_count / 4U; i++)
        {
            *p_word_aligned_dest = *p_word_aligned_src;
            p_word_aligned_dest++;
            p_word_aligned_src++;
        }
    }
    /* Half Word access */
    else if (0 == byte_count % 2U)
    {
        uint16_t * p_half_word_aligned_dest = (uint16_t *) p_dest;
        uint16_t * p_half_word_aligned_src  = (uint16_t *) p_src;
        for (i = 0; i < byte_count / 2U; i++)
        {
            *p_half_word_aligned_dest = *p_half_word_aligned_src;
            p_half_word_aligned_dest++;
            p_half_word_aligned_src++;
        }
    }
    /* Byte access */
    else
    {
        for (i = 0; i < byte_count; i++)
        {
            p_dest[i] = p_src[i];
        }
    }

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * Erase a block or sector of flash.  The byte_count must exactly match one of the erase sizes defined in spi_flash_cfg_t.
 * For chip erase, byte_count must be SPI_FLASH_ERASE_SIZE_CHIP_ERASE.
 *
 * Implements @ref spi_flash_api_t::erase.
 *
 * @retval FSP_SUCCESS                 The command to erase the flash was executed successfully.
 * @retval FSP_ERR_ASSERTION           p_instance_ctrl or p_device_address is NULL, byte_count doesn't match an erase
 *                                     size defined in spi_flash_cfg_t, or byte_count is set to 0.
 * @retval FSP_ERR_NOT_OPEN            Driver is not opened.
 * @retval FSP_ERR_DEVICE_BUSY         The device is busy.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_Erase (spi_flash_ctrl_t * p_ctrl, uint8_t * const p_device_address, uint32_t byte_count)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;

#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ASSERT(NULL != p_device_address);
    FSP_ASSERT(0 != byte_count);
    FSP_ERROR_RETURN(OSPI_PRV_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif
    spi_flash_cfg_t const * p_cfg         = p_instance_ctrl->p_cfg;
    uint16_t                erase_command = 0;
    uint32_t                chip_address  = (uint32_t) p_device_address - BSP_FEATURE_OSPI_DEVICE_0_START_ADDRESS;
    bool send_address = true;
    ospi_extended_cfg_t * p_cfg_extend = (ospi_extended_cfg_t *) p_cfg->p_extend;
    FSP_ERROR_RETURN(false == r_ospi_status_sub(p_instance_ctrl, p_cfg->write_status_bit), FSP_ERR_DEVICE_BUSY);

    for (uint32_t index = 0; index < p_cfg->erase_command_list_length; index++)
    {
        /* If requested byte_count is supported by underlying flash, store the command. */
        if (byte_count == p_cfg->p_erase_command_list[index].size)
        {
            if (p_cfg_extend->memory_size == byte_count)
            {
                /* Don't send address for chip erase. */
                send_address = false;
            }

            erase_command = p_cfg->p_erase_command_list[index].command;
            break;
        }
    }

#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(0U != erase_command);
#endif
    r_ospi_wen(p_instance_ctrl);

    spi_flash_direct_transfer_t direct_command = {0};
    direct_command.command        = erase_command;
    direct_command.address        = chip_address;
    direct_command.address_length = (true == send_address) ?
                                    (OSPI_PRV_DIRECT_ADDR_AND_DATA_MASK & (p_cfg->address_bytes + 1U)) : 0U;
    direct_command.command_length = (SPI_FLASH_PROTOCOL_EXTENDED_SPI == p_instance_ctrl->spi_protocol) ?
                                    1U : (p_cfg_extend->p_opi_commands->command_bytes & OSPI_PRV_DIRECT_COMMAND_MASK);

    r_ospi_direct_transfer(p_instance_ctrl, &direct_command, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * Gets the write or erase status of the flash.
 *
 * Implements @ref spi_flash_api_t::statusGet.
 *
 * Example:
 * @snippet r_ospi_example.c R_OSPI_StatusGet
 *
 * @retval FSP_SUCCESS                 The write status is in p_status.
 * @retval FSP_ERR_ASSERTION           p_instance_ctrl or p_status is NULL.
 * @retval FSP_ERR_NOT_OPEN            Driver is not opened.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_StatusGet (spi_flash_ctrl_t * p_ctrl, spi_flash_status_t * const p_status)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;

#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ASSERT(NULL != p_status);
    FSP_ERROR_RETURN(OSPI_PRV_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    /* Read device status. */
    p_status->write_in_progress = r_ospi_status_sub(p_instance_ctrl, p_instance_ctrl->p_cfg->write_status_bit);

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * Selects the bank to access.
 *
 * Implements @ref spi_flash_api_t::bankSet.
 *
 * @retval FSP_ERR_UNSUPPORTED         API not supported by OSPI.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_BankSet (spi_flash_ctrl_t * p_ctrl, uint32_t bank)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;

    FSP_PARAMETER_NOT_USED(p_instance_ctrl);
    FSP_PARAMETER_NOT_USED(bank);

    return FSP_ERR_UNSUPPORTED;
}

/*******************************************************************************************************************//**
 * Sets the SPI protocol.
 *
 * Implements @ref spi_flash_api_t::spiProtocolSet.
 *
 * @retval FSP_SUCCESS                SPI protocol updated on MCU peripheral.
 * @retval FSP_ERR_ASSERTION          A required pointer is NULL.
 * @retval FSP_ERR_NOT_OPEN           Driver is not opened.
 * @retval FSP_ERR_CALIBRATE_FAILED   Failed to perform auto-calibrate.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_SpiProtocolSet (spi_flash_ctrl_t * p_ctrl, spi_flash_protocol_t spi_protocol)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;

#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ERROR_RETURN(OSPI_PRV_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif
    p_instance_ctrl->spi_protocol = spi_protocol;

    /* Update the SPI protocol and its associated registers. */
    return r_ospi_spi_protocol_specific_settings(p_instance_ctrl, spi_protocol);
}

/*******************************************************************************************************************//**
 * Close the OSPI driver module.
 *
 * Implements @ref spi_flash_api_t::close.
 *
 * @retval FSP_SUCCESS             Configuration was successful.
 * @retval FSP_ERR_ASSERTION       p_instance_ctrl is NULL.
 * @retval FSP_ERR_NOT_OPEN        Driver is not opened.
 **********************************************************************************************************************/
fsp_err_t R_OSPI_Close (spi_flash_ctrl_t * p_ctrl)
{
    ospi_instance_ctrl_t * p_instance_ctrl = (ospi_instance_ctrl_t *) p_ctrl;

#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ERROR_RETURN(OSPI_PRV_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    p_instance_ctrl->open = 0U;

    /* Disable clock to the OSPI block */
    R_BSP_MODULE_STOP(FSP_IP_OSPI, 0U);

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * Get the driver version based on compile time macros.
 *
 * Implements @ref spi_flash_api_t::versionGet.
 *
 * @retval     FSP_SUCCESS          Successful close.
 * @retval     FSP_ERR_ASSERTION    p_version is NULL.
 *
 **********************************************************************************************************************/
fsp_err_t R_OSPI_VersionGet (fsp_version_t * const p_version)
{
#if OSPI_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_version);
#endif

    p_version->version_id = g_ospi_version.version_id;

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * @} (end addtogroup OSPI)
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * Perform initialization based on SPI/OPI protocol
 *
 * @param[in]   p_instance_ctrl    Pointer to OSPI specific control structure
 * @param[in]   spi_protocol       SPI/OPI protocol request
 *
 * @retval      FSP_SUCCESS                Protocol based settings completed successfully.
 * @retval      FSP_ERR_CALIBRATE_FAILED   Auto-Calibration failed.
 **********************************************************************************************************************/
static fsp_err_t r_ospi_spi_protocol_specific_settings (ospi_instance_ctrl_t * p_instance_ctrl,
                                                        spi_flash_protocol_t   spi_protocol)
{
    spi_flash_cfg_t const * p_cfg        = p_instance_ctrl->p_cfg;
    ospi_extended_cfg_t   * p_cfg_extend = (ospi_extended_cfg_t *) p_cfg->p_extend;
    fsp_err_t               ret          = FSP_SUCCESS;
    ospi_device_number_t    channel      = p_instance_ctrl->channel;

    /* In case of SOPI mode enable pre-cycle as described in hardware manual (see Section 34.2.12
     * 'CDSR : Controller and Device Setting Register' Note 1 of the RA6M4 manual R01UH0890EJ0100). */
    uint32_t cdsr = R_OSPI->CDSR;
    cdsr &= ~((uint32_t) (R_OSPI_CDSR_DV0TTYP_Msk << (channel * R_OSPI_CDSR_DV1TTYP_Pos)) |
              (uint32_t) (R_OSPI_CDSR_DV0PC_Msk << channel));

    /* Right shifted to match enum to register value */
    cdsr |= ((((uint32_t) spi_protocol) >> 1U) << (channel * R_OSPI_CDSR_DV1TTYP_Pos));

    /* Enable pre-cycle setting in case of SOPI. ANDed to filter out SOPI enum from other mode enums. */
    cdsr |= ((((uint32_t) spi_protocol) & 1U) << (R_OSPI_CDSR_DV0PC_Pos + channel));

    /* Keep DLFT disabled. No easy recovery from deadlock */
    cdsr        |= 1U << R_OSPI_CDSR_DLFT_Pos;
    R_OSPI->CDSR = cdsr;

    /* OPI mode */
    if (SPI_FLASH_PROTOCOL_EXTENDED_SPI != spi_protocol)
    {
        ospi_opi_command_set_t const * p_opi_commands = p_cfg_extend->p_opi_commands;
        R_OSPI->MRWCSR = OSPI_PRV_RMW(R_OSPI->MRWCSR,
                                      ((uint32_t) (((uint32_t) p_cfg->address_bytes + 1U) << R_OSPI_MRWCSR_MRAL0_Pos) |
                                       (uint32_t) (p_opi_commands->command_bytes << R_OSPI_MRWCSR_MRCL0_Pos) |
                                       (uint32_t) (((uint32_t) p_cfg->address_bytes + 1U) << R_OSPI_MRWCSR_MWAL0_Pos) |
                                       (uint32_t) (p_opi_commands->command_bytes << R_OSPI_MRWCSR_MWCL0_Pos)),
                                      channel);

        /* Set MDLR (Memory Map Dummy Length Reg) with Read dummy length setting */
        R_OSPI->MDLR =
            OSPI_PRV_RMW(R_OSPI->MDLR,
                         ((uint32_t) (p_cfg_extend->opi_mem_read_dummy_cycles << R_OSPI_MDLR_DV0RDL_Pos)),
                         channel);

        /* Specifies the read and write commands for Device */
        uint32_t read_command = (uint32_t) ((SPI_FLASH_PROTOCOL_SOPI == spi_protocol) ?
                                            p_opi_commands->read_command : p_opi_commands->dual_read_command);
        R_OSPI->MRWCR[channel] = (uint32_t) p_opi_commands->page_program_command << R_OSPI_MRWCR_DMWCMD0_Pos |
                                 read_command;

        /* Perform auto-calibration to appropriately update MDTR DVnDEL field */
        if (0 == p_cfg_extend->data_latch_delay_clocks)
        {
            ret = r_ospi_automatic_calibration_seq(p_instance_ctrl);
        }
        else
        {
            /* The OctalFlash is pre-calibrated with the existing clock settings. Do not auto-calibrate. */
            R_OSPI->MDTR =
                OSPI_PRV_RMW_MASKED(R_OSPI->MDTR,
                                    R_OSPI_MDTR_DV0DEL_Msk,
                                    ((uint32_t) (p_cfg_extend->data_latch_delay_clocks << R_OSPI_MDTR_DV0DEL_Pos)),
                                    channel);
        }
    }
    else
    {
        /* Command length is forced to 1 byte for SPI mode */
        R_OSPI->MRWCSR = OSPI_PRV_RMW(R_OSPI->MRWCSR,
                                      ((uint32_t) (((uint32_t) p_cfg->address_bytes + 1U) << R_OSPI_MRWCSR_MRAL0_Pos) |
                                       (uint32_t) (1U << R_OSPI_MRWCSR_MRCL0_Pos) |
                                       (uint32_t) (((uint32_t) p_cfg->address_bytes + 1U) << R_OSPI_MRWCSR_MWAL0_Pos) |
                                       (uint32_t) (1U << R_OSPI_MRWCSR_MWCL0_Pos)),
                                      channel);

        /* Specifies the read and write commands for Device 0 */
        R_OSPI->MRWCR[channel] = (uint32_t) p_cfg->page_program_command << R_OSPI_MRWCR_DMWCMD0_Pos |
                                 (uint32_t) p_cfg->read_command << R_OSPI_MRWCR_DMRCMD0_Pos;

        /* Single continuous read mode (CTR) must be disabled for SPI mode */
        R_OSPI->DRCSTR = OSPI_PRV_RMW_MASKED(R_OSPI->DRCSTR, R_OSPI_DRCSTR_CTR0_Msk, 0U, channel);

        /* Clear auto-calibration value */
        R_OSPI->MDTR = OSPI_PRV_RMW_MASKED(R_OSPI->MDTR, R_OSPI_MDTR_DV0DEL_Msk, 0U, channel);

        /* Set read/write dummy clocks to 0 in SPI mode. */
        R_OSPI->MDLR = OSPI_PRV_RMW_MASKED(R_OSPI->MDLR, R_OSPI_MDLR_DV0RDL_Msk, 0U, channel);
    }

    return ret;
}

/*******************************************************************************************************************//**
 * Gets device status.
 *
 * @param[in]  p_instance_ctrl         Pointer to a driver handle
 * @param[in]  bit_pos                 Write-in-progress bit position
 *
 * @return True if busy, false if not.
 **********************************************************************************************************************/
static bool r_ospi_status_sub (ospi_instance_ctrl_t * p_instance_ctrl, uint8_t bit_pos)
{
    spi_flash_cfg_t const     * p_cfg          = p_instance_ctrl->p_cfg;
    spi_flash_direct_transfer_t direct_command = {0};
    if (SPI_FLASH_PROTOCOL_EXTENDED_SPI == p_instance_ctrl->spi_protocol)
    {
        direct_command.command        = p_cfg->status_command;
        direct_command.command_length = 1U;
    }
    else
    {
        ospi_opi_command_set_t const * p_opi_commands = ((ospi_extended_cfg_t *) p_cfg->p_extend)->p_opi_commands;
        direct_command.command        = p_opi_commands->status_command;
        direct_command.command_length = p_opi_commands->command_bytes & OSPI_PRV_DIRECT_COMMAND_MASK;
        direct_command.address_length = (p_cfg->address_bytes + 1U) &
                                        OSPI_PRV_DIRECT_ADDR_AND_DATA_MASK;
        direct_command.dummy_cycles = 4U;
    }

    direct_command.data_length = 1U;
    r_ospi_direct_transfer(p_instance_ctrl, &direct_command, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);

    return (direct_command.data >> bit_pos) & 1U;
}

/*******************************************************************************************************************//**
 * Send Write enable command to the OctaFlash
 *
 * @param[in]   p_instance_ctrl    Pointer to OSPI specific control structure
 **********************************************************************************************************************/
static void r_ospi_wen (ospi_instance_ctrl_t * p_instance_ctrl)
{
    spi_flash_direct_transfer_t direct_command = {0};
    spi_flash_cfg_t const     * p_cfg          = p_instance_ctrl->p_cfg;
    if (SPI_FLASH_PROTOCOL_EXTENDED_SPI == p_instance_ctrl->spi_protocol)
    {
        direct_command.command        = p_cfg->write_enable_command;
        direct_command.command_length = 1U;
    }
    else
    {
        ospi_opi_command_set_t const * p_opi_commands = ((ospi_extended_cfg_t *) p_cfg->p_extend)->p_opi_commands;
        direct_command.command        = p_opi_commands->write_enable_command;
        direct_command.command_length = p_opi_commands->command_bytes & OSPI_PRV_DIRECT_COMMAND_MASK;
    }

    r_ospi_direct_transfer(p_instance_ctrl, &direct_command, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
}

/*******************************************************************************************************************//**
 * Perform Automatic Calibration
 *
 * @param[in]   p_instance_ctrl    Pointer to OSPI specific control structure
 *
 * @retval      FSP_SUCCESS                Auto-Calibration completed successfully.
 * @retval      FSP_ERR_CALIBRATE_FAILED   Auto-Calibration failed.
 **********************************************************************************************************************/
static fsp_err_t r_ospi_automatic_calibration_seq (ospi_instance_ctrl_t * p_instance_ctrl)
{
    fsp_err_t            ret     = FSP_SUCCESS;
    ospi_device_number_t channel = p_instance_ctrl->channel;

    /* Default ACTR value is long enough for stable environment */
    R_OSPI->ACAR[channel] =
        (uint32_t) ((ospi_extended_cfg_t *) p_instance_ctrl->p_cfg->p_extend)->p_autocalibration_preamble_pattern_addr;
    uint32_t cdsr = R_OSPI->CDSR;

    /* Enable auto-calibration and allow MDTR update */
    R_OSPI->CDSR = cdsr | (uint32_t) ((1U << (R_OSPI_CDSR_ACMEME0_Pos + channel))) |
                   (uint32_t) (1U << R_OSPI_CDSR_ACMODE_Pos);

    /* Using default values for DQSSOPI nad DQSDOPI counter */
    /* MDTR.DVnDEL: Typical value(VCC=3.3, 25 degree C, typical sample) is around 0x80 */
    while (0 == (R_OSPI->ACSR & (uint32_t) (R_OSPI_ACSR_ACSR0_Msk << (channel * R_OSPI_ACSR_ACSR1_Pos))))
    {
        /* TODO: Add timeout */
    }

    /* Disable automatic calibration */
    R_OSPI->CDSR = cdsr;
    if ((OSPI_PRV_AUTOMATIC_CALIBRATION_NORMAL_END << (channel * R_OSPI_ACSR_ACSR1_Pos)) !=
        (R_OSPI->ACSR & (uint32_t) (R_OSPI_ACSR_ACSR0_Msk << (channel * R_OSPI_ACSR_ACSR1_Pos))))
    {
        ret = FSP_ERR_CALIBRATE_FAILED;
    }

    /* Clear automatic calibration status */
    R_OSPI->ACSR =
        (uint32_t) (R_OSPI->ACSR &
                    ~(uint32_t) (R_OSPI_ACSR_ACSR0_Msk << (channel * R_OSPI_ACSR_ACSR1_Pos)));

    return ret;
}

/*******************************************************************************************************************//**
 * Performs direct data transfer with the OctaFlash
 *
 * @param[in]   p_instance_ctrl    Pointer to OSPI specific control structure
 * @param[in]   p_transfer             Pointer to transfer parameters
 * @param[in]   direction          Read/Write
 **********************************************************************************************************************/
static void r_ospi_direct_transfer (ospi_instance_ctrl_t              * p_instance_ctrl,
                                    spi_flash_direct_transfer_t * const p_transfer,
                                    spi_flash_direct_transfer_dir_t     direction)
{
    R_OSPI->DCR = p_transfer->command; /* Write OSPI command. */
    R_OSPI->DAR = p_transfer->address; /* Write OSPI address */
    /* Direct Read/Write settings */
    R_OSPI->DCSR = (uint32_t) (p_transfer->command_length << R_OSPI_DCSR_CMDLEN_Pos) |
                   (uint32_t) (p_transfer->address_length << R_OSPI_DCSR_ADLEN_Pos) |
                   (uint32_t) (p_transfer->dummy_cycles << R_OSPI_DCSR_DMLEN_Pos) |

                   /* Right shifted to match enum to register value */
                   (uint32_t) ((p_instance_ctrl->spi_protocol >> 2U) << R_OSPI_DCSR_DOPI_Pos) |
                   (uint32_t) (p_instance_ctrl->channel << R_OSPI_DCSR_ACDV_Pos) |
                   (uint32_t) (p_transfer->data_length << R_OSPI_DCSR_DALEN_Pos);
    if (SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE == direction)
    {
        if (0 == p_transfer->data_length)
        {
            /* Write any data. This will only send out command or command & address based on the settings above */
            R_OSPI->CWNDR = (uint32_t) OSPI_PRV_OPEN;
        }
        else
        {
            R_OSPI->CWDR = p_transfer->data;
        }
    }
    else
    {
        p_transfer->data = R_OSPI->CRR;
    }
}
