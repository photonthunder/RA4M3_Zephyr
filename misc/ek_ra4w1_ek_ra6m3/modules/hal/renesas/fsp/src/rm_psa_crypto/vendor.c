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

#if !defined(MBEDTLS_CONFIG_FILE)
 #include "mbedtls/config.h"
#else
 #include MBEDTLS_CONFIG_FILE
#endif

#include "vendor.h"
#include "asymmetric_vendor.h"
#include "aes_vendor.h"
#include "mbedtls/error.h"

uint32_t ecp_load_key_size(bool wrapped_mode_ctx, const mbedtls_ecp_group * grp);

psa_status_t vendor_bitlength_to_raw_bitlength (psa_key_type_t type, size_t vendor_bits, size_t * raw_bits)
{
    (void) vendor_bits;
    (void) raw_bits;
    (void) type;
    psa_status_t status = PSA_SUCCESS;
#if defined(MBEDTLS_AES_C) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_AES_FORMAT)))

    /* Check if the key is of AES type */
    if (PSA_KEY_TYPE_IS_AES(type))
    {
/* Check that the bit size is acceptable for the key type
 * NOTE: If adding wrapped 192 key support in future note that the size of the wrapped 192 and 256 AES keys
 * are the same (416 bits) making this logic unusable */
        *raw_bits = PSA_AES_BITS_VENDOR_RAW(vendor_bits);
        if (0 == *raw_bits)
        {
            status = PSA_ERROR_NOT_SUPPORTED;
        }
    }
    else
#endif
    {
        status = PSA_ERROR_NOT_SUPPORTED;
    }

    return status;
}

/** Calculate the size of the vendor key in the given slot, in bits.
 *
 * \param[in] slot      A key slot containing a transparent key.
 *
 * \return The key size in bits, calculated from the key data.
 */
static psa_key_bits_t calculate_key_bits_vendor (const psa_key_slot_t * slot)
{
    size_t bits = 0;                   /* return 0 on an empty slot */

    if (PSA_KEY_TYPE_IS_UNSTRUCTURED(slot->attr.type))
    {
        bits = PSA_BYTES_TO_BITS(slot->data.raw.bytes);
    }

#if defined(MBEDTLS_RSA_C) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_RSA_FORMAT)))
    else if (PSA_KEY_TYPE_IS_RSA(slot->attr.type))
    {
        bits = PSA_BYTES_TO_BITS(mbedtls_rsa_get_len(slot->data.rsa));
    }
#endif                                 /* defined(MBEDTLS_RSA_C) */
#if defined(MBEDTLS_ECP_C) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_ECC_FORMAT)))
    else if (PSA_KEY_TYPE_IS_ECC(slot->attr.type))
    {
        bits = PSA_BYTES_TO_BITS((size_t) (ecp_load_key_size(true, &slot->data.ecp->grp) * 4));
    }
#endif                                 /* defined(MBEDTLS_ECP_C) */
    else
    {
        bits = 0;
    }

    /* We know that the size fits in psa_key_bits_t thanks to checks
     * when the key was created. */
    return (psa_key_bits_t) bits;
}

/*************crypto_accel_driver.h implementations follow***************************/

/*
 * This function is based off of psa_generate_key_internal() in mbedCrypto.
 */
psa_status_t psa_generate_key_vendor (psa_key_slot_t * slot,
                                      size_t           bits,
                                      const uint8_t  * domain_parameters,
                                      size_t           domain_parameters_size)
{
    (void) slot;
    (void) bits;
    (void) domain_parameters;
    (void) domain_parameters_size;
    psa_status_t status = PSA_ERROR_NOT_SUPPORTED;

    if ((domain_parameters == NULL) && (domain_parameters_size != 0))
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

#if defined(MBEDTLS_AES_ALT) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_AES_FORMAT)))
    if (PSA_KEY_TYPE_IS_UNSTRUCTURED(slot->attr.type))
    {
        status = prepare_raw_data_slot_vendor(slot->attr.type, bits, &slot->data.raw);
        if (status != PSA_SUCCESS)
        {
            return status;
        }

        status = psa_generate_symmetric_vendor(slot->attr.type, bits, slot->data.raw.data, slot->data.raw.bytes);
        if (status != PSA_SUCCESS)
        {
            return status;
        }
    }
    else
#endif                                 /* defined(MBEDTLS_AES_ALT) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_AES_FORMAT))) */
#if defined(MBEDTLS_RSA_ALT) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_RSA_FORMAT)))
    if (slot->attr.type == PSA_KEY_TYPE_RSA_KEY_PAIR)
    {
        mbedtls_rsa_context * rsa;
        int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;;
        int exponent;
        if (bits > PSA_VENDOR_RSA_MAX_KEY_BITS)
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }

        /* Accept only byte - aligned keys, for the same reasons as *
         * in psa_import_rsa_key (). */
        if (bits % 8 != 0)
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }

        status = psa_read_rsa_exponent(domain_parameters, domain_parameters_size, &exponent);
        if (status != PSA_SUCCESS)
        {
            return status;
        }

        rsa = mbedtls_calloc(1, sizeof(*rsa));
        if (rsa == NULL)
        {
            return PSA_ERROR_INSUFFICIENT_MEMORY;
        }

        mbedtls_rsa_init(rsa, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_NONE);

        /* Setup the vendor context flag.
         * Even though vendor_ctx is a void pointer since we only need true/false info
         * we are using the pointer as a bool instead */
        rsa->vendor_ctx = (bool *) true;

        ret = mbedtls_rsa_gen_key(rsa, NULL, NULL, (unsigned int) bits, exponent);
        if (ret != 0)
        {
            mbedtls_rsa_free(rsa);
            mbedtls_free(rsa);

            return PSA_ERROR_HARDWARE_FAILURE;
        }

        slot->data.rsa = rsa;
    }
    else
#endif                                 /* defined(MBEDTLS_RSA_C) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_RSA_FORMAT))) */

#if defined(MBEDTLS_ECP_ALT) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_ECC_FORMAT)))
    if (PSA_KEY_TYPE_IS_ECC(slot->attr.type) && PSA_KEY_TYPE_IS_KEY_PAIR(slot->attr.type))
    {
        psa_ecc_curve_t      curve  = PSA_KEY_TYPE_GET_CURVE(slot->attr.type);
        mbedtls_ecp_group_id grp_id =
            mbedtls_ecc_group_of_psa(curve, PSA_BITS_TO_BYTES(bits));
        const mbedtls_ecp_curve_info * curve_info =
            mbedtls_ecp_curve_info_from_grp_id(grp_id);
        mbedtls_ecp_keypair * ecp;
        int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
        if (domain_parameters_size != 0)
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }

        if ((grp_id == MBEDTLS_ECP_DP_NONE) || (curve_info == NULL))
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }

        if (curve_info->bit_size != bits)
        {
            return PSA_ERROR_INVALID_ARGUMENT;
        }

        ecp = mbedtls_calloc(1, sizeof(*ecp));
        if (ecp == NULL)
        {
            return PSA_ERROR_INSUFFICIENT_MEMORY;
        }

        mbedtls_ecp_keypair_init(ecp);
        ret = ecp_gen_key_vendor(grp_id, ecp);
        if (ret != 0)
        {
            mbedtls_ecp_keypair_free(ecp);
            mbedtls_free(ecp);

            return PSA_ERROR_HARDWARE_FAILURE;
        }

        slot->data.ecp = ecp;

        /* Write the actual key size to the slot.
         * psa_start_key_creation() wrote the size declared by the
         * caller, which may be 0 (meaning unspecified) or wrong. */

        slot->attr.bits = calculate_key_bits_vendor(slot);
        status          = PSA_SUCCESS;
    }
    else
#endif                                 /* MBEDTLS_ECP_C && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_RSA_FORMAT))) */
    {
        status = PSA_ERROR_NOT_SUPPORTED;
    }

    return status;                     // NOLINT(readability-misleading-indentation)
}

/** Import key data into a slot. `slot->attr.type` must have been set
 * previously. This function assumes that the slot does not contain
 * any key material yet. On failure, the slot content is unchanged. */
psa_status_t psa_import_key_into_slot_vendor (psa_key_slot_t * slot,
                                              const uint8_t  * data,
                                              size_t           data_length,
                                              bool             write_to_persistent_memory)
{
    psa_status_t status = PSA_ERROR_NOT_SUPPORTED;
    (void) slot;
    (void) data;
    (void) data_length;
#if defined(MBEDTLS_AES_ALT) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_AES_FORMAT)))
    if (PSA_KEY_TYPE_IS_UNSTRUCTURED(slot->attr.type))
    {
        size_t bit_size = PSA_BYTES_TO_BITS(data_length);

        /* Ensure that the bytes-to-bit conversion didn't overflow. */
        if (data_length > SIZE_MAX / 8)
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }

        /* Enforce a size limit, and in particular ensure that the bit
         * size fits in its representation type. */
        if (bit_size > PSA_MAX_KEY_BITS)
        {
            return PSA_ERROR_NOT_SUPPORTED;
        }

        status = prepare_raw_data_slot_vendor(slot->attr.type, bit_size, &slot->data.raw);
        if (status != PSA_SUCCESS)
        {
            return status;
        }

        if (data_length != 0)
        {
            memcpy(slot->data.raw.data, data, data_length);
        }
    }
    else
#endif                                 /* defined(MBEDTLS_AES_ALT) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_AES_FORMAT))) */
#if defined(MBEDTLS_ECP_ALT) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_ECC_FORMAT)))
    if (PSA_KEY_TYPE_IS_ECC_KEY_PAIR(slot->attr.type))
    {
        status = psa_import_ec_private_key_vendor(PSA_KEY_TYPE_GET_CURVE(slot->attr.type),
                                                  data,
                                                  data_length,
                                                  &slot->data.ecp);
    }
    else if (PSA_KEY_TYPE_IS_ECC_PUBLIC_KEY(slot->attr.type))
    {
        status = psa_import_ec_public_key(PSA_KEY_TYPE_GET_CURVE(slot->attr.type), data, data_length, &slot->data.ecp);
    }
    else
#endif                                 /* MBEDTLS_ECP_C  && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_ECC_FORMAT))) */

#if defined(MBEDTLS_RSA_ALT) && defined(MBEDTLS_PK_PARSE_C) && \
    ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_RSA_FORMAT)))
    if (PSA_KEY_TYPE_IS_RSA(slot->attr.type))
    {
        status = psa_import_rsa_key(slot->attr.type, data, data_length, &slot->data.rsa);

        if (0 == status)
        {
            /* Setup the vendor context flag */
            slot->data.rsa->vendor_ctx = (bool *) true;
        }
    }
    else
#endif                                 /* defined(MBEDTLS_RSA_C) && defined(MBEDTLS_PK_PARSE_C) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_RSA_FORMAT))) */
    {
        status = PSA_ERROR_NOT_SUPPORTED;
    }

    if (status == PSA_SUCCESS)         // NOLINT(readability-misleading-indentation)
    {
        /* Write the actual key size to the slot.
         * psa_start_key_creation() wrote the size declared by the
         * caller, which may be 0 (meaning unspecified) or wrong. */
        slot->attr.bits = calculate_key_bits_vendor(slot);

        if (true == write_to_persistent_memory)
        {
            status = psa_finish_key_creation(slot, NULL);
        }
    }

    return status;
}

/*
 * This function is based off of psa_finish_key_creation() in mbedCrypto.
 */

psa_status_t psa_finish_key_creation_vendor (psa_key_slot_t * slot)
{
    (void) slot;
    psa_status_t status = PSA_SUCCESS;
#if defined(MBEDTLS_PSA_CRYPTO_STORAGE_C)
    size_t buffer_size = 0;
 #if defined(MBEDTLS_AES_C) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_AES_FORMAT)))

    /* Check if the key is of AES type */
    if (PSA_KEY_TYPE_IS_AES(slot->attr.type))
    {
        buffer_size = slot->data.raw.bytes;
    }
    else
 #endif                                // defined(MBEDTLS_AES_C) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_AES_FORMAT)))
    {
        buffer_size =
            (size_t) (PSA_KEY_EXPORT_MAX_SIZE(slot->attr.type, slot->attr.bits));
    }

    if (buffer_size == 0)
    {
        return PSA_ERROR_NOT_SUPPORTED;
    }

    uint8_t * buffer = mbedtls_calloc(1, buffer_size);
    size_t    length = 0;
    if (buffer == NULL)
    {
        return PSA_ERROR_INSUFFICIENT_MEMORY;
    }

    status = psa_internal_export_key(slot, buffer, buffer_size, &length, 0);
    if (status == PSA_SUCCESS)
    {
        status = psa_save_persistent_key(&slot->attr, buffer, length);
    }

    mbedtls_platform_zeroize(buffer, buffer_size);
    mbedtls_free(buffer);
#endif                                 // defined (MBEDTLS_PSA_CRYPTO_STORAGE_C)
    return status;
}
