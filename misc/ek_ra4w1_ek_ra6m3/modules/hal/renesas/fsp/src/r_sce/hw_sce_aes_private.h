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

#ifndef HW_SCE_AES_PRIVATE_H
#define HW_SCE_AES_PRIVATE_H

#include <stdint.h>
#include "bsp_api.h"

/* AES key lengths defined for SCE operations. */
#define SIZE_AES_128BIT_KEYLEN_BITS             (128)
#define SIZE_AES_128BIT_KEYLEN_BYTES            ((SIZE_AES_128BIT_KEYLEN_BITS) / 8)
#define SIZE_AES_128BIT_KEYLEN_WORDS            ((SIZE_AES_128BIT_KEYLEN_BITS) / 32)

#define SIZE_AES_128BIT_KEYLEN_BITS_WRAPPED     (288)
#define SIZE_AES_128BIT_KEYLEN_BYTES_WRAPPED    ((SIZE_AES_128BIT_KEYLEN_BITS_WRAPPED) / 8)
#define SIZE_AES_128BIT_KEYLEN_WORDS_WRAPPED    ((SIZE_AES_128BIT_KEYLEN_BITS_WRAPPED) / 32)

#define SIZE_AES_192BIT_KEYLEN_BITS_WRAPPED     (416)
#define SIZE_AES_192BIT_KEYLEN_BYTES_WRAPPED    ((SIZE_AES_192BIT_KEYLEN_BITS_WRAPPED) / 8)
#define SIZE_AES_192BIT_KEYLEN_WORDS_WRAPPED    ((SIZE_AES_192BIT_KEYLEN_BITS_WRAPPED) / 32)

#define SIZE_AES_256BIT_KEYLEN_BITS_WRAPPED     (416)
#define SIZE_AES_256BIT_KEYLEN_BYTES_WRAPPED    ((SIZE_AES_256BIT_KEYLEN_BITS_WRAPPED) / 8)
#define SIZE_AES_256BIT_KEYLEN_WORDS_WRAPPED    ((SIZE_AES_256BIT_KEYLEN_BITS_WRAPPED) / 32)

#define SIZE_AES_192BIT_KEYLEN_BITS             (192)
#define SIZE_AES_192BIT_KEYLEN_BYTES            ((SIZE_AES_192BIT_KEYLEN_BITS) / 8)
#define SIZE_AES_192BIT_KEYLEN_WORDS            ((SIZE_AES_192BIT_KEYLEN_BITS) / 32)

#define SIZE_AES_256BIT_KEYLEN_BITS             (256)
#define SIZE_AES_256BIT_KEYLEN_BYTES            ((SIZE_AES_256BIT_KEYLEN_BITS) / 8)
#define SIZE_AES_256BIT_KEYLEN_WORDS            ((SIZE_AES_256BIT_KEYLEN_BITS) / 32)

#define SIZE_AES_BLOCK_BITS                     (128)
#define SIZE_AES_BLOCK_BYTES                    (128 / 8)
#define SIZE_AES_BLOCK_WORDS                    ((SIZE_AES_BLOCK_BITS) / 32)

typedef struct st_sce_data
{
    uint32_t   length;
    uint32_t * p_data;
} r_sce_data_t;

extern fsp_err_t HW_SCE_AES_128EcbEncrypt(const uint32_t * InData_Key,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text);

extern fsp_err_t HW_SCE_AES_128EcbDecrypt(const uint32_t * InData_Key,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text);

extern fsp_err_t HW_SCE_AES_128CbcEncrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_128CbcDecrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_128CtrEncrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_128XtsEncrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t * InData_Len,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_128XtsDecrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t * InData_Len,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_256EcbEncrypt(const uint32_t * InData_Key,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text);

extern fsp_err_t HW_SCE_AES_256EcbDecrypt(const uint32_t * InData_Key,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text);

extern fsp_err_t HW_SCE_AES_256CbcEncrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_256CbcDecrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_256CtrEncrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t   num_words,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_256XtsEncrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t * InData_Len,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_256XtsDecrypt(const uint32_t * InData_Key,
                                          const uint32_t * InData_IV,
                                          const uint32_t * InData_Len,
                                          const uint32_t * InData_Text,
                                          uint32_t       * OutData_Text,
                                          uint32_t       * OutData_IV);

extern fsp_err_t HW_SCE_AES_128CreateEncryptedKey(uint32_t * OutData_KeyIndex);

extern fsp_err_t HW_SCE_AES_128EcbEncryptUsingEncryptedKey(const uint32_t * InData_KeyIndex,
                                                           const uint32_t   num_words,
                                                           const uint32_t * InData_Text,
                                                           uint32_t       * OutData_Text);

extern fsp_err_t HW_SCE_AES_128EcbDecryptUsingEncryptedKey(const uint32_t * InData_KeyIndex,
                                                           const uint32_t   num_words,
                                                           const uint32_t * InData_Text,
                                                           uint32_t       * OutData_Text);

extern fsp_err_t HW_SCE_AES_256CreateEncryptedKey(uint32_t * OutData_KeyIndex);

extern fsp_err_t HW_SCE_AES_256EcbEncryptUsingEncryptedKey(const uint32_t * InData_KeyIndex,
                                                           const uint32_t   num_words,
                                                           const uint32_t * InData_Text,
                                                           uint32_t       * OutData_Text);

extern fsp_err_t HW_SCE_AES_256EcbDecryptUsingEncryptedKey(const uint32_t * InData_KeyIndex,
                                                           const uint32_t   num_words,
                                                           const uint32_t * InData_Text,
                                                           uint32_t       * OutData_Text);

extern fsp_err_t HW_SCE_Aes128EncryptDecryptInit(const uint32_t * InData_Cmd,
                                                 const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_IV);

extern void HW_SCE_Aes128EncryptDecryptUpdate(const uint32_t * InData_Text,
                                              uint32_t       * OutData_Text,
                                              const uint32_t   num_words);

extern fsp_err_t HW_SCE_Aes128EncryptDecryptFinal(void);

extern fsp_err_t HW_SCE_Aes256EncryptDecryptInit(const uint32_t * InData_Cmd,
                                                 const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_IV);

extern void HW_SCE_Aes256EncryptDecryptUpdate(const uint32_t * InData_Text,
                                              uint32_t       * OutData_Text,
                                              const uint32_t   num_words);

extern fsp_err_t HW_SCE_Aes256EncryptDecryptFinal(void);

#endif                                 /* HW_SCE_AES_PRIVATE_H */
