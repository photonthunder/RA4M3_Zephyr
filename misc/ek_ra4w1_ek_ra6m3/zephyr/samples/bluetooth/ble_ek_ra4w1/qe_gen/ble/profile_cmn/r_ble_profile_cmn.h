/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
* this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2018 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

#ifndef R_BLE_PROFILE_CMN_H
#define R_BLE_PROFILE_CMN_H

#ifdef ENABLE_PROFILE_UT_TEST
#define UT_MOCK(name) __mock_ ## name
#else
#define UT_MOCK(name) name
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array)\
        (sizeof(array) / sizeof(array[0]))
#endif

/**
 *  Packing Macros.
 *
 *  Syntax: BT_PACK_<Endian-ness LE/BE>_<no_of_bytes>_BYTE
 *
 *  Usage: Based on the endian-ness defined for each protocol/profile layer,
 *  appropriate packing macros to be used by each layer.
 *
 *  Example: HCI is defined as little endian protocol,
 *  so if HCI defines HCI_PACK_2_BYTE for packing a parameter of size 2 byte,
 *  that shall be mapped to BT_PACK_LE_2_BYTE
 *
 *  By default both the packing and unpaking macros uses pointer to
 *  a single or multi-octet variable which to be packed to or unpacked from
 *  a buffer (unsinged character array).
 *
 *  For the packing macro, another variation is available,
 *  where the single or multi-octet variable itself is used (not its pointer).
 *
 *  Syntax: BT_PACK_<Endian-ness LE/BE>_<no_of_bytes>_BYTE_VAL
 */
/* Little Endian Packing Macros */
#ifndef BT_PACK_LE_1_BYTE
#define BT_PACK_LE_1_BYTE(dst, src) \
    { \
        uint8_t val; \
        val = (uint8_t)(*(src)); \
        BT_PACK_LE_1_BYTE_VAL((dst), val); \
    }
#endif

#ifndef BT_PACK_LE_1_BYTE_VAL
#define BT_PACK_LE_1_BYTE_VAL(dst, src) \
    *((uint8_t *)(dst) + 0) = (src);
#endif

#ifndef BT_PACK_LE_2_BYTE
#define BT_PACK_LE_2_BYTE(dst, src) \
    { \
        uint16_t val; \
        val = (uint16_t)(*(src)); \
        BT_PACK_LE_2_BYTE_VAL((dst), val); \
    }
#endif

#ifndef BT_PACK_LE_2_BYTE_VAL
#define BT_PACK_LE_2_BYTE_VAL(dst, src) \
    *((uint8_t *)(dst) + 0) = (uint8_t)(src); \
    *((uint8_t *)(dst) + 1) = (uint8_t)((src) >> 8);
#endif

#ifndef BT_PACK_LE_3_BYTE
#define BT_PACK_LE_3_BYTE(dst, src) \
    { \
        uint32_t val; \
        val = (uint32_t)(*(src)); \
        BT_PACK_LE_3_BYTE_VAL((dst), val); \
    }
#endif

#ifndef BT_PACK_LE_3_BYTE_VAL
#define BT_PACK_LE_3_BYTE_VAL(dst, src) \
    *((uint8_t *)(dst) + 0) = (uint8_t)(src);\
    *((uint8_t *)(dst) + 1) = (uint8_t)((src) >> 8);\
    *((uint8_t *)(dst) + 2) = (uint8_t)((src) >> 16);
#endif

#ifndef BT_PACK_LE_4_BYTE
#define BT_PACK_LE_4_BYTE(dst, src) \
    { \
        uint32_t val; \
        val = (uint32_t)(*(src)); \
        BT_PACK_LE_4_BYTE_VAL((dst), val); \
    }
#endif

#ifndef BT_PACK_LE_4_BYTE_VAL
#define BT_PACK_LE_4_BYTE_VAL(dst, src) \
    *((uint8_t *)(dst) + 0) = (uint8_t)(src);\
    *((uint8_t *)(dst) + 1) = (uint8_t)((src) >> 8);\
    *((uint8_t *)(dst) + 2) = (uint8_t)((src) >> 16);\
    *((uint8_t *)(dst) + 3) = (uint8_t)((src) >> 24);

/* Update based on 64 Bit, 128 Bit Data Types */
#endif

#ifndef BT_PACK_LE_8_BYTE
#define BT_PACK_LE_8_BYTE(dst,val)\
        memcpy ((dst), (val), 8)
#endif

#ifndef BT_PACK_LE_16_BYTE
#define BT_PACK_LE_16_BYTE(dst,val)\
        memcpy ((dst), (val), 16)
#endif

#ifndef BT_PACK_LE_N_BYTE
#define BT_PACK_LE_N_BYTE(dst,val,n)\
        memcpy ((dst), (val), (n))
#endif

/**
 *  Unpacking Macros.
 *
 *  Syntax: BT_UNPACK_<Endian-ness LE/BE>_<no_of_bytes>_BYTE
 *
 *  Usage: Based on the endian-ness defined for each protocol/profile layer,
 *  appropriate unpacking macros to be used by each layer.
 *
 *  Example: HCI is defined as little endian protocol,
 *  so if HCI defines HCI_UNPACK_4_BYTE for unpacking a parameter of size 4 byte,
 *  that shall be mapped to BT_UNPACK_LE_4_BYTE
 */
/* Little Endian Unpacking Macros */
#ifndef BT_UNPACK_LE_1_BYTE
#define BT_UNPACK_LE_1_BYTE(dst,src)\
    *((uint8_t *)(dst)) = (uint8_t)(*((uint8_t *)(src)));
#endif

#ifndef BT_UNPACK_LE_2_BYTE
#define BT_UNPACK_LE_2_BYTE(dst,src)\
        *((uint16_t *)(dst)) = (uint16_t)( \
                        (((uint16_t)(*((src) + 0))) <<  0) | \
                        (((uint16_t)(*((src) + 1))) <<  8)   \
                    );
#endif

#ifndef BT_UNPACK_LE_3_BYTE
#define BT_UNPACK_LE_3_BYTE(dst,src)\
        *((uint32_t *)(dst)) = (uint32_t)( \
                        (((uint32_t)(*((src) + 0))) <<  0) | \
                        (((uint32_t)(*((src) + 1))) <<  8) | \
                        (((uint32_t)(*((src) + 2))) << 16)   \
                    );
#endif

#ifndef BT_UNPACK_LE_4_BYTE
#define BT_UNPACK_LE_4_BYTE(dst,src)\
        *((uint32_t *)(dst)) = (uint32_t)( \
                        (((uint32_t)(*((src) + 0))) <<  0) | \
                        (((uint32_t)(*((src) + 1))) <<  8) | \
                        (((uint32_t)(*((src) + 2))) << 16) | \
                        (((uint32_t)(*((src) + 3))) << 24)   \
                    );
#endif

/* Update based on 64 Bit, 128 Bit Data Types */
#ifndef BT_UNPACK_LE_8_BYTE
#define BT_UNPACK_LE_8_BYTE(dst,src)\
        memcpy ((dst), (src), 8)
#endif

#ifndef BT_UNPACK_LE_16_BYTE
#define BT_UNPACK_LE_16_BYTE(dst,src)\
        memcpy ((dst), (src), 16)
#endif

#ifndef BT_UNPACK_LE_N_BYTE
#define BT_UNPACK_LE_N_BYTE(dst,src,n)\
        memcpy ((dst), (src), (n))
#endif

#ifndef MAX
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

/*******************************************************************************************************************//**
 * @file
 * @defgroup profile_cmn Profile Common Library
 * @{
 * @ingroup app_lib
 * @brief Profile Common Library
 * @details This library provides APIs to encode/decode default type and data types.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version Description           
*         : 23.08.2019 1.00    First Release
*         : 31.10.2019 1.01    Add doxygen comments.
***********************************************************************************************************************/

/** @defgroup profile_cmn_struct Structures
 *  @{
 *  @brief Structure definition
 */
/*******************************************************************************************************************//**
 * @brief IEEE 11073 FLOAT type.
***********************************************************************************************************************/
typedef struct {
    int8_t  exponent; /**< 8-bit exponent to base 10 */
    int32_t mantissa; /**< 24-bit mantissa */
} st_ble_ieee11073_float_t;

/*******************************************************************************************************************//**
 * @brief IEEE 11073 short FLOAT type.
***********************************************************************************************************************/
typedef struct {
    int8_t  exponent; /**< 4-bit exponent to base 10 */
    int16_t mantissa; /**< 12-bit mantissa */
} st_ble_ieee11073_sfloat_t;

/*******************************************************************************************************************//**
 * @brief Date Time characteristic parameters.
***********************************************************************************************************************/
typedef struct {
    uint16_t year;    /**< Year */
    uint8_t  month;   /**< Month */
    uint8_t  day;     /**< Day */
    uint8_t  hours;   /**< Hours */
    uint8_t  minutes; /**< Minutes */
    uint8_t  seconds; /**< Seconds */
} st_ble_date_time_t;
/*@}*/

#endif /* R_BLE_PROFILE_CMN_H */
/*@}*/