/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED  AND RENESAS MAKES NO WARRANTIES REGARDING
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
* Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version Description
*         : 05.10.2020 1.00        First Release.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include "r_sce_if.h"
#include "hw_sce_ra_private.h"

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

void HW_SCE_p_func027_r2(uint32_t ARG1)
{
    uint32_t iLoop = 0u, iLoop1 = 0u, iLoop2 = 0u, jLoop = 0u, kLoop = 0u, oLoop = 0u, oLoop1 = 0u, oLoop2 = 0u, KEY_ADR = 0u, OFS_ADR = 0u, MAX_CNT2 = 0u;
    uint32_t dummy = 0u;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    SCE->REG_ECH = 0x38000f5au;
    SCE->REG_ECH = 0x00030020u;
    SCE->REG_ECH = 0x0000b7c0u;
    SCE->REG_ECH = 0x01305c44u;
    SCE->REG_ECH = 0x00000060u;
    SCE->REG_ECH = 0x0000b7c0u;
    SCE->REG_ECH = 0x0142859du;
    SCE->REG_ECH = 0x00000080u;
    SCE->REG_C4H = 0x00443a0cu;
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = change_endian_long(0x00000000u);
    SCE->REG_C4H = 0x000c3e1cu;
    SCE->REG_E0H = 0x810103c0u;
    SCE->REG_00H = 0x00002807u;
    /* WAIT_LOOP */
    while (0u != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800u;
    SCE->REG_104H = 0x00003b62u;
    SCE->REG_D0H = 0x00000e00u;
    SCE->REG_C4H = 0x02f087bfu;
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+28 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+28 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+28 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+28 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+32 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+32 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+32 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+32 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+36 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+36 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+36 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+36 + 3];
    SCE->REG_00H = 0x00003233u;
    SCE->REG_2CH = 0x00000015u;
    /* WAIT_LOOP */
    while (0u != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800u;
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+40 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+40 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+40 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+40 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+44 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+44 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+44 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+44 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+48 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+48 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+48 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+48 + 3];
    SCE->REG_00H = 0x00003233u;
    SCE->REG_2CH = 0x00000013u;
    /* WAIT_LOOP */
    while (0u != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800u;
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+52 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+52 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+52 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+52 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+56 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+56 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+56 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+56 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+60 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+60 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+60 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+60 + 3];
    SCE->REG_00H = 0x00003233u;
    SCE->REG_2CH = 0x0000001du;
    /* WAIT_LOOP */
    while (0u != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800u;
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+64 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+64 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+64 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+64 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+68 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+68 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+68 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+68 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+72 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+72 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+72 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+72 + 3];
    SCE->REG_00H = 0x00003233u;
    SCE->REG_2CH = 0x0000001cu;
    /* WAIT_LOOP */
    while (0u != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800u;
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+76 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+76 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+76 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+76 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+80 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+80 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+80 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+80 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+84 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+84 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+84 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+84 + 3];
    SCE->REG_00H = 0x00003233u;
    SCE->REG_2CH = 0x00000014u;
    /* WAIT_LOOP */
    while (0u != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800u;
    SCE->REG_104H = 0x00000b62u;
    SCE->REG_D0H = 0x00000200u;
    SCE->REG_C4H = 0x00f087bfu;
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+88 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+88 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+88 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+88 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+92 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+92 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+92 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+92 + 3];
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+96 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+96 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+96 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+96 + 3];
    SCE->REG_00H = 0x00003233u;
    SCE->REG_2CH = 0x0000001au;
    /* WAIT_LOOP */
    while (0u != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800u;
    SCE->REG_C4H = 0x000007bdu;
    /* WAIT_LOOP */
    while (1u != SCE->REG_104H_b.B31)
    {
        /* waiting */
    }
    SCE->REG_100H = S_FLASH2[ARG1+100 + 0];
    SCE->REG_100H = S_FLASH2[ARG1+100 + 1];
    SCE->REG_100H = S_FLASH2[ARG1+100 + 2];
    SCE->REG_100H = S_FLASH2[ARG1+100 + 3];
    SCE->REG_C4H = 0x00800c45u;
    SCE->REG_00H = 0x00002213u;
    /* WAIT_LOOP */
    while (0u != SCE->REG_00H_b.B25)
    {
        /* waiting */
    }
    SCE->REG_1CH = 0x00001800u;
}

/***********************************************************************************************************************
End of function ./input_dir/S6C1/Cryptographic_PlainKey/HW_SCE_p_func027_r2.prc
***********************************************************************************************************************/
