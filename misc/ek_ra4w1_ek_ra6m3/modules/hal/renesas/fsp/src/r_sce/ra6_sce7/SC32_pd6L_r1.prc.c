/////////////////////////////////////////////////////////////////////////

// <SC32#2 P-256 key generation(hiding private key)>                   //
// Procedure number: d6                                                //
// File name      : SC32_pd6L_r1.prc                                   //
// State Diagram  : main(FSM1)                                         //
// Start State    : main03                                             //
// End State      : main03                                             //
// Input Data     : InData_DomainParam[32] (a || b || p || n)          //
// : InData_G[16] (Gx || Gy)                            //
// Output Data    : OutData_KeyIndex[13]                               //
// : OutData_PubKey[16](Px || Py)                       //
// Return Value   : Pass or Fail or Resource_Conflict                  //
// ---------------------------------------------------------------------//
// total cycle    : polling + write access + read access               //
// polling        : 410.6k cycle                                       //
// polling access :  47974 times                                       //
// write access   :  96865 times                                       //
// read  access   :   2096 times                                       //
/////////////////////////////////////////////////////////////////////////

#include "SCE_ProcCommon.h"
#include "hw_sce_ecc_private.h"
#include "hw_sce_private.h"

fsp_err_t HW_SCE_ECC_256HrkGenerateKey (const uint32_t * InData_DomainParam,
                                        const uint32_t * InData_G,
                                        uint32_t       * OutData_KeyIndex,
                                        uint32_t       * OutData_PubKey) {
    uint32_t lLoop;
    uint32_t iLoop;
    uint32_t jLoop;

    if (RD1_MASK(REG_1BCH, 0x0000001f) != 0x0000000)
    {
        // Busy now then retry later;
        // Because  1_xxxx other processing box is in duty.
        // x_1xxx resource D is in duty,then retry after you released D.
        // x_x1xx resource C is in duty,then retry after you released C.
        // x_xx1x resource B is in duty,then retry after you released B.
        // x_xxx1 resource A is in duty,then retry after you released A.

        // A,B,C,D: see Primitive Level Procedures datasheet
        return FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT;
    }

    WR1_PROG(REG_84H, 0x0000d601);
    WR1_PROG(REG_108H, 0x00000000);

    WR1_PROG(REG_1C0H, 0x00010001);

    WR1_PROG(REG_28H, 0x00000001);

    WR1_PROG(REG_28H, 0x00870001);

    WR1_PROG(REG_1CH, 0x00000088);
    WR1_PROG(REG_D0H, 0x00000100);
    WR1_PROG(REG_C4H, 0x00008887);
    WR1_PROG(REG_00H, 0x00003223);
    WR1_PROG(REG_2CH, 0x00000018);
    for (lLoop = 0; lLoop < 8; lLoop = lLoop + 4)
    {
        WAIT_STS(REG_00H, 22, 1);
        WR4_ADDR(REG_C0H, &InData_DomainParam[(0) + lLoop]);
    }

    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_1CH, 0x00000088);
    WR1_PROG(REG_D0H, 0x00000100);
    WR1_PROG(REG_C4H, 0x00008887);
    WR1_PROG(REG_00H, 0x00003223);
    WR1_PROG(REG_2CH, 0x00000014);
    for (lLoop = 0; lLoop < 8; lLoop = lLoop + 4)
    {
        WAIT_STS(REG_00H, 22, 1);
        WR4_ADDR(REG_C0H, &InData_DomainParam[(16) + lLoop]);
    }

    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_1CH, 0x00000088);
    WR1_PROG(REG_D0H, 0x00000100);
    WR1_PROG(REG_C4H, 0x00008887);
    WR1_PROG(REG_00H, 0x00003223);
    WR1_PROG(REG_2CH, 0x00000015);
    for (lLoop = 0; lLoop < 8; lLoop = lLoop + 4)
    {
        WAIT_STS(REG_00H, 22, 1);
        WR4_ADDR(REG_C0H, &InData_DomainParam[(24) + lLoop]);
    }

    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_1CH, 0x00000088);
    WR1_PROG(REG_D0H, 0x00000100);
    WR1_PROG(REG_C4H, 0x00008887);
    WR1_PROG(REG_00H, 0x00003223);
    WR1_PROG(REG_2CH, 0x0000001a);
    for (lLoop = 0; lLoop < 8; lLoop = lLoop + 4)
    {
        WAIT_STS(REG_00H, 22, 1);
        WR4_ADDR(REG_C0H, &InData_G[(0) + lLoop]);
    }

    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_1CH, 0x00000088);
    WR1_PROG(REG_D0H, 0x00000100);
    WR1_PROG(REG_C4H, 0x00008887);
    WR1_PROG(REG_00H, 0x00003223);
    WR1_PROG(REG_2CH, 0x0000001b);
    for (lLoop = 0; lLoop < 8; lLoop = lLoop + 4)
    {
        WAIT_STS(REG_00H, 22, 1);
        WR4_ADDR(REG_C0H, &InData_G[(8) + lLoop]);
    }

    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_24H, 0x000084d0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x00021128);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x10001df1);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x00001de1);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_34H, 0x00000800);
    WR1_PROG(REG_24H, 0x8000e0c1);
    WAIT_STS(REG_24H, 21, 0);

    SC32_function001(0x2c5698cb, 0x60fba181, 0x1df65186, 0x5ec39f13);

    WR1_PROG(REG_104H, 0x00000252);
    WR1_PROG(REG_C4H, 0x01003774);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x010037f4);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x010037f4);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x00060805);
    WR1_PROG(REG_00H, 0x00002213);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_104H, 0x00000052);
    WR1_PROG(REG_C4H, 0x01073644);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x000b0805);
    WR1_PROG(REG_00H, 0x00002213);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);
    WR1_PROG(REG_104H, 0x00000052);
    WR1_PROG(REG_C4H, 0x01000c84);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_00H, 0x00003213);
    WR1_PROG(REG_2CH, 0x00000011);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    SC32_function001(0xd2ec6cfd, 0x5c6d8be3, 0x9b07d59f, 0xafe984fa);

    WR1_PROG(REG_104H, 0x00000252);
    WR1_PROG(REG_C4H, 0x01003774);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x010037f4);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x010037f4);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x00060805);
    WR1_PROG(REG_00H, 0x00002213);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_104H, 0x00000052);
    WR1_PROG(REG_C4H, 0x01073644);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x000b0805);
    WR1_PROG(REG_00H, 0x00002213);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);
    WR1_PROG(REG_104H, 0x00000052);
    WR1_PROG(REG_C4H, 0x01000c84);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_00H, 0x00003213);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_24H, 0x000005c0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x000040c1);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x00000b0c);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x10001df1);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x00001de1);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x0000a0c1);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x000084d0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x00021028);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x100019b1);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x000019a1);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_34H, 0x00000800);
    WR1_PROG(REG_24H, 0x8000c0c1);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x00004404);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x0000e8d0);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_34H, 0x00000808);
    WR1_PROG(REG_24H, 0x8000f0d0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_34H, 0x00000020);
    WR1_PROG(REG_24H, 0x80009cd0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_34H, 0x00000800);
    WR1_PROG(REG_24H, 0x8000f004);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_34H, 0x00000002);
    WR1_PROG(REG_24H, 0x800088d0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_34H, 0x00000800);
    WR1_PROG(REG_24H, 0x80005004);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_34H, 0x00000802);
    WR1_PROG(REG_24H, 0x800088d0);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_34H, 0x00000002);
    WR1_PROG(REG_24H, 0x80008cd0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_34H, 0x00000800);
    WR1_PROG(REG_24H, 0x80007004);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_34H, 0x00000802);
    WR1_PROG(REG_24H, 0x8000acd0);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_ECH, 0x00000bff);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_E0H, 0x8088001f);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_00H, 0x00008323);
    WR1_PROG(REG_2CH, 0x00000021);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_ECH, 0x00000bde);
    WR1_PROG(REG_1D0H, 0x00000000);

    WR1_PROG(REG_ECH, 0x00000842);
    WR1_PROG(REG_1D0H, 0x00000000);

    WR1_PROG(REG_ECH, 0x0000b420);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_ECH, 0x00000004);
    WR1_PROG(REG_1D0H, 0x00000000);

    WR1_PROG(REG_ECH, 0x0000b480);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_ECH, 0x00000100);
    WR1_PROG(REG_1D0H, 0x00000000);

    WR1_PROG(REG_ECH, 0x0000b7a0);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_ECH, 0x000000d6);
    WR1_PROG(REG_1D0H, 0x00000000);

    WR1_PROG(REG_ECH, 0x00000b9c);
    WR1_PROG(REG_1D0H, 0x00000000);

    SC32_function001(0xd41a96a7, 0xcaad689f, 0xcfee7046, 0xaf924edf);
    WR1_PROG(REG_E0H, 0x81010380);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_04H, 0x00000607);
    WAIT_STS(REG_04H, 30, 1);
    RD1_ADDR(REG_100H, &S_RAM[0]);

    for (iLoop = 0; iLoop < 256; iLoop = iLoop + 1)
    {
        SC32_function002(0xc11fa551, 0xf4126407, 0xce905268, 0xd727007b);

        SC32_Subprc03();

        if (S_RAM[0] == 0x00000001)
        {
            break;
        }

        SC32_function002(0x2d61c703, 0x9011fe62, 0xc33622eb, 0x212f68e0);
    }

    WR1_PROG(REG_24H, 0x00001dc0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x00000591);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_1CH, 0x00001f00);
    WR1_PROG(REG_1CH, 0x00210000);

    SC32_function001(0x1151801b, 0x13b558b2, 0x376d8707, 0xb0aaa971);
    WR1_PROG(REG_1CH, 0x00400000);
    WR1_PROG(REG_1D0H, 0x00000000);

    if (CHCK_STS(REG_1CH, 22, 1))
    {
        WR1_PROG(REG_1C0H, 0x00010000);

        SC32_function003(0x22ac31bf, 0x3580d0ea, 0x0107a0c1, 0xda6e0199);
        WR1_PROG(REG_1BCH, 0x00000040);
        WAIT_STS(REG_18H, 12, 0);

        return FSP_ERR_CRYPTO_SCE_FAIL;
    }

    WR1_PROG(REG_24H, 0x00001dc0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x000080c1);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_34H, 0x00000800);
    WR1_PROG(REG_24H, 0x8000f4d0);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x000019c0);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x00000591);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x00000591);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x0000a0c1);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_24H, 0x00005004);
    WAIT_STS(REG_24H, 21, 0);
    WR1_PROG(REG_24H, 0x00008404);
    WAIT_STS(REG_24H, 21, 0);

    WR1_PROG(REG_ECH, 0x0000b420);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_ECH, 0x00000004);
    WR1_PROG(REG_1D0H, 0x00000000);

    WR1_PROG(REG_ECH, 0x00000bff);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_E0H, 0x8088001f);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_00H, 0x00008323);
    WR1_PROG(REG_2CH, 0x00000021);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    for (iLoop = 0; iLoop < 8; iLoop = iLoop + 1)
    {
        WR1_PROG(REG_ECH, 0x0000381f);
        WR1_PROG(REG_1D0H, 0x00000000);

        for (jLoop = 0; jLoop < 32; jLoop = jLoop + 1)
        {
            WR1_PROG(REG_24H, 0x0000102c);
            WAIT_STS(REG_24H, 21, 0);
            WR1_PROG(REG_24H, 0x100019b1);
            WAIT_STS(REG_24H, 21, 0);

            WR1_PROG(REG_24H, 0x000019a1);
            WAIT_STS(REG_24H, 21, 0);
            WR1_PROG(REG_24H, 0x000080c1);
            WAIT_STS(REG_24H, 21, 0);

            WR1_PROG(REG_ECH, 0x3800d81f);
            WR1_PROG(REG_1D0H, 0x00000000);
            WR1_PROG(REG_E0H, 0x00000080);
            WR1_PROG(REG_1CH, 0x00A60000);

            WR1_PROG(REG_ECH, 0x00016c00);
            WR1_PROG(REG_1D0H, 0x00000000);

            SC32_function001(0x701f17c2, 0xf9328cb5, 0x9efb2f4a, 0xd6ca1444);
            WR1_PROG(REG_1CH, 0x00400000);
            WR1_PROG(REG_1D0H, 0x00000000);

            if (CHCK_STS(REG_1CH, 22, 1))
            {
                WR1_PROG(REG_24H, 0x0000082c);
                WAIT_STS(REG_24H, 21, 0);
                WR1_PROG(REG_24H, 0x100019b1);
                WAIT_STS(REG_24H, 21, 0);

                WR1_PROG(REG_24H, 0x000019a1);
                WAIT_STS(REG_24H, 21, 0);
                WR1_PROG(REG_24H, 0x000080c1);
                WAIT_STS(REG_24H, 21, 0);

                SC32_function002(0x7bbf0897, 0x980e929e, 0x37215852, 0xcc6f5d15);
            }
            else
            {
                SC32_function002(0xe06a4305, 0x660a2257, 0x6917d826, 0x9ad28d0b);
            }
        }

        WR1_PROG(REG_ECH, 0x000027e1);
        WR1_PROG(REG_1D0H, 0x00000000);

        SC32_function002(0xc3414398, 0x17ed3821, 0xe946a664, 0x1be23fe8);
    }

    WR1_PROG(REG_ECH, 0x00008be0);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_ECH, 0x00000020);
    WR1_PROG(REG_1D0H, 0x00000000);

    WR1_PROG(REG_ECH, 0x00007c1f);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_1CH, 0x00602000);
    WR1_PROG(REG_1D0H, 0x00000000);

    SC32_Subprc04();

    SC32_function001(0x9d09c940, 0xeab518c4, 0x48e28758, 0xff2f29ac);
    WR1_PROG(REG_D0H, 0x00000100);
    WR1_PROG(REG_C4H, 0x00008887);
    WR1_PROG(REG_00H, 0x00002323);
    WR1_PROG(REG_2CH, 0x00000022);
    WR1_PROG(REG_04H, 0x00000222);
    for (lLoop = 0; lLoop < 8; lLoop = lLoop + 4)
    {
        WAIT_STS(REG_04H, 30, 1);
        RD4_ADDR(REG_C0H, &OutData_PubKey[(0) + lLoop]);
    }

    SC32_function001(0xcdd0662c, 0x55619706, 0xe2d07623, 0x2031ae93);
    WR1_PROG(REG_D0H, 0x00000100);
    WR1_PROG(REG_C4H, 0x00008887);
    WR1_PROG(REG_00H, 0x00002323);
    WR1_PROG(REG_2CH, 0x00000023);
    WR1_PROG(REG_04H, 0x00000222);
    for (lLoop = 0; lLoop < 8; lLoop = lLoop + 4)
    {
        WAIT_STS(REG_04H, 30, 1);
        RD4_ADDR(REG_C0H, &OutData_PubKey[(8) + lLoop]);
    }

    SC32_function001(0x4bfe85bd, 0xed78b78a, 0x4b08e6f8, 0xf8f56506);

    WR1_PROG(REG_104H, 0x00000252);
    WR1_PROG(REG_C4H, 0x01003774);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x010037f4);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x010037f4);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x00060805);
    WR1_PROG(REG_00H, 0x00002213);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_104H, 0x00000052);
    WR1_PROG(REG_C4H, 0x01073644);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_C4H, 0x000b0805);
    WR1_PROG(REG_00H, 0x00002213);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);
    WR1_PROG(REG_104H, 0x00000052);
    WR1_PROG(REG_C4H, 0x01000c84);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0x00000000);

    WR1_PROG(REG_E0H, 0x80040000);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_00H, 0x00008213);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    SC32_function001(0xb0b1ac36, 0x3fc63c55, 0xd666f558, 0xf26f506f);
    WR1_PROG(REG_E0H, 0x81010000);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_C4H, 0x00000884);
    WR1_PROG(REG_00H, 0x00002807);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);
    WR1_PROG(REG_04H, 0x00000207);
    WAIT_STS(REG_04H, 30, 1);
    RD1_ADDR(REG_C0H, &OutData_KeyIndex[0]);
    WR1_PROG(REG_00H, 0x0000020f);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_C4H, 0x200c3b0c);
    WR1_PROG(REG_E0H, 0x81010000);
    WR1_PROG(REG_1D0H, 0x00000000);
    WR1_PROG(REG_00H, 0x00002807);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_C4H, 0x000c0b0c);
    WAIT_STS(REG_104H, 31, 1);
    WR1_PROG(REG_100H, 0xa4cf6fd0);

    SC32_function001(0xc3c62cd2, 0xaf894fcc, 0xaa47522f, 0x47abe1d2);
    WR1_PROG(REG_D0H, 0x00000100);
    WR1_PROG(REG_C4H, 0x02e487bf);
    WR1_PROG(REG_00H, 0x00002323);
    WR1_PROG(REG_2CH, 0x0000002d);
    WAIT_STS(REG_00H, 25, 0);
    WR1_PROG(REG_1CH, 0x00001800);

    WR1_PROG(REG_104H, 0x00000352);
    WR1_PROG(REG_C4H, 0x000009cd);
    WAIT_STS(REG_104H, 31, 1);
    WR4_PROG(REG_100H, 0x00000000, 0x00000000, 0x00000000, 0x00000002);

    WR1_PROG(REG_04H, 0x00000233);
    WAIT_STS(REG_04H, 30, 1);
    RD4_ADDR(REG_C0H, &OutData_KeyIndex[1]);
    WAIT_STS(REG_04H, 30, 1);
    RD4_ADDR(REG_C0H, &OutData_KeyIndex[5]);
    WAIT_STS(REG_04H, 30, 1);
    RD4_ADDR(REG_C0H, &OutData_KeyIndex[9]);

    WR1_PROG(REG_1C0H, 0x00010000);

    SC32_function003(0x1a71ddbf, 0xd3c46d81, 0xbe81cb85, 0xa3f7a2da);
    WR1_PROG(REG_1BCH, 0x00000040);
    WAIT_STS(REG_18H, 12, 0);

    return FSP_SUCCESS;
}
