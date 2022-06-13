/* linker.ld - Linker command/script file */

/*
 * Copyright (c) 2020 MXT Creation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arch/arm/aarch32/cortex_m/scripts/linker.ld>

__StackLimit = LOADADDR(_NOINIT_SECTION_NAME);
__StackTop = _image_ram_end;

#if 0
 /* Stacks are stored in this section. */
    .stack_dummy (NOLOAD):
    {
        . = ALIGN(8);
        __StackLimit = .;
        /* Main stack */
        KEEP(*(.stack))
        __StackTop = .;
        /* Thread stacks */
        KEEP(*(.stack*))
        __StackTopAll = .;
    } > RAM

 SECTION_PROLOGUE(_STACK_DUMMY,(NOLOAD),)
        {
       // *(.stack)

        } GROUP_LINK_IN(RAMABLE_REGION)

    /* Define linker symbols */

	__StackLimit = noinit;

	// KEEP(*(.stack*))
    __StackTopAll = _image_ram_end;

    GROUP_END(RAMABLE_REGION)
     #endif