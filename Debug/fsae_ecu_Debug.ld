/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2025 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 * Generated linker script file for LPC55S06
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v24.9 [Build 25] [2024-09-26] on 14 may. 2025 20:20:57
 */

INCLUDE "fsae_ecu_Debug_library.ld"
INCLUDE "fsae_ecu_Debug_memory.ld"

ENTRY(ResetISR)

SECTIONS
{
     /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        LONG(LOADADDR(.data_RAM3));
        LONG(    ADDR(.data_RAM3));
        LONG(  SIZEOF(.data_RAM3));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        LONG(    ADDR(.bss_RAM3));
        LONG(  SIZEOF(.bss_RAM3));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > PROGRAM_FLASH
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > PROGRAM_FLASH

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > PROGRAM_FLASH
 
    _etext = .;
        
    /* DATA section for SRAMX */

    .data_RAM2 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM2 = .) ;
        PROVIDE(__start_data_SRAMX = .) ;
        *(.ramfunc.$RAM2)
        *(.ramfunc.$SRAMX)
        *(.data.$RAM2)
        *(.data.$SRAMX)
        *(.data.$RAM2.*)
        *(.data.$SRAMX.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM2 = .) ;
        PROVIDE(__end_data_SRAMX = .) ;
     } > SRAMX AT>PROGRAM_FLASH

    /* DATA section for SRAM3 */

    .data_RAM3 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM3 = .) ;
        PROVIDE(__start_data_SRAM3 = .) ;
        *(.ramfunc.$RAM3)
        *(.ramfunc.$SRAM3)
        *(.data.$RAM3)
        *(.data.$SRAM3)
        *(.data.$RAM3.*)
        *(.data.$SRAM3.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM3 = .) ;
        PROVIDE(__end_data_SRAM3 = .) ;
     } > SRAM3 AT>PROGRAM_FLASH

    /* MAIN DATA SECTION */
    .uninit_RESERVED (NOLOAD) : ALIGN(4)
    {
        _start_uninit_RESERVED = .;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > SRAM0_1_2 AT> SRAM0_1_2

    /* Main DATA section (SRAM0_1_2) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_RAM = .) ;
       PROVIDE(__start_data_SRAM0_1_2 = .) ;
       *(vtable)
       *(.ramfunc*)
       KEEP(*(CodeQuickAccess))
       KEEP(*(DataQuickAccess))
       *(RamFunction)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
       PROVIDE(__end_data_RAM = .) ;
       PROVIDE(__end_data_SRAM0_1_2 = .) ;
    } > SRAM0_1_2 AT>PROGRAM_FLASH

    /* BSS section for SRAMX */
    .bss_RAM2 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM2 = .) ;
       PROVIDE(__start_bss_SRAMX = .) ;
       *(.bss.$RAM2)
       *(.bss.$SRAMX)
       *(.bss.$RAM2.*)
       *(.bss.$SRAMX.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
       PROVIDE(__end_bss_SRAMX = .) ;
    } > SRAMX AT> SRAMX

    /* BSS section for SRAM3 */
    .bss_RAM3 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM3 = .) ;
       PROVIDE(__start_bss_SRAM3 = .) ;
       *(.bss.$RAM3)
       *(.bss.$SRAM3)
       *(.bss.$RAM3.*)
       *(.bss.$SRAM3.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM3 = .) ;
       PROVIDE(__end_bss_SRAM3 = .) ;
    } > SRAM3 AT> SRAM3

    /* MAIN BSS SECTION */
    .bss (NOLOAD) : ALIGN(4)
    {
        _bss = .;
        PROVIDE(__start_bss_RAM = .) ;
        PROVIDE(__start_bss_SRAM0_1_2 = .) ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(__end_bss_RAM = .) ;
        PROVIDE(__end_bss_SRAM0_1_2 = .) ;
        PROVIDE(end = .);
    } > SRAM0_1_2 AT> SRAM0_1_2

    /* NOINIT section for SRAMX */
    .noinit_RAM2 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM2 = .) ;
       PROVIDE(__start_noinit_SRAMX = .) ;
       *(.noinit.$RAM2)
       *(.noinit.$SRAMX)
       *(.noinit.$RAM2.*)
       *(.noinit.$SRAMX.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM2 = .) ;
       PROVIDE(__end_noinit_SRAMX = .) ;
    } > SRAMX AT> SRAMX

    /* NOINIT section for SRAM3 */
    .noinit_RAM3 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM3 = .) ;
       PROVIDE(__start_noinit_SRAM3 = .) ;
       *(.noinit.$RAM3)
       *(.noinit.$SRAM3)
       *(.noinit.$RAM3.*)
       *(.noinit.$SRAM3.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM3 = .) ;
       PROVIDE(__end_noinit_SRAM3 = .) ;
    } > SRAM3 AT> SRAM3

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_RAM = .) ;
        PROVIDE(__start_noinit_SRAM0_1_2 = .) ;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
       PROVIDE(__end_noinit_RAM = .) ;
       PROVIDE(__end_noinit_SRAM0_1_2 = .) ;        
    } > SRAM0_1_2 AT> SRAM0_1_2

    /* Reserve and place Heap within memory map */
    _HeapSize = 0x1000;
    .heap (NOLOAD) :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > SRAM0_1_2

     _StackSize = 0x1000;
     /* Reserve space in memory for Stack */
    .heap2stackfill (NOLOAD) :
    {
        . += _StackSize;
    } > SRAM0_1_2
    /* Locate actual Stack in memory map */
    .stack ORIGIN(SRAM0_1_2) + LENGTH(SRAM0_1_2) - _StackSize - 0 (NOLOAD) :  ALIGN(4)
    {
        _vStackBase = .;
        . = ALIGN(4);
        _vStackTop = . + _StackSize;
    } > SRAM0_1_2

    /* ## Create checksum value (used in startup) ## */
    PROVIDE(__valid_user_code_checksum = 0 - 
                                         (_vStackTop 
                                         + (ResetISR + 1) 
                                         + (NMI_Handler + 1) 
                                         + (HardFault_Handler + 1) 
                                         + (( DEFINED(MemManage_Handler) ? MemManage_Handler : 0 ) + 1)   /* MemManage_Handler may not be defined */
                                         + (( DEFINED(BusFault_Handler) ? BusFault_Handler : 0 ) + 1)     /* BusFault_Handler may not be defined */
                                         + (( DEFINED(UsageFault_Handler) ? UsageFault_Handler : 0 ) + 1) /* UsageFault_Handler may not be defined */
                                         ) );

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}