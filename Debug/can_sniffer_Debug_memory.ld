/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2024 NXP
 * Generated linker script file for LPC55S06
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.10.0 [Build 3148] [2024-07-03] on 19 ago 2024 21:52:28
 */

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x3d000 /* 244K bytes (alias Flash) */  
  SRAM0_1_2 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x10000 /* 64K bytes (alias RAM) */  
  SRAMX (rwx) : ORIGIN = 0x4000000, LENGTH = 0x4000 /* 16K bytes (alias RAM2) */  
  SRAM3 (rwx) : ORIGIN = 0x20010000, LENGTH = 0x4000 /* 16K bytes (alias RAM3) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x0 + 0x3d000 ; /* 244K bytes */  
  __top_Flash = 0x0 + 0x3d000 ; /* 244K bytes */  
  __base_SRAM0_1_2 = 0x20000000  ; /* SRAM0_1_2 */  
  __base_RAM = 0x20000000 ; /* RAM */  
  __top_SRAM0_1_2 = 0x20000000 + 0x10000 ; /* 64K bytes */  
  __top_RAM = 0x20000000 + 0x10000 ; /* 64K bytes */  
  __base_SRAMX = 0x4000000  ; /* SRAMX */  
  __base_RAM2 = 0x4000000 ; /* RAM2 */  
  __top_SRAMX = 0x4000000 + 0x4000 ; /* 16K bytes */  
  __top_RAM2 = 0x4000000 + 0x4000 ; /* 16K bytes */  
  __base_SRAM3 = 0x20010000  ; /* SRAM3 */  
  __base_RAM3 = 0x20010000 ; /* RAM3 */  
  __top_SRAM3 = 0x20010000 + 0x4000 ; /* 16K bytes */  
  __top_RAM3 = 0x20010000 + 0x4000 ; /* 16K bytes */  
