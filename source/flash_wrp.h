/*
 * flash_wrp.h
 *
 *  Created on: 8 mar. 2025
 *      Author: agust
 */

#ifndef FLASH_WRP_H_
#define FLASH_WRP_H_

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_iap.h"
#include "fsl_iap_ffr.h"
#include "fsl_common.h"

#define BUFFER_LEN 512 / 4
#define PAGE_INDEX_FROM_END 1U



void init_flash(void);
void erase_flash(void);
void program_flash(uint32_t *data, uint32_t size);
void read_flash(uint32_t *data, uint32_t size);



#endif /* FLASH_WRP_H_ */
