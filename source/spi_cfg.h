/*
 * spicfg.h
 *
 *  Created on: 15 nov. 2024
 *      Author: agust
 */

#ifndef SPI_CFG_H_
#define SPI_CFG_H_

#include "fsl_spi.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_debug_console.h"

#include <stdbool.h>


#define EXAMPLE_SPI_MASTER          SPI3
#define EXAMPLE_SPI_MASTER_IRQ      FLEXCOMM3_IRQn
#define EXAMPLE_SPI_MASTER_CLK_SRC  kCLOCK_Flexcomm3
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFlexCommClkFreq(3U)
#define EXAMPLE_SPI_SSEL            0
#define EXAMPLE_SPI_SPOL            kSPI_SpolActiveAllLow

#define BUFFER_SIZE (8)
static uint8_t srcBuff[BUFFER_SIZE];
static uint8_t destBuff[BUFFER_SIZE];


#endif /* SPI_CFG_H_ */
