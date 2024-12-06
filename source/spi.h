/*
 * spi.h
 *
 *  Created on: 5 dic. 2024
 *      Author: agust
 */

#ifndef SPI_H_
#define SPI_H_

#include "fsl_spi.h"
#include <stdio.h>

#define SPI_MASTER          SPI3
#define SPI_MASTER_IRQ      FLEXCOMM3_IRQn
#define SPI_MASTER_CLK_SRC  kCLOCK_Flexcomm3
#define SPI_MASTER_CLK_FREQ CLOCK_GetFlexCommClkFreq(3U)
#define SPI_SSEL            0
#define SPI_SPOL            kSPI_SpolActiveAllLow

#define BUFFER_SIZE (64)



void Init_SPI(void);
void Send_Data_SPI(uint8_t *data);
bool isMasterFinished(void);

#endif /* SPI_H_ */
