#ifndef SPI_H_
#define SPI_H_

#include "fsl_spi.h"

#define BUFFER_SIZE 64
#define MAX_SPI_INSTANCES 6

#define SPI_MASTER_CLK_SRC  kCLOCK_Flexcomm3
#define SPI_MASTER_CLK_FREQ CLOCK_GetFlexCommClkFreq(3U)
#define SPI_SSEL            0
#define SPI_SPOL            kSPI_SpolActiveAllLow

#define SPI_LORA 	1
#define SPI_LED 	3

typedef struct {
	SPI_Type *base;
	spi_master_handle_t handle;
	spi_transfer_t xfer;
	uint8_t srcBuff[BUFFER_SIZE];
	uint8_t destBuff[BUFFER_SIZE];
	volatile bool masterFinished;
} spi_instance_t;

void Init_SPI(uint8_t instance, SPI_Type *base, uint32_t srcFreq,
		spi_ssel_t sselNum, spi_spol_t sselPol);
void Send_Data_SPI(uint8_t instance, uint8_t *data, uint32_t len);
bool isMasterFinished(uint8_t instance);

#endif /* SPI_H_ */



///*
// * spi.h
// *
// *  Created on: 5 dic. 2024
// *      Author: agust
// */
//
//#ifndef SPI_H_
//#define SPI_H_
//
//#include "fsl_spi.h"
//#include <stdio.h>
//
//#define SPI_MASTER          SPI3
//#define SPI_MASTER_IRQ      FLEXCOMM3_IRQn
//#define SPI_MASTER_CLK_SRC  kCLOCK_Flexcomm3
//#define SPI_MASTER_CLK_FREQ CLOCK_GetFlexCommClkFreq(3U)
//#define SPI_SSEL            0
//#define SPI_SPOL            kSPI_SpolActiveAllLow
//
//#define BUFFER_SIZE (64)
//
//
//
//void Init_SPI(void);
//void Send_Data_SPI(uint8_t *data);
//bool isMasterFinished(void);
//
//#endif /* SPI_H_ */
