/*
 * max7219.h
 *
 *  Created on: 12 nov. 2024
 *      Author: agust
 */

#ifndef MAX7219_H_
#define MAX7219_H_

#include "fsl_spi.h"
#include "fsl_iocon.h"
#include "fsl_clock.h"
#include "spi_cfg.h"

// Definiciones para los registros del MAX7219
#define MAX7219_REG_NO_OP        0x00
#define MAX7219_REG_DIGIT0       0x01
#define MAX7219_REG_DIGIT1       0x02
#define MAX7219_REG_DIGIT2       0x03
#define MAX7219_REG_DIGIT3       0x04
#define MAX7219_REG_DIGIT4       0x05
#define MAX7219_REG_DIGIT5       0x06
#define MAX7219_REG_DIGIT6       0x07
#define MAX7219_REG_DIGIT7       0x08
#define MAX7219_REG_DECODE_MODE  0x09
#define MAX7219_REG_INTENSITY    0x0A
#define MAX7219_REG_SCAN_LIMIT   0x0B
#define MAX7219_REG_SHUTDOWN     0x0C
#define MAX7219_REG_DISPLAY_TEST 0x0F

// Funciones para controlar el MAX7219
void MAX7219_Init(SPI_Type *base, uint32_t srcClock_Hz);
void MAX7219_SendData(uint8_t reg, uint8_t data);
void MAX7219_ClearDisplay(void);
void MAX7219_SetIntensity(uint8_t intensity);

void InitPins_SPI0(void);
void Init_SPI0(void);

#endif // MAX7219_H_
