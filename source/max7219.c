/*
 * max7219.c
 *
 *  Created on: 12 nov. 2024
 *      Author: agust
 */


#include "max7219.h"

// Declaración de funciones

void InitPins_SPI0(void);
void Init_SPI0(void);


// Inicialización del MAX7219
void MAX7219_Init(SPI_Type *base, uint32_t srcClock_Hz) {

	Init_SPI0();

    spi_master_config_t spiConfig;

    // Configuración de SPI
    SPI_MasterGetDefaultConfig(&spiConfig);
    spiConfig.baudRate_Bps = 1000000U;  // Velocidad de 1 MHz para el MAX7219
    SPI_MasterInit(base, &spiConfig, srcClock_Hz);

    // Configuración inicial del MAX7219
    MAX7219_SendData(MAX7219_REG_SCAN_LIMIT, 0x07);   // Usar los 8 dígitos
    MAX7219_SendData(MAX7219_REG_DECODE_MODE, 0x00);  // Sin decodificación BCD
    MAX7219_SendData(MAX7219_REG_SHUTDOWN, 0x01);     // Salir del modo shutdown
    MAX7219_ClearDisplay();                           // Limpiar display al inicio
    MAX7219_SetIntensity(0x0F);                       // Configuración de brillo
}

// Función para enviar datos al MAX7219
void MAX7219_SendData(uint8_t reg, uint8_t data) {
    spi_transfer_t transfer;
    uint8_t buffer[2] = {reg, data};

    transfer.txData = buffer;
    transfer.rxData = NULL;
    transfer.dataSize = sizeof(buffer);
    transfer.configFlags = kSPI_FrameAssert;

    SPI_MasterTransferBlocking(SPI0, &transfer);  // SPI0 es el SPI base
}

// Función para borrar el display
void MAX7219_ClearDisplay(void) {
    for (uint8_t i = MAX7219_REG_DIGIT0; i <= MAX7219_REG_DIGIT7; i++) {
        MAX7219_SendData(i, 0x00);  // Enviar 0 para limpiar cada dígito
    }
}

// Función para configurar el brillo del display
void MAX7219_SetIntensity(uint8_t intensity) {
    MAX7219_SendData(MAX7219_REG_INTENSITY, intensity & 0x0F);  // Rango 0-15
}




// Función para inicializar SPI0
void Init_SPI0(void) {
    spi_master_config_t spiConfig;
    uint32_t srcFreq               = 0;
    uint32_t i                     = 0;
    uint32_t err                   = 0;
    spi_transfer_t xfer            = {0};

    /* attach 12 MHz clock to SPI3 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
    /* reset FLEXCOMM for SPI */
    RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);

    SPI_MasterGetDefaultConfig(&spiConfig);
    srcFreq            = EXAMPLE_SPI_MASTER_CLK_FREQ;
    spiConfig.sselNum = (spi_ssel_t)EXAMPLE_SPI_SSEL;
    spiConfig.sselPol = (spi_spol_t)EXAMPLE_SPI_SPOL;
    SPI_MasterInit(EXAMPLE_SPI_MASTER, &spiConfig, srcFreq);


}
