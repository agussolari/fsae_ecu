/*
 * max7219.c
 *
 *  Created on: 12 nov. 2024
 *      Author: agust
 */


#include "max7219.h"

// Declaración de funciones




// Inicialización del MAX7219
void MAX7219_Init(void)
{
    // Configuración inicial del MAX7219
	uint8_t data[2] = {0x00, 0x00};

	// Configuración de los registros
	//Usar los 8 dígitos
	data[0] = MAX7219_REG_SCAN_LIMIT;
	data[1] = 0x07;
	Send_Data_SPI(data);

	// Sin decodificación BCD
	data[0] = MAX7219_REG_DECODE_MODE;
	data[1] = 0x00;
	Send_Data_SPI(data);

	// Salir del modo shutdown
	data[0] = MAX7219_REG_SHUTDOWN;
	data[1] = 0x01;
	Send_Data_SPI(data);

	// Limpiar display al inicio
	MAX7219_ClearDisplay();

	// Configuración de brillo
	MAX7219_SetIntensity(0x0F);
}


// Función para borrar el display
void MAX7219_ClearDisplay(void)
{
	    // Limpiar cada dígito del display
	uint8_t data[2] = {0x00, 0x00};
    for (uint8_t i = MAX7219_REG_DIGIT0; i <= MAX7219_REG_DIGIT7; i++)
    {
    	data[0] = i;
    	Send_Data_SPI(data); // Enviar datos al MAX7219
    }
}

// Función para configurar el brillo del display
void MAX7219_SetIntensity(uint8_t intensity) {
	// Enviar datos al registro de intens
	uint8_t data[2] = {MAX7219_REG_INTENSITY, intensity & 0x0F};
	Send_Data_SPI(data); //Rango de intensidad de 0x00 a 0x0F
}

// Función para enviar datos al MAX7219
void MAX7219_SendData(uint8_t reg, uint8_t data)
{
	uint8_t frame[2] = {reg, data};
	Send_Data_SPI(frame); // Enviar datos al MAX7219
}







