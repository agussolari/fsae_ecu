/*
 * sensors.c
 *
 *  Created on: 21 ago 2024
 *      Author: asolari
 */

#include "sensors.h"

sensor_values_t sensor_values;
tps_data_t tps_data = {0};

void stop_motor();


void init_sensor(void) {
	// Inicializo los sensores
	PRINTF("Starting sensors...\n");

	//Inicializar el ADC para conectar el acelerador
	adcInit();


	PRINTF("Sensors started\n");
}

void init_buttons(void) {
	// Inicializo los botones
	PRINTF("Starting buttons...\n");

	//Inicializar los GPIO para conectar los botones
	gpioMode(START_GPIO_PORT, GPIO_INPUT);
	gpioMode(DRIVE_GPIO_PORT, GPIO_INPUT);
	gpioMode(STOP_GPIO_PORT, GPIO_INPUT);



	PRINTF("Buttons started\n");
}


// Funcion para leer los sensores y guardar los valores de
// acelerador y freno en variables globales

void run_sensors(void)
{
	// Leer el valor del freno y escalarlo a un rango de 0 a 1000
	sensor_values.brake = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_BRAKE)/ADC_MAX_VALUE)*1000);

	sensor_values.direction = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_DIRECTION)/ADC_MAX_VALUE)*1000);

    tps_data.tps1_value = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_TPS1)/ADC_MAX_VALUE)*3000);
    tps_data.tps2_value = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_TPS2)/ADC_MAX_VALUE)*3000);

}


