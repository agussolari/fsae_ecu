/*
 * sensors.c
 *
 *  Created on: 21 ago 2024
 *      Author: asolari
 */

#include "sensors.h"

sensor_values_t sensor_values;


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
	gpioMode(PRE_OP_GPIO_PORT, GPIO_INPUT);
	gpioMode(OP_GPIO_PORT, GPIO_INPUT);
	gpioMode(DRIVE_GPIO_PORT, GPIO_INPUT);
	gpioMode(STOP_GPIO_PORT, GPIO_INPUT);

	PRINTF("Buttons started\n");
}


// Funcion para leer los sensores y guardar los valores de
// acelerador y freno en variables globales

void run_sensors(void) {

	// Leer el valor del acelerador y escalarlo a un rango de 0 a 1000
	sensor_values.throttle = (uint32_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_THROTTLE)/65535)*1000);

	// Leer el valor del freno y escalarlo a un rango de 0 a 1000
	sensor_values.brake = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_BRAKE)/65535)*1000);

	// Leer el valor de la direccion y escalarlo a un rango de 0 a 1000
	sensor_values.torque = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_TORQUE)/65535)*1000);


	sensor_values.direction = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_DIRECTION)/65535)*1000);

    PRINTF("%d %d %d %d\n", sensor_values.throttle, sensor_values.brake, sensor_values.torque, sensor_values.direction);
}
