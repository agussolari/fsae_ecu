/*
 * sensors.h
 *
 *  Created on: 21 ago 2024
 *      Author: asolari
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "adc.h"
#include "fsl_debug_console.h"
#include "gpio.h"
#include "drivers.h"


#define ADC_CHANNEL_THROTTLE ADC_CH0A // Canal del ADC que se conecta al acelerador
#define ADC_CHANNEL_BRAKE ADC_CH0B // Canal del ADC que se conecta al freno
#define ADC_CHANNEL_TORQUE ADC_CH3A // Canal del ADC que se conecta al torque
#define ADC_CHANNEL_DIRECTION ADC_CH3B // Canal del ADC que se conecta a la direccion




void init_sensor(void);
void init_buttons(void);
void run_sensors(void);

typedef struct {
	uint16_t throttle;
	uint16_t brake;
	uint16_t torque;
	uint16_t direction;
} sensor_values_t;


extern sensor_values_t sensor_values;

#endif /* SENSORS_H_ */
