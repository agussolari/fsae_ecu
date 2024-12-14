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
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>



#define ADC_CHANNEL_BRAKE ADC_CH3A // Canal del ADC que se conecta al freno
#define ADC_CHANNEL_TPS1 ADC_CH0A
#define ADC_CHANNEL_TPS2 ADC_CH0B
#define ADC_CHANNEL_TORQUE ADC_CH3A // Canal del ADC que se conecta al torque
#define ADC_CHANNEL_DIRECTION ADC_CH3B // Canal del ADC que se conecta a la direccion

#define IMPLAUSIBILITY_THRESHOLD 0.10 // 10% threshold
#define IMPLAUSIBILITY_TIME_MS 100

#define SENSOR_READ_INTERVAL 100 // Intervalo de lectura de los sensores en ms



void init_sensor(void);
void init_buttons(void);
void run_sensors(void);
void read_tps_values(void);
void handle_implausibility(void);

typedef struct {
	uint32_t throttle;
	uint16_t brake;
	uint16_t torque;
	uint16_t direction;
} sensor_values_t;

typedef struct {
    uint16_t tps1_value;
    uint16_t tps2_value;
    bool implausibility_detected;
    time_t implausibility_start_time;
} tps_data_t;

extern tps_data_t tps_data;
extern sensor_values_t sensor_values;

#endif /* SENSORS_H_ */
