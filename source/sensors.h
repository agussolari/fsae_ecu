/*
 * sensors.h
 *
 *  Created on: 21 ago 2024
 *      Author: asolari
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "common.h"
#include "adc.h"
#include "fsl_debug_console.h"
#include "gpio.h"
#include "millis.h"
#include "drivers.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>




#define ADC_CHANNEL_TPS1 ADC_CH0A //OK
#define ADC_CHANNEL_TPS2 ADC_CH2A //OK

#define ADC_CHANNEL_REAR_BRAKE ADC_CH3A
#define ADC_CHANNEL_FRONT_BRAKE ADC_CH0B //

#define ADC_CHANNEL_DIRECTION ADC_CH3B // Canal del ADC que se conecta a la direccion

#define SENSOR_READ_INTERVAL 100 // Intervalo de lectura de los sensores en ms

#define START_GPIO_PORT 		PORTNUM2PIN(1, 29)
#define DRIVE_GPIO_PORT 		PORTNUM2PIN(0, 14)
#define STOP_GPIO_PORT 			PORTNUM2PIN(0, 0)

#define CALIBRATION_GPIO_PORT 	PORTNUM2PIN(0, 26)

#define IMPLAUSIBILITY_THRESHOLD 100 // 100 milliseconds
#define PEDAL_TRAVEL_THRESHOLD 100 // 10% of 1000

#define CURRENT_NODE_ID 0x70F

#define ADC_0_5V_VALUE (uint16_t)9930 // 3.3V in ADC units

#define MAX_DEGREE ((float)(3600.0f))




void init_sensor(void);
void flash_read_calibration_values(void);

void init_buttons(void);
void run_sensors(void);
bool check_implausibility_tps(void);


typedef struct
{
	float ac_current_n1;
	float ac_current_n2;
	float dc_current_n1;
	float dc_current_n2;

}current_sense_data_t;

typedef struct {
	uint16_t tps1_value;
	uint16_t tps2_value;
	uint16_t tps_value;
	int16_t front_brake_value;
	int16_t rear_brake_value;
	int16_t direction_value;
} sensor_values_t;

typedef struct
{
	int16_t brake_value;
	uint16_t calibration_break_value;
}break_data_t;

typedef struct{
	float direction_value;
	uint16_t calibration_direction_value;
} direction_data_t;

typedef struct {
    uint16_t tps1_value;
    uint16_t tps2_value;

    bool implausibility_detected;

    time_t implausibility_start_time;
    uint32_t tps_time_stamp;

    uint16_t tps1_min_value;
    uint16_t tps1_max_value;
    uint16_t tps2_min_value;
    uint16_t tps2_max_value;

} tps_data_t;

#define FILTER_WINDOW_SIZE 50

typedef struct {
    uint16_t values[FILTER_WINDOW_SIZE];
    uint8_t index;
    uint32_t sum;
} filter_t;

void init_filter(filter_t *filter);
int16_t apply_filter(filter_t *filter, int16_t new_value);

extern tps_data_t tps_data;

extern break_data_t front_break_data;
extern break_data_t rear_break_data;

extern direction_data_t direction_data;

extern sensor_values_t sensor_values;

extern current_sense_data_t current_sense_data;

extern filter_t ac_n1_filter;
extern filter_t ac_n2_filter;
extern filter_t dc_n1_filter;
extern filter_t dc_n2_filter;

#endif /* SENSORS_H_ */
