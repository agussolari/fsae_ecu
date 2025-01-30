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


#define FILTER_SIZE 10


uint16_t tps1_values[FILTER_SIZE] = {0};
uint16_t tps2_values[FILTER_SIZE] = {0};
uint8_t tps_index = 0;

#define ALPHA 0.1  // Smoothing factor for the EMA filter

uint16_t tps1_filtered = 0;
uint16_t tps2_filtered = 0;

uint16_t moving_average(uint16_t* values) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        sum += values[i];
    }
    return (uint16_t)(sum / FILTER_SIZE);
}

void run_sensors(void) {
    // Read the brake value and scale it to a range of 0 to 1000
    sensor_values.brake = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_BRAKE) / ADC_MAX_VALUE) * 1000);

    sensor_values.direction = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_DIRECTION) / ADC_MAX_VALUE) * 1000);

    // Read the TPS values
    uint16_t tps1_raw = (uint16_t)(((54800.0 - (double)adcReadChannelBlocking(ADC_CHANNEL_TPS1)) / (6704.0)) * 3000.0);
    uint16_t tps2_raw = (uint16_t)((((double)adcReadChannelBlocking(ADC_CHANNEL_TPS2) - 9024.0) / (1216.0)) * 3000.0);

    // Apply the EMA filter
    tps1_filtered = (uint16_t)(ALPHA * tps1_raw + (1 - ALPHA) * tps1_filtered);
    tps2_filtered = (uint16_t)(ALPHA * tps2_raw + (1 - ALPHA) * tps2_filtered);

    // Update the TPS data
    tps_data.tps1_value = tps1_filtered;
    tps_data.tps2_value = tps2_filtered;
}


/************************************
 *       TPS sensor functions		*
 ************************************/

//void read_tps_values() {
//    tps_data.tps1_value = adc_read(TPS1_ADC_CHANNEL);
//    tps_data.tps2_value = adc_read(TPS2_ADC_CHANNEL);
//}
bool check_implausibility() {
    float tps1_percentage = (float)(tps_data.tps1_value) / (3000.0f);
    float tps2_percentage = (float)(tps_data.tps2_value) / (3000.0f);
    float deviation = fabs(tps1_percentage - tps2_percentage);

    return deviation > IMPLAUSIBILITY_THRESHOLD;
}

bool handle_implausibility() {
	if (check_implausibility()) {
		if (!tps_data.implausibility_detected) {
			tps_data.implausibility_detected = true;
			tps_data.implausibility_start_time = millis();
		} else if (millis()
				- tps_data.implausibility_start_time> IMPLAUSIBILITY_TIME_MS) {
			stop_motor();
			return true; // Motor stopped due to implausibility
		}
	} else {
		tps_data.implausibility_detected = false;
	}
	return false; // No implausibility detected or motor not stopped
}


void stop_motor()
{
    // Implement motor stop logic here
    // For example, send a command to the motor controller to stop the motor

    PRINTF("Motor stopped\n");
}

