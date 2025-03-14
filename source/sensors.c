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
	gpioMode(START_GPIO_PORT, GPIO_INPUT_PULLDOWN);
	gpioMode(DRIVE_GPIO_PORT, GPIO_INPUT_PULLDOWN);
	gpioMode(STOP_GPIO_PORT, GPIO_INPUT_PULLDOWN);

	//Inicializar los GPIO para conectar los botones
	gpioMode(CALIBRATION_GPIO_PORT, GPIO_INPUT_PULLUP);



	PRINTF("Buttons started\n");
}


// Funcion para leer los sensores y guardar los valores de
// acelerador y freno en variables globales



void run_sensors(void) {
	// Read the TPS values
	uint16_t raw_tps1 = adcReadChannelBlocking(ADC_CHANNEL_TPS1);
	uint16_t raw_tps2 = adcReadChannelBlocking(ADC_CHANNEL_TPS2);

	tps_data.tps_time_stamp = millis();

	// Map the TPS1 value
	if (raw_tps1 <= tps_data.tps1_min_value) {
		tps_data.tps1_value = 0;
	} else if (raw_tps1 >= tps_data.tps1_max_value) {
		tps_data.tps1_value = 1000;
	} else {
		tps_data.tps1_value = (uint16_t) (((float) (raw_tps1 - tps_data.tps1_min_value)
				/ (tps_data.tps1_max_value - tps_data.tps1_min_value)) * 1000);
	}

	// Map the TPS2 value (inverted)
	if (raw_tps2 <= tps_data.tps2_min_value) {
		tps_data.tps2_value = 1000;
	} else if (raw_tps2 >= tps_data.tps2_max_value) {
		tps_data.tps2_value = 0;
	} else {
		tps_data.tps2_value = (uint16_t) (((float) (tps_data.tps2_max_value - raw_tps2)
				/ (tps_data.tps2_max_value - tps_data.tps2_min_value)) * 1000);
	}


}

bool check_implausibility_tps(void)
{
    // Calculate the absolute difference between the two TPS values
    uint16_t tps_difference =  abs(tps_data.tps1_value - tps_data.tps2_value);

    // Check if the difference exceeds the threshold
    if (tps_difference > PEDAL_TRAVEL_THRESHOLD)
    {
        // Get the current time in milliseconds
        uint32_t current_time = millis();

        // Check if this is the first time implausibility is detected
        if (tps_data.implausibility_start_time == 0)
        {
        	tps_data.implausibility_start_time = current_time;
        }
        else if ((current_time - tps_data.implausibility_start_time) > IMPLAUSIBILITY_THRESHOLD)
        {
            // Implausibility has persisted for more than 100 milliseconds
            return true;
        }
    }
    else
    {
        // Reset the implausibility start time if the difference is within the threshold
    	tps_data.implausibility_start_time = 0;
    }

    return false;
}



