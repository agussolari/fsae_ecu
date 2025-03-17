/*
 * sensors.c
 *
 *  Created on: 21 ago 2024
 *      Author: asolari
 */

#include "sensors.h"

sensor_values_t sensor_values;

break_data_t front_break_data;
break_data_t rear_break_data;

direction_data_t direction_data;

tps_data_t tps_data = {0};

bool check_breaks(void);


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

	uint16_t raw_front_brake = adcReadChannelBlocking(ADC_CHANNEL_FRONT_BRAKE);
	uint16_t raw_rear_brake = adcReadChannelBlocking(ADC_CHANNEL_REAR_BRAKE);

	uint16_t raw_direction = adcReadChannelBlocking(ADC_CHANNEL_DIRECTION);

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

	front_break_data.brake_value = raw_front_brake - front_break_data.calibration_break_value;
	rear_break_data.brake_value = raw_rear_brake - rear_break_data.calibration_break_value;

	direction_data.direction_value = raw_direction - direction_data.calibration_direction_value;

	//Check if the breaks are pressed
	if(check_breaks())
	{
		//If the breaks are pressed, TPS values are set to 0
		tps_data.tps1_value = 0;
		tps_data.tps2_value = 0;
	}



	//Save values
	sensor_values.tps1_value = tps_data.tps1_value;
	sensor_values.tps2_value = tps_data.tps2_value;
	sensor_values.front_brake_value = raw_front_brake;
	sensor_values.rear_brake_value = raw_rear_brake;
	sensor_values.direction_value = raw_direction;
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

bool check_breaks(void)
{
	// Check if the front and rear brakes are both pressed
	if (front_break_data.brake_value
			> (front_break_data.calibration_break_value * 1.1)
			&& rear_break_data.brake_value
					> (rear_break_data.calibration_break_value * 1.1)) {
		// STOP TPS
		return true;
	}

	return false;
}




