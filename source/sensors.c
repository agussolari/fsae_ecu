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

current_sense_data_t current_sense_data = {0};

bool check_breaks(void);



void init_sensor(void) {
	// Inicializo los sensores
	uartWriteStr("Starting sensors...\n");

	//Inicializar el ADC para conectar el acelerador
	adcInit();



	uartWriteStr("Sensors started\n");
}

void init_buttons(void) {
	// Inicializo los botones
	uartWriteStr("Starting buttons...\n");

	//Inicializar los GPIO para conectar los botones
	gpioMode(START_GPIO_PORT, GPIO_INPUT_PULLDOWN);
	gpioMode(DRIVE_GPIO_PORT, GPIO_INPUT_PULLDOWN);
	gpioMode(STOP_GPIO_PORT, GPIO_INPUT_PULLDOWN);

	//Inicializar los GPIO para conectar los botones
	gpioMode(CALIBRATION_GPIO_PORT, GPIO_INPUT_PULLUP);



	uartWriteStr("Buttons started\n");
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




    //Map the TPS1 value
    if (raw_tps1 > tps_data.tps1_min_value)
    {
		tps_data.tps1_value = 0;
	} else if (raw_tps1 < tps_data.tps1_max_value) {
		tps_data.tps1_value = 1000;
	} else {
	    tps_data.tps1_value = (uint16_t) (((float) (tps_data.tps1_min_value - raw_tps1)
	            / (tps_data.tps1_min_value - tps_data.tps1_max_value)) * 1000);
    }

    // Map the TPS2 value
	if (raw_tps2 < tps_data.tps2_min_value) {
		tps_data.tps2_value = 0;
	} else if (raw_tps2 > tps_data.tps2_max_value) {
		tps_data.tps2_value = 1000;
	} else {
	    tps_data.tps2_value = (uint16_t) (((float) (raw_tps2 - tps_data.tps2_min_value)
	            / (tps_data.tps2_max_value - tps_data.tps2_min_value)) * 1000);
	}




	//Break values in PSI
	// P [PSI] = 400*(V - 0.5V)
	// V = raw_value/65535.0 * 3.3
	front_break_data.brake_value =  (int16_t)(400.0*(((float)(raw_front_brake + front_break_data.calibration_break_value)/65535.0)*3.3 - 0.5));
	rear_break_data.brake_value =   (int16_t)(400.0*(((float)(raw_rear_brake + rear_break_data.calibration_break_value)/65535.0)*3.3 - 0.5));





	direction_data.direction_value = (float)((float)raw_direction - (float)direction_data.calibration_direction_value)/((float)(ADC_MAX_VALUE))*MAX_DEGREE;




	//Save values
	sensor_values.tps1_value = tps_data.tps1_value;
	sensor_values.tps2_value = tps_data.tps2_value;
	sensor_values.tps_value = (tps_data.tps1_value + tps_data.tps2_value) / 2;
    sensor_values.front_brake_value = front_break_data.brake_value;
    sensor_values.rear_brake_value = rear_break_data.brake_value;
    sensor_values.direction_value = direction_data.direction_value;
}

bool check_implausibility_tps(void)
{
    // Calculate the absolute difference between the two TPS values
    uint16_t tps_difference =  abs(tps_data.tps1_value - tps_data.tps2_value);

    // Check if the difference exceeds the threshold
	if (tps_difference > PEDAL_TRAVEL_THRESHOLD)
	{
		if (tps_data.implausibility_detected == false)
		{
			// If the implausibility condition has just been detected, save the time
			tps_data.implausibility_detected = true;
			tps_data.implausibility_start_time = millis();
		}
		if (tps_data.implausibility_detected == true)
		{
			// If the implausibility condition has been detected for too long, return true
			if (millis() - tps_data.implausibility_start_time> IMPLAUSIBILITY_THRESHOLD)
			{
				return true;
			}
		}
	}
	else
	{
		tps_data.implausibility_detected = false;
	}


	return false; // If the implausibility condition is not detected or has not been detected for too long, return false
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


void flash_read_calibration_values(void)
{
	//Read flash memory and save the calibration values
	uint32_t data[7];
	read_flash(data, 7);

	tps_data.tps1_min_value = (uint16_t)data[0];
	tps_data.tps1_max_value = (uint16_t)data[1];
	tps_data.tps2_min_value = (uint16_t)data[2];
	tps_data.tps2_max_value = (uint16_t)data[3];

	tps_data.implausibility_detected = false;
	tps_data.implausibility_start_time = 0;
	tps_data.tps_time_stamp = 0;

	front_break_data.calibration_break_value = (uint16_t)data[4];
	rear_break_data.calibration_break_value = (uint16_t)data[5];

	direction_data.calibration_direction_value = (uint16_t)data[6];

//	//Initialize the calibration flag
//	if (tps_data.tps1_min_value == 0
//			|| tps_data.tps1_max_value == 0
//			|| tps_data.tps2_min_value == 0
//			|| tps_data.tps2_max_value == 0
//			|| front_break_data.calibration_break_value == 0
//			|| rear_break_data.calibration_break_value == 0
//			|| direction_data.calibration_direction_value == 0)
//	{
//		driver_1.calibration_needed = true;
//		driver_2.calibration_needed = true;
//
//	} else {
//		driver_1.calibration_needed = false;
//		driver_2.calibration_needed = false;
//	}

	driver_1.calibration_needed = false;
	driver_2.calibration_needed = false;

}



