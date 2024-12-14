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
	sensor_values.brake = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_BRAKE)/65535)*1000);

	sensor_values.direction = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_DIRECTION)/65535)*1000);

    tps_data.tps1_value = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_TPS1)/65535)*1000);
    tps_data.tps2_value = (uint16_t)(((float)adcReadChannelBlocking(ADC_CHANNEL_TPS2)/65535)*1000);

}

/************************************
 *       TPS sensor functions		*
 ************************************/

//void read_tps_values() {
//    tps_data.tps1_value = adc_read(TPS1_ADC_CHANNEL);
//    tps_data.tps2_value = adc_read(TPS2_ADC_CHANNEL);
//}
bool check_implausibility()
{
    float tps1_percentage = (float)tps_data.tps1_value / ADC_MAX_VALUE;
    float tps2_percentage = (float)tps_data.tps2_value / ADC_MAX_VALUE;
    float deviation = fabs(tps1_percentage - tps2_percentage);

    return deviation > IMPLAUSIBILITY_THRESHOLD;
}
void handle_implausibility() {
    if (check_implausibility()) {
        if (!tps_data.implausibility_detected) {
            tps_data.implausibility_detected = true;
            tps_data.implausibility_start_time = millis();
        } else if (millis() - tps_data.implausibility_start_time > IMPLAUSIBILITY_TIME_MS) {
            stop_motor();
        }
    } else {
        tps_data.implausibility_detected = false;
    }
}

void stop_motor()
{
    // Implement motor stop logic here
    // For example, send a command to the motor controller to stop the motor

    PRINTF("Motor stopped\n");
}
