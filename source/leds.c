/*
 * leds.c
 *
 *  Created on: 3 dic. 2024
 *      Author: agust
 */

#include "leds.h"

void gpioBlink_led_START(uint32_t interval);
void gpioBlink_led_DRIVE(uint32_t interval);
void gpioBlink_led_STOP(uint32_t interval);


void init_leds(void)
{
	// Initialize the Cockpit LEDs
	gpioMode(PIN_LED_START, GPIO_OUTPUT);
	gpioMode(PIN_LED_DRIVE, GPIO_OUTPUT);
	gpioMode(PIN_LED_STOP, GPIO_OUTPUT);

	// Initialize the RPM LEDs
	gpioMode(PIN_LED_G1, GPIO_OUTPUT);
	gpioMode(PIN_LED_G2, GPIO_OUTPUT);
	gpioMode(PIN_LED_A1, GPIO_OUTPUT);
	gpioMode(PIN_LED_A2, GPIO_OUTPUT);
	gpioMode(PIN_LED_A3, GPIO_OUTPUT);
	gpioMode(PIN_LED_R1, GPIO_OUTPUT);
	gpioMode(PIN_LED_R2, GPIO_OUTPUT);


	// Turn off the LEDs
	gpioWrite(PIN_LED_START, !LED_ACTIVE);
	gpioWrite(PIN_LED_DRIVE, !LED_ACTIVE);
	gpioWrite(PIN_LED_STOP, !LED_ACTIVE);
	gpioWrite(PIN_LED_G1, !LED_ACTIVE);
	gpioWrite(PIN_LED_G2, !LED_ACTIVE);
	gpioWrite(PIN_LED_A1, !LED_ACTIVE);
	gpioWrite(PIN_LED_A2, !LED_ACTIVE);
	gpioWrite(PIN_LED_A3, !LED_ACTIVE);
	gpioWrite(PIN_LED_R1, !LED_ACTIVE);
	gpioWrite(PIN_LED_R2, !LED_ACTIVE);

	PRINTF("LEDs initialized\n");
}

void update_leds_by_state(nmt_state_t nmt_state, driver_state_t driver_state)
{

	// Update the LEDs based on the NMT state
	if (nmt_state == NMT_STATE_BOOTUP)
	{
		if (driver_state == STATE_CALIBRATION_1)
		{
			gpioBlink_led_START(500);
			gpioWrite(PIN_LED_DRIVE, LED_ACTIVE);
			gpioWrite(PIN_LED_STOP, !LED_ACTIVE);
		}
		else if (driver_state == STATE_CALIBRATION_2)
		{
			gpioWrite(PIN_LED_START, !LED_ACTIVE);
			gpioBlink_led_DRIVE(500);
			gpioWrite(PIN_LED_STOP, !LED_ACTIVE);
		}
		else
		{
			gpioWrite(PIN_LED_START, LED_ACTIVE);
			gpioWrite(PIN_LED_DRIVE, !LED_ACTIVE);
			gpioWrite(PIN_LED_STOP, !LED_ACTIVE);
		}

	}
	else if (nmt_state == NMT_STATE_PRE_OPERATIONAL)
	{
		if (driver_state == STATE_CALIBRATION_1)
		{
			gpioBlink_led_START(500);
			gpioWrite(PIN_LED_DRIVE, LED_ACTIVE);
			gpioWrite(PIN_LED_STOP, !LED_ACTIVE);
		}
		else if (driver_state == STATE_CALIBRATION_2)
		{
			gpioWrite(PIN_LED_START, !LED_ACTIVE);
			gpioBlink_led_DRIVE(500);
			gpioWrite(PIN_LED_STOP, !LED_ACTIVE);
		}
		else
		{
			gpioWrite(PIN_LED_START, LED_ACTIVE);
			gpioWrite(PIN_LED_DRIVE, !LED_ACTIVE);
			gpioWrite(PIN_LED_STOP, !LED_ACTIVE);
		}


	}
	else if (nmt_state == NMT_STATE_OPERATIONAL)
	{
		gpioWrite(PIN_LED_START, !LED_ACTIVE);
		gpioWrite(PIN_LED_DRIVE, LED_ACTIVE);
		gpioWrite(PIN_LED_STOP, !LED_ACTIVE);

	}
	else if (nmt_state == NMT_STATE_DRIVE)
	{
		gpioWrite(PIN_LED_START, !LED_ACTIVE);
		gpioWrite(PIN_LED_DRIVE, !LED_ACTIVE);
		gpioWrite(PIN_LED_STOP, LED_ACTIVE);

	}
	else if (nmt_state == NMT_STATE_STOPPED)
	{
		gpioWrite(PIN_LED_START, LED_ACTIVE);
		gpioWrite(PIN_LED_DRIVE, LED_ACTIVE);
		gpioWrite(PIN_LED_STOP, LED_ACTIVE);

	}
	else if (nmt_state == NMT_STATE_ERROR)
	{
		gpioBlink_led_DRIVE(250);
		gpioBlink_led_START(250);
		gpioBlink_led_STOP(250);
	}
}

void update_leds_by_rpm(uint16_t tps_data)
{
	//TPS_data is a value from 0 to 1000
	//Update the 7 RPM LEDs based on the TPS value
if (tps_data < 50) {
	gpioWrite(PIN_LED_G1, !LED_ACTIVE);
	gpioWrite(PIN_LED_G2, !LED_ACTIVE);
	gpioWrite(PIN_LED_A1, !LED_ACTIVE);
	gpioWrite(PIN_LED_A2, !LED_ACTIVE);
	gpioWrite(PIN_LED_A3, !LED_ACTIVE);
	gpioWrite(PIN_LED_R1, !LED_ACTIVE);
	gpioWrite(PIN_LED_R2, !LED_ACTIVE);
} else if (tps_data < 143) {
	gpioWrite(PIN_LED_G1, LED_ACTIVE);
	gpioWrite(PIN_LED_G2, !LED_ACTIVE);
	gpioWrite(PIN_LED_A1, !LED_ACTIVE);
	gpioWrite(PIN_LED_A2, !LED_ACTIVE);
	gpioWrite(PIN_LED_A3, !LED_ACTIVE);
	gpioWrite(PIN_LED_R1, !LED_ACTIVE);
	gpioWrite(PIN_LED_R2, !LED_ACTIVE);
} else if (tps_data < 286) {
	gpioWrite(PIN_LED_G1, LED_ACTIVE);
	gpioWrite(PIN_LED_G2, LED_ACTIVE);
	gpioWrite(PIN_LED_A1, !LED_ACTIVE);
	gpioWrite(PIN_LED_A2, !LED_ACTIVE);
	gpioWrite(PIN_LED_A3, !LED_ACTIVE);
	gpioWrite(PIN_LED_R1, !LED_ACTIVE);
	gpioWrite(PIN_LED_R2, !LED_ACTIVE);
} else if (tps_data < 429) {
	gpioWrite(PIN_LED_G1, LED_ACTIVE);
	gpioWrite(PIN_LED_G2, LED_ACTIVE);
	gpioWrite(PIN_LED_A1, LED_ACTIVE);
	gpioWrite(PIN_LED_A2, !LED_ACTIVE);
	gpioWrite(PIN_LED_A3, !LED_ACTIVE);
	gpioWrite(PIN_LED_R1, !LED_ACTIVE);
	gpioWrite(PIN_LED_R2, !LED_ACTIVE);
} else if (tps_data < 572) {
	gpioWrite(PIN_LED_G1, LED_ACTIVE);
	gpioWrite(PIN_LED_G2, LED_ACTIVE);
	gpioWrite(PIN_LED_A1, LED_ACTIVE);
	gpioWrite(PIN_LED_A2, LED_ACTIVE);
	gpioWrite(PIN_LED_A3, !LED_ACTIVE);
	gpioWrite(PIN_LED_R1, !LED_ACTIVE);
	gpioWrite(PIN_LED_R2, !LED_ACTIVE);
} else if (tps_data < 715) {
	gpioWrite(PIN_LED_G1, LED_ACTIVE);
	gpioWrite(PIN_LED_G2, LED_ACTIVE);
	gpioWrite(PIN_LED_A1, LED_ACTIVE);
	gpioWrite(PIN_LED_A2, LED_ACTIVE);
	gpioWrite(PIN_LED_A3, LED_ACTIVE);
	gpioWrite(PIN_LED_R1, !LED_ACTIVE);
	gpioWrite(PIN_LED_R2, !LED_ACTIVE);
} else if (tps_data < 858) {
	gpioWrite(PIN_LED_G1, LED_ACTIVE);
	gpioWrite(PIN_LED_G2, LED_ACTIVE);
	gpioWrite(PIN_LED_A1, LED_ACTIVE);
	gpioWrite(PIN_LED_A2, LED_ACTIVE);
	gpioWrite(PIN_LED_A3, LED_ACTIVE);
	gpioWrite(PIN_LED_R1, LED_ACTIVE);
	gpioWrite(PIN_LED_R2, !LED_ACTIVE);
} else {
	gpioWrite(PIN_LED_G1, LED_ACTIVE);
	gpioWrite(PIN_LED_G2, LED_ACTIVE);
	gpioWrite(PIN_LED_A1, LED_ACTIVE);
	gpioWrite(PIN_LED_A2, LED_ACTIVE);
	gpioWrite(PIN_LED_A3, LED_ACTIVE);
	gpioWrite(PIN_LED_R1, LED_ACTIVE);
	gpioWrite(PIN_LED_R2, LED_ACTIVE);
	}
}

void gpioBlink_led_START(uint32_t interval)
{
	static uint32_t last_time = 0;
	static bool led_state = false;

	uint32_t current_time = millis();

	if (current_time - last_time >= interval) {
		last_time = current_time;
		led_state = !led_state;
		gpioWrite(PIN_LED_START, led_state);
	}
}

void gpioBlink_led_DRIVE(uint32_t interval) {
	static uint32_t last_time = 0;
	static bool led_state = false;

	uint32_t current_time = millis();

	if (current_time - last_time >= interval) {
		last_time = current_time;
		led_state = !led_state;
		gpioWrite(PIN_LED_DRIVE, led_state);
	}
}

void gpioBlink_led_STOP(uint32_t interval) {
	static uint32_t last_time = 0;
	static bool led_state = false;

	uint32_t current_time = millis();

	if (current_time - last_time >= interval) {
		last_time = current_time;
		led_state = !led_state;
		gpioWrite(PIN_LED_STOP, led_state);
	}
}







