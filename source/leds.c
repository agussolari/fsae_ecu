/*
 * leds.c
 *
 *  Created on: 3 dic. 2024
 *      Author: agust
 */

#include "leds.h"

void init_leds(void)
{
	// Initialize the GPIO
	gpioMode(PIN_LED_RED, GPIO_OUTPUT);
	gpioMode(PIN_LED_GREEN, GPIO_OUTPUT);
	gpioMode(PIN_LED_BLUE, GPIO_OUTPUT);

	// Turn off the LEDs
	gpioWrite(PIN_LED_RED, HIGH);
	gpioWrite(PIN_LED_GREEN, HIGH);
	gpioWrite(PIN_LED_BLUE, HIGH);

}

void update_leds(state_t state)
{
	// Turn off the LEDs
	gpioWrite(PIN_LED_RED, HIGH);
	gpioWrite(PIN_LED_GREEN, HIGH);
	gpioWrite(PIN_LED_BLUE, HIGH);

	// Update the LEDs based on the NMT state
	if (state == BOOTUP) {
		gpioWrite(PIN_LED_RED, LOW);

	} else if (state == PRE_OPERATIONAL) {
		gpioWrite(PIN_LED_RED, LOW);
		gpioWrite(PIN_LED_GREEN, LOW);

	} else if (state == OPERATIONAL) {
		gpioWrite(PIN_LED_GREEN, LOW);

	} else if (state == DRIVE) {
		gpioWrite(PIN_LED_BLUE, LOW);

	} else if (state == STOPPED) {
		gpioWrite(PIN_LED_BLUE, LOW);
		gpioWrite(PIN_LED_RED, LOW);

	}
}




