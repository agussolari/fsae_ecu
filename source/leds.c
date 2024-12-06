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

	// Initialize the CTIMER
	millis_init();
}

void blink_led(pin_t pin)
{
    static uint32_t lastToggleTime = 0;
    static bool ledState = false;

    // Get the current time
    uint32_t currentTime = millis();

    // If 500ms have passed since the last toggle
    if ((currentTime - lastToggleTime) >= 1000 ) {
        // Toggle the LED
        ledState = !ledState;
        gpioWrite(pin, ledState);

        // Update the last toggle time
        lastToggleTime = currentTime;
    }
}


