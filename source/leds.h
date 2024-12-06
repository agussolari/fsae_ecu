/*
 * leds.h
 *
 *  Created on: 3 dic. 2024
 *      Author: agust
 */

#ifndef LEDS_H_
#define LEDS_H_

#include "gpio.h"
#include "millis.h"



void init_leds(void);
void blink_led(pin_t pin);

#endif /* LEDS_H_ */
