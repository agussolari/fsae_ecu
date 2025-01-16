/*
 * leds.h
 *
 *  Created on: 3 dic. 2024
 *      Author: agust
 */

#ifndef LEDS_H_
#define LEDS_H_

#include "gpio.h"

typedef enum {
	BOOTUP,
	PRE_OPERATIONAL,
	OPERATIONAL,
	DRIVE,
	STOPPED,
} state_t;



void init_leds(void);
void update_leds(state_t state);

#endif /* LEDS_H_ */
