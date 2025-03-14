/*
 * leds.h
 *
 *  Created on: 3 dic. 2024
 *      Author: agust
 */

#ifndef LEDS_H_
#define LEDS_H_

#include "common.h"
#include "gpio.h"
#include "drivers.h"
#include "fsl_debug_console.h"



//On Boards LEDS
//#define PIN_LED_RED     	PORTNUM2PIN(0,21)
//#define PIN_LED_GREEN     	PORTNUM2PIN(0,22)
//#define PIN_LED_BLUE     	PORTNUM2PIN(0,18)

//Cockpit LEDS
#define PIN_LED_START    PORTNUM2PIN(0, 2)
#define PIN_LED_DRIVE    PORTNUM2PIN(0, 4)
#define PIN_LED_STOP     PORTNUM2PIN(0, 6)

//RPM LEDS
#define PIN_LED_G1 	     PORTNUM2PIN(0, 21)
#define PIN_LED_G2       PORTNUM2PIN(0, 22)
#define PIN_LED_A1       PORTNUM2PIN(0, 20)
#define PIN_LED_A2       PORTNUM2PIN(0, 13)
#define PIN_LED_A3       PORTNUM2PIN(1, 9)
#define PIN_LED_R1       PORTNUM2PIN(0, 1)
#define PIN_LED_R2       PORTNUM2PIN(0, 7)




void init_leds(void);
void update_leds_by_state(nmt_state_t nmt_state, driver_state_t driver_state);
void update_leds_by_rpm(uint16_t tps_data);

#endif /* LEDS_H_ */
