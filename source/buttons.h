/*
 * buttons.h
 *
 *  Created on: 16 may. 2025
 *      Author: agust
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "pin_mux.h"
#include "board.h"
#include "fsl_pint.h"
#include "fsl_common.h"
#include "fsl_inputmux.h"

//Define the button objects
typedef struct {
	pint_pin_int_t pinit_pin;
	inputmux_connection_t mux_connection;
	bool pressed_flag;
} button_t;



#define PINIT_PIN_START kPINT_PinInt0
#define PINIT_PIN_DRIVE kPINT_PinInt1
#define PINIT_PIN_STOP  kPINT_PinInt2
#define PINIT_PIN_CALIBRATION kPINT_PinInt3

#define INPUTMUX_START_GPIO_PORT 			kINPUTMUX_GpioPort1Pin29ToPintsel //PORTNUM2PIN(1, 29)
#define INPUTMUX_DRIVE_GPIO_PORT 			kINPUTMUX_GpioPort0Pin14ToPintsel //PORTNUM2PIN(0, 14)
#define INPUTMUX_STOP_GPIO_PORT 			kINPUTMUX_GpioPort0Pin0ToPintsel //PORTNUM2PIN(0, 0)
#define INPUTMUX_CALIBRATION_GPIO_PORT 		kINPUTMUX_GpioPort0Pin26ToPintsel //PORTNUM2PIN(0, 26)

void init_buttons(void);
bool start_button_pressed(void);
bool drive_button_pressed(void);
bool stop_button_pressed(void);
bool calibration_button_pressed(void);




#endif /* BUTTONS_H_ */
