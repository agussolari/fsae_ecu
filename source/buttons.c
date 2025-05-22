/*
 * buttons.c
 *
 *  Created on: 15 may. 2025
 *      Author: agust
 */



#include "buttons.h"

button_t start_button = {PINIT_PIN_START, INPUTMUX_START_GPIO_PORT, false};
button_t drive_button = {PINIT_PIN_DRIVE, INPUTMUX_DRIVE_GPIO_PORT, false};
button_t stop_button = {PINIT_PIN_STOP, INPUTMUX_STOP_GPIO_PORT, false};
button_t calibration_button = {PINIT_PIN_CALIBRATION, INPUTMUX_CALIBRATION_GPIO_PORT, false};


void start_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{

	// Set the pressed flag and state to true
	start_button.pressed_flag = !start_button.pressed_flag;
}

void drive_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	drive_button.pressed_flag = !drive_button.pressed_flag;
}

void stop_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	stop_button.pressed_flag = !stop_button.pressed_flag;
}

void calibration_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	calibration_button.pressed_flag = !calibration_button.pressed_flag;
}


bool start_button_pressed(void)
{
	return start_button.pressed_flag;
}
bool drive_button_pressed(void)
{
	return drive_button.pressed_flag;
}

bool stop_button_pressed(void)
{
	return stop_button.pressed_flag;
}

bool calibration_button_pressed(void)
{
	return calibration_button.pressed_flag;
}


void init_buttons(void)
{
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);    // Inicializar el GPIO

	//Inicializar el INPUTMUX para conectar los botones
	INPUTMUX_Init(INPUTMUX);
	INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, kINPUTMUX_GpioPort1Pin29ToPintsel);
	INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt1, kINPUTMUX_GpioPort0Pin14ToPintsel);
	INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt2, kINPUTMUX_GpioPort0Pin0ToPintsel);
	INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt3, kINPUTMUX_GpioPort0Pin26ToPintsel);
    INPUTMUX_Deinit(INPUTMUX);


	//Inicializar el PINT para conectar los botones
	PINT_Init(PINT);

	//Configuro la interrupcion de cada boton
	PINT_PinInterruptConfig(PINT, kPINT_PinInt0,
			kPINT_PinIntEnableBothEdges, start_button_callback);
	PINT_PinInterruptConfig(PINT, kPINT_PinInt1,
			kPINT_PinIntEnableBothEdges, drive_button_callback);
	PINT_PinInterruptConfig(PINT, kPINT_PinInt2,
			kPINT_PinIntEnableBothEdges, stop_button_callback);
	PINT_PinInterruptConfig(PINT, kPINT_PinInt3,
			kPINT_PinIntEnableBothEdges, calibration_button_callback);

	//Habilito la interrupcion de cada boton
	PINT_EnableCallbackByIndex(PINT, kPINT_PinInt0);
	PINT_EnableCallbackByIndex(PINT, kPINT_PinInt1);
	PINT_EnableCallbackByIndex(PINT, kPINT_PinInt2);
	PINT_EnableCallbackByIndex(PINT, kPINT_PinInt3);

}










