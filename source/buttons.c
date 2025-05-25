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

	if (start_button.pressed_flag == false) {
		start_button.pressed_flag = true;
	} else {
		start_button.pressed_flag = false;
	}
}

void drive_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	if (drive_button.pressed_flag == false) {
		drive_button.pressed_flag = true;
	} else {
		drive_button.pressed_flag = false;
	}
}

void stop_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	if (stop_button.pressed_flag == false) {
		stop_button.pressed_flag = true;
	} else {
		stop_button.pressed_flag = false;
	}
}

void calibration_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	if (calibration_button.pressed_flag == false) {
		calibration_button.pressed_flag = true;
	} else {
		calibration_button.pressed_flag = false;
	}
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
	INPUTMUX_AttachSignal(INPUTMUX, start_button.pinit_pin, start_button.mux_connection);
	INPUTMUX_AttachSignal(INPUTMUX, drive_button.pinit_pin, drive_button.mux_connection);
	INPUTMUX_AttachSignal(INPUTMUX, stop_button.pinit_pin, stop_button.mux_connection);
	INPUTMUX_AttachSignal(INPUTMUX, calibration_button.pinit_pin, calibration_button.mux_connection);
    INPUTMUX_Deinit(INPUTMUX);


	//Inicializar el PINT para conectar los botones
	PINT_Init(PINT);

	//Configuro la interrupcion de cada boton
	PINT_PinInterruptConfig(PINT, start_button.pinit_pin, kPINT_PinIntEnableBothEdges, start_button_callback);
	PINT_PinInterruptConfig(PINT, drive_button.pinit_pin, kPINT_PinIntEnableBothEdges, drive_button_callback);
	PINT_PinInterruptConfig(PINT, stop_button.pinit_pin, kPINT_PinIntEnableBothEdges, stop_button_callback);
	PINT_PinInterruptConfig(PINT, calibration_button.pinit_pin, kPINT_PinIntEnableBothEdges, calibration_button_callback);

	//Habilito la interrupcion de cada boton
	PINT_EnableCallbackByIndex(PINT, start_button.pinit_pin);
	PINT_EnableCallbackByIndex(PINT, drive_button.pinit_pin);
	PINT_EnableCallbackByIndex(PINT, stop_button.pinit_pin);
	PINT_EnableCallbackByIndex(PINT, calibration_button.pinit_pin);

}










