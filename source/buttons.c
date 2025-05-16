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
	start_button.pressed_flag = true;
}

void drive_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	drive_button.pressed_flag = true;
}

void stop_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	stop_button.pressed_flag = true;
}

void calibration_button_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	calibration_button.pressed_flag = true;
}


bool start_button_pressed(void)
{
	if (start_button.pressed_flag)
	{
		start_button.pressed_flag = false;
		return true;
	}
	return false;
}
bool drive_button_pressed(void)
{
	if (drive_button.pressed_flag)
	{
		drive_button.pressed_flag = false;
		return true;
	}
	return false;
}

bool stop_button_pressed(void)
{
	if (stop_button.pressed_flag)
	{
		stop_button.pressed_flag = false;
		return true;
	}
	return false;
}

bool calibration_button_pressed(void)
{
	if (calibration_button.pressed_flag)
	{
		calibration_button.pressed_flag = false;
		return true;
	}
	return false;
}


void init_buttons(void)
{

	//Inicializar el INPUTMUX para conectar los botones
	INPUTMUX_Init(INPUTMUX);
	INPUTMUX_AttachSignal(INPUTMUX, start_button.pinit_pin,
			start_button.mux_connection);
	INPUTMUX_AttachSignal(INPUTMUX, drive_button.pinit_pin,
			drive_button.mux_connection);
	INPUTMUX_AttachSignal(INPUTMUX, stop_button.pinit_pin,
			stop_button.mux_connection);
	INPUTMUX_AttachSignal(INPUTMUX, calibration_button.pinit_pin,
			calibration_button.mux_connection);

	//Inicializar el PINT para conectar los botones
	PINT_Init(PINT);

	//Configuro la interrupcion de cada boton
	PINT_PinInterruptConfig(PINT, start_button.pinit_pin,
			kPINT_PinIntEnableRiseEdge, start_button_callback);
	PINT_PinInterruptConfig(PINT, drive_button.pinit_pin,
			kPINT_PinIntEnableRiseEdge, drive_button_callback);
	PINT_PinInterruptConfig(PINT, stop_button.pinit_pin,
			kPINT_PinIntEnableRiseEdge, stop_button_callback);
	PINT_PinInterruptConfig(PINT, calibration_button.pinit_pin,
			kPINT_PinIntEnableFallEdge, calibration_button_callback);

	//Habilito la interrupcion de cada boton
	PINT_EnableCallbackByIndex(PINT, start_button.pinit_pin);
	PINT_EnableCallbackByIndex(PINT, drive_button.pinit_pin);
	PINT_EnableCallbackByIndex(PINT, stop_button.pinit_pin);
	PINT_EnableCallbackByIndex(PINT, calibration_button.pinit_pin);

}










