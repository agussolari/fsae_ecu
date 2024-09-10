/***************************************************************************//**
  @file     main.c
  @brief    Aplicación
  @author   Agustin Solari
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "common.h"

#include "millis.h"
#include "can.h"
#include "uart.h"
#include "gpio.h"
#include "fsl_debug_console.h"
#include "can.h"
#include "gpio.h"


#include <stdio.h>



#include "pin_mux.h"
#include "board.h"

#include "sensors.h"
#include "drivers.h"




int main(void)
{

	BOARD_InitPins();
	BOARD_BootClockFROHF96M();
	SystemCoreClockUpdate();
	BOARD_InitDebugConsole();

	can_init(CAN_BAUDRATE);
	init_leds();
	init_buttons();
	init_sensor();
	init_drivers();

    while (1) {
        update_state_machine(0x001);

    }
}








