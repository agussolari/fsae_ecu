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

    while (true) {
        update_nmt_state_machine(NODE_ID_1);
        //PRINTF GPIO PINS FOR TEST
//        PRINTF("%d %d %d %d\n", gpioRead(PRE_OP_GPIO_PORT), gpioRead(OP_GPIO_PORT), gpioRead(DRIVE_GPIO_PORT), gpioRead(STOP_GPIO_PORT));
        //PRINTF for ADC values
//        run_sensors();
    }
}








