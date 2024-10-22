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


driver_t driver_1;
driver_t driver_2;

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

	//Init drivers
	driver_1.node_id = NODE_ID_1;
	driver_2.node_id = NODE_ID_2;


	init_drivers(&driver_1);
	init_drivers(&driver_2);

    while (1) {
        update_state_machine(&driver_1);
        update_state_machine(&driver_2);

    	//PRINF SENSOR VALUES
//    	run_sensors();
//    	PRINTF("%d %d %d %d \n", sensor_values.throttle, sensor_values.brake, sensor_values.torque, sensor_values.direction);

    	//PRINTF BUTTONS VALUES
//    	PRINTF("%d %d %d %d \n", gpioRead(PRE_OP_GPIO_PORT), gpioRead(OP_GPIO_PORT), gpioRead(DRIVE_GPIO_PORT), gpioRead(STOP_GPIO_PORT));

    }
}








