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
#include "leds.h"
#include "max7219.h"



driver_t driver_1;
driver_t driver_2;

int main(void)
{

	BOARD_InitPins();
	BOARD_BootClockFROHF96M();
	SystemCoreClockUpdate();
	BOARD_InitDebugConsole();

	can_init(CAN_BAUDRATE);
	init_buttons();
	init_sensor();
	uartInit();
	Init_SPI();
	init_leds(); //ECU LEDS
	LEDS_Init(); //ON BOARD LEDS

//
//	//Init drivers
	driver_1.node_id = NODE_ID_1;
	driver_2.node_id = NODE_ID_2;
//
//
	init_drivers(&driver_1);
	init_drivers(&driver_2);





    while (1) {


        update_state_machine(&driver_1);
        update_state_machine(&driver_2);



        send_motor_data_uart(&driver_1);
        send_motor_data_uart(&driver_2);

        update_driver_leds(&driver_1);



//    	update_driver_leds(&driver_1);

    	//PRINF SENSOR VALUES
//    	run_sensors();
//    	PRINTF("%d %d %d %d \n", sensor_values.throttle, sensor_values.brake, sensor_values.torque, sensor_values.direction);

//        debounce_button(&start_button, START_GPIO_PORT);
//        debounce_button(&drive_button, DRIVE_GPIO_PORT);
//        debounce_button(&stop_button, STOP_GPIO_PORT);
    	//PRINTF BUTTONS VALUES
//        PRINTF("%d %d %d \n", start_button.last_button_state, drive_button.button_state, stop_button.button_state);

//    	PRINTF LEDS VALUES
//    	PRINTF("%d %d %d %d %d \n", gpioRead(LED_1_PORT), gpioRead(LED_2_PORT), gpioRead(LED_3_PORT), gpioRead(LED_4_PORT), gpioRead(LED_5_PORT));
    }
}








