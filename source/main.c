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
#include "display.h"
#include "spi.h"
#include "systick.h"

void update_gui(void);
void update_led(void);

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
	SysTick_Init();

//
//	//Init drivers
	driver_1.node_id = NODE_ID_1;
	driver_2.node_id = NODE_ID_2;
//
//
	init_drivers(&driver_1);
	init_drivers(&driver_2);

	//Initialice periodic interrupt for uart and led control
	SysTick_RegisterCallback(&update_gui, 500);
	SysTick_RegisterCallback(&update_led, 500);







    while (1) {


        update_state_machine(&driver_1);
        update_state_machine(&driver_2);







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



void update_led(void)
{
    update_driver_leds(&driver_1);
    update_driver_leds(&driver_2);
}

void update_gui(void)
{
    send_motor_data_uart(&driver_1);
    send_motor_data_uart(&driver_2);
}









