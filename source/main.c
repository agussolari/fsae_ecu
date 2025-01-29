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
#include "leds.h"

#include "fsl_common.h"



#include <stdio.h>



#include "pin_mux.h"
#include "board.h"

#include "sensors.h"
#include "drivers.h"
#include "systick.h"
#include "adc.h"

void update_data(void);
void update_driver_leds(void);

driver_t driver_1;
driver_t driver_2;



int main(void)
{
	/* Init board hardware. */

	BOARD_InitPins();
	BOARD_BootClockFROHF96M();
	SystemCoreClockUpdate();
	BOARD_InitDebugConsole();


	can_init(CAN_BAUDRATE);
	init_buttons();
	init_sensor();
	uartInit();
	init_leds();

	SysTick_Init();


	//Init drivers
	driver_1.node_id = NODE_ID_1;
	driver_2.node_id = NODE_ID_2;


	init_drivers(&driver_1);
	init_drivers(&driver_2);

	//Initialice periodic interrupt for uart and led control and lora
	SysTick_RegisterCallback(update_data, 1000);
	SysTick_RegisterCallback(update_driver_leds, 1000);
//	SysTick_RegisterCallback(run_sensors, 50);

	PRINTF("Init complete\n");

    while (1)
    {
    	//PRINTF TPS
//    	run_sensors();
//    	PRINTF("TPS2: %d\n", tps_data.tps2_value);
//    	PRINTF("TPS1: %d\n", tps_data.tps1_value);

        update_state_machine(&driver_1);
        update_state_machine(&driver_2);

    }
}




void update_data(void)
{
	send_motor_data_uart(&driver_1);
	send_motor_data_uart(&driver_2);
}

void update_driver_leds(void) {
	update_leds(driver_1.nmt_state);
}








