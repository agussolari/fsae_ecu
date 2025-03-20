/***************************************************************************//**
  @file     main.c
  @brief    Aplicación
  @author   Agustin Solari
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "common.h"
#include "drivers.h"


#include "millis.h"
#include "can.h"
#include "uart.h"
#include "gpio.h"
#include "fsl_debug_console.h"
#include "can.h"
#include "leds.h"
#include "flash_wrp.h"


#include "fsl_common.h"



#include <stdio.h>



#include "pin_mux.h"
#include "board.h"

#include "sensors.h"
#include "systick.h"
#include "adc.h"

void update_data(void);
void update_driver_leds(void);




int main(void)
{
	/* Init board hardware. */



	BOARD_InitPins();
	BOARD_BootClockFROHF96M();
	SystemCoreClockUpdate();
	BOARD_InitDebugConsole();


	can_init(CAN_BAUDRATE);	//INIT CAN
	init_buttons();  //INIT START, DRIVE AND STOP BUTTONS
	init_sensor();	//INIT ADC FOR SENSORS
	init_leds();	//INIT LEDS
	uartInit();		//INIT UART
	init_flash();	//INIT FLASH MEMORY
	millis_init();	//INIT MILLIS
	SysTick_Init();	//INIT SYSTICK


	//Initialice periodic interrupts
	SysTick_RegisterCallback(update_data, 100);
	SysTick_RegisterCallback(update_driver_leds, 100);
	SysTick_RegisterCallback(recive_pdo_message, 100);
	SysTick_RegisterCallback(run_sensors, 10);


	//Init drivers
	driver_1.node_id = NODE_ID_1;
	driver_2.node_id = NODE_ID_2;


	init_drivers(&driver_1);	//INIT DRIVER 1
	init_drivers(&driver_2);	//INIT DRIVER 2
	boot_drivers(); //BOOT BOTH DRIVERS


	flash_read_calibration_values();	//READ CALIBRATION VALUES FROM FLASH



	PRINTF("Init complete\n");


    while (1)
    {

//    	can_msg_t msg;
//    	if(can_isNewRxMsg())
//    	{
//    		can_readRxMsg(&msg);
//    		PRINTF("%d %d ", msg.id, msg.len);
//			for (int i = 0; i < msg.len; i++) {
//				PRINTF("%d ", msg.data[i]);
//			}
//			PRINTF("\n");
//    	}
//
//    	if (gpioRead(STOP_GPIO_PORT))
//    	{
//			PRINTF("STOP\n");
//			send_nmt_command(NMT_CMD_RESET_NODE, NODE_ID_1);
//			send_nmt_command(NMT_CMD_RESET_NODE, NODE_ID_2);
//		}
       update_state_machine(&driver_1);
       update_state_machine(&driver_2);


       driver_1.time_stamp = millis();
       driver_2.time_stamp = millis();

    	//PRINTF ADC VALUES



    }
}




void update_data(void)
{
	send_motor_data_uart(&driver_1);
	send_motor_data_uart(&driver_2);
}

void update_driver_leds(void)
{
	update_leds_by_state(driver_1.nmt_state, driver_1.state);
	update_leds_by_rpm(tps_data.tps1_value);

}




