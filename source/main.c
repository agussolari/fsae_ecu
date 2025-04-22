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

void send_data_uart(void);
void send_data_led(void);
void send_data_motec(void);
void recive_data(void);



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
	SysTick_RegisterCallback(send_data_led, 100);
	SysTick_RegisterCallback(send_data_uart, 10);
	SysTick_RegisterCallback(send_data_motec, 10);
	SysTick_RegisterCallback(recive_data, 10);
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

       update_state_machine(&driver_1);
       update_state_machine(&driver_2);
    	//PRINTF BOTTOMS
//    	PRINTF("Start: %d Drive: %d Stop: %d\n", gpioRead(START_GPIO_PORT), gpioRead(DRIVE_GPIO_PORT), gpioRead(STOP_GPIO_PORT));

       driver_1.time_stamp = millis();
       driver_2.time_stamp = millis();


    }
}




void send_data_uart(void)
{
	send_data_gui_uart(&driver_1);
	send_data_gui_uart(&driver_2);
	send_data_rf_uart();

}

void send_data_led(void)
{
	update_leds_by_state(driver_1.nmt_state, driver_1.state);
	update_leds_by_rpm(tps_data.tps1_value);
}

void send_data_motec(void)
{
	//b[0], b[1]: TPS Value
	//b[2], b[3]: Front Brake Value
	//b[4], b[5]: Rear Brake Value
	//b[6], b[7]: Direction Value

	uint8_t b[8];

	b[0] = (uint8_t)(tps_data.tps1_value >> 8);
	b[1] = (uint8_t)(tps_data.tps1_value);

	b[2] = (uint8_t)(front_break_data.brake_value >> 8);
	b[3] = (uint8_t)(front_break_data.brake_value);

	b[4] = (uint8_t)(rear_break_data.brake_value >> 8);
	b[5] = (uint8_t)(rear_break_data.brake_value);

	b[6] = (uint8_t)(direction_data.direction_value >> 8);
	b[7] = (uint8_t)(direction_data.direction_value);

	if (can_isTxReady())
	{
		can_msg_t msg;
		msg.id = 0x501;
		for (int i = 0; i < 8; i++)
		{
			msg.data[i] = b[i];
		}
		msg.len = 8;
		can_sendTxMsg(&msg);
	}

}

void recive_data(void)
{
	if(can_isNewRxMsg())
	{
		can_msg_t rx_msg;
	    can_readRxMsg(&rx_msg);

	    recive_bootup_message(rx_msg);
	    recive_current_message(rx_msg);
		recive_pdo_message(rx_msg);
	}
}






