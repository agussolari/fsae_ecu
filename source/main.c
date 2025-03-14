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

#define DISABLE_PRINTF



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



	SysTick_Init();


	//Init drivers
	driver_1.node_id = NODE_ID_1;
	driver_2.node_id = NODE_ID_2;


	init_drivers(&driver_1);
	init_drivers(&driver_2);

	//Initialice periodic interrupt for uart and led control and lora
	SysTick_RegisterCallback(update_data, 100);
	SysTick_RegisterCallback(update_driver_leds, 100);
	SysTick_RegisterCallback(run_sensors, 10);


	PRINTF("Init complete\n");

    while (1)
    {

       update_state_machine(&driver_1);
       update_state_machine(&driver_2);


       driver_1.time_stamp = millis();
       driver_2.time_stamp = millis();

//    	gpioWrite(PIN_LED_G1, LED_ACTIVE);
//    	gpioWrite(PIN_LED_G2, LED_ACTIVE);
//    	gpioWrite(PIN_LED_A1, LED_ACTIVE);
//    	gpioWrite(PIN_LED_A2, LED_ACTIVE);
//    	gpioWrite(PIN_LED_A3, LED_ACTIVE);
//    	gpioWrite(PIN_LED_R1, LED_ACTIVE);
//    	gpioWrite(PIN_LED_R2, LED_ACTIVE);


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










////FLASH EXAMPLE
//#include "flash_wrp.h"
//#include "fsl_debug_console.h"
//#include "pin_mux.h"
//#include "board.h"
//#include "fsl_iap.h"
//#include "fsl_iap_ffr.h"
//#include "fsl_common.h"
//#include "gpio.h"
//#include "leds.h"
//
//
//
//uint32_t s_buffer[BUFFER_LEN] = {1, 2, 3, 4};
//
//int main(void)
//{
//
////    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
////    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
////    /* enable clock for GPIO*/
////    CLOCK_EnableClock(kCLOCK_Gpio0);
////    CLOCK_EnableClock(kCLOCK_Gpio1);
//
//	BOARD_InitPins();
//	BOARD_BootClockFROHF96M();
//	SystemCoreClockUpdate();
//	BOARD_InitDebugConsole();
//
//	gpioMode(PIN_LED_A2, GPIO_OUTPUT);
//	gpioWrite(PIN_LED_A2, LED_ACTIVE);
//
//	init_flash();
//
////	program_flash(s_buffer, sizeof(s_buffer));
//
//	uint32_t data[BUFFER_LEN];
//	read_flash(data, BUFFER_LEN);
//
//}
