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
#include "can.h"
#include "leds.h"
#include "flash_wrp.h"
#include "buttons.h"


#include "fsl_common.h"



#include <stdio.h>



#include "pin_mux.h"
#include "board.h"

#include "sensors.h"
#include "SysTick.h"
#include "adc.h"





//#define DISABLE_PRINTF


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
	SysTick_RegisterCallback(send_sync_message, 10);


	SysTick_RegisterCallback(recive_data, 1);
	SysTick_RegisterCallback(run_sensors, 10);



	//Init drivers
	driver_1.node_id = NODE_ID_1;
	driver_2.node_id = NODE_ID_2;


	init_drivers(&driver_1);	//INIT DRIVER 1
	init_drivers(&driver_2);	//INIT DRIVER 2


	flash_read_calibration_values();	//READ CALIBRATION VALUES FROM FLASH

	boot_drivers(); //BOOT BOTH DRIVERS



	uartWriteStr("Init complete\n");


    while (1)
    {

       update_state_machine(&driver_1);
       driver_1.time_stamp = millis();

       update_state_machine(&driver_2);
       driver_2.time_stamp = millis();

    	//PRINTF BUTTONS
//    	PRINTF("%d %d %d %d\n", start_button_pressed(), drive_button_pressed(), stop_button_pressed(), calibration_button_pressed());


    }
}







void send_data_uart(void)
{
	send_data_rf_uart();

}

void send_data_led(void)
{
	update_leds_by_state(driver_1.nmt_state, driver_1.state);
	update_leds_by_rpm(tps_data.tps1_value);
}


bool is_bootup_message(uint16_t id, uint8_t node_id) {
    return id == (BOOTUP_BASE_ID + node_id);
}

bool is_current_message(uint16_t id) {
    return id == CURRENT_NODE_ID;
}

bool is_pdo_message(uint16_t id, uint8_t node_id) {
    return (id == (TPDO1_ID + node_id)) ||
           (id == (TPDO2_ID + node_id)) ||
           (id == (TPDO3_ID + node_id)) ||
           (id == (TPDO4_ID + node_id));
}

void recive_data(void)
{
	if(can_isNewRxMsg())
	{
		can_msg_t rx_msg;
	    can_readRxMsg(&rx_msg);

        uint16_t id = rx_msg.id;

        // Verificar y procesar mensajes
        if (is_bootup_message(id, NODE_ID_1) || is_bootup_message(id, NODE_ID_2))
        {
            recive_bootup_message(rx_msg);
            return; // Salir después de procesar
        }

        if (is_current_message(id))
        {
            recive_current_message(rx_msg);
            return; // Salir después de procesar
        }

        if (is_pdo_message(id, NODE_ID_1) || is_pdo_message(id, NODE_ID_2))
        {
            recive_pdo_message(rx_msg);
            return; // Salir después de procesar
        }

	}
}






