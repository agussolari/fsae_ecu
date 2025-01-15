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

#include "fsl_common.h"



#include <stdio.h>



#include "pin_mux.h"
#include "board.h"

#include "sensors.h"
#include "drivers.h"
#include "leds.h"
#include "display.h"
#include "spi.h"
#include "systick.h"
#include "lora.h"

void update_data(void);

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

	SysTick_Init();
	Init_SPI(SPI_LORA, SPI1, SPI_MASTER_CLK_FREQ, SPI_SSEL, SPI_SPOL);
	Init_SPI(SPI_LED, SPI3, SPI_MASTER_CLK_FREQ, SPI_SSEL, SPI_SPOL);
//
	init_leds(); //ECU LEDS
	LEDS_Init(); //ON BOARD LEDS
	LoRa_Init();

//
//
//	//Init drivers
	driver_1.node_id = NODE_ID_1;
	driver_2.node_id = NODE_ID_2;
//
//
	init_drivers(&driver_1);
	init_drivers(&driver_2);

//	//Initialice periodic interrupt for uart and led control and lora
	SysTick_RegisterCallback(&update_data, 1000);
	PRINTF("Init complete\n");
//

    while (1)
    {
//        update_state_machine(&driver_1);
//        update_state_machine(&driver_2);

    }
}




void update_data(void)
{
//	update_driver_leds(&driver_1);
//
//	send_motor_data_uart(&driver_1);
//	send_motor_data_uart(&driver_2);
//
//	send_motor_data_lora(&driver_1);
//	send_motor_data_lora(&driver_2);
	uint8_t data[2] = {0x01, 0x02};
	Send_Data_SPI(SPI_LORA, data, 2);
	Send_Data_SPI(SPI_LED, data, 2);
}







