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

void test(driver_t* driver)
{
    const int32_t throttle_threshold = 5; // Define un umbral adecuado
    int32_t current_throttle = (int32_t)tps_data.tps1_value;



    int32_t error_throttle = current_throttle - driver->prev_throttle;

    // Determina el estado del movimiento
    if (error_throttle > throttle_threshold) {
        driver->moving_state = STATE_ACCELERATING;
    } else if (error_throttle < -throttle_threshold) {
        driver->moving_state = STATE_DECELERATING;
    } else {
        driver->moving_state = STATE_IDLE;
    }

    // Control de LEDs según el estado
    switch (driver->moving_state)
    {
    case STATE_ACCELERATING:
    	PRINTF("ACCELERATING\n");
        gpioWrite(PIN_LED_RED, LOW);
        gpioWrite(PIN_LED_GREEN, HIGH);
        gpioWrite(PIN_LED_BLUE, HIGH);
        break;

    case STATE_DECELERATING:
    	PRINTF("DECELERATING\n");
        gpioWrite(PIN_LED_RED, HIGH);
        gpioWrite(PIN_LED_GREEN, LOW);
        gpioWrite(PIN_LED_BLUE, HIGH);
        break;

    case STATE_IDLE:
    	PRINTF("IDLE\n");
        gpioWrite(PIN_LED_RED, HIGH);
        gpioWrite(PIN_LED_GREEN, HIGH);
        gpioWrite(PIN_LED_BLUE, LOW);
        break;
    }

    // Actualiza el valor previo del throttle
    driver->prev_throttle = current_throttle;
}




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
//	uartInit();
	init_leds();

//	SysTick_Init();


	//Init drivers
	driver_1.node_id = NODE_ID_1;
	driver_2.node_id = NODE_ID_2;


	init_drivers(&driver_1);
	init_drivers(&driver_2);


//	SysTick_RegisterCallback(run_sensors, 50);
	PRINTF("Init complete\n");

    while (1)
    {

        update_state_machine(&driver_1);
        update_state_machine(&driver_2);


    }
}










