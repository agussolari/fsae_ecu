/*
 * drivers.c
 *
 *  Created on: 22 ago 2024
 *      Author: asolari
 */
#include "drivers.h"
#include <stdio.h>

static bool flag = false;


void run_motors (driver_t* driver);
void send_pdo_sync_message(driver_t* driver);
void recive_pdo_message(driver_t* driver);
void send_pdo_message(driver_t* driver);

bool send_controlword(uint8_t controlword, uint16_t node_id);

bool send_sdo_mode_of_operation(int8_t mode, uint16_t node_id);

void map_rpdo(uint16_t node_id);
void map_tpdo(uint16_t node_id);

bool check_alignment_status(driver_t* driver);
void align_motors(uint16_t node_id);
void handle_errors(void);

/**
 * @brief Initialize the drivers
 * @return void
 */
void init_drivers(driver_t* driver)
{
	driver->state = STATE_RESET_NODE;
	driver->time_stamp = 0;
	driver->sensor_time_stamp = 0;
    driver->last_uart_time = 0;


	//Clean data of the PDO and TPDO
	for (int i = 0; i < 8; i++) {
		driver->tpdo1_data.b[i] = 0;
		driver->tpdo2_data.b[i] = 0;
		driver->tpdo3_data.b[i] = 0;
		driver->tpdo4_data.b[i] = 0;
		driver->pdo1_data.b[i] = 0;
		driver->pdo2_data.b[i] = 0;
	}

	millis_init();

}


/**
 * @brief Update the state machine
 * @param node_id Node ID
 * @return void
 */
void update_state_machine(driver_t* driver) {
    debounce_button(&start_button, START_GPIO_PORT);
    debounce_button(&drive_button, DRIVE_GPIO_PORT);
    debounce_button(&stop_button, STOP_GPIO_PORT);

    switch (driver->state) {
        case STATE_RESET_NODE:
            send_nmt_command(0x81, driver->node_id);
            PRINTF("Reset Node\n");

            driver->align = false;
            driver->state = STATE_POWER_ON_RESET;
            driver->mode = MODE_TORQUE;
            break;

        case STATE_POWER_ON_RESET:
            can_msg_t rx_msg;
            if (can_readRxMsg(&rx_msg) && (rx_msg.id == (0x700 + driver->node_id)) && (rx_msg.data[0] == 0x00)) {
                PRINTF("Boot-up message received.\n");
                driver->nmt_state = NMT_STATE_BOOTUP;
                driver->state = STATE_INITIALIZATION;
                break;
            }
            if (stop_button.last_button_state) {
                driver->state = STATE_RESET_NODE;
                break;

            }
            break;

        case STATE_INITIALIZATION:
            if (send_sdo_write_command(0x40, 0x1810, 0x01, 0x00000000, driver->state)) {
                PRINTF("Init SDO command sent\n");
            }
            driver->state = STATE_WAIT_START;
            PRINTF("Initialization Mode\n");

//            map_rpdo(driver->node_id);
//            map_tpdo(driver->node_id);

            break;

        case STATE_WAIT_START:
            if (start_button.last_button_state)
            {
            	PRINTF("Start Mode\n");
                driver->state = STATE_START;
                break;

//                PRINTF("Start Mode\n");
            }
//            PRINTF("TEST\n");
            break;

        case STATE_START:
            send_nmt_command(0x80, driver->node_id);
            driver->nmt_state = NMT_STATE_PRE_OPERATIONAL;
            PRINTF("Pre-Operational Mode\n");



            if (driver->align)
            {
            	 send_sdo_mode_of_operation(MODE_VELOCITY, driver->node_id);
            	 driver->mode = MODE_VELOCITY;
            	 PRINTF("Mode of operation set to Velocity\n");

				PRINTF("Operational Mode\n");
				send_nmt_command(0x01, driver->node_id);
				driver->nmt_state = NMT_STATE_OPERATIONAL;
                driver->state = STATE_WAIT_DRIVE;
                break;
            }
            else
            {
                align_motors(driver->node_id);
                driver->mode = MODE_ALIGNMENT;
                PRINTF("Aligning motors\n");
                send_nmt_command(0x01, driver->node_id);
                PRINTF("Operational Mode for align\n");
                driver->nmt_state = NMT_STATE_OPERATIONAL;

                driver->state = STATE_ALIGNING_MOTORS;
                break;

            }
            break;


        case STATE_ALIGNING_MOTORS:
            if (check_alignment_status(driver))
            {
                PRINTF("Alignment Complete\n");
                driver->align = true;

                send_nmt_command(0x80, driver->node_id);
                driver->nmt_state = NMT_STATE_PRE_OPERATIONAL;
                PRINTF("Pre-Operational Mode\n");

                send_sdo_mode_of_operation(MODE_VELOCITY, driver->node_id);
                driver->mode = MODE_VELOCITY;
                PRINTF("Mode of operation set to Velocity\n");

                PRINTF("Operational Mode\n");
                send_nmt_command(0x01, driver->node_id);
                driver->nmt_state = NMT_STATE_OPERATIONAL;

                driver->state = STATE_WAIT_DRIVE;
            }
            else if (start_button.last_button_state)
            {
                driver->state = STATE_WAIT_START;
                break;
            }
            else if (stop_button.last_button_state)
            {
                driver->state = STATE_STOPPED;
                break;

            }
            break;



        case STATE_WAIT_DRIVE:
            if (drive_button.last_button_state) {
                PRINTF("Drive Mode\n");
                send_controlword(0x06, driver->node_id);
                send_controlword(0x07, driver->node_id);
                send_controlword(0x0F, driver->node_id);
                driver->state = STATE_DRIVE;
                driver->nmt_state = NMT_STATE_DRIVE;
            }
            if (start_button.last_button_state) {
                driver->state = STATE_WAIT_START;
            }
            if (stop_button.last_button_state) {
                driver->state = STATE_STOPPED;
            }
            break;

        case STATE_DRIVE:
            run_sensors();
            run_motors(driver);
            handle_errors();

            if (stop_button.last_button_state) {
                driver->state = STATE_STOPPED;
            }

            break;

        case STATE_STOPPED:
            PRINTF("Stop Mode\n");
            send_nmt_command(0x80, driver->node_id);
            driver->nmt_state = NMT_STATE_STOPPED;
            driver->state = STATE_WAIT_START;
            break;
    }
}


void handle_errors()
{
	handle_implausibility(); // Check for implausibility of TPS sensors
}



// Function to align motors
void align_motors(uint16_t node_id)
{
    send_sdo_mode_of_operation(-4, node_id);
}

static time_t last_time = 0;


bool check_alignment_status(driver_t* driver)
{
	// Leer el estado de alineación de los motores
	// Si los motores están alineados, retornar true
	// De lo contrario, retornar false
	int32_t state;

    time_t current_time = millis();
    if (current_time - last_time >= 3000)
    {
        last_time = current_time;
        send_sdo_read_command(0x6060, 0x00, driver->node_id);

    }



	//Wait for controller response
	state = recive_sdo_read_command(0x43, 0x6060, 0x00, driver->node_id);


	if((state == 0x0A)) //Torque mode or pre op
		return true;
	else
		return false;
}




/**
 * @brief Run the motors
 * void run_motors (uint16_t node_id)
 *
 * @return void
 */
void run_motors (driver_t* driver)
{


	recive_pdo_message(driver);	//Receive PDO message

	send_pdo_message(driver);		//Send PDO message


}


/**
 * @brief Send a PDO SYNC message
 * void send_pdo_message(void)
 *
 * @return void
 */
void send_pdo_sync_message(driver_t* driver)
{
	uint32_t current_time = millis();
	if (current_time - driver->time_stamp >= 5000) {
		driver->time_stamp = current_time;

		//Send SYNC message
		can_msg_t sync_msg;
		sync_msg.id = SYNC_MESSAGE_ID;
		sync_msg.len = 1;
		sync_msg.data[0] = 0x00;

		if(can_isTxReady());
		{
			PRINTF("SYNC message sent\n");
			can_sendTxMsg(&sync_msg);
		}
	}
}


/**
 * @brief Receive a PDO message
 * void recive_pdo_message(uint8_t node_id)
 *
 * save the received PDO message in the driver_data  structure
 *
 * @return void
 */
void recive_pdo_message(driver_t* driver)
{
	can_msg_t rx_msg;
	if (can_isNewRxMsg()) {
		can_readRxMsg(&rx_msg);

		if (rx_msg.id == (TPDO1_ID + driver->node_id ))
		{
//			PRINTF("TPDO1 received\n");
			for (int i = 0; i < 8; i++)
				driver->tpdo1_data.b[i] = rx_msg.data[i];
		}
		if ( rx_msg.id == (TPDO2_ID + driver->node_id ))
		{
//			PRINTF("TPDO2 received\n");
			for (int i = 0; i < 8; i++)
				driver->tpdo2_data.b[i] = rx_msg.data[i];
		}
		if ( rx_msg.id == (TPDO3_ID + driver->node_id ))
		{
//			PRINTF("TPDO3 received\n");
			for (int i = 0; i < 8; i++)
				driver->tpdo3_data.b[i] = rx_msg.data[i];
		}
		if (rx_msg.id == (TPDO4_ID + driver->node_id)) {
			for (int i = 0; i < 8; i++)
				driver->tpdo4_data.b[i] = rx_msg.data[i];
		}
	}
}



void send_pdo_message(driver_t* driver)
{
    const int32_t throttle_threshold = 5; // Define a suitable threshold
    const int16_t torque_threshold = 5;   // Define a suitable threshold

    int32_t current_throttle = sensor_values.throttle;
    int16_t current_torque = sensor_values.torque;

    driver->pdo1_data.data.control_word = 0x0F | driver->node_id << 8;
    driver->pdo1_data.data.target_velocity = current_throttle;
    driver->pdo1_data.data.target_torque = current_torque;

    // Check if sensor values are zero
    if (current_throttle == 0 && current_torque == 0) {
        if (!driver->zero_message_sent) {
            can_msg_t pdo_msg;
            pdo_msg.id = RPDO1_ID + driver->node_id;
            pdo_msg.len = 8;

            pdo_msg.data[0] = 0x0F;
            pdo_msg.data[1] = 00;

            pdo_msg.data[2] = 0;
            pdo_msg.data[3] = 0;
            pdo_msg.data[4] = 0;
            pdo_msg.data[5] = 0;
            driver->pdo1_data.data.target_velocity = 0;

            pdo_msg.data[6] = 0;
            pdo_msg.data[7] = 0;
            driver->pdo1_data.data.target_torque = 0;

            if (can_isTxReady())
            {
                can_sendTxMsg(&pdo_msg);
                PRINTF("%d %d\n", current_throttle, current_torque);
            }

            driver->zero_message_sent = true;
        }
    } else {
        driver->zero_message_sent = false;

        if ((abs(current_throttle - driver->prev_throttle) > throttle_threshold ||
             abs(current_torque - driver->prev_torque) > torque_threshold) &&
            (millis() - driver->sensor_time_stamp >= SENSOR_READ_INTERVAL))
        {
            can_msg_t pdo_msg;
            pdo_msg.id = RPDO1_ID + driver->node_id;
            pdo_msg.len = 8;

            pdo_msg.data[0] = 0x0F;
            pdo_msg.data[1] = driver->node_id;

            pdo_msg.data[2] = (uint8_t)(current_throttle & 0x000000FF);
            pdo_msg.data[3] = (uint8_t)((current_throttle >> 8) & 0x000000FF);
            pdo_msg.data[4] = (uint8_t)((current_throttle >> 16) & 0x000000FF);
            pdo_msg.data[5] = (uint8_t)((current_throttle >> 24) & 0x000000FF);

            pdo_msg.data[6] = (uint8_t)(current_torque & 0x00FF);
            pdo_msg.data[7] = (uint8_t)((current_torque >> 8) & 0x00FF);

            if (can_isTxReady())
            {
                can_sendTxMsg(&pdo_msg);
                PRINTF("%d %d\n", current_throttle, current_torque);
                driver->sensor_time_stamp = millis(); // Update the timestamp
            }

            // Update previous values
            driver->prev_throttle = current_throttle;
            driver->prev_torque = current_torque;
        }
    }
}



/**
 * @brief Send Control Word
 * void send_controlword(uint8_t controlword)
 *
 * OD_INDEX = 0x6040
 * OD_SUB_INDEX = 0x00
 *
 * Command = 0x2B for 2 bytes of write
 *
 */

bool send_controlword(uint8_t controlword, uint16_t node_id) {
    can_msg_t control_msg;
    control_msg.id = RPDO1_ID + node_id;  // RPDO1 para el control del driver
    control_msg.len = 8;        // Longitud de datos (8 bytes)

    // Configurar el Controlword
    control_msg.data[0] = controlword;  // Byte bajo del Controlword
    control_msg.data[1] = 0x01;         // Byte alto del Controlword (0 si no se usa)
    // Otros bytes pueden ser configurados según sea necesario para parámetros adicionales
    for (int i = 2; i < 8; i++) {
        control_msg.data[i] = 0x00;  // Rellenar los bytes restantes con ceros
    }

    // Enviar el mensaje CAN
    while (!can_isTxReady());
    can_sendTxMsg(&control_msg);
    PRINTF("Enviado Controlword 0x%02X al ID %03X\n", controlword, control_msg.id);
    return true;
}




/**
 * @brief Send a Mode of operation
 * void send_sdo_mode_of_operation(uint8_t mode)
 *
 * OD_INDEX = 0x6060
 * OD_SUB_INDEX = 0x00
 *
 * Command = 0x2F for 1 byte of write
 *
 * @param mode Mode of operation
 * 	    0x0A: Torque
 * 	    0x09: Velocity
 * 	    0x08: Position
 * @return bool : 1 = Send success
 */
bool send_sdo_mode_of_operation(int8_t mode, uint16_t node_id)
{

	send_sdo_write_command(0x2F, 0x6060, 0x00, (int32_t)mode, node_id);

	//Wait for controller response
	if (recive_sdo_write_command(0x60, 0x6060, 0x00, 0x00000000, node_id)) {
		//Generate a SDO Write to save parameters
		send_sdo_write_command(0x23, 0x1010, 0x01, SAVE_PARAM, node_id);

		//Wait for controller response
		if (recive_sdo_write_command(0x60, 0x1010, 0x01, 0x00000000, node_id)) {
			//Use NMT command to Reset Node
			send_nmt_command(0x81, node_id); //Reset Node command
			return 1;

		}
	}

}

/**
 * @brief Map RPDO
 * void map_rpdo(void)
 *
 * @return void
 */
void map_rpdo(uint16_t node_id)
{
    // Disable PDO communication
    send_sdo_write_command(0x23, 0x1800, 0x01, 0x80000000, node_id);

    // Disable PDO mapping
    send_sdo_write_command(0x2B, 0x1A00, 0x00, 0x00, node_id);

    // Map RPDO
    // b[0], b[1]: Control word (0x6040, 0x00)
    // b[2], b[3], b[4], b[5]: Target velocity 0x60FF, 0x00
    // b[6], b[7]: Target torque 0x6071, 0x00


    //Map number of entry: 3 entry
    send_sdo_write_command(0x27, 0x1600, 0x00, 0x03, node_id);

    //Map Control Word: 0x6040, 0x00 (2 bytes)
    send_sdo_write_command(0x23, 0x1600, 0x01, 0x60400010, node_id);
    //Mpa Target velocity: 0x60FF, 0x00 (4 bytes)
    send_sdo_write_command(0x23, 0x1600, 0x02, 0x60FF0020, node_id);
    //Map Target torque: 0x6071, 0x00 (2 bytes)
    send_sdo_write_command(0x23, 0x1600, 0x03, 0x60710010, node_id);



    //Set transmission type to aynchronous
    send_sdo_write_command(0x2B, 0x1400, 0x02, 0xFE, node_id);

    //Set COB-ID to 0x200 + node_id
    send_sdo_write_command(0x2B, 0x1400, 0x01, RPDO1_ID + node_id, node_id);

    //Save parameters
    send_sdo_write_command(0x23, 0x1010, 0x01, SAVE_PARAM, node_id);

    //Reset node
    send_nmt_command(0x81, node_id);

}



/**
 * @brief Map TPDO
 * void map_tpdo(void)
 *
 * @return void
 */
void map_tpdo(uint16_t node_id)
{
	// Disable PDO communication
	send_sdo_write_command(0x2B, 0x1800, 0x01, 0x80000000, node_id);

	// Disable PDO mapping
	send_sdo_write_command(0x2B, 0x1A00, 0x00, 0x00, node_id);

	// Map TPDO1
	// b[0], b[1]: Status word (0x6041, 0x00)
	// b[2], b[3], b[4], b[5]: Actual velocity 0x606C, 0x00
	// b[6], b[7]: Actual torque 0x6077, 0x00

	//Map number of entry: 3 entry
	send_sdo_write_command(0x27, 0x1A00, 0x00, 0x03, node_id);

	//Map Status Word: 0x6041, 0x00 (2 bytes)
	send_sdo_write_command(0x23, 0x1A00, 0x01, 0x60410010, node_id);
	//Map Actual velocity: 0x606C, 0x00 (4 bytes)
	send_sdo_write_command(0x23, 0x1A00, 0x02, 0x606C0020, node_id);
	//Map Actual torque: 0x6077, 0x00 (2 bytes)
	send_sdo_write_command(0x23, 0x1A00, 0x03, 0x60770010, node_id);

	//Set transmission type to sync
	send_sdo_write_command(0x2B, 0x1800, 0x02, 0x01, node_id);

	//Set COB-ID to 0x180 + node_id
	send_sdo_write_command(0x2B, 0x1800, 0x01, TPDO1_ID + node_id, node_id);

	//Map TPDO2
	// b[0], b[1], b[2], b[3]: Actual velocity value 0x606C, 0x00
	// b[4]: Controller temperature 0x2026, 0x01
	// b[5]: Motor temperature 0x2027, 0x01
	// b[6], b[7]: Actual motor current 0x6078, 0x02

	//Map number of entry: 2 entry
	send_sdo_write_command(0x2F, 0x1A01, 0x00, 0x04, node_id);

	//Map Actual velocity value: 0x606C, 0x00 (4 bytes)
	send_sdo_write_command(0x23, 0x1A01, 0x01, 0x606C0020, node_id);
	//Map Controller temperature: 0x2026, 0x01 (1 byte)
	send_sdo_write_command(0x23, 0x1A01, 0x02, 0x20260108, node_id);
	//Map Motor temperature: 0x2027, 0x01 (1 byte)
	send_sdo_write_command(0x23, 0x1A01, 0x03, 0x20250108, node_id);
	//Map Actual motor current: 0x6078, 0x02 (2 bytes)
	send_sdo_write_command(0x23, 0x1A01, 0x04, 0x60780210, node_id);

	//Set transmission type to sync
	send_sdo_write_command(0x2B, 0x1801, 0x02, 0x01, node_id);

	//Set COB-ID to 0x280 + node_id
	send_sdo_write_command(0x2B, 0x1801, 0x01, TPDO2_ID + node_id, node_id);

	//Save parameters
	send_sdo_write_command(0x23, 0x1010, 0x01, SAVE_PARAM, node_id);

	//Reset node
	send_nmt_command(0x81, node_id);

}



void send_motor_data_uart(driver_t *driver) {
	uint32_t current_time = millis();
	if (current_time - driver->last_uart_time >= 500) {
		driver->last_uart_time = current_time;

		char buffer[512];

		// Formatear los datos en un mensaje legible
		int len = snprintf(buffer, sizeof(buffer),
				":%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d:", driver->node_id,
				driver->state,
				driver->nmt_state,
				driver->mode,
				driver->tpdo4_data.data.motor_temperature,
				driver->tpdo2_data.data.controller_temperature,
				driver->tpdo3_data.data.motor_current_actual_value,
				driver->pdo1_data.data.target_torque,
				driver->tpdo1_data.data.actual_torque,
				driver->pdo1_data.data.target_velocity,
				driver->tpdo4_data.data.actual_velocity,
				driver->error_code);

		// Enviar los datos por UART
		uartWriteMsg(buffer, len);
	}
}


void update_driver_leds(driver_t *driver)
{
//UPDATE ECU LEDS
	// Update the LEDs based on the NMT state
	if (driver->nmt_state == NMT_STATE_BOOTUP) {
		blink_led(PIN_LED_RED);

	} else if (driver->nmt_state == NMT_STATE_PRE_OPERATIONAL) {
		gpioWrite(PIN_LED_RED, LOW);

	} else if (driver->nmt_state == NMT_STATE_OPERATIONAL) {
		gpioWrite(PIN_LED_GREEN, LOW);

	} else if (driver->nmt_state == NMT_STATE_DRIVE) {
		blink_led(PIN_LED_GREEN);

	} else if (driver->nmt_state == NMT_STATE_STOPPED) {
		blink_led(PIN_LED_BLUE);

	} else {
		gpioWrite(PIN_LED_GREEN, LOW);
		gpioWrite(PIN_LED_RED, LOW);
		gpioWrite(PIN_LED_RED, LOW);
	}
//UPDATE ONBOARD LEDS
	if (driver->nmt_state == NMT_STATE_BOOTUP)
	{

	}


}





