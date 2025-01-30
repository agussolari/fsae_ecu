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

void recive_pdo_message(driver_t* driver);
void send_pdo_message(driver_t* driver);

bool send_controlword(uint8_t controlword, uint16_t node_id);

bool send_sdo_mode_of_operation(int8_t mode, uint16_t node_id);

bool check_alignment_status(driver_t* driver);
void align_motors(uint16_t node_id);
void handle_errors(driver_t* driver);

/**
 * @brief Initialize the drivers
 * @return void
 */
void init_drivers(driver_t* driver)
{
	driver->state = STATE_RESET_NODE;
	driver->time_stamp = 0;
	driver->sensor_time_stamp = 0;


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
void update_state_machine(driver_t* driver)
{

    switch (driver->state) {
        case STATE_RESET_NODE:
            send_nmt_command(0x81, driver->node_id);
            PRINTF("Reset Node\n");

            driver->align = false;
            driver->state = STATE_POWER_ON_RESET;
            driver->mode = MODE_TORQUE;
            break;

        case STATE_POWER_ON_RESET:
            if(can_isNewRxMsg())
            {
                can_msg_t rx_msg;
            	can_readRxMsg(&rx_msg);

                if ((rx_msg.id == (0x700 + driver->node_id)) && (rx_msg.data[1] == 0x00))
                {
                    PRINTF("Boot-up message received.\n");
                    driver->nmt_state = NMT_STATE_BOOTUP;
                    driver->state = STATE_INITIALIZATION;
                    break;
                }
            }
            if (gpioRead(STOP_GPIO_PORT))
            {
                driver->state = STATE_RESET_NODE;
            }
            break;

        case STATE_INITIALIZATION:
            if (send_sdo_write_command(0x40, 0x1810, 0x01, 0x00000000, driver->state))
            {
                PRINTF("Init SDO command sent\n");
            }
            driver->state = STATE_WAIT_START;
            PRINTF("Initialization Mode\n");

            break;

        case STATE_WAIT_START:
            if (gpioRead(START_GPIO_PORT))
            {
            	PRINTF("Start Mode\n");
                driver->state = STATE_START;
                break;
            }
            break;

        case STATE_START:
            if (driver->align)
            {
            	send_nmt_command(NMT_CMD_ENTER_PRE_OPERATIONAL, driver->node_id);
            	driver->nmt_state = NMT_STATE_PRE_OPERATIONAL;
            	PRINTF("Pre-Operational Mode\n");

            	send_sdo_mode_of_operation(MODE_VELOCITY, driver->node_id);
            	driver->mode = MODE_VELOCITY;
            	PRINTF("Mode of operation set to Velocity\n");

				PRINTF("Operational Mode\n");
				send_nmt_command(NMT_CMD_ENTER_OPERATIONAL, driver->node_id);
				driver->nmt_state = NMT_STATE_OPERATIONAL;
                driver->state = STATE_WAIT_DRIVE;
            }
            else
            {
            	send_nmt_command(NMT_CMD_ENTER_PRE_OPERATIONAL, driver->node_id);
            	driver->nmt_state = NMT_STATE_PRE_OPERATIONAL;
            	PRINTF("Pre-Operational Mode\n");

                align_motors(driver->node_id);
                driver->mode = MODE_ALIGNMENT;
                PRINTF("Aligning motors\n");

                delay(5000);
                send_nmt_command(NMT_CMD_ENTER_OPERATIONAL, driver->node_id);
                PRINTF("Operational Mode for align\n");

                driver->nmt_state = NMT_STATE_OPERATIONAL;
                driver->state = STATE_ALIGNING_MOTORS;
            }
            break;


        case STATE_ALIGNING_MOTORS:
            if (check_alignment_status(driver))
            {
                PRINTF("Alignment Complete\n");
                driver->align = true;

//                send_nmt_command(0x80, driver->node_id);
//                driver->nmt_state = NMT_STATE_PRE_OPERATIONAL;
//                PRINTF("Pre-Operational Mode\n");

                send_sdo_mode_of_operation(MODE_VELOCITY, driver->node_id);
                driver->mode = MODE_VELOCITY;
                PRINTF("Mode of operation set to Velocity\n");


                PRINTF("Operational Mode\n");
                delay(5000);
                send_nmt_command(0x01, driver->node_id);
                driver->nmt_state = NMT_STATE_OPERATIONAL;

                driver->state = STATE_WAIT_DRIVE;
            }
            else if (gpioRead(STOP_GPIO_PORT))
            {
                driver->state = STATE_STOPPED;
                break;

            }
            break;



        case STATE_WAIT_DRIVE:
            if (gpioRead(DRIVE_GPIO_PORT)) {
                PRINTF("Drive Mode\n");
                send_controlword(0x06, driver->node_id);
                send_controlword(0x07, driver->node_id);
                send_controlword(0x0F, driver->node_id);
                driver->state = STATE_DRIVE;
                driver->nmt_state = NMT_STATE_DRIVE;
            }
            if (gpioRead(START_GPIO_PORT)) {
                driver->state = STATE_START;
            }
            if (gpioRead(STOP_GPIO_PORT)) {
                driver->state = STATE_STOPPED;
            }
            break;

        case STATE_DRIVE:
            run_motors(driver);

            if (gpioRead(STOP_GPIO_PORT)) {
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




// Function to align motors
void align_motors(uint16_t node_id)
{
    send_sdo_mode_of_operation(MODE_ALIGNMENT, node_id);
}

static time_t last_time = 0;


bool check_alignment_status(driver_t* driver)
{
    time_t current_time = millis();
    if (current_time - last_time >= 3000)
    {
        last_time = current_time;
        //Read mode of operation
        //Od_index: 0x6061 and Od_sub_index: 0x00
        send_sdo_read_command(0x6060, 0x00, driver->node_id);
    }


	//Wait for controller response
    //Command: 0x43 for 4 bytes of read
    //Od_index: 0x6060 and Od_sub_index: 0x00
    int32_t state = recive_sdo_read_command(0x43, 0x6060, 0x00, driver->node_id);


    //After alignment, the mode of operation changes to TORQUE
	if(state == MODE_TORQUE)
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
	run_sensors();	           		//Run sensors
	recive_pdo_message(driver);		//Receive PDO message
	send_pdo_message(driver);		//Send PDO message
	handle_errors(driver);				//Handle errors
}


void handle_errors(driver_t* driver)
{
	if(handle_implausibility()) // Check for implausibility of TPS sensors
	{
		driver->error_code = ERROR_IMPLAUSIBILITY;
		driver->state = STATE_STOPPED;
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
			for (int i = 0; i < 8; i++)
				driver->tpdo1_data.b[i] = rx_msg.data[i];
		}
		if ( rx_msg.id == (TPDO2_ID + driver->node_id ))
		{
			for (int i = 0; i < 8; i++)
				driver->tpdo2_data.b[i] = rx_msg.data[i];
		}
		if ( rx_msg.id == (TPDO3_ID + driver->node_id ))
		{
			for (int i = 0; i < 8; i++)
				driver->tpdo3_data.b[i] = rx_msg.data[i];
		}
		if (rx_msg.id == (TPDO4_ID + driver->node_id)) {
			for (int i = 0; i < 8; i++)
				driver->tpdo4_data.b[i] = rx_msg.data[i];
		}
	}
}



void send_pdo_message(driver_t *driver) {
	const int32_t throttle_threshold = 5; // Define a suitable threshold to handle noise

	int32_t current_throttle = 0;
	int32_t current_torque = 0;

	if (driver->node_id == NODE_ID_1) {
		current_throttle = (int32_t) (tps_data.tps1_value);
	} else if (driver->node_id == NODE_ID_2) {
		current_throttle = -(int32_t) (tps_data.tps1_value);
	}

	driver->pdo1_data.data.control_word = 0x0F | driver->node_id << 8;
	driver->pdo1_data.data.target_velocity = current_throttle;
	driver->pdo1_data.data.target_torque = current_torque;

	bool is_accelerating = (current_throttle - driver->prev_throttle) > throttle_threshold;

	if (is_accelerating) {
		can_msg_t pdo_msg;
		pdo_msg.id = RPDO1_ID + driver->node_id;
		pdo_msg.len = 8;

		pdo_msg.data[0] = 0x0F;
		pdo_msg.data[1] = driver->node_id;

		pdo_msg.data[2] = (uint8_t) (current_throttle & 0x000000FF);
		pdo_msg.data[3] = (uint8_t) ((current_throttle >> 8) & 0x000000FF);
		pdo_msg.data[4] = (uint8_t) ((current_throttle >> 16) & 0x000000FF);
		pdo_msg.data[5] = (uint8_t) ((current_throttle >> 24) & 0x000000FF);

		pdo_msg.data[6] = (uint8_t) (current_torque & 0x00FF);
		pdo_msg.data[7] = (uint8_t) ((current_torque >> 8) & 0x00FF);

		if (can_isTxReady()) {
			can_sendTxMsg(&pdo_msg);
			driver->sensor_time_stamp = millis(); // Update the timestamp
		}

		// Update previous values
		driver->prev_throttle = current_throttle;
		driver->prev_torque = current_torque;
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
    for (int i = 2; i < 8; i++)
    {
        control_msg.data[i] = 0x00;  // Rellenar los bytes restantes con ceros
    }

    // Enviar el mensaje CAN
    while (!can_isTxReady())
    {
        // Wait until Tx is ready
    }
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
 * 	    -4: Alignment
 * @return bool : 1 = Send success
 */
bool send_sdo_mode_of_operation(int8_t mode, uint16_t node_id)
{

	send_sdo_write_command(0x2F, 0x6060, 0x00, (int32_t)mode, node_id);

	//Wait for controller response
	while(!recive_sdo_write_command(0x60, 0x6060, 0x00, 0x00000000, node_id))
	{
		//Generate a SDO Write to save parameters
		send_sdo_write_command(0x23, 0x1010, 0x01, SAVE_PARAM, node_id);

		//Wait for controller response
		while(!recive_sdo_write_command(0x60, 0x1010, 0x01, 0x00000000, node_id))
		{
			return 1;

		}
	}

}


void send_motor_data_uart(driver_t *driver)
{
	char buffer[64];

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








