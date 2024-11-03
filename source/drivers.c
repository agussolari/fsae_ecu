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

	millis_init();

}


/**
 * @brief Update the state machine
 * @param node_id Node ID
 * @return void
 */
void update_state_machine(driver_t* driver)
{
    switch (driver->state)
    {
    	case STATE_RESET_NODE:
    		// Enviar comando para RESET NODE
            send_nmt_command(0x81, driver->node_id);
            PRINTF("Reset Node\n");

            //Align in false
            driver->align = false;

//            while(!can_isTxReady());

            // Mapp the PDO's
//        	map_rpdo(driver->node_id);
//        	map_tpdo(driver->node_id);

        	driver->state = STATE_POWER_ON_RESET;
        	break;

        case STATE_POWER_ON_RESET:


            can_msg_t rx_msg;
            if (can_readRxMsg(&rx_msg)
              		&& (rx_msg.id == (0x700 + driver->node_id))
					&& (rx_msg.data[0] == 0x00) )
            {
            	 PRINTF("Boot-up message received.\n");
                 driver->state = STATE_INITIALIZATION;
            }
		if (gpioRead(STOP_GPIO_PORT) == HIGH) {
			driver->state = STATE_RESET_NODE;
		}

            break;

        case STATE_INITIALIZATION:
            // Acción: Enviar SDO request para leer el Object Dictionary (0x1018, subindex 0x01)
            if(send_sdo_write_command(0x40, 0x1810, 0x01, 0x00000000, driver->state))
            	PRINTF("Init SDO command sent\n");

            driver->state = STATE_WAIT_PRE_OPERATIONAL;
            PRINTF("Initialization Mode\n");

            break;

        case STATE_WAIT_PRE_OPERATIONAL:
            // Esperar a que el botón de PRE-OP sea presionado
            if (gpioRead(PRE_OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón
            {
            	driver->state = STATE_PRE_OPERATIONAL;
                PRINTF("Pre-Operational Mode\n");
            }
            break;

        case STATE_PRE_OPERATIONAL:
        	 // Enviar comando para Pre-operational
            send_nmt_command(0x80, driver->node_id);
            PRINTF("Pre-Operational Mode\n");

            driver->state = STATE_ALIGN_MOTORS;
            break;


        case STATE_ALIGN_MOTORS:
        	if(driver->align == true)
        	{
        		driver->state = STATE_WAIT_OPERATIONAL;
        	}
        	else
        	{
				align_motors(driver->node_id);
				PRINTF("Aligning motors\n");
				driver->state = STATE_WAIT_ALIGN_MOTORS;
        	}

			break;

		case STATE_WAIT_ALIGN_MOTORS:
			if(gpioRead(OP_GPIO_PORT) == HIGH)
			{
	            send_nmt_command(0x01, driver->node_id);
                PRINTF("Operational Mode for align\n");
                driver->state = STATE_ALIGNING_MOTORS;
			}
			if (gpioRead(PRE_OP_GPIO_PORT) == HIGH)
			{
				driver->state = STATE_PRE_OPERATIONAL;
			}

			break;

		case STATE_ALIGNING_MOTORS:
			//Send SDO Read for state of alignment
//			send_sdo_read_command(0x6060, 0x00, driver->node_id);
            if (check_alignment_status(driver))
            {
                PRINTF("Alignment Complete\n");

                driver->align = true;

                //Set mode in pre operational
                send_nmt_command(0x80, driver->node_id);
                PRINTF("Pre-Operational Mode\n");

                //Set mode of operation to velocity
                send_sdo_mode_of_operation(0x09, driver->node_id);
                PRINTF("Mode of operation set to Velocity\n");


                PRINTF("Operational Mode\n");
                driver->state = STATE_OPERATIONAL;
            }

            else if (gpioRead(PRE_OP_GPIO_PORT) == HIGH)
			{
				driver->state = STATE_PRE_OPERATIONAL;
			}

            else if(gpioRead(STOP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón)
			{
				driver->state = STATE_STOPPED;
			}

            break;

        case STATE_WAIT_OPERATIONAL:  // Pre Operational mode

            if(gpioRead(OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón
            {
            	driver->state = STATE_OPERATIONAL;
                PRINTF("Operational Mode\n");
            }
            break;

        case STATE_OPERATIONAL:
            // Enviar comando NMT para pasar a Operational
            send_nmt_command(0x01, driver->node_id);
            driver->state = STATE_WAIT_DRIVE;
            break;

        case STATE_WAIT_DRIVE:  //In Operational mode
            if (gpioRead(DRIVE_GPIO_PORT) == HIGH)
            {
                PRINTF("Drive Mode\n");

                // Secuencia para habilitar el drive
                bool cw_6 = send_controlword(0x06, driver->node_id);   // Enviar CW 6 para preparar el sistema
                bool cw_7 = send_controlword(0x07, driver->node_id);   // Enviar CW 7 para habilitar el modo de operación
                bool cw_15 = send_controlword(0x0F, driver->node_id);  // Enviar CW 15 para habilitar la operación


                driver->state = STATE_DRIVE;
            }

			if (gpioRead(PRE_OP_GPIO_PORT) == HIGH)
			{
				driver->state = STATE_PRE_OPERATIONAL;
			}

			if (gpioRead(OP_GPIO_PORT) == HIGH)
			{
				driver->state = STATE_OPERATIONAL;
			}
            break;

        case STATE_DRIVE:
            // Aquí puedes comenzar a enviar comandos de torque o velocidad
            run_sensors();
            run_motors(driver);
            handle_errors();

            if (gpioRead(STOP_GPIO_PORT) == HIGH)
            {
                driver->state = STATE_STOPPED;
            }
			if (gpioRead(PRE_OP_GPIO_PORT) == HIGH)
			{
				driver->state = STATE_PRE_OPERATIONAL;
			}
			if(gpioRead(OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón))
			{
				driver->state = STATE_OPERATIONAL;
			}
			break;

        case STATE_STOPPED:
            PRINTF("Stop Mode\n");
            send_nmt_command(0x80, driver->node_id);  // Volver a Pre-operational
            driver->state = STATE_PRE_OPERATIONAL;
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

	if(state == 0x0A) //Torque mode
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

	send_pdo_sync_message(driver);		//Send SYNC message every 2 seconds

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
	if (current_time - driver->time_stamp >= 2000) {
		driver->time_stamp = current_time;

		//Send SYNC message
		can_msg_t sync_msg;
		sync_msg.id = SYNC_MESSAGE_ID;
		sync_msg.rtr = 0;
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

//		PRINTF("Received message ID: %03X\n", rx_msg.id);


		if (rx_msg.id == (TPDO1_ID + driver->node_id ))
		{
			PRINTF("TPDO1 received\n");
			for (int i = 0; i < 8; i++)
				driver->tpdo1_data.b[i] = rx_msg.data[i];
		}
		if ( rx_msg.id == (TPDO2_ID + driver->node_id ))
		{
			PRINTF("TPDO2 received\n");
			for (int i = 0; i < 8; i++)
				driver->tpdo2_data.b[i] = rx_msg.data[i];
		}
		if ( rx_msg.id == (TPDO3_ID + driver->node_id ))
		{
			PRINTF("TPDO3 received\n");
			for (int i = 0; i < 8; i++)
				driver->tpdo3_data.b[i] = rx_msg.data[i];
		}
	}
}

/**
 * @brief Send a PDO message
 * void send_pdo_message(uint8_t node_id)
 *
 * Send a PDO message to the node_id
 *
 * b[0], b[1]: Control word
 * b[2], b[3], b[4], b[5]: Target velocity
 * b[6], b[7]: Target torque
 *
 * @return void
 *
 */

void send_pdo_message(driver_t* driver)
{
    const int32_t throttle_threshold = 5; // Define a suitable threshold
    const int16_t torque_threshold = 5;   // Define a suitable threshold

    int32_t current_throttle = sensor_values.throttle;
    int16_t current_torque = sensor_values.torque;

    // Check if sensor values are zero
    if (current_throttle == 0 && current_torque == 0) {
        if (!driver->zero_message_sent) {
            can_msg_t pdo_msg;
            pdo_msg.id = RPDO1_ID + driver->node_id;
            pdo_msg.rtr = 0;
            pdo_msg.len = 8;

            pdo_msg.data[0] = 0x0F;
            pdo_msg.data[1] = driver->node_id;

            pdo_msg.data[2] = 0;
            pdo_msg.data[3] = 0;
            pdo_msg.data[4] = 0;
            pdo_msg.data[5] = 0;
            driver->pdo1_data.data.target_velocity = 0;

            pdo_msg.data[6] = 0;
            pdo_msg.data[7] = 0;
            driver->pdo1_data.data.target_torque = 0;

            if (can_isTxReady())
                can_sendTxMsg(&pdo_msg);

            driver->zero_message_sent = true;
        }
    } else {
        driver->zero_message_sent = false;

        if (abs(current_throttle - driver->prev_throttle) > throttle_threshold ||
            abs(current_torque - driver->prev_torque) > torque_threshold)
        {
            can_msg_t pdo_msg;
            pdo_msg.id = RPDO1_ID + driver->node_id;
            pdo_msg.rtr = 0;
            pdo_msg.len = 8;

            pdo_msg.data[0] = 0x0F;
            pdo_msg.data[1] = driver->node_id;
            driver->pdo1_data.data.control_word = 0x0F | driver->node_id << 8;

            pdo_msg.data[2] = (uint8_t)(current_throttle & 0x000000FF);
            pdo_msg.data[3] = (uint8_t)((current_throttle >> 8) & 0x000000FF);
            pdo_msg.data[4] = (uint8_t)((current_throttle >> 16) & 0x000000FF);
            pdo_msg.data[5] = (uint8_t)((current_throttle >> 24) & 0x000000FF);
            driver->pdo1_data.data.target_velocity = current_throttle;

            pdo_msg.data[6] = (uint8_t)(current_torque & 0x00FF);
            pdo_msg.data[7] = (uint8_t)((current_torque >> 8) & 0x00FF);
            driver->pdo1_data.data.target_torque = current_torque;

            if (can_isTxReady())
                can_sendTxMsg(&pdo_msg);

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
    send_sdo_write_command(0x2B, 0x1400, 0x01, 0x80000000, node_id);

    // Disable PDO mapping
    send_sdo_write_command(0x2B, 0x1600, 0x00, 0x00, node_id);

    // Map RPDO
    // b[0], b[1]: Control word (0x6040, 0x00)
    // b[2], b[3], b[4], b[5]: Target velocity 0x60FF, 0x00
    // b[6], b[7]: Target torque 0x6071, 0x00


    //Map number of entry: 3 entry
    send_sdo_write_command(0x2F, 0x1600, 0x00, 0x03, node_id);

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
	send_sdo_write_command(0x2F, 0x1A00, 0x00, 0x03, node_id);

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



//void send_motor_data_uart(driver_t* driver) {
//    const char* motor_state;
//    const char* operation_mode;
//    int target_velocity = driver->target_velocity; // Reemplazar con el valor real
//
//    //tpdo[1] = b[2], b[3], b[4], b[5]
//    int actual_velocity = driver->tpdo1_data[2] | driver->tpdo1_data[3] << 8 | driver->tpdo1_data[4] << 16 | driver->tpdo1_data[5] << 24;
//    float current = driver->tpdo2_data[6] | driver->tpdo2_data[7] << 8; // Reemplazar con el valor real
//    float actual_torque = driver->tpdo1_data[6] | driver->tpdo1_data[7] << 8; // Reemplazar con el valor real
//    float desired_torque = (float)(driver->target_torque); // Reemplazar con el valor real
//    float temperature = driver->tpdo2_data[5]; // Reemplazar con el valor real
//    int error_indicator = 0; // Reemplazar con el valor real
//
//    // Determinar el estado del motor
//    switch (driver->state) {
//        case STATE_PRE_OPERATIONAL:
//            motor_state = "Pre-operacional";
//            break;
//        case STATE_OPERATIONAL:
//            motor_state = "Operacional";
//            break;
//        case STATE_DRIVE:
//            motor_state = "Drive";
//            break;
//        case STATE_STOPPED:
//            motor_state = "Stop";
//            break;
//        default:
//            motor_state = "Unknown";
//            break;
//    }
//
//    // Determinar el modo de operación
////    switch (driver->mode) {
////        case MODE_VELOCITY:
////            operation_mode = "Velocidad";
////            break;
////        case MODE_TORQUE:
////            operation_mode = "Torque";
////            break;
////        case MODE_ALIGNMENT:
////            operation_mode = "Alineación";
////            break;
////        default:
////            operation_mode = "Unknown";
////            break;
////    }
//
//    // Formatear los datos manualmente
//    char buffer[256] = {0};
//    char temp[50];
//
//    // Concatenar node_id
//    itoa(driver->node_id, temp, 10);
//    strcat(buffer, temp);
//    strcat(buffer, ",");
//
//    // Concatenar motor_state
//    strcat(buffer, motor_state);
//    strcat(buffer, ",");
//
//    // Concatenar operation_mode
//    strcat(buffer, operation_mode);
//    strcat(buffer, ",");
//
//    // Concatenar target_velocity
//    itoa(target_velocity, temp, 10);
//    strcat(buffer, temp);
//    strcat(buffer, ",");
//
//    // Concatenar actual_velocity
//    itoa(actual_velocity, temp, 10);
//    strcat(buffer, temp);
//    strcat(buffer, ",");
//
//    // Concatenar current
//    snprintf(temp, sizeof(temp), "%.2f", current);
//    strcat(buffer, temp);
//    strcat(buffer, ",");
//
//    // Concatenar actual_torque
//    snprintf(temp, sizeof(temp), "%.2f", actual_torque);
//    strcat(buffer, temp);
//    strcat(buffer, ",");
//
//    // Concatenar desired_torque
//    snprintf(temp, sizeof(temp), "%.2f", desired_torque);
//    strcat(buffer, temp);
//    strcat(buffer, ",");
//
//    // Concatenar temperature
//    snprintf(temp, sizeof(temp), "%.2f", temperature);
//    strcat(buffer, temp);
//    strcat(buffer, ",");
//
//    // Concatenar error_indicator
//    itoa(error_indicator, temp, 10);
//    strcat(buffer, temp);
//
//    strcat(buffer, "\n");
//
//    // Enviar los datos por UART
//    uartWriteStr(buffer);
//
//
//}







