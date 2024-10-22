/*
 * drivers.c
 *
 *  Created on: 22 ago 2024
 *      Author: asolari
 */
#include "drivers.h"


void run_motors (driver_t* driver);
void send_pdo_sync_message(driver_t* driver);
void recive_pdo_message(driver_t* driver);
void send_pdo_message(uint16_t node_id);

//bool send_controlword(uint8_t controlword, uint16_t node_id);
bool send_controlword(uint8_t controlword, uint16_t node_id);

bool send_sdo_mode_of_operation(int8_t mode, uint16_t node_id);

void map_rpdo(uint16_t node_id);
void map_tpdo(uint16_t node_id);

bool check_alignment_status(driver_t* driver);
void align_motors(uint16_t node_id);

/**
 * @brief Initialize the drivers
 * @return void
 */
void init_drivers(driver_t* driver)
{
	driver->state = STATE_POWER_ON_RESET;
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
        case STATE_POWER_ON_RESET:
            // Wait boot up message
            gpioBlink(PIN_LED_RED);

            // Enviar comando para RESET NODE
            send_nmt_command(0x81, driver->node_id);

            while(!can_isNewRxMsg());

            // Mapp the PDO's
//        	map_rpdo(node_id);
//        	map_tpdo(node_id);

            if (can_isNewRxMsg())
            {
                can_msg_t rx_msg;
                if (can_readRxMsg(&rx_msg)
                		&& (rx_msg.id == (0x700 + driver->node_id))
						&& (rx_msg.data[0] == 0x7F) )
                {
                    PRINTF("Boot-up message received.\n");
                    driver->state = STATE_INITIALIZATION;
                }

            }
            break;

        case STATE_INITIALIZATION:

            gpioBlink(PIN_LED_RED);

            // Acción: Enviar SDO request para leer el Object Dictionary (0x1018, subindex 0x01)
            if(send_sdo_write_command(0x40, 0x1810, 0x01, 0x00000000, driver->state))
            	PRINTF("Init SDO command sent\n");


            driver->state = STATE_WAIT_PRE_OPERATIONAL;
            PRINTF("Initialization Mode\n");

            if(gpioRead(PIN_LED_RED) == LED_ACTIVE)
            	gpioWrite(PIN_LED_RED, !LED_ACTIVE);

            break;

        case STATE_WAIT_PRE_OPERATIONAL:
            gpioBlink(PIN_LED_RED);
            // Esperar a que el botón de PRE-OP sea presionado

            if (gpioRead(PRE_OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón
            {
            	driver->state = STATE_PRE_OPERATIONAL;
                PRINTF("Pre-Operational Mode\n");
                if(gpioRead(PIN_LED_RED) == LED_ACTIVE)
                	gpioWrite(PIN_LED_RED, !LED_ACTIVE);
            }
            break;

        case STATE_PRE_OPERATIONAL:
        	 // Enviar comando para Pre-operational
            send_nmt_command(0x80, driver->node_id);
            PRINTF("Pre-Operational Mode\n");

            driver->state = STATE_ALIGN_MOTORS;
            break;


        case STATE_ALIGN_MOTORS:
			align_motors(driver->node_id);
			PRINTF("Aligning motors\n");
			driver->state = STATE_WAIT_ALIGN_MOTORS;
			break;

		case STATE_WAIT_ALIGN_MOTORS:
			if(gpioRead(OP_GPIO_PORT) == HIGH)
			{
	            send_nmt_command(0x01, driver->node_id);
                PRINTF("Operational Mode for align\n");
                driver->state = STATE_ALIGNING_MOTORS;

			}
			break;

		case STATE_ALIGNING_MOTORS:
            gpioBlink(PIN_LED_GREEN);
            if (check_alignment_status(driver)) {
                PRINTF("Alignment Complete\n");

                //Set mode in pre operational
                send_nmt_command(0x80, driver->node_id);
                PRINTF("Pre-Operational Mode\n");

                //Set mode of operation to velocity
                send_sdo_mode_of_operation(0x09, driver->node_id);

                driver->state = STATE_WAIT_OPERATIONAL;
            }
            break;

        case STATE_WAIT_OPERATIONAL:  // Pre Operational mode
            gpioBlink(PIN_LED_GREEN);

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
            gpioWrite(PIN_LED_GREEN, HIGH);

            if (gpioRead(DRIVE_GPIO_PORT) == HIGH)
            {
                PRINTF("Drive Mode\n");

                // Secuencia para habilitar el drive
                bool cw_6 = send_controlword(0x06, driver->node_id);   // Enviar CW 6 para preparar el sistema
                bool cw_7 = send_controlword(0x07, driver->node_id);   // Enviar CW 7 para habilitar el modo de operación
                bool cw_15 = send_controlword(0x0F, driver->node_id);  // Enviar CW 15 para habilitar la operación

                PRINTF("Control Word messages sent: cw_6: %d - cw_7: %d - cw_15: %d\n", cw_6, cw_7, cw_15);

                driver->state = STATE_DRIVE;
                if(gpioRead(PIN_LED_GREEN) == LED_ACTIVE)
                	gpioWrite(PIN_LED_GREEN, !LED_ACTIVE);
                if(gpioRead(PIN_LED_BLUE) != LED_ACTIVE)
                	gpioWrite(PIN_LED_BLUE, LED_ACTIVE);
            }

			if (gpioRead(PRE_OP_GPIO_PORT) == HIGH)
			{
				driver->state = STATE_PRE_OPERATIONAL;
				if (gpioRead(PIN_LED_RED) == LED_ACTIVE)
					gpioWrite(PIN_LED_RED, !LED_ACTIVE);
			}
            break;

        case STATE_DRIVE:
            // Aquí puedes comenzar a enviar comandos de torque o velocidad
            run_sensors();
            run_motors(driver);

            if (gpioRead(STOP_GPIO_PORT) == HIGH) {
                // Enviar comando NMT para pasar a STOP Mode
                gpioMode(PIN_LED_RED, HIGH);
                PRINTF("Stop Mode\n");
                driver->state = STATE_STOPPED;
            }
			if (gpioRead(PRE_OP_GPIO_PORT) == HIGH)
			{
				driver->state = STATE_PRE_OPERATIONAL;
				if (gpioRead(PIN_LED_RED) == LED_ACTIVE)
					gpioWrite(PIN_LED_RED, !LED_ACTIVE);
			}
			if(gpioRead(OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón))
			{
				driver->state = STATE_OPERATIONAL;
				if (gpioRead(PIN_LED_GREEN) == LED_ACTIVE)
					gpioWrite(PIN_LED_GREEN, !LED_ACTIVE);
			}
			break;

        case STATE_STOPPED:
            send_nmt_command(0x80, driver->node_id);  // Volver a Pre-operational
            driver->state = STATE_PRE_OPERATIONAL;
            break;
    }
}



// Function to align motors
void align_motors(uint16_t node_id)
{
//    PRINTF("ALINEACION %d\n", node_id);
    send_sdo_mode_of_operation(-4, node_id);
}


bool check_alignment_status(driver_t* driver)
{
	// Leer el estado de alineación de los motores
	// Si los motores están alineados, retornar true
	// De lo contrario, retornar false
	int32_t state;

	//Send SDO Read command
	send_sdo_read_command(0x6060, 0x00, driver->node_id);

	//Wait for controller response
	state = recive_sdo_read_command(0x4F, 0x6060, 0x00, driver->node_id);

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

	send_pdo_message(driver->node_id);		//Send PDO message
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
			can_sendTxMsg(&sync_msg);
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
		if (can_readRxMsg(&rx_msg) && rx_msg.id == (TPDO1_ID + driver->node_id ))
		{
			for (int i = 0; i < 8; i++)
				driver->tpdo1_data[i] = rx_msg.data[i];
		}
		if (can_readRxMsg(&rx_msg) && rx_msg.id == (TPDO2_ID + driver->node_id ))
		{
			for (int i = 0; i < 8; i++)
				driver->tpdo2_data[i] = rx_msg.data[i];
		}
		if (can_readRxMsg(&rx_msg) && rx_msg.id == (TPDO3_ID + driver->node_id ))
		{
			for (int i = 0; i < 8; i++)
				driver->tpdo3_data[i] = rx_msg.data[i];
		}
		if (can_readRxMsg(&rx_msg) && rx_msg.id == (TPDO4_ID + driver->node_id ))
		{
			for (int i = 0; i < 8; i++)
				driver->tpdo4_data[i] = rx_msg.data[i];
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
 * uint32_t 0x00000000
 */
void send_pdo_message(uint16_t node_id)
{

	can_msg_t pdo_msg;
	pdo_msg.id = RPDO1_ID + node_id;
	pdo_msg.rtr = 0;
	pdo_msg.len = 8;

	pdo_msg.data[0] = 0x0F;
	pdo_msg.data[1] = node_id;


	pdo_msg.data[2] = (uint8_t)(sensor_values.throttle & 0x000000FF) ;
	pdo_msg.data[3] = (uint8_t)((sensor_values.throttle >> 8) & 0x000000FF);
	pdo_msg.data[4] = (uint8_t)((sensor_values.throttle >> 16) & 0x000000FF);
	pdo_msg.data[5] = (uint8_t)((sensor_values.throttle >> 24) & 0x000000FF);
	pdo_msg.data[6] = (uint8_t)(sensor_values.torque & 0x00FF);
	pdo_msg.data[7] = (uint8_t)((sensor_values.torque >> 8) & 0x00FF);

	if(can_isTxReady())
		can_sendTxMsg(&pdo_msg);

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

//bool send_controlword(uint8_t controlword, uint16_t node_id)
//{
//    //Send SDO Write command
//	send_sdo_write_command(0x2B, 0x6040, 0x00, (int32_t)(controlword), node_id); //data = 0x control_word + node_id
//
//	//Wait for controller response
//	return recive_sdo_write_command(0x60, 0x6040, 0x00, 0x00000000, node_id);
//
//}

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
	//Mpa Actual velocity: 0x606C, 0x00 (4 bytes)
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

	//Map number of entry: 2 entry
	send_sdo_write_command(0x2F, 0x1A01, 0x00, 0x02, node_id);

	//Map Actual velocity value: 0x606C, 0x00 (4 bytes)
	send_sdo_write_command(0x23, 0x1A01, 0x01, 0x606C0020, node_id);
	//Map Controller temperature: 0x2026, 0x01 (1 byte)
	send_sdo_write_command(0x23, 0x1A01, 0x02, 0x20260108, node_id);

	//Set transmission type to sync
	send_sdo_write_command(0x2B, 0x1801, 0x02, 0x01, node_id);

	//Set COB-ID to 0x280 + node_id
	send_sdo_write_command(0x2B, 0x1801, 0x01, TPDO2_ID + node_id, node_id);

	//Save parameters
	send_sdo_write_command(0x23, 0x1010, 0x01, SAVE_PARAM, node_id);

	//Reset node
	send_nmt_command(0x81, node_id);

}






