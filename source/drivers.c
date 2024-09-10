/*
 * drivers.c
 *
 *  Created on: 22 ago 2024
 *      Author: asolari
 */
#include "drivers.h"

static nmt_state_t current_state = STATE_POWER_ON_RESET;
static uint32_t last_send_time = 0;

void run_motors (uint16_t node_id);
void send_pdo_sync_message(void);
void recive_pdo_message(uint16_t node_id);
void send_pdo_message(uint16_t node_id);
bool send_controlword(uint8_t controlword, uint16_t node_id);
bool send_sdo_mode_of_operation(uint8_t mode, uint16_t node_id);


/**
 * @brief Initialize the drivers
 * @return void
 */
void init_drivers(void)
{
	current_state = STATE_POWER_ON_RESET;
	millis_init();

}


/**
 * @brief Update the state machine
 * @param node_id Node ID
 * @return void
 */
void update_state_machine(uint16_t node_id)
{
    switch (current_state)
    {
        case STATE_POWER_ON_RESET:
            // Wait boot up message
            gpioBlink(PIN_LED_RED);

            if (can_isNewRxMsg()) {
                can_msg_t rx_msg;
                if (can_readRxMsg(&rx_msg)
                		&& (rx_msg.id == (0x700 + node_id))
						&& (rx_msg.data[1] == 0x00) )
                {
                    PRINTF("Boot-up message received.\n");
                    current_state = STATE_INITIALIZATION;

                }
            }
            break;

        case STATE_INITIALIZATION:

            gpioBlink(PIN_LED_RED);

            // Acción: Enviar SDO request para leer el Object Dictionary (0x1018, subindex 0x01)
            if(send_sdo_write_command(0x40, 0x1810, 0x01, 0x00000000, node_id))
            	PRINTF("Init SDO command sent\n");

			// Esperar la respuesta del esclavo
			while(!recive_sdo_write_command(0x43, 0x1810, 0x01, 0x19030000, node_id))
			PRINTF("Received SDO response\n");



            current_state = STATE_WAIT_PRE_OPERATIONAL;
            PRINTF("Initialization Mode\n");

            if(gpioRead(PIN_LED_RED) == LED_ACTIVE)
            	gpioWrite(PIN_LED_RED, !LED_ACTIVE);

            break;

        case STATE_WAIT_PRE_OPERATIONAL:
            gpioBlink(PIN_LED_RED);
            // Esperar a que el botón de PRE-OP sea presionado

            if (gpioRead(PRE_OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón
            {
                current_state = STATE_PRE_OPERATIONAL;
                PRINTF("Pre-Operational Mode\n");
                if(gpioRead(PIN_LED_RED) == LED_ACTIVE)
                	gpioWrite(PIN_LED_RED, !LED_ACTIVE);
            }
            break;

        case STATE_PRE_OPERATIONAL:

            send_nmt_command(0x80, node_id);  // Enviar comando para Pre-operational
            // Configurar el modo de operación
            send_sdo_mode_of_operation(0x09, node_id);  // 0x09: Velocity mode

            PRINTF("Pre-Operational Mode\n");

            current_state = STATE_WAIT_OPERATIONAL;
            break;

        case STATE_WAIT_OPERATIONAL:  // Pre Operational mode
            gpioBlink(PIN_LED_GREEN);

            if(gpioRead(OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón
            {
                current_state = STATE_OPERATIONAL;
                PRINTF("Operational Mode\n");
            }
            break;

        case STATE_OPERATIONAL:
            // Enviar comando NMT para pasar a Operational
            send_nmt_command(0x01, node_id);
            current_state = STATE_WAIT_DRIVE;
            break;

        case STATE_WAIT_DRIVE:  //In Operational mode
            gpioWrite(PIN_LED_GREEN, HIGH);

            if (gpioRead(DRIVE_GPIO_PORT) == HIGH)
            {
                PRINTF("Drive Mode\n");

                // Secuencia para habilitar el drive
                bool cw_6 = send_controlword(0x06, node_id);   // Enviar CW 6 para preparar el sistema
                bool cw_7 = send_controlword(0x07, node_id);   // Enviar CW 7 para habilitar el modo de operación
                bool cw_15 = send_controlword(0x0F, node_id);  // Enviar CW 15 para habilitar la operación

                if(cw_6 && cw_7 && cw_15)
                	PRINTF("Control Word messages sent: cw_6: %d - cw_7: %d - cw_15: %d\n", cw_6, cw_7, cw_15);

                current_state = STATE_DRIVE;
                if(gpioRead(PIN_LED_GREEN) == LED_ACTIVE)
                	gpioWrite(PIN_LED_GREEN, !LED_ACTIVE);
                if(gpioRead(PIN_LED_BLUE) != LED_ACTIVE)
                	gpioWrite(PIN_LED_BLUE, LED_ACTIVE);
            }

			if (gpioRead(PRE_OP_GPIO_PORT) == HIGH)
			{
				current_state = STATE_PRE_OPERATIONAL;
				if (gpioRead(PIN_LED_RED) == LED_ACTIVE)
					gpioWrite(PIN_LED_RED, !LED_ACTIVE);
			}
            break;

        case STATE_DRIVE:
            // Aquí puedes comenzar a enviar comandos de torque o velocidad
            run_sensors();
            run_motors(node_id);

            if (gpioRead(STOP_GPIO_PORT) == HIGH) {
                // Enviar comando NMT para pasar a STOP Mode
                gpioMode(PIN_LED_RED, HIGH);
                PRINTF("Stop Mode\n");
                current_state = STATE_STOPPED;
            }
			if (gpioRead(PRE_OP_GPIO_PORT) == HIGH)
			{
				current_state = STATE_PRE_OPERATIONAL;
				if (gpioRead(PIN_LED_RED) == LED_ACTIVE)
					gpioWrite(PIN_LED_RED, !LED_ACTIVE);
			}
			if(gpioRead(OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón))
			{
				current_state = STATE_OPERATIONAL;
				if (gpioRead(PIN_LED_GREEN) == LED_ACTIVE)
					gpioWrite(PIN_LED_GREEN, !LED_ACTIVE);
			}
			break;

        case STATE_STOPPED:
            send_nmt_command(0x80, node_id);  // Volver a Pre-operational
            current_state = STATE_PRE_OPERATIONAL;
            break;
    }
}










void run_motors (uint16_t node_id)
{
	//Send SYNC message every 2 seconds
	send_pdo_sync_message();

	recive_pdo_message(node_id);

	send_pdo_message(node_id);
}

//    // Enviar los RPDOs con los comandos de control
//    //PDO1: control word[2 bytes], target velocity[4 bytes], max torque[2 bytes]
//    //PDO2: targuet position [4 bytes], other reserved[4 bytes]
//
////    uint8_t rpdo1Data[8] = { 0x0F, 0x00, 0x00, 0x00, (sensor_values.throttle) & 0xFF, (sensor_values.throttle >> 8) & 0xFF, sensor_values.torque & 0xFF, (sensor_values.torque >> 8) & 0xFF };
////    uint8_t rpdo1Data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, sensor_values.torque & 0xFF, (sensor_values.torque >> 8) & 0xFF };
////    uint8_t rpdo1Data[8] = { 0x0F, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0, 0 };
//
////    uint8_t rpdo2Data[8] = {0x0F, 0x00, 0x00, 0x00};
//
////    send_can_message(RPDO1_ID, rpdo1Data, 8);
////    send_can_message(RPDO2_ID, rpdo2Data, 4);
//
//    can_msg_t sdo_msg;
//    sdo_msg.id = 0x581 + NODE_ID; // COB-ID del SDO Write
//    sdo_msg.len = 8;
//    sdo_msg.data[0] = 0x43;        // Comando SDO Write (4 bytes)
//    sdo_msg.data[1] = 0xFF;        // Índice (byte bajo de 0x60FF)
//    sdo_msg.data[2] = 0x60;        // Índice (byte alto de 0x60FF)
//    sdo_msg.data[3] = 0x00;        // Subíndice (0x00)
//    sdo_msg.data[4] = (sensor_values.throttle & 0xFF);       // Byte 0 del valor
//    sdo_msg.data[5] = (sensor_values.throttle >> 8) & 0xFF;  // Byte 1 del valor
//    sdo_msg.data[6] = (sensor_values.throttle >> 16) & 0xFF; // Byte 2 del valor
//    sdo_msg.data[7] = (sensor_values.throttle >> 24) & 0xFF; // Byte 3 del valor
//
//    can_sendTxMsg(&sdo_msg);
//
//    can_msg_t sdo_response;
//    if (can_isNewRxMsg()) {
//        if (can_readRxMsg(&sdo_response) && sdo_response.id == (0x580 + NODE_ID)) {
//            if (sdo_response.data[0] == 0x60 && sdo_response.data[1] == 0xFF && sdo_response.data[2] == 0x60 && sdo_response.data[3] == 0x00) {
//                PRINTF("Target Velocity set successfully.\n");
//            } else {
//                PRINTF("Error setting Target Velocity.\n");
//            }
//        }
//    }




/**
 * @brief Send a PDO SYNC message
 * void send_pdo_message(void)
 *
 * @return void
 */

void send_pdo_sync_message(void)
{
	uint32_t current_time = millis();
	if (current_time - last_send_time >= 2000) {
		last_send_time = current_time;

		//Send SYNC message
		can_msg_t sync_msg;
		sync_msg.id = SYNC_MESSAGE_ID;
		sync_msg.rtr = 0;
		sync_msg.len = 1;
		sync_msg.data[0] = 0x00;

		while (!can_isTxReady());
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
void recive_pdo_message(uint16_t node_id)
{
	can_msg_t rx_msg;
	if (can_isNewRxMsg()) {
		if (can_readRxMsg(&rx_msg) && rx_msg.id == (TPDO1_ID + node_id ))
		{

		}
		if (can_readRxMsg(&rx_msg) && rx_msg.id == (TPDO2_ID + node_id ))
		{

		}
		if (can_readRxMsg(&rx_msg) && rx_msg.id == (TPDO3_ID + node_id ))
		{

		}
		if (can_readRxMsg(&rx_msg) && rx_msg.id == (TPDO4_ID + node_id ))
		{

		}
	}
}

/**
 * @brief Send a PDO message
 * void send_pdo_message(uint8_t node_id)
 *
 * @return void
 */
void send_pdo_message(uint16_t node_id)
{

	can_msg_t pdo_msg;
	pdo_msg.id = RPDO1_ID + node_id;
	pdo_msg.rtr = 0;
	pdo_msg.len = 8;
	pdo_msg.data[0] = 0x0F;
	pdo_msg.data[1] = node_id;

	pdo_msg.data[2] = 0x00;
	pdo_msg.data[3] = 0x00;
	pdo_msg.data[4] = 0xFF;
	pdo_msg.data[5] = 0xFF;
	pdo_msg.data[6] = 0x00;
	pdo_msg.data[7] = 0x00;

	while (!can_isTxReady())
		;
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

bool send_controlword(uint8_t controlword, uint16_t node_id)
{
    //Send SDO Write command
	send_sdo_write_command(0x2B, 0x6040, 0x00, (uint32_t)( (controlword) | (node_id<<8) ), node_id); //data = 0x control_word + node_id

	//Wait for controller response
	return recive_sdo_write_command(0x60, 0x6040, 0x00, 0x00000000, node_id);

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
bool send_sdo_mode_of_operation(uint8_t mode, uint16_t node_id)
{
	send_sdo_write_command(0x2F, 0x6060, 0x00, (uint32_t)mode, node_id);

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



