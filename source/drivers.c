/*
 * drivers.c
 *
 *  Created on: 22 ago 2024
 *      Author: asolari
 */
#include "drivers.h"

nmt_state_t current_state = STATE_POWER_ON_RESET;
can_msg_t rx_msg;

#define EMCY_ID (0x080 + NODE_ID)

void send_controlword(uint8_t controlword);
void send_sdo_mode_of_operation(uint8_t mode);


void init_drivers(void)
{
	current_state = STATE_POWER_ON_RESET;

}

void send_nmt_command(uint8_t command, uint8_t node_id) {
    can_msg_t nmt_msg;
    nmt_msg.id = 0x000;  // COB-ID para NMT
    nmt_msg.len = 2;
    nmt_msg.data[0] = command;  // Comando NMT (0x01 para Operational, 0x80 para Pre-operational)
    nmt_msg.data[1] = node_id;  // Node ID del esclavo
    can_sendTxMsg(&nmt_msg);
}

void run_motors (void)
{
    uint8_t syncData[1] = {0x00}; // Datos del mensaje SYNC
    uint8_t rxBuffer[8]; // Buffer para recibir mensajes

    // Enviar el mensaje SYNC
    send_can_message(SYNC_MESSAGE_ID, syncData, 1);


    // Esperar los TPDOs
    if (receive_can_message(TPDO1_ID, rxBuffer)) {
        // Procesar TPDO1: Estado del sistema y posición real
    	//Print Status Word : b[0] and b[1]
    	//PRINTF("Status Word: %d\n", rxBuffer[0] | (rxBuffer[1] << 8));
    	//PRINTF("Position Actual: %d\n", rxBuffer[2] | (rxBuffer[3] << 8) | (rxBuffer[4] << 16) | (rxBuffer[5] << 24));
//    	PRINTF("Velocity Actual: %d\n", rxBuffer[6] | (rxBuffer[7] <<8));
    }
    if (receive_can_message(TPDO2_ID, rxBuffer)) {
        // Procesar TPDO2: Temperatura, voltaje, corriente
    	//PRINTF("Controller temp: %d\n", rxBuffer[0]);
    	//PRINTF("Motor temp: %d\n", rxBuffer[1]);
    	//PRINTF("DC Bus voltage: %d\n", rxBuffer[2] | (rxBuffer[3] << 8));
    	//PRINTF("DC Bus current: %d\n", rxBuffer[4] | (rxBuffer[5] << 8));
    	//PRINTF("Current demand: %d\n", rxBuffer[6] | (rxBuffer[7] << 8));

    }
    if (receive_can_message(TPDO3_ID, rxBuffer)) {
        // Procesar TPDO3: Corriente y fase del motor
//    	PRINTF("Motor current: %d\n", rxBuffer[0] | (rxBuffer[1] << 8));
    	//PRINTF("Electrical angle: %d\n", rxBuffer[2] | (rxBuffer[3] << 8));
    	//PRINTF("Phase A current: %d\n", rxBuffer[4] | (rxBuffer[5] << 8));
    	//PRINTF("Phase B current: %d\n", rxBuffer[6] | (rxBuffer[7] << 8));
    }

    // Enviar los RPDOs con los comandos de control
    //PDO1: control word[2 bytes], target velocity[4 bytes], max torque[2 bytes]
    //PDO2: targuet position [4 bytes], other reserved[4 bytes]

//    uint8_t rpdo1Data[8] = { 0x0F, 0x00, 0x00, 0x00, (sensor_values.throttle) & 0xFF, (sensor_values.throttle >> 8) & 0xFF, sensor_values.torque & 0xFF, (sensor_values.torque >> 8) & 0xFF };
//    uint8_t rpdo1Data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, sensor_values.torque & 0xFF, (sensor_values.torque >> 8) & 0xFF };
//    uint8_t rpdo1Data[8] = { 0x0F, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0, 0 };

//    uint8_t rpdo2Data[8] = {0x0F, 0x00, 0x00, 0x00};

//    send_can_message(RPDO1_ID, rpdo1Data, 8);
//    send_can_message(RPDO2_ID, rpdo2Data, 4);

    can_msg_t sdo_msg;
    sdo_msg.id = 0x581 + NODE_ID; // COB-ID del SDO Write
    sdo_msg.len = 8;
    sdo_msg.data[0] = 0x43;        // Comando SDO Write (4 bytes)
    sdo_msg.data[1] = 0xFF;        // Índice (byte bajo de 0x60FF)
    sdo_msg.data[2] = 0x60;        // Índice (byte alto de 0x60FF)
    sdo_msg.data[3] = 0x00;        // Subíndice (0x00)
    sdo_msg.data[4] = (sensor_values.throttle & 0xFF);       // Byte 0 del valor
    sdo_msg.data[5] = (sensor_values.throttle >> 8) & 0xFF;  // Byte 1 del valor
    sdo_msg.data[6] = (sensor_values.throttle >> 16) & 0xFF; // Byte 2 del valor
    sdo_msg.data[7] = (sensor_values.throttle >> 24) & 0xFF; // Byte 3 del valor

    can_sendTxMsg(&sdo_msg);

    can_msg_t sdo_response;
    if (can_isNewRxMsg()) {
        if (can_readRxMsg(&sdo_response) && sdo_response.id == (0x580 + NODE_ID)) {
            if (sdo_response.data[0] == 0x60 && sdo_response.data[1] == 0xFF && sdo_response.data[2] == 0x60 && sdo_response.data[3] == 0x00) {
                PRINTF("Target Velocity set successfully.\n");
            } else {
                PRINTF("Error setting Target Velocity.\n");
            }
        }
    }


}


void update_nmt_state_machine(uint8_t node_id) {
    switch (current_state) {
        case STATE_POWER_ON_RESET:
            // Acción: Esperar al "Boot-Up" message
            gpioBlink(PIN_LED_RED);

            if (can_isNewRxMsg()) {
                can_msg_t rx_msg;
                if (can_readRxMsg(&rx_msg) && (rx_msg.id == (0x700 + node_id))) {
                    PRINTF("Boot-up message received.\n");
                    current_state = STATE_INITIALIZATION;
                }
            }
            break;

        case STATE_INITIALIZATION:
            // Acción: Enviar SDO request para leer el Object Dictionary (0x1018, subindex 0x01)
            can_msg_t sdo_request;
            sdo_request.id = 0x600 + node_id;  // SDO request
            sdo_request.len = 8;
            sdo_request.data[0] = 0x40;  // Comando de lectura SDO
            sdo_request.data[1] = 0x18;  // Índice (0x1018)
            sdo_request.data[2] = 0x10;
            sdo_request.data[3] = 0x01;  // Subíndice (0x01)
            memset(&sdo_request.data[4], 0x00, 4);  // Rellenar los bytes restantes con 0

            can_sendTxMsg(&sdo_request);
            gpioBlink(PIN_LED_RED);

            // Esperar la respuesta del esclavo
            if (can_isNewRxMsg()) {
                can_readRxMsg(&rx_msg);
                PRINTF("Received SDO response: %02X %02X %02X %02X\n",
                       rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
                current_state = STATE_WAIT_PRE_OPERATIONAL;
                PRINTF("Initialization Mode\n");
                gpioWrite(PIN_LED_RED, !LED_ACTIVE);
            }
            break;

        case STATE_WAIT_PRE_OPERATIONAL:
            gpioBlink(PIN_LED_RED);  // Initialization mode
            // Esperar a que el botón de PRE-OP sea presionado

            if (gpioRead(PRE_OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón
            {
                send_nmt_command(0x80, node_id);  // Enviar comando para Pre-operational
                current_state = STATE_PRE_OPERATIONAL;
                PRINTF("Pre-Operational Mode\n");
                gpioWrite(PIN_LED_RED, !LED_ACTIVE);
            }
            break;

        case STATE_PRE_OPERATIONAL:
            send_nmt_command(0x80, node_id);  // Enviar comando NMT para cambiar a Pre-operational

            // Configurar el modo de operación
            // Aquí seleccionas el modo: Torque (0x0A) o Velocidad (0x09)
            send_sdo_mode_of_operation(0x09);  // 0x0A: Torque, 0x09: Velocidad

            PRINTF("Pre-Operational Mode\n");

            current_state = STATE_WAIT_OPERATIONAL;
            break;

        case STATE_WAIT_OPERATIONAL:
            gpioBlink(PIN_LED_GREEN);  // Pre Operational mode

            if(gpioRead(OP_GPIO_PORT) == HIGH) // Esperar a que se presione el botón
            {
                current_state = STATE_OPERATIONAL;
                PRINTF("Operational Mode\n");
            }
            break;

        case STATE_OPERATIONAL:
            // Enviar comando NMT para pasar a Operational
            uint8_t msg[2] = {0x01, 0x00};
            send_can_message(0x000, msg, 2);  // Enviar comando NMT
            current_state = STATE_WAIT_DRIVE;
            break;

        case STATE_WAIT_DRIVE:
            gpioWrite(PIN_LED_GREEN, HIGH);
            PRINTF("Waiting for Drive Mode\n");

            if (gpioRead(DRIVE_GPIO_PORT) == HIGH)
            {
                PRINTF("Drive Mode\n");

                // Secuencia para habilitar el drive
                send_controlword(6);   // Enviar CW 6 para preparar el sistema
                send_controlword(15);  // Enviar CW 15 para habilitar la operación
                PRINTF("Control Word messages sent\n");

                current_state = STATE_DRIVE;
                gpioWrite(PIN_LED_GREEN, LOW);
                gpioWrite(PIN_LED_BLUE, HIGH);
            }
            break;

        case STATE_DRIVE:
            // Aquí puedes comenzar a enviar comandos de torque o velocidad
            run_sensors();
            run_motors();
            error_handler(NODE_ID_1);

            if (gpioRead(STOP_GPIO_PORT) == HIGH) {
                // Enviar comando NMT para pasar a STOP Mode
                gpioMode(PIN_LED_RED, HIGH);
                PRINTF("Stop Mode\n");
                current_state = STATE_STOPPED;
            }
            break;

        case STATE_STOPPED:
            send_nmt_command(0x80, node_id);  // Volver a Pre-operational
            current_state = STATE_PRE_OPERATIONAL;
            break;
    }
}

void error_handler(uint8_t node_id) {
	// Manejo de errores
	uint8_t rxBuffer[8];
	if (receive_can_message(0x080 + node_id, rxBuffer))
	{
		uint16_t error_code = rxBuffer[0] | (rxBuffer[1] << 8);
		PRINTF("Error Code: %d\n", error_code);
		if(error_code == 0x1000)
		{
			PRINTF("Generic error\n");
			current_state = STATE_STOPPED;
			send_nmt_command(0x02, node_id);
		}
		//Any other error that start with 0xFFxx
		else if (error_code >> 8 == 0xFF) {
			PRINTF("Hardware error\n");
			current_state = STATE_STOPPED;
			send_nmt_command(0x02, node_id);
		}
	}
}

void send_controlword(uint8_t controlword) {
    can_msg_t control_msg;
    control_msg.id = RPDO1_ID;  // RPDO1 para el control del driver
    control_msg.len = 8;        // Longitud de datos (8 bytes)

    // Configurar el Controlword
    control_msg.data[0] = controlword;  // Byte bajo del Controlword
    control_msg.data[1] = 0x00;         // Byte alto del Controlword (0 si no se usa)
    // Otros bytes pueden ser configurados según sea necesario para parámetros adicionales
    for (int i = 2; i < 8; i++) {
        control_msg.data[i] = 0x00;  // Rellenar los bytes restantes con ceros
    }

    // Enviar el mensaje CAN
    while (!can_isTxReady());
    can_sendTxMsg(&control_msg);
    PRINTF("Enviado Controlword 0x%02X al ID %03X\n", controlword, RPDO1_ID);
}

void send_sdo_mode_of_operation(uint8_t mode) {
    can_msg_t sdo_msg;
    sdo_msg.id = 0x600 + NODE_ID;  // SDO request (COB-ID 0x600 + Node ID)
    sdo_msg.len = 8;               // Longitud de datos (8 bytes)

    // Comando SDO para escribir el modo de operación (0x6060, subíndice 0)
    sdo_msg.data[0] = 0x2F;  // Comando SDO para escribir 1 byte
    sdo_msg.data[1] = 0x60;  // Índice bajo (0x6060)
    sdo_msg.data[2] = 0x60;  // Índice alto
    sdo_msg.data[3] = 0x00;  // Subíndice (0)
    sdo_msg.data[4] = mode;  // Valor del modo de operación
    sdo_msg.data[5] = 0x00;  // Rellenar con 0
    sdo_msg.data[6] = 0x00;
    sdo_msg.data[7] = 0x00;

    // Enviar el mensaje CAN
    while (!can_isTxReady());
    can_sendTxMsg(&sdo_msg);
    PRINTF("Enviado SDO para cambiar a modo 0x%02X\n", mode);

    // Esperar la confirmación del cambio de modo
    can_msg_t sdo_response;
    while (1) {
        if (can_isNewRxMsg()) {
            if (can_readRxMsg(&sdo_response)) {
                if (sdo_response.id == (0x580 + NODE_ID)) {
                    // Verificar si la respuesta es una confirmación exitosa
                    if (sdo_response.data[0] == 0x60 && sdo_response.data[1] == 0x60 && sdo_response.data[2] == 0x60) {
                        PRINTF("Cambio de modo confirmado: modo 0x%02X\n", mode);
                    } else {
                        PRINTF("Error en la confirmación del cambio de modo.\n");
                    }
                    break;
                }
            }
        }
    }
}

void check_for_emcy() {
    can_msg_t emcy_msg;

    // Monitorear mensajes EMCY
    if (can_isNewRxMsg()) {
        if (can_readRxMsg(&emcy_msg) && emcy_msg.id == EMCY_ID) {
            uint16_t error_code = (emcy_msg.data[1] << 8) | emcy_msg.data[0];
            uint8_t error_register = emcy_msg.data[2];

            PRINTF("Received EMCY message: Error Code: 0x%04X, Error Register: 0x%02X\n", error_code, error_register);

            // Aquí puedes analizar el código de error y registrar la advertencia
            if (error_register & 0x01) {  // Por ejemplo, si el bit 0x01 indica una advertencia específica
                PRINTF("Warning: Overcurrent detected!\n");
            }
        }
    }
}
