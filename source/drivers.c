/*
 * drivers.c
 *
 *  Created on: 22 ago 2024
 *      Author: asolari
 */
#include "drivers.h"

driver_t driver_1;
driver_t driver_2;
bool enable_read_data = false;

static bool flag = false;
static bool button_pressed_calibration = false;
static bool button_pressed_stop = false;




void run_motors (driver_t* driver);
void send_pdo_message(driver_t* driver);
bool send_controlword(uint8_t controlword, uint16_t node_id);
bool send_sdo_mode_of_operation(int8_t mode, uint16_t node_id);
void handle_errors(driver_t *driver);
void set_calibration_1(void);
void set_calibration_2(void);


/**
 * @brief Initialize the drivers
 * @return void
 */
void init_drivers(driver_t* driver)
{
	driver->time_stamp = 0;
	driver->error_time_stamp = 0;
    driver->mode = MODE_TORQUE;

    driver->align = true;
	driver->calibration_needed = true;
	driver->bootup_message_received = false;

    driver->state = STATE_POWER_ON_RESET;
    driver->nmt_state = NMT_STATE_BOOTUP;



	//Clean data of the PDO and TPDO
	for (int i = 0; i < 8; i++) {
		driver->tpdo1_data.b[i] = 0;
		driver->tpdo2_data.b[i] = 0;
		driver->tpdo3_data.b[i] = 0;
		driver->tpdo4_data.b[i] = 0;
		driver->pdo1_data.b[i] = 0;
		driver->pdo2_data.b[i] = 0;
	}

}

void recive_bootup_message(can_msg_t rx_msg)
{
	if ((rx_msg.id == (0x700 + NODE_ID_1) && rx_msg.data[0] == 0x00) && (driver_1.bootup_message_received == false))
	{
		uartWriteStr("Boot-up message received from Node 1\n");
		// Initialize the node
		if (send_sdo_write_command(0x40, 0x1810, 0x01, 0x00000000,NODE_ID_1))
		{
			uartWriteStr("Init SDO command sent from Node 1\n");
			driver_1.state = STATE_WAIT_START;
			driver_1.nmt_state = NMT_STATE_BOOTUP;
			driver_1.bootup_message_received = true;
		}
	}
	if ((rx_msg.id == (0x700 + NODE_ID_2) && rx_msg.data[0] == 0x00) && (driver_2.bootup_message_received == false))
	{
		uartWriteStr("Boot-up message received from Node 2\n");
		// Initialize the node
		if (send_sdo_write_command(0x40, 0x1810, 0x01, 0x00000000, NODE_ID_2))
		{
			uartWriteStr("Init SDO command sent from Node 2\n");
			driver_2.state = STATE_WAIT_START;
			driver_2.nmt_state = NMT_STATE_BOOTUP;
			driver_2.bootup_message_received = true;
		}
	}
}



void boot_drivers(void)
{
	// Send the NMT command to reset the nodes
	send_nmt_command(NMT_CMD_RESET_NODE, NODE_ID_1);
	send_nmt_command(NMT_CMD_RESET_NODE, NODE_ID_2);

	while (1)
	{
		if (driver_1.state == STATE_WAIT_START
				&& driver_2.state == STATE_WAIT_START)
		{
			// Both nodes are ready
			driver_1.nmt_state = NMT_STATE_PRE_OPERATIONAL;
			driver_2.nmt_state = NMT_STATE_PRE_OPERATIONAL;


			map_tpdo(&driver_1);
			map_tpdo(&driver_2);

			uartWriteStr("Boot-up complete\n");

			break;
		}

		// Check for the boot-up message
		if(stop_button_pressed())
		{
			// Send again the reset command
			uartWriteStr("Resetting nodes\n");

			send_nmt_command(NMT_CMD_RESET_NODE, NODE_ID_1);
			send_nmt_command(NMT_CMD_RESET_NODE, NODE_ID_2);
		}

		// CALIBRATION MODE IF START AND DRIVE BUTTONS ARE PRESSED
		if(start_button_pressed() && drive_button_pressed())
		{
			driver_1.state = STATE_CALIBRATION_1;
			driver_2.state = STATE_IDLE;

			// Wait for calibration step 1 to complete
			while (driver_1.state == STATE_CALIBRATION_1)
			{
				if (start_button_pressed() && drive_button_pressed())
				{
					set_calibration_1();
					driver_1.state = STATE_CALIBRATION_2;
					driver_2.state = STATE_IDLE;
				}
			}

			// Wait for calibration step 2 to complete
			while (driver_1.state == STATE_CALIBRATION_2)
			{
				if (start_button_pressed() && drive_button_pressed())
				{
					set_calibration_2();
				    driver_1.nmt_state = NMT_STATE_BOOTUP;
				    driver_2.nmt_state = NMT_STATE_BOOTUP;
				}
			}
		}
	}
}


/**
 * @brief Update the state machine
 * @param node_id Node ID
 * @return void
 */
void update_state_machine(driver_t* driver)
{

    switch (driver->state) {

        case STATE_WAIT_START:
            if (start_button_pressed())
            {
				uartWriteStr("Start Mode\n");
				driver->state = STATE_START;
				break;
            }
			if (calibration_button_pressed())
			{
				uartWriteStr("Calibration Mode\n");
				driver_1.state = STATE_CALIBRATION_1;
				driver_2.state = STATE_IDLE;
				break;
			}
			break;

        case STATE_CALIBRATION_1:
            if (calibration_button_pressed())
            {
            	set_calibration_1();
            	break;
            }
            if (stop_button_pressed())
            {
            	driver->state = STATE_WAIT_START;
            	break;
            }
            break;

        case STATE_CALIBRATION_2:
			if (calibration_button_pressed())
			{
				set_calibration_2();
				break;
			}
            if (stop_button_pressed())
            {
            	driver->state = STATE_WAIT_START;
            	break;
            }
			break;




        case STATE_START:
        		//Inicialmente se inicia en modo de torque
            	driver->mode = MODE_TORQUE;
            	uartWriteStr("Mode of operation set to MODE_TORQUE\n");

				uartWriteStr("Operational Mode\n");
				send_nmt_command(NMT_CMD_ENTER_OPERATIONAL, driver->node_id);
				driver->nmt_state = NMT_STATE_OPERATIONAL;
                driver->state = STATE_WAIT_DRIVE;

                //Enable reading data with TPDO messages
    			enable_read_data = true;


            break;



        case STATE_WAIT_DRIVE:
            if (drive_button_pressed())
            {
                uartWriteStr("Drive Mode\n");

                send_controlword(0x06, driver->node_id);
                send_controlword(0x07, driver->node_id);
                send_controlword(0x0F, driver->node_id);



                driver->state = STATE_DRIVE;
                driver->nmt_state = NMT_STATE_DRIVE;
            }
            if (stop_button_pressed())
            {
                driver->state = STATE_STOPPED;
                break;
            }
            break;

        case STATE_DRIVE:
            run_motors(driver);
//            handle_errors(driver);

            if (stop_button_pressed())
            {
                driver->state = STATE_STOPPED;
                break;
            }

            break;

        case STATE_STOPPED:
            uartWriteStr("Stop Mode\n");
			send_nmt_command(NMT_CMD_ENTER_PRE_OPERATIONAL, driver->node_id);
			enable_read_data = false;

            driver->nmt_state = NMT_STATE_PRE_OPERATIONAL;
            driver->state = STATE_WAIT_START;

            break;

        case STATE_ERROR:
            // delay for 5 seconds non blocking
            if ((driver->time_stamp - driver->error_time_stamp) >= 5000)
            {
            	driver->error_time_stamp = driver->time_stamp;
    			enable_read_data = false;

            	driver->state = STATE_WAIT_START;
            }
            break;
    }

}










/**
 * @brief Run the motors
 * void run_motors (uint16_t node_id)
 *
 * @return void
 */
void run_motors (driver_t* driver)
{
	send_pdo_message(driver);		//Send PDO message
}





/**
 * @brief Receive a PDO message
 * void recive_pdo_message(uint8_t node_id)
 *
 * save the received PDO message in the driver_data  structure
 *
 * @return void
 */
void recive_pdo_message(can_msg_t rx_msg)
{
		if (rx_msg.id == (TPDO1_ID + NODE_ID_1))
		{
			for (int i = 0; i < 8; i++)
				driver_1.tpdo1_data.b[i] = rx_msg.data[i];
		}
		if (rx_msg.id == (TPDO2_ID + NODE_ID_1)) {
			for (int i = 0; i < 8; i++)
				driver_1.tpdo2_data.b[i] = rx_msg.data[i];
		}
		if (rx_msg.id == (TPDO3_ID + NODE_ID_1)) {
			for (int i = 0; i < 8; i++)
				driver_1.tpdo3_data.b[i] = rx_msg.data[i];
		}
		if (rx_msg.id == (TPDO4_ID + NODE_ID_1)) {
			for (int i = 0; i < 8; i++)
				driver_1.tpdo4_data.b[i] = rx_msg.data[i];
		}
		if (rx_msg.id == (TPDO1_ID + NODE_ID_2)) {
			for (int i = 0; i < 8; i++)
				driver_2.tpdo1_data.b[i] = rx_msg.data[i];
		}
		if (rx_msg.id == (TPDO2_ID + NODE_ID_2)) {
			for (int i = 0; i < 8; i++)
				driver_2.tpdo2_data.b[i] = rx_msg.data[i];
		}
		if (rx_msg.id == (TPDO3_ID + NODE_ID_2)) {
			for (int i = 0; i < 8; i++)
				driver_2.tpdo3_data.b[i] = rx_msg.data[i];
		}
		if (rx_msg.id == (TPDO4_ID + NODE_ID_2)) {
			for (int i = 0; i < 8; i++)
				driver_2.tpdo4_data.b[i] = rx_msg.data[i];
		}

}

void recive_current_message(can_msg_t rx_msg)
{
	if (rx_msg.id == CURRENT_NODE_ID)
	{
		//Convert int16_t to float
		int16_t ac_current_n1 = (int16_t) (rx_msg.data[0] | rx_msg.data[1] << 8);
		int16_t ac_current_n2 = (int16_t) (rx_msg.data[2] | rx_msg.data[3] << 8);
		int16_t dc_current_n1 = (int16_t) (rx_msg.data[4] | rx_msg.data[5] << 8);
		int16_t dc_current_n2 = (int16_t) (rx_msg.data[6] | rx_msg.data[7] << 8);



		//Save the current values
		current_sense_data.ac_current_n1 = ((float)ac_current_n1)/100.0;
		current_sense_data.ac_current_n2 = ((float)ac_current_n2)/100.0;
		current_sense_data.dc_current_n1 = ((float)dc_current_n1)/100.0;
		current_sense_data.dc_current_n2 = ((float)dc_current_n2)/100.0;
	}
}

void calculate_tps(driver_t *driver, uint16_t sensor_value) {
    // Constants

}


#define TPS_THRESHOLD 10
#define TPS_INTERVAL_WAIT 100
#define TPS_INTERVAL_RUN 10


void send_pdo_message(driver_t *driver)
{
	uint32_t interval = TPS_INTERVAL_WAIT; // Default interval is 100ms

	// Read the TPS values
	float sensor_value = (float)(tps_data.tps1_value + tps_data.tps2_value)/(2.0f);
	float velocity_value = (float) abs(driver->tpdo2_data.data.actual_velocity);

	// Calculate the factor
	float Mmin = (MAX_DC_CURRENT)/((float)(driver->tpdo2_data.data.motor_rated_current)/1000.0f);

	float M = Mmin;

	if(sensor_value > 10.0)
	{
		M  = Mmin *(1000.0f / sensor_value);
	}
	else
	{
		M = 100.0;
	}

	if(velocity_value > M * MAX_VELOCITY)
	{
		//tps = Mmin * 1000.0 * (VEL_MAX / velocity)
		driver->tps_value = (uint16_t)((Mmin * 1000.0f * MAX_VELOCITY) / velocity_value);
	}
	else
	{
		driver->tps_value = (uint16_t)sensor_value;
	}


	if(driver->tps_value < 0)
	{
		driver->tps_value = 0;
	}


	driver->tps_time_stamp = millis();



	// Check for TPS variation
	if (abs(driver->tps_value - driver->last_tps_value) > TPS_THRESHOLD) {
		interval = TPS_INTERVAL_RUN; // If accelerating, set interval to 25ms
		driver->last_tps_value = driver->tps_value;
	}

	// Check if the required interval has passed
	if (driver->tps_time_stamp - driver->last_tps_time_stamp >= interval)
	{
		driver->last_tps_time_stamp = driver->tps_time_stamp;

	    int32_t current_torque = 0;
	    int32_t current_throttle = 0;

		if (driver->node_id == NODE_ID_1)
		{
			current_torque = (int32_t) (-1)*(driver->tps_value);
		} else if (driver->node_id == NODE_ID_2)
		{
			current_torque = -(int32_t) (driver->tps_value);
		}

		driver->pdo1_data.data.control_word = 0x0F | driver->node_id << 8;
		driver->pdo1_data.data.target_velocity = current_throttle;
		driver->pdo1_data.data.target_torque = current_torque;

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

		if (can_isTxReady())
		{
			can_sendTxMsg(&pdo_msg);
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

bool send_controlword(uint8_t controlword, uint16_t node_id)
{

    can_msg_t control_msg;
    control_msg.id = RPDO1_ID + node_id;  // RPDO1 para el control del driver
    control_msg.len = 8;        // Longitud de datos (8 bytes)

    // Configurar el Controlword
    control_msg.data[0] = controlword;  // Byte bajo del Controlword
    control_msg.data[1] = 0x00;         // Byte alto del Controlword (0 si no se usa)
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
//    int i = 1000000;
//    while(i--);
	uartWriteStr("Controlword sent\n");
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
			//Use NMT command to Reset Node
//			send_nmt_command(0x81, node_id); //Reset Node command
			return 1;

		}
	}

}



void send_data_gui_uart(driver_t *driver)
{
	char buffer[64];

	int len = snprintf(buffer, sizeof(buffer),
			":%d;%d;%d;%d;%d:",

			driver->node_id,
			driver->state,
			driver->nmt_state,
			driver->mode,
			driver->error_code);

//			driver->pdo1_data.data.target_torque,
//			driver->pdo1_data.data.target_velocity,
//
//			driver->tpdo1_data.data.actual_torque,
//
//			driver->tpdo2_data.data.controller_temperature,
//			driver->tpdo2_data.data.current_demand,
//
//			driver->tpdo4_data.data.torque_regulator,
//			driver->tpdo4_data.data.actual_velocity,
//			driver->tpdo4_data.data.motor_temperature,
//
//
//			tps_data.tps1_value, tps_data.tps2_value,
//
//			driver->tps_value,
//
//			front_break_data.brake_value, rear_break_data.brake_value,
//
//			direction_data.direction_value);

	uartWriteMsg(buffer, len);



}

void send_data_rf_uart(void) {

	// Send the data over UART
	uint32_t start = 0xFFFFFFFF;
	uartWriteMsg((uint8_t*) &start, sizeof(start));


    uartWriteMsg((uint8_t*)&current_sense_data.ac_current_n1, sizeof(float));
    uartWriteMsg((uint8_t*)&current_sense_data.ac_current_n2, sizeof(float));
    uartWriteMsg((uint8_t*)&current_sense_data.dc_current_n1, sizeof(float));
    uartWriteMsg((uint8_t*)&current_sense_data.dc_current_n2, sizeof(float));


    float temp;

    temp = (float)tps_data.tps1_value;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)tps_data.tps2_value;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)front_break_data.brake_value;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)rear_break_data.brake_value;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)direction_data.direction_value;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)driver_1.tpdo2_data.data.actual_velocity;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)driver_2.tpdo2_data.data.actual_velocity;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)driver_1.tps_value;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)driver_2.tps_value;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));


    temp = (float)driver_1.tpdo1_data.data.motor_temperature;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)driver_2.tpdo1_data.data.motor_temperature;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    //DC Link voltage
    temp = (float)driver_1.tpdo1_data.data.dc_link_voltage;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));

    temp = (float)driver_2.tpdo1_data.data.dc_link_voltage;
    uartWriteMsg((uint8_t*)&temp, sizeof(float));




}


void handle_errors(driver_t *driver)
{
	// Check for errors
	if(check_implausibility_tps())
	{
		driver->error_code = ERROR_IMPLAUSIBILITY;
		driver->error_time_stamp = driver->time_stamp;
		driver->state = STATE_ERROR;
		driver->nmt_state = NMT_STATE_ERROR;
		send_nmt_command(NMT_CMD_ENTER_PRE_OPERATIONAL, driver->node_id);
	}
//	else if (driver->tpdo2_data.data.controller_temperature > 70)
//	{
//		driver->error_code = ERROR_CONTROLLER_OVERTEMP;
//	}
//	else if (driver->tpdo4_data.data.motor_temperature > 70)
//	{
//		driver->error_code = ERROR_MOTOR_OVERTEMP;
//	}
//	else if (driver->tpdo2_data.data.current_demand > 100)
//	{
//		driver->error_code = ERROR_CURRENT;
//	}
	else
	{
		driver->error_code = ERROR_NONE;
	}

}

void set_calibration_1(void)
{
    // Read the TPS values
    uint16_t tps1_min = adcReadChannelBlocking(ADC_CHANNEL_TPS1);
    uint16_t tps2_min = adcReadChannelBlocking(ADC_CHANNEL_TPS2);

    uint16_t front_brake = adcReadChannelBlocking(ADC_CHANNEL_FRONT_BRAKE);
    uint16_t rear_brake = adcReadChannelBlocking(ADC_CHANNEL_REAR_BRAKE);

    uint16_t direction = adcReadChannelBlocking(ADC_CHANNEL_DIRECTION);


	tps_data.tps1_min_value = tps1_min;
	tps_data.tps2_min_value = tps2_min;
	uartWriteStr("TPS 0 value saved\n");

	//Save the calibration values in flash memory
	front_break_data.calibration_break_value = ADC_0_5V_VALUE - front_brake;
	rear_break_data.calibration_break_value = ADC_0_5V_VALUE - rear_brake;
	uartWriteStr("Brake calibration value saved Front\n");

	direction_data.calibration_direction_value = direction;
	uartWriteStr("Direction calibration value saved\n");

	driver_1.state = STATE_CALIBRATION_2;
	driver_2.state = STATE_IDLE;
}

void set_calibration_2(void)
{
	// Read the TPS values
	uint16_t tps1_max = adcReadChannelBlocking(ADC_CHANNEL_TPS1);
	uint16_t tps2_max = adcReadChannelBlocking(ADC_CHANNEL_TPS2);
	tps_data.tps1_max_value = tps1_max;
	tps_data.tps2_max_value = tps2_max;

	uartWriteStr("TPS 100 value saved\n");



	//Save the calibration values in flash memory
	uint32_t data[BUFFER_LEN];
	data[0] = (uint32_t)tps_data.tps1_min_value;
	data[1] = (uint32_t)tps_data.tps1_max_value;
	data[2] = (uint32_t)tps_data.tps2_min_value;
	data[3] = (uint32_t)tps_data.tps2_max_value;

	data[4] = (uint32_t)front_break_data.calibration_break_value;
	data[5] = (uint32_t)rear_break_data.calibration_break_value;

	data[6] = (uint32_t)direction_data.calibration_direction_value;

	program_flash(data, sizeof(data));

	driver_1.calibration_needed = false;
	driver_2.calibration_needed = false;

	uartWriteStr("Calibration complete\n");

	if(driver_1.nmt_state == NMT_STATE_BOOTUP)
	{
		driver_1.state = STATE_POWER_ON_RESET;
		driver_2.state = STATE_POWER_ON_RESET;
	}
	else
	{
		driver_1.state = STATE_WAIT_START;
		driver_2.state = STATE_WAIT_START;
	}

}

void map_tpdo(driver_t *driver)
{
	//TPD1
	//Map de number of entries = 3
	send_sdo_write_command(0x2F, 0x1A00, 0x00, 3 , driver->node_id);
	//Map the entries
	//Motor Temperature 0x2025 0x00 8 bits = 08 hexa
	// 0x20250008 = 539295752dec
	send_sdo_write_command(0x23, 0x1A00, 0x01, 539295752 , driver->node_id);
	//Controller Temperature 0x2026 0x01 8 bits = 08 hexa
	// 0x20260108 = 539361544dec
	send_sdo_write_command(0x23, 0x1A00, 0x02, 539361544 , driver->node_id);
	//DC Link voltage 0x6079 0x00 16 bits = 10 hexa
	// 0x60790010 = 1618542608
	send_sdo_write_command(0x23, 0x1A00, 0x03, 1618542608 , driver->node_id);

	//Map the COB ID
	send_sdo_write_command(0x2B, 0x1800, 0x01, TPDO1_ID + driver->node_id , driver->node_id);
	//Map the Transmission type : Acyclic synchronous transmission
	send_sdo_write_command(0x2F, 0x1800, 0x02, 1 , driver->node_id);

	//Generate a SDO Write to save parameters
	send_sdo_write_command(0x23, 0x1010, 0x01, SAVE_PARAM, driver->node_id);



	//TPDO2
	//Map de number of entries = 2
	send_sdo_write_command(0x2F, 0x1A01, 0x00, 2 , driver->node_id);

	//Map the entries
	// Velocity 0x606C 0x00 32 bits = 20 hexa
	// 0x606C0020 = 1617690656
	send_sdo_write_command(0x23, 0x1A01, 0x01, 1617690656 , driver->node_id);
	// Motor Rated Current 0x6075 0x00 32 bits = 20 hexa
	// 0x60750020 = 1618280480
	send_sdo_write_command(0x23, 0x1A01, 0x02, 1618280480 , driver->node_id);

	//Map the COB ID
	send_sdo_write_command(0x2B, 0x1801, 0x01, TPDO2_ID + driver->node_id , driver->node_id);
	//Map the Transmission type : Acyclic synchronous transmission
	send_sdo_write_command(0x2F, 0x1801, 0x02, 1 , driver->node_id);

	//Generate a SDO Write to save parameters
	send_sdo_write_command(0x23, 0x1010, 0x01, SAVE_PARAM, driver->node_id);

}

void send_sync_message(void)
{
	if(enable_read_data)
	{
		can_msg_t sync_msg;
		sync_msg.id = SYNC_ID;
		sync_msg.len = 1;

		sync_msg.data[0] = 0x00;
		sync_msg.data[1] = 0x00;
		sync_msg.data[2] = 0x00;
		sync_msg.data[3] = 0x00;
		sync_msg.data[4] = 0x00;
		sync_msg.data[5] = 0x00;
		sync_msg.data[6] = 0x00;
		sync_msg.data[7] = 0x00;

		can_sendTxMsg(&sync_msg);
	}
}


void send_data_motec(void)
{

	can_msg_t msg;

	if (can_isTxReady())
	{
		//SEND SENSOR DATA
		// 0x501
		//b[0], b[1]: TPS Value
		//b[2], b[3]: Front Brake Value
		//b[4], b[5]: Rear Brake Value
		//b[6], b[7]: Direction Value

		msg.id = 0x501;

		msg.data[0] = (uint8_t)(sensor_values.tps_value >> 8);
		msg.data[1] = (uint8_t)(sensor_values.tps_value);

		msg.data[2] = (uint8_t)(front_break_data.brake_value >> 8);
		msg.data[3] = (uint8_t)(front_break_data.brake_value);

		msg.data[4] = (uint8_t)(rear_break_data.brake_value >> 8);
		msg.data[5] = (uint8_t)(rear_break_data.brake_value);

		int16_t direction = (int16_t)(100.0f * direction_data.direction_value);
		msg.data[6] = (uint8_t)(direction >> 8);
		msg.data[7] = (uint8_t)(direction);

		msg.len = 8;

		can_sendTxMsg(&msg);
	}



	if (can_isTxReady())
	{
		//SEND VELOCITY AND TEMPERATURE DATA
		// 0x502
		// b[0], b[1]: Driver 1 Velocity
		// b[2], b[3]: Driver 2 Velocity
		// b[4], b[5]: Motor Temp Driver 1
		// b[6], b[7]: Motor Temp Driver 2

		msg.id = 0x502;

		int32_t vel_1 = (int32_t)((-1)*(driver_1.tpdo2_data.data.actual_velocity)/60);
		msg.data[0] = (uint8_t) (vel_1 >> 8);
		msg.data[1] = (uint8_t) (vel_1);

		int32_t vel_2 = (int32_t)((-1)*(driver_2.tpdo2_data.data.actual_velocity)/60);
		msg.data[2] = (uint8_t) (vel_2 >> 8);
		msg.data[3] = (uint8_t) (vel_2);


		msg.data[4] = (uint8_t) (driver_1.tpdo1_data.data.motor_temperature >> 8);
		msg.data[5] = (uint8_t) (driver_1.tpdo1_data.data.motor_temperature);

		msg.data[6] = (uint8_t) (driver_2.tpdo1_data.data.motor_temperature >> 8);
		msg.data[7] = (uint8_t) (driver_2.tpdo1_data.data.motor_temperature);

		msg.len = 8;

		can_sendTxMsg(&msg);
	}

	if (can_isTxReady())
	{
		//SEND DC LINK VOLTAGE AND DRIVER TEMP DATA
		// 0x503
		// b[0], b[1]: Driver 1 DC Link Voltage
		// b[2], b[3]: Temperature Driver 1
		// b[4], b[5]: Temperature Driver 2

		msg.id = 0x503;

		msg.data[0] = (uint8_t) (driver_1.tpdo1_data.data.dc_link_voltage >> 8);
		msg.data[1] = (uint8_t) (driver_1.tpdo1_data.data.dc_link_voltage);

		msg.data[2] = (uint8_t) (driver_1.tpdo1_data.data.controller_temperature >> 8);
		msg.data[3] = (uint8_t) (driver_1.tpdo1_data.data.controller_temperature);

		msg.data[4] = (uint8_t) (driver_2.tpdo1_data.data.controller_temperature >> 8);
		msg.data[5] = (uint8_t) (driver_2.tpdo1_data.data.controller_temperature);

		msg.data[6] = 0x00;
		msg.data[7] = 0x00;

		msg.len = 8;

		can_sendTxMsg(&msg);


	}

	//CURRENT DATA
	if (can_isTxReady())
	{
		//SEND CURRENT DATA
		// 0x504
		// b[0], b[1]: AC Current Driver 1
		// b[2], b[3]: AC Current Driver 2
		// b[4], b[5]: DC Current Driver 1
		// b[6], b[7]: DC Current Driver 2

		msg.id = 0x504;

		uint16_t temp = (uint16_t)((10.0)*current_sense_data.ac_current_n1);
		msg.data[0] = (uint8_t) (temp >> 8);
		msg.data[1] = (uint8_t) (temp);

		temp = (uint16_t)((10.0)*current_sense_data.ac_current_n2);
		msg.data[2] = (uint8_t) (temp >> 8);
		msg.data[3] = (uint8_t) (temp);

		temp = (uint16_t)((-10.0)*current_sense_data.dc_current_n1);
		msg.data[4] = (uint8_t) (temp >> 8);
		msg.data[5] = (uint8_t) (temp);

		temp = (uint16_t)((-10.0)*current_sense_data.dc_current_n2);
		msg.data[6] = (uint8_t) (temp >> 8);
		msg.data[7] = (uint8_t) (temp);


		msg.len = 8;

		can_sendTxMsg(&msg);
	}
}










