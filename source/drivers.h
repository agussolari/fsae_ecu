/*
 * drivers.h
 *
 *  Created on: 22 ago 2024
 *      Author: asolari
 */

#ifndef DRIVERS_H_
#define DRIVERS_H_




#define CAN_BAUDRATE			500000
#define NODE_ID_1					1
#define NODE_ID_2					2
#define BASE_NODE 				0x100


#define SYNC_MESSAGE_ID 		0x080
#define RPDO1_ID 				(0x200)
#define RPDO2_ID 				(0x300)

#define TPDO1_ID 				(0x180)
#define TPDO2_ID 				(0x280)
#define TPDO3_ID 				(0x380)
#define TPDO4_ID 				(0x480)



#define SAVE_PARAM 				(uint32_t)(0x65766173)



typedef enum {
	STATE_RESET_NODE,
    STATE_POWER_ON_RESET,
    STATE_INITIALIZATION,
	STATE_ALIGN_MOTORS,
	STATE_WAIT_ALIGN_MOTORS,
	STATE_ALIGNING_MOTORS,
	STATE_CALIBRATION_1,
	STATE_CALIBRATION_2,
	STATE_WAIT_START,
    STATE_START,
	STATE_WAIT_DRIVE,
    STATE_DRIVE,
    STATE_STOPPED,
} driver_state_t;

typedef enum {
	NMT_STATE_BOOTUP,
	NMT_STATE_PRE_OPERATIONAL,
	NMT_STATE_OPERATIONAL,
	NMT_STATE_DRIVE,
	NMT_STATE_STOPPED,
} nmt_state_t;

typedef enum {
	ERROR_NONE,
	ERROR_GENERIC,
	ERROR_CURRENT,
	ERROR_VOLTAGE,
	ERROR_TEMPERATURE,
	ERROR_COMMUNICATION,
	ERROR_DEVICE,
	ERROR_SOFTWARE,
	ERROR_INTERNAL,
	ERROR_USER,
	ERROR_OTHER,
} nmt_error_t;

typedef enum {
	NMT_CMD_ENTER_OPERATIONAL = 0x01,
	NMT_CMD_STOP_REMOTE_NODE = 0x02,
	NMT_CMD_ENTER_PRE_OPERATIONAL = 0x80,
	NMT_CMD_RESET_NODE = 0x81,
	NMT_CMD_RESET_COMMUNICATION = 0x82,
} nmt_cmd_t;

//Mode of operation
typedef enum {
	MODE_VELOCITY = 0x09,
	MODE_TORQUE = 0x0A,
	MODE_ALIGNMENT = -4
} mode_t;

#include "leds.h"
#include <stdint.h>
#include "can.h"
#include "sensors.h"
#include "gpio.h"
#include "can_open.h"
#include "millis.h"
#include "uart.h"
#include "flash_wrp.h"



//Make union of TPDO1 data
typedef union {
	uint8_t b[8];
	struct {
		uint16_t status_word;
		int32_t actual_position;
		int16_t actual_torque;
	} data;
} tpdo1_data_t;


//Make union of TPDO2 data
typedef union {
	uint8_t b[8];
	struct {
		uint32_t actual_velocity;
		uint8_t controller_temperature;
		uint16_t current_demand;
		uint8_t resv_1;
	} data;
} tpdo2_data_t;

//Make union of TPDO3 data
typedef union {
	uint8_t b[8];
	struct {
		uint16_t motor_current_actual_value;
		uint16_t electrical_angle;
		uint8_t phase_a_current;
		uint8_t phase_b_current;
		uint16_t resv_2;
	} data;
} tpdo3_data_t;

//Make union of TPDO4 data
typedef union {
	uint8_t b[8];
	struct {
		uint16_t torque_regulator;
		uint32_t actual_velocity;
		uint8_t resv_3;
		uint8_t motor_temperature;
	} data;
} tpdo4_data_t;

//Make union of PDO1 data
typedef union {
	uint8_t b[8];
	struct {
		uint32_t control_word;
		int32_t target_velocity;
		int16_t target_torque;
	} data;
} pdo1_data_t;

//Make union of PDO2 data
typedef union {
	uint8_t b[8];
	struct {
		uint32_t target_position;
		uint8_t res_1;
		uint8_t res_2;
		uint8_t res_3;
		uint8_t res_4;
	} data;
} pdo2_data_t;


//Object of the driver
typedef struct {
    uint16_t node_id;
    driver_state_t state;
    nmt_state_t nmt_state;
    mode_t mode;

    uint16_t error_code;

    	//STATES
    bool align;
    bool calibration_needed;


       //TPDO data
    tpdo1_data_t tpdo1_data;
    tpdo2_data_t tpdo2_data;
    tpdo3_data_t tpdo3_data;
    tpdo4_data_t tpdo4_data;

    	//RPDO data
    pdo1_data_t pdo1_data;
    pdo2_data_t pdo2_data;

    	//TPS
    uint32_t tps_time_stamp;
    uint32_t last_tps_time_stamp;
    uint32_t tps_value;
    uint32_t last_tps_value;


} driver_t;




void init_drivers(driver_t* driver);
void update_state_machine(driver_t* driver);
void send_motor_data_uart(driver_t* driver);


#endif /* DRIVERS_H_ */
