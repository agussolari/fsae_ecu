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
#define SYNC_ID 				0x080
#define BOOTUP_BASE_ID			0x700



#define SYNC_MESSAGE_ID 		0x080
#define RPDO1_ID 				(0x200)
#define RPDO2_ID 				(0x300)

#define TPDO1_ID 				(0x180)
#define TPDO2_ID 				(0x280)
#define TPDO3_ID 				(0x380)
#define TPDO4_ID 				(0x480)



#define SAVE_PARAM 				(uint32_t)(0x65766173)

#define MAX_VELOCITY			((int32_t)2500) //RPM
#define MAX_DC_CURRENT			((int32_t)50) //A



typedef enum {
	STATE_IDLE,
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
	STATE_ERROR,
} driver_state_t;

typedef enum {
	NMT_STATE_BOOTUP,
	NMT_STATE_PRE_OPERATIONAL,
	NMT_STATE_OPERATIONAL,
	NMT_STATE_DRIVE,
	NMT_STATE_STOPPED,
	NMT_STATE_ERROR,
} nmt_state_t;

typedef enum {
	ERROR_NONE,
	ERROR_GENERIC,
	ERROR_CURRENT,
	ERROR_VOLTAGE,
	ERROR_MOTOR_OVERTEMP,
	ERROR_CONTROLLER_OVERTEMP,
	ERROR_COMMUNICATION,
	ERROR_IMPLAUSIBILITY,
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

#include "common.h"
#include "leds.h"
#include <stdio.h>
#include <stdint.h>
#include "can.h"
#include "sensors.h"
#include "gpio.h"
#include "can_open.h"
#include "millis.h"
#include "uart.h"
#include "flash_wrp.h"
#include "buttons.h"




//Make union of TPDO1 data
typedef union {
	uint8_t b[8];
	struct {
		uint8_t motor_temperature;
		uint8_t controller_temperature;
		int16_t dc_link_voltage;
	} data;
} tpdo1_data_t;


//Make union of TPDO2 data
typedef union {
	uint8_t b[8];
	struct {
		int32_t actual_velocity;
		int32_t motor_rated_current;
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
		int32_t actual_velocity;
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
    uint16_t error_code;
    mode_t mode;

    uint32_t time_stamp;
    uint32_t error_time_stamp;


    bool align;
    bool calibration_needed;
    bool bootup_message_received;

    //TPS Data
    int16_t tps_value;
    uint16_t last_tps_value;
    uint32_t tps_time_stamp;
    uint32_t last_tps_time_stamp;


       //TPDO data
    tpdo1_data_t tpdo1_data;
    tpdo2_data_t tpdo2_data;
    tpdo3_data_t tpdo3_data;
    tpdo4_data_t tpdo4_data;

    	//RPDO data
    pdo1_data_t pdo1_data;
    pdo2_data_t pdo2_data;

} driver_t;

extern driver_t driver_1;
extern driver_t driver_2;
extern bool enable_read_data;


void init_drivers(driver_t* driver);
void boot_drivers(void);
void map_tpdo(driver_t *driver);
void send_sync_message(void);


void update_state_machine(driver_t* driver);

void recive_bootup_message(can_msg_t rx_msg);
void recive_pdo_message(can_msg_t rx_msg);
void recive_current_message(can_msg_t rx_msg);

void send_data_gui_uart(driver_t *driver);
void send_data_rf_uart(void);
void send_data_motec(void);




#endif /* DRIVERS_H_ */
