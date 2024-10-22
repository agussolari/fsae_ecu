/*
 * drivers.h
 *
 *  Created on: 22 ago 2024
 *      Author: asolari
 */

#ifndef DRIVERS_H_
#define DRIVERS_H_

#include <stdint.h>
#include "can.h"
#include "sensors.h"
#include "gpio.h"
#include "can_open.h"
#include "millis.h"

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

#define PRE_OP_GPIO_PORT 		PORTNUM2PIN(1, 21)
#define OP_GPIO_PORT 			PORTNUM2PIN(0, 28)
#define DRIVE_GPIO_PORT 		PORTNUM2PIN(0, 27)
#define STOP_GPIO_PORT 			PORTNUM2PIN(0, 14)

#define SAVE_PARAM 				(uint32_t)(0x65766173)





typedef enum {
    STATE_POWER_ON_RESET,
    STATE_INITIALIZATION,
	STATE_ALIGN_MOTORS,
	STATE_WAIT_ALIGN_MOTORS,
	STATE_ALIGNING_MOTORS,
	STATE_WAIT_PRE_OPERATIONAL,
    STATE_PRE_OPERATIONAL,
	STATE_WAIT_OPERATIONAL,
    STATE_OPERATIONAL,
	STATE_WAIT_DRIVE,
	STATE_DRIVE,
    STATE_STOPPED,
} nmt_state_t;

//Object of the driver
typedef struct {
	uint16_t node_id;
	nmt_state_t state;
	uint16_t error_code;
	uint32_t time_stamp;
	bool align;
	uint8_t tpdo1_data[8];
	uint8_t tpdo2_data[8];
	uint8_t tpdo3_data[8];
	uint8_t tpdo4_data[8];
} driver_t;


void init_drivers(driver_t* driver);
void update_state_machine(driver_t* driver);


#endif /* DRIVERS_H_ */
