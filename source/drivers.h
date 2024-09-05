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

#define CAN_BAUDRATE			500000
#define NODE_ID_1					1
#define NODE_ID_2					11

#define NODE_ID 				NODE_ID_1

#define SYNC_MESSAGE_ID 		0x080
#define RPDO1_ID 				(0x200 + NODE_ID)
#define RPDO2_ID 				(0x300 + NODE_ID)
#define TPDO1_ID 				(0x280 + NODE_ID)
#define TPDO2_ID 				(0x300 + NODE_ID)
#define TPDO3_ID 				(0x380 + NODE_ID)

#define PRE_OP_GPIO_PORT 		PORTNUM2PIN(1, 21)
#define OP_GPIO_PORT 			PORTNUM2PIN(0, 28)
#define DRIVE_GPIO_PORT 		PORTNUM2PIN(0, 27)
#define STOP_GPIO_PORT 			PORTNUM2PIN(0, 14)





typedef enum {
    STATE_POWER_ON_RESET,
    STATE_INITIALIZATION,
	STATE_WAIT_PRE_OPERATIONAL,
    STATE_PRE_OPERATIONAL,
	STATE_WAIT_OPERATIONAL,
    STATE_OPERATIONAL,
	STATE_WAIT_DRIVE,
	STATE_DRIVE,
    STATE_STOPPED,
} nmt_state_t;


void send_nmt_command(uint8_t command, uint8_t node_id);
void run_motors (void);
void update_nmt_state_machine(uint8_t node_id);
void error_handler(uint8_t node_id);
void init_drivers(void);


extern nmt_state_t current_state;
extern can_msg_t rx_msg;




#endif /* DRIVERS_H_ */
