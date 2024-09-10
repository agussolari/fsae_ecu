/*
 * can_open.h
 *
 *  Created on: 9 sept 2024
 *      Author: asolari
 */

#ifndef CAN_OPEN_H_
#define CAN_OPEN_H_

#include "can.h"
////////////////////////////////////////////////////
//                 SDO WRITE                      //
////////////////////////////////////////////////////

/**
 * @brief Send a CAN message
 * void send_sdo_command (uint8_t command, uint16_t od_index,
 * uint8_t od_sub_index, uint32_t data, uint8_t node_id)
 *
 * @param id CAN ID
 * @param data Pointer to the data
 * @param len Length of the data
 * @param command
 *       0x23: 4 bytes sent
 *       0x27: 3 bytes sent
 *       0x2B: 2 bytes sent
 *       0x2F: 1 byte sent
 * @return 1 = Send success
 */
bool send_sdo_write_command (uint8_t command, uint16_t od_index, uint8_t od_sub_index, uint32_t data, uint16_t node_id);

/**
 * @brief Receive a SDO Write message
 *	bool recive_sdo_command(uint8_t command, uint16_t od_index,
 *	 uint8_t od_sub_index, uint32_t data, uint8_t node_id)
 *
 * 	@return 1 = Recive success
 */
bool recive_sdo_write_command(uint8_t command, uint16_t od_index, uint8_t od_sub_index, uint32_t data, uint16_t node_id);


////////////////////////////////////////////////////
//                 SDO READ                       //
////////////////////////////////////////////////////
/**
 * @brief Send a SDO Read command
 * bool send_sdo_command(uint8_t command, uint16_t od_index,
 * uint8_t od_sub_index, uint32_t data, uint8_t node_id)
 *
 * @return bool : 1 = send success
 */
bool send_sdo_read_command(uint16_t od_index, uint8_t od_sub_index, uint16_t node_id);


/**
 * @brief Receive a SDO Read message
 *	bool recive_sdo_command(uint8_t command, uint16_t od_index,
 *	 uint8_t od_sub_index, uint32_t data, uint8_t node_id)
 *
 *	 0x43 : 4 bytes sent
 *	 0x47 : 3 bytes sent
 *	 0x4B : 2 bytes sent
 *	 0x4F : 1 byte sent
 *
 * 	@return data : 4 bytes
 */
int32_t recive_sdo_read_command(uint8_t command, uint16_t od_index, uint8_t od_sub_index, uint16_t node_id);


/**
 * @brief Send a NMT CAN message
 * void send_nmt_command(uint8_t command, uint8_t node_id)
 *
 * @param command NMT command :
 * 0x01 for Operational
 * 0x02 for Stop
 * 0x80 for Pre-operational
 * 0x81 for Reset Node
 * 0x82 for Reset Communication
 * @param node_id Node ID of the slave
 * @return void
 */
void send_nmt_command(uint8_t command, uint16_t node_id);


#endif /* CAN_OPEN_H_ */
