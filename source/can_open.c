/*
 * can_open.c
 *
 *  Created on: 9 sept 2024
 *      Author: asolari
 */

#include  "can_open.h"



////////////////////////////////////////////////////
//                 SDO WRITE                      //
////////////////////////////////////////////////////


bool send_sdo_write_command (uint8_t command, uint16_t od_index, uint8_t od_sub_index, int32_t data, uint16_t node_id)
{
	if(can_isTxReady())
	{
		can_msg_t sdo_msg;
		sdo_msg.id = 0x600 + node_id;  // COB-ID del SDO
		sdo_msg.rtr = 0;
		sdo_msg.len = 8;

		sdo_msg.data[0] = command;  // Comando SDO
		sdo_msg.data[1] = (uint8_t)(od_index & 0x00FF);  // Índice bajo
		sdo_msg.data[2] = (uint8_t)((od_index >> 8) & 0x00FF);   // Índice alto
		sdo_msg.data[3] = od_sub_index;     // Subíndice
		sdo_msg.data[4] = (uint8_t)(data & 0x000000FF);      // Byte 0 del valor
		sdo_msg.data[5] = (uint8_t)((data >> 8) & 0x000000FF);  // Byte 1 del valor
		sdo_msg.data[6] = (uint8_t)((data >> 16) & 0x000000FF); // Byte 2 del valor
		sdo_msg.data[7] = (uint8_t)((data >> 24) & 0x000000FF); // Byte 3 del valor

		while(!can_isTxReady());
		bool flag = can_sendTxMsg(&sdo_msg);

		return flag;
	}
}


/**
 * @brief Receive a SDO Write message
 *	bool recive_sdo_command(uint8_t command, uint16_t od_index,
 *	 uint8_t od_sub_index, uint32_t data, uint8_t node_id)
 *
 * 	@return 1 = Recive success
 */
bool recive_sdo_write_command(uint8_t command, uint16_t od_index, uint8_t od_sub_index, int32_t data, uint16_t node_id)
{
	can_msg_t sdo_reply;

	if(can_isNewRxMsg())
	{
		if (can_readRxMsg(&sdo_reply) && sdo_reply.id == (0x580 + node_id))
		{
			if (	   (sdo_reply.data[0] == 0x60) //Command for response
					&& (sdo_reply.rtr == 0)
					&& (sdo_reply.data[1] == (uint8_t) (od_index & 0x00FF))
					&& (sdo_reply.data[2] == (uint8_t) ((od_index >> 8) & 0x00FF))
					&& (sdo_reply.data[3] == od_sub_index)
					&& (sdo_reply.data[4] == (uint8_t)(0x00))
					&& (sdo_reply.data[5] == (uint8_t)(0x00))
					&& (sdo_reply.data[6] == (uint8_t)(0x00))
					&& (sdo_reply.data[7] == (uint8_t)(0x00)))
			{

				return true;
			}
		}
		else
			return false;
	}
}

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
bool send_sdo_read_command(uint16_t od_index, uint8_t od_sub_index, uint16_t node_id)
{
	can_msg_t sdo_msg;
	sdo_msg.id = 0x600 + node_id;  // COB-ID del SDO
	sdo_msg.rtr = 0;
	sdo_msg.len = 8;

	sdo_msg.data[0] = 0x40;  // Comando SDO
	sdo_msg.data[1] = (uint8_t) (od_index & 0x00FF);  // Índice bajo
	sdo_msg.data[2] = (uint8_t) ((od_index >> 8) & 0x00FF);   // Índice alto
	sdo_msg.data[3] = od_sub_index;     // Subíndice
	sdo_msg.data[4] = 0x00;      // Byte 0 del valor
	sdo_msg.data[5] = 0x00;  // Byte 1 del valor
	sdo_msg.data[6] = 0x00; // Byte 2 del valor
	sdo_msg.data[7] = 0x00; // Byte 3 del valor


	while(!can_isTxReady());
	bool flag = can_sendTxMsg(&sdo_msg);

	return flag;
}

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
int32_t recive_sdo_read_command(uint8_t command, uint16_t od_index, uint8_t od_sub_index, uint16_t node_id)
{
	can_msg_t sdo_reply;

	if (can_isNewRxMsg()) {
		if (can_readRxMsg(&sdo_reply) && sdo_reply.id == (0x580 + node_id))
		{
			if (       (sdo_reply.rtr == 0)
					&& (sdo_reply.data[1] == (uint8_t) (od_index & 0x00FF))
					&& (sdo_reply.data[2] == (uint8_t) ((od_index >> 8) & 0x00FF))
					&& (sdo_reply.data[3] == od_sub_index) )
			{
				if((sdo_reply.data[0] == 0x43) || (sdo_reply.data[0] == 0x47) || (sdo_reply.data[0] == 0x4B) || (sdo_reply.data[0] == 0x4F))
					return  (int32_t)(sdo_reply.data[4] | (sdo_reply.data[5] << 8) | (sdo_reply.data[6] << 16) | (sdo_reply.data[7] << 24));
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////

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
void send_nmt_command(uint8_t command, uint16_t node_id) {
    can_msg_t nmt_msg;
    nmt_msg.id = 0x000;  // COB-ID para NMT
    nmt_msg.rtr = 0;
    nmt_msg.len = 2;
    nmt_msg.data[0] = command;  // Comando NMT (0x01 para Operational, 0x80 para Pre-operational)
    nmt_msg.data[1] = (uint8_t)node_id;  // Node ID del esclavo

    if(can_isTxReady())
    {
    	PRINTF("Sending NMT command\n");
    	can_sendTxMsg(&nmt_msg);
    }
}
