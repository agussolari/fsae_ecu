/***************************************************************************//**
  @file     can.h
  @brief    CAN wrapper
  @author   Nicolás Magliola// Agustin Solari 08/2024
 ******************************************************************************/

#ifndef _CAN_H_
#define _CAN_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define CAN_MAX_BYTES_PER_MSG   8

#define CAN_ID_LEN   11

#define CAN_ID_MASK  ((1<<CAN_ID_LEN)-1)
#define CAN_MIN_ID   0
#define CAN_MAX_ID   CAN_ID_MASK



/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
    uint16_t id : 11; // 11 bits for CAN ID
    uint8_t rtr : 1; // 1 bit for RTR
    uint8_t len : 4; // 4 bits for Data Length
    uint8_t data[8]; // 0 - 8 bytes for Data
} can_msg_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize can driver
 */
bool can_init(uint32_t baudrate);


/******************* RX *******************/

/**
 * @brief Verify if a new CAN message was received
 * @return 1 = new CAN message available
 */
bool can_isNewRxMsg(void);

/**
 * @brief Read a received CAN message (Non Blocking)
 * @param msg Pointer to copy received message
 * @return 1 = Read success
 */
bool can_readRxMsg(can_msg_t* msg);


/******************* TX *******************/

/**
 * @brief Verify if CAN driver is ready to send a new CAN message
 * @return 1 = CAN driver ready
 */
bool can_isTxReady(void);

/**
 * @brief Send a CAN message (Non Blocking)
 * @param msg Message to send
 * @return 1 = Send success
 */
bool can_sendTxMsg(const can_msg_t* msg);

/**
 * @brief Send a CAN message (Blocking)
 * @param msg Message to send
 * @return 1 = Send success
 * @return 0 = Send failed
 */
void send_can_message(uint16_t id, uint8_t *data, uint8_t length);


/**
 * @brief Receive a CAN message (Blocking)
 * @param expected_id Message ID to receive
 * @param buffer Pointer to store received message
 * @return 1 = Receive success
 * @return 0 = Receive failed
 */
bool receive_can_message(uint16_t expected_id, uint8_t *buffer);


/*******************************************************************************
 ******************************************************************************/

#endif // _CAN_H_
