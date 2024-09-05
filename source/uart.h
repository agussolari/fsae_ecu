/***************************************************************************//**
  @file     uart.h
  @brief    UART wrapper for LPC55X06. Non-Blocking and using FIFO feature
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _UART_H_
#define _UART_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "common.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize UART driver
*/
void uartInit (void);


/******************* RX *******************/

/**
 * @brief Check if a new byte was received
 * @return A new byte has being received
*/
bool uartIsRxMsg(void);

/**
 * @brief Check how many bytes were received
 * @return Quantity of received bytes
*/
uint8_t uartGetRxMsgLength(void);

/**
 * @brief Read a received message. Non-Blocking
 * @param msg Buffer to paste the received bytes
 * @param cant Desired quantity of bytes to be pasted
 * @return Real quantity of pasted bytes
*/
uint8_t uartReadMsg(char* msg, uint8_t cant);


/******************* TX *******************/

/**
 * @brief Write a message to be transmitted. Non-Blocking
 * @param msg Buffer with the bytes to be transfered
 * @param cant Desired quantity of bytes to be transfered
 * @return Real quantity of bytes to be transfered
*/
uint8_t uartWriteMsg(const char* msg, uint8_t cant);

/**
 * @brief Write a string to be transmitted. Non-Blocking
 * @param msg string to be transfered
 * @return Real quantity of bytes to be transfered
*/
uint8_t uartWriteStr(const char* str);

/**
 * @brief Check if all bytes were transfered
 * @return All bytes were transfered
*/
bool uartIsTxMsgComplete(void);


/*******************************************************************************
 ******************************************************************************/

#endif // _UART_H_
