/***************************************************************************//**
  @file     millis.h
  @brief    Milliseconds counter library
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _MILLIS_H_
#define _MILLIS_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
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
 * @brief Initialize millis library
 */
void millis_init(void);


/**
 * @brief Returns the number of milliseconds passed since the board
 *        began running the current program.
 *        This number will overflow after approximately 50 days.
 * @return Number of milliseconds passed since the program started
 */
uint32_t millis(void);


/**
 * @brief Pauses the program for the amount of time (in milliseconds).
 * @param ms the number of milliseconds to pause
 */
void delay(uint32_t ms);


/*******************************************************************************
 ******************************************************************************/

#endif // _SYSTICK_H_
