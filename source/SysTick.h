/***************************************************************************//**
  @file     SysTick.h
  @brief    SysTick driver
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _SYSTICK_H_
#define _SYSTICK_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define SYSTICK_ISR_FREQUENCY_HZ 1000U

#define SYSTICK_PISR_CANT        8


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef void (*systick_callback_t) (void);


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize SysTick driver
 * @return Initialization succeed
 */
bool SysTick_Init (void);


/**
 * @brief Register PISR function
 * @param fun PISR function to be call
 * @param period PISR period in SysTick ISR ticks
 * @return Registration succeed
 */
bool SysTick_RegisterCallback (systick_callback_t fun, int period);


/*******************************************************************************
 ******************************************************************************/

#endif // _SYSTICK_H_
