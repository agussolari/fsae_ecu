/***************************************************************************//**
  @file     gpio.h
  @brief    GPIO wrapper, similar to Arduino
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _GPIO_H_
#define _GPIO_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "fsl_debug_console.h"
#include "millis.h"



/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define GPIO_CANT_PORTS		2
#define GPIO_INT_EDGE_RISING    0


// Convert port and number into pin ID
// Ex: PIO0_8  -> PORTNUM2PIN(0,8)  -> 0x0008
//     PIO1_21 -> PORTNUM2PIN(1,21) -> 0x0115
#define PORTNUM2PIN(p,n)    (((p)<<8) + (n))
#define PIN2PORT(p)         (((p)>>8) & 0x0001)
#define PIN2NUM(p)          ((p) & 0x001F)


// Modes
#define GPIO_INPUT               0
#define GPIO_OUTPUT              1
#define GPIO_INPUT_PULLUP        2
#define GPIO_INPUT_PULLDOWN      3
#define GPIO_OUTPUT_OPENDRAIN    4
#define GPIO_CANT_MODES	         5


#define LED_ACTIVE			HIGH


// Digital values
#ifndef LOW
#define LOW     0
#define HIGH    1
#endif // LOW



/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef uint16_t pin_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Configures the specified pin to behave either as an input or an output
 * @param pin the pin whose mode you wish to set (according PORTNUM2PIN)
 * @param mode GPIO_INPUT, GPIO_OUTPUT, etc.
 */
void gpioMode (pin_t pin, uint8_t mode);

/**
 * @brief Write a HIGH or a LOW value to a digital pin
 * @param pin the pin to write (according PORTNUM2PIN)
 * @param val Desired value (HIGH or LOW)
 */
void gpioWrite (pin_t pin, bool value);

/**
 * @brief Toggle the value of a digital pin (HIGH<->LOW)
 * @param pin the pin to toggle (according PORTNUM2PIN)
 */
void gpioToggle (pin_t pin);

/**
 * @brief Reads the value from a specified digital pin, either HIGH or LOW.
 * @param pin the pin to read (according PORTNUM2PIN)
 * @return HIGH or LOW
 */
bool gpioRead (pin_t pin);





/*******************************************************************************
 ******************************************************************************/

#endif // _GPIO_H_
