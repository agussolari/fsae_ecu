/***************************************************************************//**
  @file     gpio.c
  @brief    GPIO wrapper, similar to Arduino
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include "drivers.h"
#include "fsl_common.h"
#include "fsl_gpio.h"



/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef union {
	pin_t pin;
	struct {
		uint8_t num;
		uint8_t port;
	};
} portNum_t;


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static const gpio_pin_config_t gpio_config[GPIO_CANT_MODES] =
{  // gpio_pin_direction_t     outputLogic
    { kGPIO_DigitalInput,      0           }, // GPIO_INPUT
	{ kGPIO_DigitalOutput,     0           }, // GPIO_OUTPUT
	{ kGPIO_DigitalInput,      0           }, // GPIO_INPUT_PULLUP
	{ kGPIO_DigitalInput,      0           }, // GPIO_INPUT_PULLDOWN
    { kGPIO_DigitalOutput,     0           }  // GPIO_OUTPUT_OPENDRAIN
};

static const uint32_t pio_config[GPIO_CANT_MODES] =
{
    IOCON_PIO_FUNC(0) | IOCON_PIO_MODE(0) | IOCON_PIO_SLEW(0) | IOCON_PIO_INVERT(0) | IOCON_PIO_DIGIMODE(1) | IOCON_PIO_OD (0) | IOCON_PIO_ASW(0), // GPIO_INPUT
	IOCON_PIO_FUNC(0) | IOCON_PIO_MODE(0) | IOCON_PIO_SLEW(0) | IOCON_PIO_INVERT(0) | IOCON_PIO_DIGIMODE(1) | IOCON_PIO_OD (0) | IOCON_PIO_ASW(0), // GPIO_OUTPUT
	IOCON_PIO_FUNC(0) | IOCON_PIO_MODE(2) | IOCON_PIO_SLEW(0) | IOCON_PIO_INVERT(0) | IOCON_PIO_DIGIMODE(1) | IOCON_PIO_OD (0) | IOCON_PIO_ASW(0), // GPIO_INPUT_PULLUP
	IOCON_PIO_FUNC(0) | IOCON_PIO_MODE(1) | IOCON_PIO_SLEW(0) | IOCON_PIO_INVERT(0) | IOCON_PIO_DIGIMODE(1) | IOCON_PIO_OD (0) | IOCON_PIO_ASW(0), // GPIO_INPUT_PULLDOWN
	IOCON_PIO_FUNC(0) | IOCON_PIO_MODE(0) | IOCON_PIO_SLEW(0) | IOCON_PIO_INVERT(0) | IOCON_PIO_DIGIMODE(1) | IOCON_PIO_OD (1) | IOCON_PIO_ASW(0)  // GPIO_OUTPUT_OPENDRAIN
};


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static bool cg_iocon;

debounce_t pre_op_button = {0, false, false};
debounce_t op_button = {0, false, false};
debounce_t drive_button = {0, false, false};
debounce_t stop_button = {0, false, false};

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void gpioMode (pin_t pin, uint8_t mode)
{
	portNum_t p;
	p.pin = pin;

	// 1) clock gating
	if (!cg_iocon)
	{
		CLOCK_EnableClock(kCLOCK_Iocon);
		cg_iocon = true;
	}
	// NO HACE FALTA HACER CLOCK GATING DE GPIOx! YA LO HACE GPIO_PinInit()

	// 2) Initialize GPIO functionality
	GPIO_PinInit(GPIO, p.port, p.num, &gpio_config[mode]);

	// 3) Configure PIO on IOCON
	IOCON->PIO[p.port][p.num] = pio_config[mode];
}


void gpioWrite (pin_t pin, bool value)
{
	portNum_t p;
	p.pin = pin;

	GPIO->B[p.port][p.num] = (value != 0);
}


void gpioToggle (pin_t pin)
{
	portNum_t p;
	p.pin = pin;

	GPIO->NOT[p.port] = 1U << p.num;
}


bool gpioRead (pin_t pin)
{
	portNum_t p;
	p.pin = pin;

	return (GPIO->B[p.port][p.num] != 0);
}


bool read_button(uint8_t port)
{
    return gpioRead(port) == HIGH;
}

void debounce_button(debounce_t* button, uint8_t port)
{
    bool current_state = read_button(port);
    if (current_state != button->last_button_state) {
        button->last_debounce_time = millis();
    }
    if ((millis() - button->last_debounce_time) > DEBOUNCE_DELAY_MS) {
        button->button_state = current_state;
    }
    button->last_button_state = current_state;
}


/*******************************************************************************
 ******************************************************************************/

