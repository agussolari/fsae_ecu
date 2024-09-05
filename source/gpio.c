/***************************************************************************//**
  @file     gpio.c
  @brief    GPIO wrapper, similar to Arduino
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "gpio.h"

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

#include "fsl_ctimer.h"


#include "fsl_ctimer.h"

#define TIME_TOGGLE  500// Reemplaza 1000 con el valor correcto para tu plataforma

void gpioBlink(pin_t pin)
{
    static uint32_t lastToggleTime = 0;
    static bool ledState = false;

    // Get the current time
    uint32_t currentTime = millis();

    // If 500ms have passed since the last toggle
    if ((currentTime - lastToggleTime) >= TIME_TOGGLE ) {
        // Toggle the LED
        ledState = !ledState;
        gpioWrite(pin, ledState);

        // Update the last toggle time
        lastToggleTime = currentTime;
    }
}


void init_leds(void)
{
	// Initialize the GPIO
	gpioMode(PIN_LED_RED, GPIO_OUTPUT);
	gpioMode(PIN_LED_GREEN, GPIO_OUTPUT);
	gpioMode(PIN_LED_BLUE, GPIO_OUTPUT);

	// Turn off the LEDs
	gpioWrite(PIN_LED_RED, LOW);
	gpioWrite(PIN_LED_GREEN, LOW);
	gpioWrite(PIN_LED_BLUE, LOW);

	// Initialize the CTIMER
	millis_init();
}







/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/



/*******************************************************************************
 ******************************************************************************/

