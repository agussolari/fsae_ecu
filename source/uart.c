/***************************************************************************//**
  @file     uart.c
  @brief    UART wrapper for LPC55X06. Non-Blocking and using FIFO feature
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "uart.h"
#include "uart_cfg.h"

#include "fsl_usart.h"
#include "fsl_iocon.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


#define USARTX				REG2_DEF(USART, UART_NUM)
#define FLEXCOMMX_IRQn		REG3_DEF(FLEXCOMM, UART_NUM,_IRQn)

#if UART_DEVELOPMENT_MODE
//    LED
#define PIN_LED_RED     	PORTNUM2PIN(0,21)
#define PIN_LED_GREEN     	PORTNUM2PIN(0,18)
#define PIN_LED_BLUE     	PORTNUM2PIN(0,22)
#define LED_ACTIVE			HIGH
enum {
	LED_RED,
	LED_GREEN,
	LED_BLUE,
	CAN_LEDS
};
#include "gpio.h"
#endif // UART_DEVELOPMENT_MODE


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
	uint8_t iPut;
	uint8_t iGet;
	uint8_t count;
	uint8_t size;
	uint8_t *buffer;
} queue_t;



/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

#if UART_TX_BUFFER_SIZE > 254
#error overflow de indices, debe entra en un uint8_t}
#endif

static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static queue_t tx_queue = {0, 0, 0, UART_TX_BUFFER_SIZE, tx_buffer};



/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/




/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void uartInit(void)
{
	static bool yaInit = FALSE;
	if (yaInit)
		return;

	// 1) Config pins
#if UART_NUM == 0
	// RXD: P0.29 ; TXD: P0.30
	CLOCK_EnableClock(kCLOCK_Iocon);

#define IOCON_PIO_DIGITAL_EN 0x0100u  /*!<@brief Enables digital function */
#define IOCON_PIO_FUNC1 0x01u         /*!<@brief Selects pin function 1 */
#define IOCON_PIO_INV_DI 0x00u        /*!<@brief Input function is not inverted */
#define IOCON_PIO_MODE_INACT 0x00u    /*!<@brief No addition pin function */
#define IOCON_PIO_OPENDRAIN_DI 0x00u  /*!<@brief Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD 0x00u /*!<@brief Standard mode, output slew rate control is enabled */

	const uint32_t port0_pin29_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
			IOCON_PIO_FUNC1 |
			/* No addition pin function */
			IOCON_PIO_MODE_INACT |
			/* Standard mode, output slew rate control is enabled */
			IOCON_PIO_SLEW_STANDARD |
			/* Input function is not inverted */
			IOCON_PIO_INV_DI |
			/* Enables digital function */
			IOCON_PIO_DIGITAL_EN |
			/* Open drain is disabled */
			IOCON_PIO_OPENDRAIN_DI);
	/* PORT0 PIN29 (coords: 61) is configured as FC0_RXD_SDA_MOSI_DATA */
	IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);

	const uint32_t port0_pin30_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
			IOCON_PIO_FUNC1 |
			/* No addition pin function */
			IOCON_PIO_MODE_INACT |
			/* Standard mode, output slew rate control is enabled */
			IOCON_PIO_SLEW_STANDARD |
			/* Input function is not inverted */
			IOCON_PIO_INV_DI |
			/* Enables digital function */
			IOCON_PIO_DIGITAL_EN |
			/* Open drain is disabled */
			IOCON_PIO_OPENDRAIN_DI);
	/* PORT0 PIN30 (coords: 63) is configured as FC0_TXD_SCL_MISO_WS */
	IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config);

#endif // UART_NUM == 0

	// 2) attach 12 MHz clock to FLEXCOMMx
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);


	// 3) Configuro USARTX usando
	usart_config_t config;
	USART_GetDefaultConfig(&config);
	config.baudRate_Bps = UART_BAUDRATE;
	config.enableTx     = true;
	config.enableRx     = true;

	USART_Init(USARTX, &config, CLOCK_GetFlexCommClkFreq(UART_NUM));

	// 4) Configuro TX FIFO
	// no hay que hacer nda, ya se inicializa en la definición


	// 5) Configuro ISR
	//USART_EnableInterrupts(USARTX, kUSART_TxLevelInterruptEnable);
	EnableIRQ(FLEXCOMMX_IRQn);

#if UART_DEVELOPMENT_MODE
	gpioMode(PIN_LED_RED, GPIO_OUTPUT);
	gpioWrite(PIN_LED_RED, !LED_ACTIVE);
	gpioMode(PIN_LED_GREEN, GPIO_OUTPUT);
	gpioWrite(PIN_LED_GREEN, !LED_ACTIVE);
	gpioMode(PIN_LED_BLUE, GPIO_OUTPUT);
	gpioWrite(PIN_LED_BLUE, !LED_ACTIVE);
#endif// UART_DEVELOPMENT_MODE

	// Listo! Inicializada
	yaInit = true;

}


/******************* RX *******************/


bool uartIsRxMsg(void)
{
	return (USART_GetRxFifoCount(USARTX) != 0);
}


uint8_t uartGetRxMsgLength(void)
{
	return USART_GetRxFifoCount(USARTX);
}


uint8_t uartReadMsg(char* msg, uint8_t cant)
{
	uint8_t i;

	for (i=0 ; i<cant && USART_GetRxFifoCount(USARTX) ; ++i)
	{
		msg[i] = USART_ReadByte(USARTX);
	}

	return i;
}


/******************* TX *******************/


uint8_t uartWriteMsg(const char* msg, uint8_t cant)
{
	uint8_t i;

	for (i=0 ; i<cant ; ++i)
	{
		if(tx_queue.count >= tx_queue.size)
		{ // cola llena
			cant = i; // guardo cantidad de elementos ingresados a la queue
		}
		else
		{ // cola no llena
			tx_queue.buffer[tx_queue.iPut] = msg[i];
			INCMOD(tx_queue.iPut, tx_queue.size);
			tx_queue.count++; // Esto debe ser ATOMIC!!!
		}
	}

	USART_EnableInterrupts(USARTX, kUSART_TxLevelInterruptEnable); // Enable TX ISR

	return cant;
}


uint8_t uartWriteStr(const char* str)
{
	uint8_t len=0;

	while(str[len])
	{
		++len;
	}

	return uartWriteMsg(str, len);
}


bool uartIsTxMsgComplete(void)
{
	// TX is complete if TX FIFO is empty and USART TX is idle
	return (tx_queue.count == 0) && (kUSART_TxIdleFlag & USART_GetStatusFlags(USARTX));
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/



#if UART_NUM == 0
void FLEXCOMM0_IRQHandler (void)
#endif // UART_NUM == x
{
#if UART_DEVELOPMENT_MODE
	gpioWrite(PIN_LED_RED, LED_ACTIVE);
#endif // UART_DEVELOPMENT_MODE
	if (kUSART_TxFifoEmptyFlag & USART_GetStatusFlags(USARTX)) // TX IRQ
	{
		while((tx_queue.count > 0) && (USART_FIFOSTAT_TXNOTFULL_MASK & USARTX->FIFOSTAT))
		{ // mientras que haya elementos en el SW TX FIFO y no esté llena la HW FIFO
			USART_WriteByte(USARTX, tx_queue.buffer[tx_queue.iGet]);
			INCMOD(tx_queue.iGet, tx_queue.size);
			tx_queue.count--;
		}

		if(tx_queue.count == 0)
		{
			USART_DisableInterrupts(USARTX, kUSART_TxLevelInterruptEnable); // Disable TX ISR
		}
	}

#if UART_DEVELOPMENT_MODE
	gpioWrite(PIN_LED_RED, !LED_ACTIVE);
#endif // UART_DEVELOPMENT_MODE

	SDK_ISR_EXIT_BARRIER;
}




