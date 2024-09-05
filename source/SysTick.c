/***************************************************************************//**
  @file     SysTick.c
  @brief    SysTick driver
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "SysTick.h"

#include "pin_mux.h"
#include "board.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE    1



/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
    int count;
    int period;
    systick_callback_t fun;
} sys_timer_t;


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static sys_timer_t pisr[SYSTICK_PISR_CANT];
static int pisr_counter=0;



/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

bool SysTick_Init (void)
{
	static bool yaInit = 0;
	if (yaInit)
		return true;

    return !SysTick_Config(SystemCoreClock/SYSTICK_ISR_FREQUENCY_HZ);
}


bool SysTick_RegisterCallback (systick_callback_t fun, int period)
{
#if DEVELOPMENT_MODE
    if (!fun || period<=0 || pisr_counter>=SYSTICK_PISR_CANT)
    	return false;
#endif // DEVELOPMENT_MODE
    pisr[pisr_counter].count = 1;
    pisr[pisr_counter].period = period;
    pisr[pisr_counter].fun = fun;
    ++pisr_counter;

    return true;
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void SysTick_Handler (void)
{
    int i;
    for (i=0 ; i<pisr_counter ; ++i)
    {
        if (--(pisr[i].count) <= 0)
        {
            pisr[i].count = pisr[i].period;
            pisr[i].fun();
        }
    }
}


/******************************************************************************/
