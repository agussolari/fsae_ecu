/***************************************************************************//**
  @file     adc.c
  @brief    ADC wrapper for LPC55X06
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "adc.h"
#include "adc_cfg.h"

#include "fsl_iocon.h"
#include "fsl_lpadc.h"
#include "fsl_clock.h"
#include "fsl_power.h"
#include "fsl_anactrl.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


#define DEMO_LPADC_CMD_ID		1
#define DEMO_LPADC_TRIGGER_ID	0

#if ADC_DEVELOPMENT_MODE
//    LED

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



/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/
#if ADC_DEVELOPMENT_MODE
static int adc_ticks;
#endif // ADC_DEVELOPMENT_MODE

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static const uint8_t ch2adch[ADC_CANT_CH] = {
	0,  // ADC_CH0A
	0,  // ADC_CH0B
	1,  // ADC_CH1A
	1,  // ADC_CH1B
	2,  // ADC_CH2A
	2,  // ADC_CH2B
	3,  // ADC_CH3A
	3,  // ADC_CH3B
	4,  // ADC_CH4B
	12, // ADC_CHVDDA
	13, // ADC_CH1VREF
	26  // ADC_CHTEMP
};

static const uint8_t ch2ctype[ADC_CANT_CH] = {
	kLPADC_SampleChannelSingleEndSideA,  // ADC_CH0A
	kLPADC_SampleChannelSingleEndSideB,  // ADC_CH0B
	kLPADC_SampleChannelSingleEndSideA,  // ADC_CH1A
	kLPADC_SampleChannelSingleEndSideB,  // ADC_CH1B
	kLPADC_SampleChannelSingleEndSideA,  // ADC_CH2A
	kLPADC_SampleChannelSingleEndSideB,  // ADC_CH2B
	kLPADC_SampleChannelSingleEndSideA,  // ADC_CH3A
	kLPADC_SampleChannelSingleEndSideB,  // ADC_CH3B
	kLPADC_SampleChannelSingleEndSideB,  // ADC_CH4B
	kLPADC_SampleChannelSingleEndSideA, // ADC_CHVDDA
	kLPADC_SampleChannelSingleEndSideA, // ADC_CH1VREF
	kLPADC_SampleChannelSingleEndSideA  // ADC_CHTEMP
};


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void adcInit(void)
{
	static bool yaInit = FALSE;
	if (yaInit)
		return;




	// 2) Set clock source for ADC
    CLOCK_SetClkDiv(kCLOCK_DivAdcAsyncClk, 2U, true);
    CLOCK_AttachClk(kFRO_HF_to_ADC_CLK);

    // 3) Disable LDOGPADC power down
    POWER_DisablePD(kPDRUNCFG_PD_LDOGPADC);

    // 4) Configura ADC
    lpadc_config_t lpadc_config;
    LPADC_GetDefaultConfig(&lpadc_config);
    lpadc_config.enableAnalogPreliminary = true;
    lpadc_config.referenceVoltageSource = ADC_VREF;
    lpadc_config.conversionAverageMode = kLPADC_ConversionAverage64;
    LPADC_Init(ADC0, &lpadc_config);


    // 5) Calibrate
    ANACTRL_Init(ANACTRL);// Enable 1V ref
    ANACTRL_EnableVref1V(ANACTRL, true);

    LPADC_DoOffsetCalibration(ADC0);
    LPADC_DoAutoCalibration(ADC0);

    // 6) Set conversion CMD configuration
    lpadc_conv_command_config_t lpadc_cmd_config;;
    LPADC_GetDefaultConvCommandConfig(&lpadc_cmd_config);
    lpadc_cmd_config.sampleTimeMode = kLPADC_SampleTimeADCK7;
    lpadc_cmd_config.hardwareAverageMode = ADC_AVERAGE_2POW;
#if ADC_RESOLUTION_BITS == 12
#elif ADC_RESOLUTION_BITS == 16
    lpadc_cmd_config.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
#else
#error resolución invalida para el ADC! Debe ser 12 o 16
#endif // ADC_RESOLUTION_BITS = xx

    int i;
    // channel
    for (i=0 ; i<ADC_CANT_CH ; ++i)
    { // configuro 1 comando por channel
    	lpadc_cmd_config.channelNumber = ch2adch[i];
    	lpadc_cmd_config.sampleChannelMode = ch2ctype[i];
    	LPADC_SetConvCommandConfig(ADC0, i+1, &lpadc_cmd_config);
    }

    // pair
    lpadc_cmd_config.sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;
    for (i=0 ; i</*ADC_CANT_PAIR*/3 ; ++i)
    {
    	lpadc_cmd_config.channelNumber = i;
    	LPADC_SetConvCommandConfig(ADC0, ADC_CANT_CH+1+i, &lpadc_cmd_config);
    }


    // 7) Set trigger configuration
    lpadc_conv_trigger_config_t lpadc_trigger_config;
    LPADC_GetDefaultConvTriggerConfig(&lpadc_trigger_config);
    lpadc_trigger_config.enableHardwareTrigger = false;
    lpadc_trigger_config.channelBFIFOSelect = 1; // en dual single-ended mode guardo A en FIFO0 y B en FIFO1
    for (i=0 ; i<ADC_CANT_CH+/*ADC_CANT_PAIR*/3 ; ++i)
    { // configuro 1 trigger por comando
    	lpadc_trigger_config.targetCommandId = i+1;
    	LPADC_SetConvTriggerConfig(ADC0, i, &lpadc_trigger_config);
    }


	// Listo! Inicializada
	yaInit = true;

}



adc_ch_res_t adcReadChannelBlocking(adc_ch_t ch)
{
	adc_ch_res_t rta;
	lpadc_conv_result_t lpadc_result;
#if ADC_DEVELOPMENT_MODE
	adc_ticks = 0;
#endif // ADC_DEVELOPMENT_MODE

	LPADC_DoSoftwareTrigger(ADC0, 1<<ch);
	while (!LPADC_GetConvResult(ADC0, &lpadc_result, 0))
	{
#if ADC_DEVELOPMENT_MODE
		++adc_ticks;
#endif // ADC_DEVELOPMENT_MODE
	}

#if ADC_RESOLUTION_BITS == 12
	rta = lpadc_result.convValue << 1;
#else // ADC_RESOLUTION_BITS == 16
	rta = lpadc_result.convValue;
#endif // ADC_RESOLUTION_BITS = xx

	return rta;
}


adc_pair_res_t adcReadPairBlocking(adc_pair_t pair)
{
	adc_pair_res_t rta;
	lpadc_conv_result_t lpadc_result;
#if ADC_DEVELOPMENT_MODE
	adc_ticks = 0;
#endif // ADC_DEVELOPMENT_MODE

	LPADC_DoSoftwareTrigger(ADC0, 1<<(pair+ADC_CANT_CH));

	// channel A
	while (!LPADC_GetConvResult(ADC0, &lpadc_result, 0))
	{
#if ADC_DEVELOPMENT_MODE
		++adc_ticks;
#endif // ADC_DEVELOPMENT_MODE
	}
#if ADC_RESOLUTION_BITS == 12
	rta.a = lpadc_result.convValue << 1;
#else // ADC_RESOLUTION_BITS == 16
	rta.a = lpadc_result.convValue;
#endif // ADC_RESOLUTION_BITS = xx

	// channel B
	while (!LPADC_GetConvResult(ADC0, &lpadc_result, 1))
	{
#if ADC_DEVELOPMENT_MODE
		++adc_ticks;
#endif // ADC_DEVELOPMENT_MODE
	}
#if ADC_RESOLUTION_BITS == 12
	rta.b = lpadc_result.convValue << 1;
#else // ADC_RESOLUTION_BITS == 16
	rta.b = lpadc_result.convValue;
#endif // ADC_RESOLUTION_BITS = xx

	return rta;
}


