/***************************************************************************//**
  @file     adc.h
  @brief    ADC wrapper for LPC55X06
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _ADC_H_
#define _ADC_H_

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

typedef uint16_t adc_ch_res_t;
typedef struct {
	uint16_t a;
	uint16_t b;
} adc_pair_res_t;


typedef enum {
	ADC_CH0A,
	ADC_CH0B,
	ADC_CH1A,
	ADC_CH1B,
	ADC_CH2A,
	ADC_CH2B,
	ADC_CH3A,
	ADC_CH3B,
	ADC_CH4B,
	ADC_CHVDDA,
	ADC_CH1VREF,
	ADC_CHTEMP,
	ADC_CANT_CH
} adc_ch_t;

typedef enum {
	ADC_PAIR0,
	ADC_PAIR1,
	ADC_PAIR2,
	ADC_PAIR3,
	ADC_CANT_PAIR
} adc_pair_t;

#define ADC_MAX_VALUE 65535


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize ADC driver
*/
void adcInit (void);

/*
 * @brief Init DMA for ADC
 */
void adcInitDMA (void);


/**
 * @brief Performs an ADC reading
 * @return channel to read
*/
adc_ch_res_t adcReadChannelBlocking(adc_ch_t ch);


/**
 * @brief Performs an ADC reading
 * @return pair to read
*/
adc_pair_res_t adcReadPairBlocking(adc_pair_t pair);



/*******************************************************************************
 ******************************************************************************/

#endif // _ADC_H_
