/***************************************************************************//**
  @file     adc_cfg.h
  @brief    ADC configuration file
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _ADC_CFG_H_
#define _ADC_CFG_H_


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


#define ADC_CH0A_USED	1
#define ADC_CH0B_USED	1
#define ADC_CH3A_USED	1
#define ADC_CH3B_USED	1

#define ADC_RESOLUTION_BITS		12 // 12 o 16

enum { ADC_VREF_VDDA = 1, ADC_VREF_VREFP };
#define ADC_VREF		ADC_VREF_VDDA

#define ADC_AVERAGE_2POW		5

#define ADC_DEVELOPMENT_MODE	1


/*******************************************************************************
 ******************************************************************************/

#endif // _ADC_CFG_H_
