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


#define ADC_RESOLUTION_BITS		16 // 12 o 16

enum { ADC_VREF_VDDA = 1, ADC_VREF_VREFP };
#define ADC_VREF		ADC_VREF_VDDA

#define ADC_AVERAGE_2POW		5

#define ADC_DEVELOPMENT_MODE	0


/*******************************************************************************
 ******************************************************************************/

#endif // _ADC_CFG_H_
