/*
 * sensors.h
 *
 *  Created on: 21 ago 2024
 *      Author: asolari
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "common.h"
#include "adc.h"
#include "gpio.h"
#include "millis.h"
#include "fsl_dma.h"
#include "flash_wrp.h"
#include "drivers.h"
#include "fsl_dma.h"
#include "fsl_lpadc.h"
#include "can.h"




#define ADC_CHANNEL_TPS1 ADC_CH0A //OK
#define ADC_CHANNEL_TPS2 ADC_CH2A //OK

#define ADC_CHANNEL_REAR_BRAKE ADC_CH3A
#define ADC_CHANNEL_FRONT_BRAKE ADC_CH0B //

#define ADC_CHANNEL_DIRECTION ADC_CH3B // Canal del ADC que se conecta a la direccion

#define SENSOR_READ_INTERVAL 100 // Intervalo de lectura de los sensores en ms


#define IMPLAUSIBILITY_THRESHOLD 100 // 100 milliseconds
#define PEDAL_TRAVEL_THRESHOLD 100 // 10% of 1000

#define CURRENT_NODE_ID 0x70F

#define ADC_0_5V_VALUE (uint16_t)9930 // 3.3V in ADC units

#define MAX_DEGREE ((float)(1800.0f))
#define STEARING_RELATION ((float)(0.5f))


#define ADC_CHANNEL_COUNT ADC_CANT_CH /* Number of ADC channels */
#define DMA_DESCRIPTOR_NUM  2U /* Number of DMA descriptors */
#define LPADC_RESFIFO_REG_ADDR      ((uint32_t)(&(ADC0->RESFIFO[0])))
#define ADC_USER_CMDID            1U /* CMD1 */
#define DMA_ADC_CHANNEL             21U



void init_sensor(void);
void flash_read_calibration_values(void);

void run_sensors(void);
void trigger_adc(void);
bool check_implausibility_tps(void);
void recive_current_message(can_msg_t rx_msg);



typedef struct
{
	float ac_current_n1;
	float ac_current_n2;
	float dc_current_n1;
	float dc_current_n2;

}current_sense_data_t;

typedef struct {
	uint16_t tps1_value;
	uint16_t tps2_value;
	uint16_t tps_value;
	int16_t front_brake_value;
	int16_t rear_brake_value;
	int16_t direction_value;
} sensor_values_t;

typedef struct
{
	int16_t brake_value;
	uint16_t calibration_break_value;
}break_data_t;

typedef struct{
	float direction_value;
	uint16_t calibration_direction_value;
} direction_data_t;

typedef struct {
    uint16_t tps1_value;
    uint16_t tps2_value;

    bool implausibility_detected;

    time_t implausibility_start_time;
    uint32_t tps_time_stamp;

    uint16_t tps1_min_value;
    uint16_t tps1_max_value;
    uint16_t tps2_min_value;
    uint16_t tps2_max_value;

} tps_data_t;

extern volatile bool using_adc_array;
extern uint16_t adc_sensor_values[ADC_CHANNEL_COUNT];

extern tps_data_t tps_data;

extern break_data_t front_break_data;
extern break_data_t rear_break_data;

extern direction_data_t direction_data;

extern sensor_values_t sensor_values;

extern current_sense_data_t current_sense_data;


#endif /* SENSORS_H_ */
