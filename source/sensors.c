/*
 * sensors.c
 *
 *  Created on: 21 ago 2024
 *      Author: asolari
 */

#include "sensors.h"

#include "FreeRTOS.h"
#include "semphr.h"


/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

SemaphoreHandle_t adcMutex; // Declarar el mutex


void DMA_Callback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

sensor_values_t sensor_values;

break_data_t front_break_data;
break_data_t rear_break_data;

direction_data_t direction_data;

tps_data_t tps_data = {0};

current_sense_data_t current_sense_data = {0};

static uint16_t raw_tps1 = 0;
static uint16_t raw_tps2 = 0;
static uint16_t raw_front_brake = 0;
static uint16_t raw_rear_brake = 0;
static uint16_t raw_direction = 0;



//DMA variables
uint32_t g_AdcConvResult[ADC_CHANNEL_COUNT] = {0};
uint16_t adc_sensor_values[ADC_CHANNEL_COUNT] = {0};

dma_handle_t g_DmaHandleStruct; /* DMA handler */
dma_channel_config_t dmaChannelConfigStruct;
/* DMA descripter table used for ping-pong mode. */
SDK_ALIGN(uint32_t s_dma_table[DMA_DESCRIPTOR_NUM * sizeof(dma_descriptor_t)],
		FSL_FEATURE_DMA_LINK_DESCRIPTOR_ALIGN_SIZE);

volatile bool g_DmaTransferDoneFlag = false;

/* Configure DMA */
static void DMA_Configuration(void)
{

    uint32_t g_XferConfig = DMA_CHANNEL_XFER(true,                          /* Reload link descriptor after current exhaust, */
                                    true,                          /* Clear trigger status. */
                                    true,                          /* Enable interruptA. */
                                    false,                         /* Not enable interruptB. */
                                    sizeof(uint32_t),              /* Dma transfer width. */
                                    kDMA_AddressInterleave0xWidth, /* Dma source address no interleave  */
                                    kDMA_AddressInterleave1xWidth, /* Dma destination address no interleave  */
									ADC_CHANNEL_COUNT * sizeof(uint32_t) /* Dma transfer byte. */
    );
    /* Initialize DMA */
    DMA_Init(DMA0);
    DMA_EnableChannel(DMA0, DMA_ADC_CHANNEL);
    DMA_CreateHandle(&g_DmaHandleStruct, DMA0, DMA_ADC_CHANNEL);
    DMA_SetCallback(&g_DmaHandleStruct, DMA_Callback, NULL);

    /* Configure transfer */
    DMA_PrepareChannelTransfer(&dmaChannelConfigStruct,
                               (void *)LPADC_RESFIFO_REG_ADDR,
                               (void *)g_AdcConvResult,
                               g_XferConfig,
							   kDMA_PeripheralToMemory,
                               NULL,
							   (dma_descriptor_t *)&(s_dma_table[0])
							   );

    DMA_SubmitChannelTransfer(&g_DmaHandleStruct, &dmaChannelConfigStruct);

    /* Set two DMA descripters to use ping-pong mode.  */
    DMA_SetupDescriptor((dma_descriptor_t *)&(s_dma_table[0]), g_XferConfig, (void *)LPADC_RESFIFO_REG_ADDR,
                        (void *)g_AdcConvResult, (dma_descriptor_t *)&(s_dma_table[4]));
    DMA_SetupDescriptor((dma_descriptor_t *)&(s_dma_table[4]), g_XferConfig, (void *)LPADC_RESFIFO_REG_ADDR,
                        (void *)g_AdcConvResult, (dma_descriptor_t *)&(s_dma_table[0]));
}

void DMA_Callback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
	if (transferDone)
	{
		g_DmaTransferDoneFlag = true;

		for (int i = 0; i < ADC_CHANNEL_COUNT; i++)
		{
			uint32_t raw_data = g_AdcConvResult[i];
			uint16_t adc_value = (uint16_t) (raw_data & 0xFFFF); // Extract bits 0:15
			uint8_t trigger_source = (uint8_t) ((raw_data >> 16) & 0x0F); // Extract bits 19:16

            // Map trigger source to corresponding channel or action
	        if (xSemaphoreTake(adcMutex, portMAX_DELAY) == pdTRUE)
	        {
	        	adc_sensor_values[trigger_source] = adc_value;
	            xSemaphoreGive(adcMutex); // Liberar el mutex

	        }
		}
	}
}



void init_sensor(void) {
	// Inicializo los sensores
	uartWriteStr("Starting sensors...\n");

    adcMutex = xSemaphoreCreateMutex();
    if (adcMutex == NULL) {
        uartWriteStr("Failed to create ADC mutex\n");
        return;
    }


	//Inicializar el ADC
	adcInit();
	adcInitDMA();




	uartWriteStr("Sensors started\n");
}




// Funcion para leer los sensores y guardar los valores de
// acelerador y freno en variables globales


void trigger_adc(void)
{
	if (g_DmaTransferDoneFlag == false)
	{
		return;
	}

	// Start the ADC conversion
    g_DmaTransferDoneFlag = false;

    LPADC_DoSoftwareTrigger(ADC0,
    			(1 << ADC_CHANNEL_TPS1) |
				(1 << ADC_CHANNEL_TPS2) |
				(1 << ADC_CHANNEL_REAR_BRAKE) |
				(1 << ADC_CHANNEL_FRONT_BRAKE) |
				(1 << ADC_CHANNEL_DIRECTION)
				);
    DMA_StartTransfer(&g_DmaHandleStruct);

}




void run_sensors(void)
{

	trigger_adc();

    // Tomar el mutex antes de leer adc_sensor_values
    if (xSemaphoreTake(adcMutex, portMAX_DELAY) == pdTRUE)
    {
        uint16_t raw_tps1 = adc_sensor_values[ADC_CHANNEL_TPS1];
        uint16_t raw_tps2 = adc_sensor_values[ADC_CHANNEL_TPS2];
        uint16_t raw_front_brake = adc_sensor_values[ADC_CHANNEL_FRONT_BRAKE];
        uint16_t raw_rear_brake = adc_sensor_values[ADC_CHANNEL_REAR_BRAKE];
        uint16_t raw_direction = adc_sensor_values[ADC_CHANNEL_DIRECTION];
        xSemaphoreGive(adcMutex); // Liberar el mutex
    }

	tps_data.tps_time_stamp = millis();




    //Map the TPS1 value
    if (raw_tps1 > tps_data.tps1_min_value)
    {
		tps_data.tps1_value = 0;
	} else if (raw_tps1 < tps_data.tps1_max_value) {
		tps_data.tps1_value = 1000;
	} else {
	    tps_data.tps1_value = (uint16_t) (((float) (tps_data.tps1_min_value - raw_tps1)
	            / (tps_data.tps1_min_value - tps_data.tps1_max_value)) * 1000);
    }

    // Map the TPS2 value
	if (raw_tps2 < tps_data.tps2_min_value) {
		tps_data.tps2_value = 0;
	} else if (raw_tps2 > tps_data.tps2_max_value) {
		tps_data.tps2_value = 1000;
	} else {
	    tps_data.tps2_value = (uint16_t) (((float) (raw_tps2 - tps_data.tps2_min_value)
	            / (tps_data.tps2_max_value - tps_data.tps2_min_value)) * 1000);
	}




	//Break values in PSI
	// P [PSI] = 400*(V - 0.5V)
	// V = raw_value/65535.0 * 3.3
	front_break_data.brake_value =  (int16_t)(400.0*(((float)(raw_front_brake + front_break_data.calibration_break_value)/65535.0)*3.3 - 0.5));
	rear_break_data.brake_value =   (int16_t)(400.0*(((float)(raw_rear_brake + rear_break_data.calibration_break_value)/65535.0)*3.3 - 0.5));





	direction_data.direction_value = (float)((float)raw_direction - (float)direction_data.calibration_direction_value)/((float)(ADC_MAX_VALUE))*MAX_DEGREE;




	//Save values
	sensor_values.tps1_value = tps_data.tps1_value;
	sensor_values.tps2_value = tps_data.tps2_value;
	sensor_values.tps_value = (tps_data.tps1_value + tps_data.tps2_value) / 2;
    sensor_values.front_brake_value = front_break_data.brake_value;
    sensor_values.rear_brake_value = rear_break_data.brake_value;
    sensor_values.direction_value = direction_data.direction_value;
}

bool check_implausibility_tps(void)
{
    // Calculate the absolute difference between the two TPS values
    uint16_t tps_difference =  abs(tps_data.tps1_value - tps_data.tps2_value);

    // Check if the difference exceeds the threshold
	if (tps_difference > PEDAL_TRAVEL_THRESHOLD)
	{
		if (tps_data.implausibility_detected == false)
		{
			// If the implausibility condition has just been detected, save the time
			tps_data.implausibility_detected = true;
			tps_data.implausibility_start_time = millis();
		}
		if (tps_data.implausibility_detected == true)
		{
			// If the implausibility condition has been detected for too long, return true
			if (millis() - tps_data.implausibility_start_time> IMPLAUSIBILITY_THRESHOLD)
			{
				return true;
			}
		}
	}
	else
	{
		tps_data.implausibility_detected = false;
	}


	return false; // If the implausibility condition is not detected or has not been detected for too long, return false
}



void flash_read_calibration_values(void)
{
	//Read flash memory and save the calibration values
	uint32_t data[7];
	read_flash(data, 7);

	tps_data.tps1_min_value = (uint16_t)data[0];
	tps_data.tps1_max_value = (uint16_t)data[1];
	tps_data.tps2_min_value = (uint16_t)data[2];
	tps_data.tps2_max_value = (uint16_t)data[3];

	tps_data.implausibility_detected = false;
	tps_data.implausibility_start_time = 0;
	tps_data.tps_time_stamp = 0;

	front_break_data.calibration_break_value = (uint16_t)data[4];
	rear_break_data.calibration_break_value = (uint16_t)data[5];

	direction_data.calibration_direction_value = (uint16_t)data[6];

//	//Initialize the calibration flag
//	if (tps_data.tps1_min_value == 0
//			|| tps_data.tps1_max_value == 0
//			|| tps_data.tps2_min_value == 0
//			|| tps_data.tps2_max_value == 0
//			|| front_break_data.calibration_break_value == 0
//			|| rear_break_data.calibration_break_value == 0
//			|| direction_data.calibration_direction_value == 0)
//	{
//		driver_1.calibration_needed = true;
//		driver_2.calibration_needed = true;
//
//	} else {
//		driver_1.calibration_needed = false;
//		driver_2.calibration_needed = false;
//	}

	driver_1.calibration_needed = false;
	driver_2.calibration_needed = false;

}



