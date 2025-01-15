#ifndef LORA_H_
#define LORA_H_

#include "spi.h"
#include <stdint.h>
#include <stdbool.h>

#define LORA_BUFFER_SIZE 256

typedef struct {
    uint8_t buffer[LORA_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} CircularBuffer;

void LoRa_Init(void);
void LoRa_Send(uint8_t *data, uint8_t length);
uint8_t LoRa_Receive(uint8_t *data, uint8_t length);

#endif /* LORA_H_ */
