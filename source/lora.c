#include "lora.h"

static CircularBuffer rxBuffer = {{0}, 0, 0};
static volatile bool dataReceived = false;


static void LoRa_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg | 0x80, value}; // Set MSB to 1 for write operation
    Send_Data_SPI(SPI_LORA, data, 2);
}

static uint8_t LoRa_ReadRegister(uint8_t reg)
{
    uint8_t data[2] = {reg & 0x7F, 0x00}; // Set MSB to 0 for read operation
    uint8_t rxData[2] = {0};
    Send_Data_SPI(SPI_LORA, data, 2);
    // Assuming SPI_ReadData is a function to read data from SPI
//    SPI_ReadData(spiBase, rxData);
    return rxData[1];
}

void LoRa_Init(void)
{
    // Set LoRa mode
    LoRa_WriteRegister(0x01, 0x80); // RegOpMode: Set LoRa mode

    // Set frequency to 915 MHz (example)
    LoRa_WriteRegister(0x06, 0xE4); // RegFrMsb
    LoRa_WriteRegister(0x07, 0xC0); // RegFrMid
    LoRa_WriteRegister(0x08, 0x00); // RegFrLsb

    // Set output power to max
    LoRa_WriteRegister(0x09, 0xFF); // RegPaConfig

    // Set spreading factor to 7 and bandwidth to 125 kHz
    LoRa_WriteRegister(0x1D, 0x72); // RegModemConfig1
    LoRa_WriteRegister(0x1E, 0x74); // RegModemConfig2

    // Set preamble length to 8
    LoRa_WriteRegister(0x20, 0x00); // RegPreambleMsb
    LoRa_WriteRegister(0x21, 0x08); // RegPreambleLsb

    // Enable IRQ for RxDone
    LoRa_WriteRegister(0x40, 0x40); // RegDioMapping1: DIO0 -> RxDone
    LoRa_WriteRegister(0x11, 0xFF); // RegIrqFlagsMask: Enable all IRQs

    // Set to standby mode
    LoRa_WriteRegister(0x01, 0x81); // RegOpMode: Set to standby mode
}

void LoRa_Send(uint8_t *data, uint8_t length)
{
    // Set to standby mode
    LoRa_WriteRegister(0x01, 0x81); // RegOpMode: Set to standby mode

    // Set payload length
    LoRa_WriteRegister(0x22, length); // RegPayloadLength

    // Write payload to FIFO
    for (uint8_t i = 0; i < length; i++) {
        LoRa_WriteRegister(0x00, data[i]); // RegFifo
    }

    // Set to Tx mode
    LoRa_WriteRegister(0x01, 0x83); // RegOpMode: Set to Tx mode
}

uint8_t LoRa_Receive(uint8_t *data, uint8_t length)
{
    uint8_t bytesRead = 0;

    while (bytesRead < length && rxBuffer.head != rxBuffer.tail) {
        data[bytesRead++] = rxBuffer.buffer[rxBuffer.tail];
        rxBuffer.tail = (rxBuffer.tail + 1) % LORA_BUFFER_SIZE;
    }

    return bytesRead;
}

//void LoRa_IRQHandler(SPI_Type *spiBase)
//{
//    if (dataReceived) {
//        dataReceived = false;
//
//        // Read data from SPI and store it in the circular buffer
//        uint8_t receivedData;
//        SPI_ReadData(spiBase, &receivedData);
//
//        uint16_t nextHead = (rxBuffer.head + 1) % LORA_BUFFER_SIZE;
//        if (nextHead != rxBuffer.tail) {
//            rxBuffer.buffer[rxBuffer.head] = receivedData;
//            rxBuffer.head = nextHead;
//        }
//    }
//}
