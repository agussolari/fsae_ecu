#include "spi.h"



static uint8_t srcBuff[BUFFER_SIZE];
static uint8_t destBuff[BUFFER_SIZE];

spi_master_handle_t handle;
static volatile bool masterFinished = false;

spi_transfer_t xfer = {0};
spi_master_config_t userConfig;
uint32_t err     = 0;
uint32_t i       = 0;
uint32_t srcFreq = 0;


static void masterCallback(SPI_Type *base, spi_master_handle_t *masterHandle, status_t status, void *userData)
{
    masterFinished = true;
}

void Init_SPI(void)
{


    SPI_MasterGetDefaultConfig(&userConfig);
    srcFreq            = SPI_MASTER_CLK_FREQ;
    userConfig.sselNum = (spi_ssel_t)SPI_SSEL;
    userConfig.sselPol = (spi_spol_t)SPI_SPOL;
    SPI_MasterInit(SPI_MASTER, &userConfig, srcFreq);
    // Enable SPI interrupt in NVIC

    for (i = 0; i < BUFFER_SIZE; i++) {
		srcBuff[i] = 0;
		destBuff[i] = 0;
    }

    xfer.txData = srcBuff;
    xfer.rxData = destBuff;
    xfer.dataSize = BUFFER_SIZE;
    xfer.configFlags = kSPI_FrameAssert;

    SPI_MasterTransferCreateHandle(SPI_MASTER, &handle, masterCallback, NULL);
    SPI_MasterTransferNonBlocking(SPI_MASTER, &handle, &xfer);

}

void Send_Data_SPI(uint8_t *data)
{
	if (masterFinished == true)
	{
		masterFinished = false;

		uint8_t len = sizeof(data)/sizeof(data[0]);

		for (i = 0; i < len; i++) {
			srcBuff[i] = data[i];
			destBuff[i] = 0;
		}

        xfer.txData = srcBuff;
        xfer.rxData = destBuff;
		xfer.dataSize = len;
		xfer.configFlags = kSPI_FrameAssert;
		SPI_MasterTransferNonBlocking(SPI_MASTER, &handle, &xfer);
	}
}

bool isMasterFinished(void) {
	return masterFinished;
}

// EXAMPLE
// int main(void)
// {
//     CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
//     CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);


// 	BOARD_InitPins();
// 	BOARD_BootClockFROHF96M();
// 	BOARD_InitDebugConsole();

// 	Init_SPI();
// 	PRINTF("SPI Initialized\n");

// 	uint8_t data[4] = {0x00, 0x01, 0x02, 0x03};

// 	while (1) {
// 		Send_Data_SPI(data);
// 		while (isMasterFinished() == false)
// 			;
// 		PRINTF("Data sent: %x\n", data);
// 	}

// 	return 0;
// }