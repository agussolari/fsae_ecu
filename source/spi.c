#include "spi.h"



static spi_instance_t spiInstances[MAX_SPI_INSTANCES];

static void masterCallback(SPI_Type *base, spi_master_handle_t *masterHandle,
		status_t status, void *userData) {
	spi_instance_t *instance = (spi_instance_t*) userData;
	instance->masterFinished = true;
}

void Init_SPI(uint8_t instance, SPI_Type *base, uint32_t srcFreq,
		spi_ssel_t sselNum, spi_spol_t sselPol) {
	if (instance >= MAX_SPI_INSTANCES)
		return;

	spi_instance_t *spi = &spiInstances[instance];
	spi->base = base;
	spi->masterFinished = false;

	spi_master_config_t userConfig;
	SPI_MasterGetDefaultConfig(&userConfig);
	userConfig.sselNum = sselNum;
	userConfig.sselPol = sselPol;
	SPI_MasterInit(base, &userConfig, srcFreq);

	for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
		spi->srcBuff[i] = 0;
		spi->destBuff[i] = 0;
	}

	spi->xfer.txData = spi->srcBuff;
	spi->xfer.rxData = spi->destBuff;
	spi->xfer.dataSize = BUFFER_SIZE;
	spi->xfer.configFlags = kSPI_FrameAssert;

	SPI_MasterTransferCreateHandle(base, &spi->handle, masterCallback, spi);
	SPI_MasterTransferNonBlocking(base, &spi->handle, &spi->xfer);
}

void Send_Data_SPI(uint8_t instance, uint8_t *data, uint32_t len) {
	if (instance >= MAX_SPI_INSTANCES)
		return;

	spi_instance_t *spi = &spiInstances[instance];
	if (spi->masterFinished) {
		spi->masterFinished = false;

		for (uint32_t i = 0; i < len; i++) {
			spi->srcBuff[i] = data[i];
			spi->destBuff[i] = 0;
		}

		spi->xfer.txData = spi->srcBuff;
		spi->xfer.rxData = spi->destBuff;
		spi->xfer.dataSize = len;
		spi->xfer.configFlags = kSPI_FrameAssert;
		SPI_MasterTransferNonBlocking(spi->base, &spi->handle, &spi->xfer);
	}
}

bool isMasterFinished(uint8_t instance) {
	if (instance >= MAX_SPI_INSTANCES)
		return false;
	return spiInstances[instance].masterFinished;
}




//#include "spi.h"
//
//
//
//static uint8_t srcBuff[BUFFER_SIZE];
//static uint8_t destBuff[BUFFER_SIZE];
//
//spi_master_handle_t handle;
//static volatile bool masterFinished = false;
//
//spi_transfer_t xfer = {0};
//spi_master_config_t userConfig;
//uint32_t err     = 0;
//uint32_t i       = 0;
//uint32_t srcFreq = 0;
//
//
//static void masterCallback(SPI_Type *base, spi_master_handle_t *masterHandle, status_t status, void *userData)
//{
//    masterFinished = true;
//}
//
//void Init_SPI(void)
//{
//
//
//    SPI_MasterGetDefaultConfig(&userConfig);
//    srcFreq            = SPI_MASTER_CLK_FREQ;
//    userConfig.sselNum = (spi_ssel_t)SPI_SSEL;
//    userConfig.sselPol = (spi_spol_t)SPI_SPOL;
//    SPI_MasterInit(SPI_MASTER, &userConfig, srcFreq);
//    // Enable SPI interrupt in NVIC
//
//    for (i = 0; i < BUFFER_SIZE; i++) {
//		srcBuff[i] = 0;
//		destBuff[i] = 0;
//    }
//
//    xfer.txData = srcBuff;
//    xfer.rxData = destBuff;
//    xfer.dataSize = BUFFER_SIZE;
//    xfer.configFlags = kSPI_FrameAssert;
//
//    SPI_MasterTransferCreateHandle(SPI_MASTER, &handle, masterCallback, NULL);
//    SPI_MasterTransferNonBlocking(SPI_MASTER, &handle, &xfer);
//
//}
//
//void Send_Data_SPI(uint8_t *data)
//{
//	if (masterFinished == true)
//	{
//		masterFinished = false;
//
//		uint8_t len = sizeof(data)/sizeof(data[0]);
//
//		for (i = 0; i < len; i++) {
//			srcBuff[i] = data[i];
//			destBuff[i] = 0;
//		}
//
//        xfer.txData = srcBuff;
//        xfer.rxData = destBuff;
//		xfer.dataSize = len;
//		xfer.configFlags = kSPI_FrameAssert;
//		SPI_MasterTransferNonBlocking(SPI_MASTER, &handle, &xfer);
//	}
//}
//
//bool isMasterFinished(void) {
//	return masterFinished;
//}
