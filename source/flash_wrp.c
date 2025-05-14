/*
 * flash_wrp.c
 *
 *  Created on: 8 mar. 2025
 *      Author: agust
 */

#include "flash_wrp.h"


static void verify_status(status_t status);
static void error_trap();


static flash_config_t flashInstance;
static uint32_t status;
static uint32_t destAdrss; /* Address of the target location */
static uint32_t failedAddress, failedData;
static uint32_t pflashBlockBase            = 0;
static uint32_t pflashTotalSize            = 0;
static uint32_t pflashSectorSize           = 0;
static uint32_t PflashPageSize             = 0;




void init_flash(void)
{


    /* Initialize flash driver */
    uartWriteStr("Initializing flash driver...\n");
    if (FLASH_Init(&flashInstance) == kStatus_Success)
    {
        uartWriteStr("Flash init successfull!!. Halting...\n");
    }
    else
    {
        error_trap();
    }
    /* Get flash properties kFLASH_ApiEraseKey */
    FLASH_GetProperty(&flashInstance, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
    FLASH_GetProperty(&flashInstance, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);
    FLASH_GetProperty(&flashInstance, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
    FLASH_GetProperty(&flashInstance, kFLASH_PropertyPflashPageSize, &PflashPageSize);

    destAdrss = pflashBlockBase + (pflashTotalSize - (PAGE_INDEX_FROM_END * PflashPageSize));


}

void erase_flash(void)
{

}


void program_flash(uint32_t *data, uint32_t size)
{
    uartWriteStr("Calling FLASH_Erase() API...\n");
    status = FLASH_Erase(&flashInstance, destAdrss, PflashPageSize, kFLASH_ApiEraseKey);
    verify_status(status);
    uartWriteStr("Done!\n");

    /* Verify if the given flash range is successfully erased. */
    uartWriteStr("Calling FLASH_VerifyErase() API...\n");
    status = FLASH_VerifyErase(&flashInstance, destAdrss, PflashPageSize);
    verify_status(status);
    if (status == kStatus_Success)
    {
        uartWriteStr("FLASH Verify erase successful!\n");
    }
    else
    {
        error_trap();
    }

    /* Start programming specified flash region */
    uartWriteStr("Calling FLASH_Program() API...\n");
    status = FLASH_Program(&flashInstance, destAdrss, (uint8_t *)data, size);
    verify_status(status);

    /* Verify if the given flash region is successfully programmed with given data */
    uartWriteStr("Calling FLASH_VerifyProgram() API...\n");
    status = FLASH_VerifyProgram(&flashInstance, destAdrss, size, (uint8_t *)data, &failedAddress,
                                 &failedData);
    verify_status(status);

    if (status == kStatus_Success)
    {
        uartWriteStr("FLASH Verify Program successful!\n");
    }
    else
    {
        error_trap();
    }
}

void read_flash(uint32_t *data, uint32_t size)
{
    /* Check if the flash page is blank before reading to avoid hardfault. */
    status = FLASH_VerifyErase(&flashInstance, destAdrss, PflashPageSize);
    if (status == kStatus_Success)
    {
        uartWriteStr("Error: trying to Read blank flash page!\n");
		for (uint32_t i = 0; i < size; i++)
		{
			data[i] = 0;
		}
		return;
    }
    else
    {
    	uartWriteStr("Reading flash memory...\r\n");
    	/* Verify programming by reading back from flash directly */
        for (uint32_t i = 0; i < size; i++)
        {
        	data[i] = *(volatile uint32_t *)(destAdrss + i * 4);
        }
    }

}


void verify_status(status_t status) {
	char *tipString = "Unknown status";
	switch (status) {
	case kStatus_Success:
		tipString = "Done.";
		break;
	case kStatus_InvalidArgument:
		tipString = "Invalid argument.";
		break;
	case kStatus_FLASH_AlignmentError:
		tipString = "Alignment Error.";
		break;
	case kStatus_FLASH_AccessError:
		tipString = "Flash Access Error.";
		break;
	case kStatus_FLASH_CommandNotSupported:
		tipString = "This API is not supported in current target.";
		break;
	default:
		break;
	}
}


void error_trap(void)
{
    uartWriteStr("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");

}
