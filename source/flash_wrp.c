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


    /* Print basic information for Flash Driver API.*/
    PRINTF("\r\nFlash driver API tree Demo Application...\r\n");
    /* Initialize flash driver */
    PRINTF("Initializing flash driver...\r\n");
    if (FLASH_Init(&flashInstance) == kStatus_Success)
    {
        PRINTF("Flash init successfull!!. Halting...\r\n");
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

    /* print welcome message */
    PRINTF("\r\n PFlash Example Start \r\n");
    /* Print flash information - PFlash. */
    PRINTF("\tkFLASH_PropertyPflashBlockBaseAddr = 0x%X\r\n", pflashBlockBase);
    PRINTF("\tkFLASH_PropertyPflashSectorSize = %d\r\n", pflashSectorSize);
    PRINTF("\tkFLASH_PropertyPflashTotalSize = %d\r\n", pflashTotalSize);
    PRINTF("\tkFLASH_PropertyPflashPageSize = 0x%X\r\n", PflashPageSize);

    destAdrss = pflashBlockBase + (pflashTotalSize - (PAGE_INDEX_FROM_END * PflashPageSize));
    PRINTF("destAdrss: %x\r\n", destAdrss);


}

void erase_flash(void)
{

}


void program_flash(uint32_t *data, uint32_t size)
{
    PRINTF("\r\nCalling FLASH_Erase() API...\r\n");
    status = FLASH_Erase(&flashInstance, destAdrss, PflashPageSize, kFLASH_ApiEraseKey);
    verify_status(status);
    PRINTF("Done!\r\n");

    /* Verify if the given flash range is successfully erased. */
    PRINTF("Calling FLASH_VerifyErase() API...\r\n");
    status = FLASH_VerifyErase(&flashInstance, destAdrss, PflashPageSize);
    verify_status(status);
    if (status == kStatus_Success)
    {
        PRINTF("FLASH Verify erase successful!\n");
    }
    else
    {
        error_trap();
    }

    /* Start programming specified flash region */
    PRINTF("Calling FLASH_Program() API...\r\n");
    status = FLASH_Program(&flashInstance, destAdrss, (uint8_t *)data, size);
    verify_status(status);

    /* Verify if the given flash region is successfully programmed with given data */
    PRINTF("Calling FLASH_VerifyProgram() API...\r\n");
    status = FLASH_VerifyProgram(&flashInstance, destAdrss, size, (uint8_t *)data, &failedAddress,
                                 &failedData);
    verify_status(status);

    if (status == kStatus_Success)
    {
        PRINTF("FLASH Verify Program successful!\n");
    }
    else
    {
        error_trap();
    }
}

void read_flash(uint32_t *data, uint32_t size)
{

	PRINTF("Reading flash memory...\r\n");
	/* Verify programming by reading back from flash directly */
    for (uint32_t i = 0; i < size; i++)
    {
    	data[i] = *(volatile uint32_t *)(destAdrss + i * 4);
    	PRINTF("0x%x ", data[i]);
    }
    PRINTF("\r\n Successfully Programmed and Verified Location 0x%x -> 0x%x \r\n", destAdrss,
           (destAdrss + size ));

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
	PRINTF("Status: %s\r\n", tipString);
}


void error_trap(void)
{
    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1)
    {
    }
}
