/*
 * MyFlash.c
 *
 *  Created on: 18 мар. 2022 г.
 *      Author: User
 */
#include "MyFlash.h"
#include "local_files.h"
#define CONFIGURATION_START_ADDR 0x080e0000
void clearFlash(){
	static FLASH_EraseInitTypeDef EraseInitStruct;
	/* Get the 1st sector to erase */
	uint32_t FirstSector = 11;//flash memory sector
	/* Get the number of sector to erase from 1st sector*/
	uint32_t NbOfSectors = 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = NbOfSectors;
	uint32_t SectorError = 0;
	HAL_FLASH_Unlock();
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		/*Error occurred while sector erase.
User can add here some code to deal with this error.
SectorError will contain the faulty sector and then to know the code error on this sector,
user can call function 'HAL_FLASH_GetError()'
		 */
		/*FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		 */
		//Error_Handler();

	}
	HAL_FLASH_Lock();

}
void WriteDeviceAddressOffset(char* data, int size, int offset) {
	uint32_t Address = CONFIGURATION_START_ADDR+offset;
	HAL_FLASH_Unlock();
	for (int i = 0; i<size; i++){
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address+i, data[i]) != HAL_OK){
			/* Error occurred while writing data in Flash memory.
User can add here some code to deal with this error */
			/*
FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
			 */
			//Error_Handler();
			// int error = HAL_FLASH_GetError();

			break;
		}
	}
	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
}
void ReadDeviceAddressOffset(char* Dout, int size, int offset)
{
	uint32_t Address = CONFIGURATION_START_ADDR+offset;

	for (int i = 0; i<size; i++){
		Dout[i] = *(__IO char*)(Address+i);
	}
}
