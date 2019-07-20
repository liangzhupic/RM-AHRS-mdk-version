#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "uc_memory.h"

/*****************************************************************************
 * Allows to use the internal flash to store non volatile data. To initialize
 * the functionality use the FEE_Init() function. Be sure that by reprogramming
 * of the controller just affected pages will be deleted. In other case the non
 * volatile data will be lost.
******************************************************************************/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/
FLASH_EraseInitTypeDef erase_handle;
uint32_t err_flag;
uint8_t DataBuf[FEE_PAGE_SIZE];
/*****************************************************************************
*  Delete Flash Space used for user Data, deletes the whole space between
*  RW_PAGE_BASE_ADDRESS and the last uC Flash Page
******************************************************************************/
uint16_t
FEE_Init(void) {
    erase_handle.TypeErase = FLASH_TYPEERASE_PAGES;

	// unlock flash
    HAL_FLASH_Unlock();

	// Clear Flags
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR);
	return FEE_DENSITY_BYTES;
}
/*****************************************************************************
*  Erase the whole reserved Flash Space used for user Data
******************************************************************************/
void
FEE_Erase (void) {

//	int page_num = 0;

    erase_handle.PageAddress = FEE_PAGE_BASE_ADDRESS;
    erase_handle.NbPages = FEE_DENSITY_PAGES;
    // delete all pages from specified start page to the last page
    HAL_FLASHEx_Erase(&erase_handle, &err_flag);
//	do {

//		FLASH_ErasePage(FEE_PAGE_BASE_ADDRESS + (page_num * FEE_PAGE_SIZE));
//		page_num++;
//	} while (page_num < FEE_DENSITY_PAGES);
}
/*****************************************************************************
*  Writes once data byte to flash on specified address. If a byte is already
*  written, the whole page must be copied to a buffer, the byte changed and
*  the manipulated buffer written after PageErase.
*******************************************************************************/
uint16_t
FEE_WriteDataByte (uint16_t Address, uint8_t DataByte) {

    HAL_StatusTypeDef FlashStatus = HAL_OK;

	uint32_t page;
	int i;

	// exit if desired address is above the limit (e.G. under 2048 Bytes for 4 pages)
	if (Address > FEE_DENSITY_BYTES) {
		return 0;
	}

	// calculate which page is affected (Pagenum1/Pagenum2...PagenumN)
	page = (FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address)) & 0x00000FFF;

	if (page % FEE_PAGE_SIZE) page = page + FEE_PAGE_SIZE;
	page = (page / FEE_PAGE_SIZE) - 1;

	// if current data is 0xFF, the byte is empty, just overwrite with the new one
	if ((*(uint16_t*)(FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address))) == FEE_EMPTY_WORD) {
        FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address), (uint16_t)(0x00FF & DataByte) );
        //FlashStatus = FLASH_ProgramHalfWord(FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address), (uint16_t)(0x00FF & DataByte));
	}
	else {

		// Copy Page to a buffer
		memcpy(DataBuf, (uint8_t*)FEE_PAGE_BASE_ADDRESS + (page * FEE_PAGE_SIZE), FEE_PAGE_SIZE); // !!! Calculate base address for the desired page

		// check if new data is differ to current data, return if not, proceed if yes
		if (DataByte == *(uint8_t*)(FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address))) {
			return 0;
		}

		// manipulate desired data byte in temp data array if new byte is differ to the current
		DataBuf[FEE_ADDR_OFFSET(Address)] = DataByte;

		//Erase Page
        //FlashStatus = FLASH_ErasePage(FEE_PAGE_BASE_ADDRESS + page);
        erase_handle.PageAddress = FEE_PAGE_BASE_ADDRESS;
        erase_handle.NbPages = 1;
        HAL_FLASHEx_Erase(&erase_handle, &err_flag);
		// Write new data (whole page) to flash if data has beed changed
		for(i = 0; i < (FEE_PAGE_SIZE / 2); i++) {
			if ((uint16_t)(0xFF00 | DataBuf[FEE_ADDR_OFFSET(i)]) != 0xFFFF) {
                FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,  (FEE_PAGE_BASE_ADDRESS + (page * FEE_PAGE_SIZE)) + (i * 2), (uint16_t)(0xFF00 | DataBuf[FEE_ADDR_OFFSET(i)]));
                //FlashStatus = FLASH_ProgramHalfWord((FEE_PAGE_BASE_ADDRESS + (page * FEE_PAGE_SIZE)) + (i * 2), (uint16_t)(0xFF00 | DataBuf[FEE_ADDR_OFFSET(i)]));
			}
		}

	}
	return FlashStatus;
}
/*****************************************************************************
*  Read once data byte from a specified address.
*******************************************************************************/
uint8_t
FEE_ReadDataByte (uint16_t Address) {

	uint8_t DataByte = 0xFF;

	// Get Byte from specified address
	DataByte = (*(uint8_t*)(FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address)));

	return DataByte;
}
uint8_t bytes[4];
float FEE_ReadDataFloat(uint16_t Add)
{
    float data;

    bytes[3] = FEE_ReadDataByte(Add) ;
    bytes[2] = FEE_ReadDataByte(Add+1) ;
    bytes[1] = FEE_ReadDataByte(Add+2) ;
    bytes[0] = FEE_ReadDataByte(Add+3) ;
//    data = bytes[0]| (bytes[1]<< 8) |( bytes[2] << 16) | (bytes[3] << 24) ;
    data = *((float*)bytes);
    return data;
}

void FEE_WriteDataFloat(uint16_t Add, float data)
{
   // uint8_t bytes[4];
    uint8_t * ptr = (uint8_t*)(&data);
    bytes[0] = ptr[0];
    bytes[1] = ptr[1];
    bytes[2] = ptr[2];
    bytes[3] = ptr[3];

    FEE_WriteDataByte(Add, bytes[3]);
    FEE_WriteDataByte(Add+1, bytes[2]);
    FEE_WriteDataByte(Add+2, bytes[1]);
    FEE_WriteDataByte(Add+3, bytes[0]);
}

