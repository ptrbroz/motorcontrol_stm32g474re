/*
Utils meant to replace Ben Katz's original FlashWriter/PreferenceWriter libraries (which cannot be easily ported as they use a
non-HAL flash library not available on STM32G474RET6).

Purpose: facilitate reading/writing callibration data between flash memory and global variables declared in user_config.h
Maybe replace with EEPROM emulation or implement wear-leveling in the future. (10 000 write/erase cycles guaranteed per datasheet
- probably good enough if recallibration occurs infrequently enough)
*/


//#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx.h"
#include "flash_access.h"
#include "user_config.h"

#include "stdio.h"


#define PAGELEN 2048
#define BANK2_START 0x08040000
#define BANK1_START 0x08000000
#define RESERVED_PAGE  126      //max prog. upload reduced by 10k in platformio.ini, reserving pages 123 to 127 of bank 2
#define RESERVED_ADDR BANK2_START + PAGELEN*RESERVED_PAGE

#define FLOATSCOUNT 64
#define INTSCOUNT 256

//#define FLOATS_ADDR RESERVED_ADDR
#define FLOATS_ADDR RESERVED_ADDR
#define INTS_ADDR RESERVED_ADDR + FLOATSCOUNT*sizeof(float)


/**
  * @brief  Loads floats and ints from flash memory into global arrays
  */
void load_from_flash(){
    for(int i = 0;i<FLOATSCOUNT;i=i+2){
    	//int temp = FLOATS_ADDR + i*sizeof(float);
    	//float read = *((float*)(FLOATS_ADDR + i*sizeof(float)));

    	//heureka!!
    	int betterAdr = FLOATS_ADDR + i*4;
    	uint64_t doubleWord = *((uint64_t*)(betterAdr));

    	uint32_t word1 = doubleWord&0x00000000ffffffff;
    	uint32_t word2 = (doubleWord&0xffffffff00000000) >> 32;

    	float float1 = *((float*)(&word1));
    	float float2 = *((float*)(&word2));

    	__float_reg[i] =   float1;
    	__float_reg[i+1] = float2;

    }
    for(int i = 0;i<INTSCOUNT;i++){
        __int_reg[i] = *((int*)(INTS_ADDR + i*sizeof(float)));
    }
}

/**
  * @brief  Erase used page of flash memory to prepare for reprogramming
  * @retval uint32_t PageError. 0xFFFFFFFF means no problem
  */
int erase_reserved_flash(){
    FLASH_EraseInitTypeDef eraseStruct;
    eraseStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseStruct.Banks = FLASH_BANK_2;
    eraseStruct.Page = RESERVED_PAGE;
    eraseStruct.NbPages = 1;
    uint32_t error;
    HAL_FLASHEx_Erase(&eraseStruct, &error);
    printf("Leaving erase flash.");
    return error;
}


/**
  * @brief  Saves floats and ints from global arrays into flash memory.
  * This should be done sparingly as it causes wear of flash memory and shortens it's lifespan.
  * Should it become necessary to perform this often, consider implementing some form of wear leveling.
  * @retval Zero when OK, nonzero when an error was encountered
  */
int save_to_flash(){
	printf("SaveToFlashy: ");

	HAL_StatusTypeDef status = HAL_FLASH_Unlock();
	if(status!=HAL_OK) return 1;

	status = HAL_FLASH_OB_Unlock();
	if(status!=HAL_OK) return 2;

	printf(" >unlocked ");

    unsigned int eraseError = erase_reserved_flash();
    if(eraseError!=0xFFFFFFFF) return 3;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_SR_ERRORS);

    printf(" >erased ");

    status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
    if(status!=HAL_OK) return 5;

    printf(" >floatss ");

    for(int i=0;i<FLOATSCOUNT;i=i+2){
        uint64_t doubleWord = *((uint64_t*) (__float_reg + i)); //read two floats from array as one uint64
        //float floatArr[2] = {4.2, 4.22};
        //doubleWord = *((uint64_t*) (floatArr));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLOATS_ADDR + i*sizeof(float), doubleWord);
        if(status!=HAL_OK) {printf("fail f %d", i);return 6;}
    }
    printf(" >ints ");
    printf("SKIP ");
    for(int i=300;i<INTSCOUNT;i=i+2){
        uint64_t doubleWord = *((uint64_t*) __int_reg + i);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLOATS_ADDR + i*sizeof(int), doubleWord);
        if(status!=HAL_OK) {printf("fail i %d", i);return 7;}
    }
    HAL_FLASH_Lock();
    HAL_FLASH_OB_Lock();
    printf("> SaveToFlash All Ok\n");
    return 0;
}
