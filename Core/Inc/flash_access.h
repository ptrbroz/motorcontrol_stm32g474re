#ifndef FLASH_ACCESS_H
#define FLASH_ACCESS_H

//#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx.h"
#include "user_config.h"



void load_from_flash();

int erase_reserved_flash();

int save_to_flash();


#endif
