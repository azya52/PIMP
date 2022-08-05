#ifndef _MSD_FLASH_CONFIG_H_
#define _MSD_FLASH_CONFIG_H_

#include "hardware/flash.h"

#define FLASH_MSD_START_ADDR (128 * 1024)
#define FLASH_MSD_SIZE (2000 * 1024 - FLASH_MSD_START_ADDR)
#define FLASH_MSD_SECTOR_SIZE (FLASH_SECTOR_SIZE)

#endif