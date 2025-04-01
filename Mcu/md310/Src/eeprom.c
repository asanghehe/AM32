/*
 * bootloader.c
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 *
 */

#include "eeprom.h"

#include <string.h>

// #define APP_START (uint32_t)0x08001000
// #define FLASH_STORAGE 0x08005000  // at the 31kb mark
// #define sect_size 0x00001000U // sector size 4Kb

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{

    volatile uint32_t data_length = length / 4;

		/* Unlock FLASH */
		LL_FLASH_Unlock(FLASH);

		uint32_t flash_program_start = add ;                                /* Start address of user erase page */
		uint32_t flash_program_end = (add + data_length);                  /* End address of user write flash */

		while (flash_program_start < flash_program_end)
		{
			/* Wait Busy=0 */
			while(LL_FLASH_IsActiveFlag_BUSY(FLASH)==1);

			/* Enable EOP */
			/* Enable Page Erase */
			FLASH->CR |= FLASH_CR_PER | FLASH_CR_EOPIE;

			/* Set Erase Address */
			LL_FLASH_SetEraseAddress(FLASH,flash_program_start);

			/* Wait Busy=0 */
			while(LL_FLASH_IsActiveFlag_BUSY(FLASH)==1);

			/* Wait EOP=1 */
			while(LL_FLASH_IsActiveFlag_EOP(FLASH)==0);

			/* Clear EOP */
			LL_FLASH_ClearFlag_EOP(FLASH);

			/* Disable EOP */
			/* Disable Page Erase */
			FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_EOPIE);
			
			flash_program_start += FLASH_PAGE_SIZE;                                           /* Point to the start address of the next page to be erase */
		}
		
		flash_program_start = add;
		uint32_t index = 0;
		while (flash_program_start < flash_program_end)
		{
			
			uint32_t data_to_FLASH[FLASH_PAGE_SIZE];
			
			memcpy((void*)&data_to_FLASH[0], &data[index*4], FLASH_PAGE_SIZE);

			/* Wait Busy=0 */
			while(LL_FLASH_IsActiveFlag_BUSY(FLASH)==1);
			
			/* Enable EOP */
			/* Enable Program */
			FLASH->CR |= FLASH_CR_PG | FLASH_CR_EOPIE;

			/* Page Program */
			LL_FLASH_PageProgram(FLASH, flash_program_start, (uint32_t *)data_to_FLASH);
			
			/* Wait Busy=0 */
			while(LL_FLASH_IsActiveFlag_BUSY(FLASH)==1);
			
			/* Wait EOP=1 */
			while(LL_FLASH_IsActiveFlag_EOP(FLASH)==0);
			
			/* Clear EOP */
			LL_FLASH_ClearFlag_EOP(FLASH);
		 
			/* Disable EOP */
			/* Disable Program */
			FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_EOPIE);
			
			flash_program_start += FLASH_PAGE_SIZE;                                           /* Point to the start address of the next page to be written */
			index += FLASH_PAGE_SIZE;
		}
		
		/* Lock FLASH */
		LL_FLASH_Lock(FLASH);
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    // volatile uint32_t read_data;
    //for (int i = 0; i < out_buff_len; i++) {
    //    data[i] = *(uint8_t*)(add + i);
    //}
		memcpy(data, (void*)add, out_buff_len);
}
