
#include "flash.h"

void flash_unlock(void)
{
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
}

void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

void flash_erase_sector(uint8_t sector)
{
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR = FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
}

void flash_program_word(uint32_t addr, uint32_t data)
{
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR = FLASH_CR_PG;
    *(volatile uint32_t*)addr = data;
    while (FLASH->SR & FLASH_SR_BSY);
}
