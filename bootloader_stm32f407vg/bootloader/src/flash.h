
#pragma once
#include "stm32f4xx.h"
#include <stdint.h>

void flash_unlock(void);
void flash_lock(void);
void flash_erase_sector(uint8_t sector);
void flash_program_word(uint32_t addr, uint32_t data);

static inline void erase_flash_region(uint32_t start, uint32_t size)
{
    // Map region to STM32F407 sectors
    for (uint32_t addr = start; addr < start + size; addr += 0x20000) {
        uint8_t sector = (addr - 0x08000000) / 0x20000;
        flash_erase_sector(sector);
    }
}
