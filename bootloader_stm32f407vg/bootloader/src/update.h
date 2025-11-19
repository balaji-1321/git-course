
#pragma once
#include <stdint.h>

#define APP_SLOT_A_START  0x08009000
#define APP_SLOT_A_SIZE   (460 * 1024)

#define APP_SLOT_B_START  0x0807F000
#define APP_SLOT_B_SIZE   (460 * 1024)

int is_valid_app(uint32_t addr);
void try_apply_update(void);
void copy_slot_b_to_slot_a(void);
