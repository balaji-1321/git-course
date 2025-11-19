
#include "update.h"
#include "flash.h"
#include "uart.h"
#include "settings.h"

void copy_slot_b_to_slot_a(void)
{
    uart_send_str("[UPDATE] Copying slot B to slot A...\r\n");

    erase_flash_region(APP_SLOT_A_START, APP_SLOT_A_SIZE);

    uint32_t *src = (uint32_t*)APP_SLOT_B_START;
    uint32_t *dst = (uint32_t*)APP_SLOT_A_START;

    for (uint32_t i = 0; i < APP_SLOT_A_SIZE; i += 4)
        flash_program_word((uint32_t)&dst[i/4], src[i/4]);
}

int is_valid_app(uint32_t addr)
{
    uint32_t sp = *(uint32_t*)addr;
    return (sp > 0x20000000 && sp < 0x20030000);
}

void try_apply_update(void)
{
    if (settings.update_flag && settings.use_slot_b)
    {
        uart_send_str("[UPDATE] Pending update found.\r\n");

        if (is_valid_app(APP_SLOT_B_START)) {
            copy_slot_b_to_slot_a();
            settings.update_flag = 0;
            settings.use_slot_b = 0;
            save_settings();
            uart_send_str("[UPDATE] Update applied.\r\n");
        }
        else {
            uart_send_str("[UPDATE] Invalid firmware in slot B!\r\n");
            settings.update_flag = 0;
            save_settings();
        }
    }
}
