
#include "settings.h"
#include "flash.h"
#include <string.h>

settings_t settings;

void load_settings(void)
{
    settings_t *mem = (settings_t*)SETTINGS_START;

    if (mem->magic == SETTINGS_MAGIC) {
        memcpy(&settings, mem, sizeof(settings));
    } else {
        settings.magic = SETTINGS_MAGIC;
        settings.baudrate = 115200;
        settings.fw_version_a = 0;
        settings.fw_version_b = 0;
        settings.update_flag = 0;
        settings.use_slot_b = 0;
    }
}

void save_settings(void)
{
    flash_unlock();
    flash_erase_sector(2);

    uint32_t *src = (uint32_t*)&settings;
    uint32_t *dst = (uint32_t*)SETTINGS_START;

    for (int i = 0; i < sizeof(settings)/4; i++)
        flash_program_word((uint32_t)&dst[i], src[i]);

    flash_lock();
}
