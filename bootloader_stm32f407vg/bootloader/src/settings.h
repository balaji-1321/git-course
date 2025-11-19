
#pragma once
#include <stdint.h>

#define SETTINGS_START    0x08008000
#define SETTINGS_MAGIC    0xDEADBEEF

typedef struct {
    uint32_t magic;
    uint32_t baudrate;
    uint32_t fw_version_a;
    uint32_t fw_version_b;
    uint32_t update_flag;
    uint32_t use_slot_b;
} settings_t;

extern settings_t settings;

void load_settings(void);
void save_settings(void);
