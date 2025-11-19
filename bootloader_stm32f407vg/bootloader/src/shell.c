
#include "shell.h"
#include "uart.h"
#include "flash.h"
#include "settings.h"
#include "update.h"
#include "jump.h"
#include <string.h>
#include <stdlib.h>

static char line[128];
static int idx = 0;

void write_to_slot(char *p, uint32_t base)
{
    uint32_t offset = strtoul(p, &p, 16);
    while (*p == ' ') p++;
    uint32_t addr = base + offset;

    uint8_t buf[64];
    int len = 0;
    while (p[0] && p[1]) {
        char b[3] = {p[0], p[1], 0};
        buf[len++] = strtoul(b,0,16);
        p+=2;
    }

    flash_unlock();
    for (int i=0;i<len;i+=4){
        uint32_t w = buf[i] | (buf[i+1]<<8) | (buf[i+2]<<16) | (buf[i+3]<<24);
        flash_program_word(addr, w);
        addr+=4;
    }
    flash_lock();
    uart_send_str("OK\r\n");
}

void process_command(char *cmd)
{
    if (!strcmp(cmd,"info")){
        uart_send_str("[INFO] Bootloader STM32F407VG\r\n");
    }
    else if (!strcmp(cmd,"reboot")){
        NVIC_SystemReset();
    }
    else if (!strncmp(cmd,"flash erase_app",15)){
        erase_flash_region(APP_SLOT_A_START, APP_SLOT_A_SIZE);
        uart_send_str("App erased\r\n");
    }
    else if (!strncmp(cmd,"flash erase_update",18)){
        erase_flash_region(APP_SLOT_B_START, APP_SLOT_B_SIZE);
        uart_send_str("Update erased\r\n");
    }
    else if (!strncmp(cmd,"flash write_app ",16)){
        write_to_slot(cmd+16, APP_SLOT_A_START);
    }
    else if (!strncmp(cmd,"flash write_update ",20)){
        write_to_slot(cmd+20, APP_SLOT_B_START);
    }
    else if (!strcmp(cmd,"update commit")){
        settings.update_flag = 1;
        settings.use_slot_b = 1;
        save_settings();
        uart_send_str("Update scheduled\r\n");
    }
    else if (!strcmp(cmd,"flash run")){
        jump_to_application(APP_SLOT_A_START);
    }
    else {
        uart_send_str("Unknown\r\n");
    }
}

void shell_loop(void)
{
    uart_send_str("> ");

    while(1){
        char c = uart_read();
        if (c=='\r' || c=='\n'){
            line[idx]=0;
            process_command(line);
            idx=0;
            uart_send_str("> ");
        } else {
            line[idx++] = c;
            uart_send(c);
        }
    }
}
