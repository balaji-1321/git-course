#!/usr/bin/env python3
import os

PROJECT = "bootloader_stm32f407vg"

# --- File contents -------------------------------------------------------------

files = {
# -----------------------------
# Bootloader main.c
# -----------------------------
"bootloader/src/main.c": r'''
#include "uart.h"
#include "settings.h"
#include "update.h"
#include "jump.h"
#include "flash.h"
#include "shell.h"

#include "stm32f4xx.h"

int main(void)
{
    SystemInit();
    uart_init(115200);

    uart_send_str("\r\n[BOOT] STM32F407VG Bootloader\r\n");

    load_settings();
    try_apply_update();

    if (is_valid_app(APP_SLOT_A_START)) {
        uart_send_str("[BOOT] Jumping to Application\r\n");
        jump_to_application(APP_SLOT_A_START);
    }

    uart_send_str("[BOOT] Entering shell\r\n");
    shell_loop();
}
''',

# -----------------------------
# flash.c
# -----------------------------
"bootloader/src/flash.c": r'''
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
''',

# -----------------------------
# flash.h
# -----------------------------
"bootloader/src/flash.h": r'''
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
''',

# -----------------------------
# uart.c/h
# -----------------------------
"bootloader/src/uart.c": r'''
#include "uart.h"
#include "stm32f4xx.h"

void uart_init(uint32_t baud)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER |= (2 << (2*2)) | (2 << (2*3));
    GPIOA->AFR[0] |= (7 << (4*2)) | (7 << (4*3));

    USART2->BRR = SystemCoreClock / baud;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void uart_send(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

char uart_read(void)
{
    while (!(USART2->SR & USART_SR_RXNE));
    return USART2->DR;
}

void uart_send_str(const char *s)
{
    while (*s) uart_send(*s++);
}
''',

"bootloader/src/uart.h": r'''
#pragma once
#include <stdint.h>

void uart_init(uint32_t baud);
void uart_send(char c);
char uart_read(void);
void uart_send_str(const char *s);
''',

# -----------------------------
# settings.c/h
# -----------------------------
"bootloader/src/settings.c": r'''
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
''',

"bootloader/src/settings.h": r'''
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
''',

# -----------------------------
# update.c/h
# -----------------------------
"bootloader/src/update.c": r'''
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
''',

"bootloader/src/update.h": r'''
#pragma once
#include <stdint.h>

#define APP_SLOT_A_START  0x08009000
#define APP_SLOT_A_SIZE   (460 * 1024)

#define APP_SLOT_B_START  0x0807F000
#define APP_SLOT_B_SIZE   (460 * 1024)

int is_valid_app(uint32_t addr);
void try_apply_update(void);
void copy_slot_b_to_slot_a(void);
''',

# -----------------------------
# jump.c/h
# -----------------------------
"bootloader/src/jump.c": r'''
#include "jump.h"
#include "stm32f4xx.h"

void jump_to_application(uint32_t app_addr)
{
    uint32_t app_stack = *(uint32_t*)app_addr;
    uint32_t app_reset = *(uint32_t*)(app_addr + 4);

    __disable_irq();

    SCB->VTOR = app_addr;
    __set_MSP(app_stack);

    void (*app)(void) = (void*)app_reset;

    __enable_irq();
    app();
}
''',

"bootloader/src/jump.h": r'''
#pragma once
#include <stdint.h>

void jump_to_application(uint32_t app_addr);
''',

# -----------------------------
# shell.c/h
# -----------------------------
"bootloader/src/shell.c": r'''
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
''',

"bootloader/src/shell.h": r'''
#pragma once
void shell_loop(void);
''',

# -----------------------------
# system_stm32f4xx.c (minimal)
# -----------------------------
"bootloader/src/system_stm32f4xx.c": r'''
#include "stm32f4xx.h"
uint32_t SystemCoreClock = 16000000;
void SystemInit(void){}
''',

# -----------------------------
# startup_stm32f407.s (abbreviated)
# -----------------------------
"bootloader/src/startup_stm32f407.s": r'''
.syntax unified
.cpu cortex-m4
.thumb

.global _estack
_estack = 0x20020000

.global Reset_Handler

.section .isr_vector,"a",%progbits
isr_vector:
    .word _estack
    .word Reset_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0

.section .text.Reset_Handler
.thumb_func
Reset_Handler:
    bl main
    b .
''',

# -----------------------------
# linker_boot.ld
# -----------------------------
"bootloader/linker_boot.ld": r'''
ENTRY(Reset_Handler)

MEMORY {
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
    RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS {
    .isr_vector : {
        KEEP(*(.isr_vector))
    } > FLASH

    .text : {
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > FLASH

    .data : AT ( _etext ) {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM

    .bss : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM
}
''',

# -----------------------------
# bootloader Makefile
# -----------------------------
"bootloader/Makefile": r'''
CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-mcpu=cortex-m4 -mthumb -Os -Wall -I./src
LDFLAGS=-T linker_boot.ld

SRCS=$(wildcard src/*.c)
OBJS=$(SRCS:.c=.o)

all: bootloader.bin

bootloader.elf: $(OBJS) src/startup_stm32f407.o
	$(CC) $(CFLAGS) $(OBJS) src/startup_stm32f407.o -o $@ $(LDFLAGS)

bootloader.bin: bootloader.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f src/*.o *.elf *.bin
''',

# -----------------------------
# Application main.c
# -----------------------------
"application/src/main.c": r'''
#include "stm32f4xx.h"

int main(void)
{
    SystemInit();

    while(1){
        // your main firmware
    }
}
''',

"application/src/system_stm32f4xx.c": r'''
#include "stm32f4xx.h"
uint32_t SystemCoreClock = 16000000;
void SystemInit(void){}
''',

"application/src/startup_stm32f407.s": r'''
.syntax unified
.cpu cortex-m4
.thumb

.global _estack
_estack = 0x20020000

.global Reset_Handler

.section .isr_vector,"a",%progbits
isr_vector:
    .word _estack
    .word Reset_Handler

.section .text.Reset_Handler
.thumb_func
Reset_Handler:
    bl main
    b .
''',

# -----------------------------
# linker_app.ld
# -----------------------------
"application/linker_app.ld": r'''
ENTRY(Reset_Handler)

MEMORY {
    FLASH (rx) : ORIGIN = 0x08009000, LENGTH = 460K
    RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS {
    .isr_vector : {
        KEEP(*(.isr_vector))
    } > FLASH

    .text : {
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > FLASH

    .data : AT ( _etext ) {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM

    .bss : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM
}
''',

# -----------------------------
# tools uploader
# -----------------------------
"tools/uploader.py": r'''
import serial, sys, time

ser = serial.Serial("COM3",115200,timeout=1)

def send(cmd):
    ser.write((cmd+"\r\n").encode())
    print(ser.readline().decode(), end='')

send("flash erase_update")

data = open("app.bin","rb").read()
offset = 0

for i in range(0,len(data),32):
    chunk = data[i:i+32]
    hexstr = chunk.hex()
    send(f"flash write_update {offset} {hexstr}")
    offset += 32

send("update commit")
print("Done. Reset device.")
'''
}

# ---------------------------------------------------------------------

def write_files():
    print(f"Generating project: {PROJECT}")
    for path, content in files.items():
        full = os.path.join(PROJECT, path)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, "w") as f:
            f.write(content)
            print("✓", full)

if __name__ == "__main__":
    write_files()
    print("\nProject created successfully!")
