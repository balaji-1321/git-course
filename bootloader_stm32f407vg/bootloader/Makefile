
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
