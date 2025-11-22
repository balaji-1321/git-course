\
/* FLASH (where code is stored) */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   (rwx): ORIGIN = 0x20000000, LENGTH = 128K
}

ENTRY(Reset_Handler)

SECTIONS
{
    .isr_vector :
    {
        _isr_vector_ram = .;
        KEEP(*(.isr_vector))
        _isr_vector_ram_end = .;
    } > RAM AT > FLASH

    .text :
    {
        _text_ram = .;
        *(.text*)
        *(.rodata*)
        _text_ram_end = .;
    } > RAM AT > FLASH

    _text_flash = LOADADDR(.text);

    .data :
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM AT > FLASH

    _data_flash = LOADADDR(.data);

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM
}
