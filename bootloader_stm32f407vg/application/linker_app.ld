
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
