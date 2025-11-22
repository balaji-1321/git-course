#include <stdint.h>

extern uint32_t _isr_vector_ram, _isr_vector_ram_end;
extern uint32_t _text_ram, _text_ram_end, _text_flash;
extern uint32_t _sdata, _edata, _data_flash;
extern uint32_t _sbss, _ebss;

int main(void);

/* ------------ VECTOR TABLE (FLASH, but copied to RAM) ------------ */
__attribute__((section(".isr_vector")))
const void *vector_table_flash[] = {
    (void*)0x20020000,    /* Initial SP */
    (void*)Reset_Handler, /* Reset */
};

/* ------------ RESET HANDLER (in Flash, but moves system to RAM) ------------ */
void Reset_Handler(void)
{
    uint32_t *src, *dst;

    /* Copy vector table (FLASH → RAM) */
    src = (uint32_t*)vector_table_flash;
    dst = &_isr_vector_ram;
    while (dst < &_isr_vector_ram_end)
        *dst++ = *src++;

    /* Copy .text (FLASH → RAM) */
    src = &_text_flash;
    dst = &_text_ram;
    while (dst < &_text_ram_end)
        *dst++ = *src++;

    /* Copy .data (FLASH → RAM) */
    src = &_data_flash;
    dst = &_sdata;
    while (dst < &_edata)
        *dst++ = *src++;

    /* Clear .bss */
    dst = &_sbss;
    while (dst < &_ebss)
        *dst++ = 0;

    /* Set vector table to RAM */
    *((volatile uint32_t*)0xE000ED08) = (uint32_t)&_isr_vector_ram;

    /* Set MSP from RAM vector table */
    __asm volatile ("ldr r0, =_isr_vector_ram\n"
                    "ldr r0, [r0]\n"
                    "msr msp, r0");

    /* Jump to main in RAM */
    main();

    while (1);
}
