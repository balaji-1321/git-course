
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
