
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
