
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
