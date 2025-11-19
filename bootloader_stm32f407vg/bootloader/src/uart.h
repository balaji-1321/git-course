
#pragma once
#include <stdint.h>

void uart_init(uint32_t baud);
void uart_send(char c);
char uart_read(void);
void uart_send_str(const char *s);
