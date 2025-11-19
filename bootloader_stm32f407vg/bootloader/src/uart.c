
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
