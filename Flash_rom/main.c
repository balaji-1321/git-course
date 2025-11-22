\
#include <stdint.h>

#define PERIPH_BASE   0x40000000
#define AHB1_OFFSET   0x00020000
#define AHB1_BASE     (PERIPH_BASE + AHB1_OFFSET)

#define RCC_BASE      (AHB1_BASE + 0x3800)
#define GPIOD_BASE    (AHB1_BASE + 0x0C00)

#define RCC_AHB1ENR   (*(volatile uint32_t*)(RCC_BASE  + 0x30))
#define GPIOD_MODER   (*(volatile uint32_t*)(GPIOD_BASE + 0x00))
#define GPIOD_ODR     (*(volatile uint32_t*)(GPIOD_BASE + 0x14))

void delay(volatile uint32_t t)
{
    while (t--);
}

int main(void)
{
    RCC_AHB1ENR |= (1 << 3);

    GPIOD_MODER &= ~(3 << (12 * 2));
    GPIOD_MODER |=  (1 << (12 * 2));

    while (1)
    {
        GPIOD_ODR ^= (1 << 12);
        delay(400000);
    }
}
