#include <stdint.h>

/* ===================== WAV DATA ===================== */
extern const unsigned char wav[];
extern const unsigned int wav_len;

/* Skip WAV header */
#define WAV_DATA_OFFSET 44

/* ===================== REGISTER ADDRESSES ===================== */

/* RCC */
#define RCC_BASE        0x40023800
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_PLLI2SCFGR  (*(volatile uint32_t *)(RCC_BASE + 0x84))

/* GPIO */
#define GPIOA_BASE      0x40020000
#define GPIOC_BASE      0x40020800
#define GPIOD_BASE      0x40020C00

#define GPIO_MODER(x)   (*(volatile uint32_t *)(x + 0x00))
#define GPIO_AFRL(x)    (*(volatile uint32_t *)(x + 0x20))
#define GPIO_AFRH(x)    (*(volatile uint32_t *)(x + 0x24))
#define GPIO_ODR(x)     (*(volatile uint32_t *)(x + 0x14))

/* SPI3 / I2S */
#define SPI3_BASE       0x40003C00
#define SPI_I2SCFGR     (*(volatile uint32_t *)(SPI3_BASE + 0x1C))
#define SPI_I2SPR       (*(volatile uint32_t *)(SPI3_BASE + 0x20))
#define SPI_DR          (*(volatile uint32_t *)(SPI3_BASE + 0x0C))
#define SPI_SR          (*(volatile uint32_t *)(SPI3_BASE + 0x08))

/* ===================== SIMPLE DELAY ===================== */
void delay(volatile uint32_t d)
{
    while (d--);
}

/* ===================== GPIO INIT ===================== */
void gpio_init(void)
{
    /* Enable GPIOA, GPIOC, GPIOD */
    RCC_AHB1ENR |= (1 << 0) | (1 << 2) | (1 << 3);

    /* PA4 → AF6 (I2S_WS) */
    GPIO_MODER(GPIOA_BASE) |= (2 << (4 * 2));
    GPIO_AFRL(GPIOA_BASE) |= (6 << (4 * 4));

    /* PC7, PC10, PC12 → AF6 */
    GPIO_MODER(GPIOC_BASE) |= (2 << (7 * 2)) | (2 << (10 * 2)) | (2 << (12 * 2));
    GPIO_AFRL(GPIOC_BASE) |= (6 << (7 * 4));
    GPIO_AFRH(GPIOC_BASE) |= (6 << ((10 - 8) * 4)) | (6 << ((12 - 8) * 4));

    /* PD4 → Codec Reset */
    GPIO_MODER(GPIOD_BASE) |= (1 << (4 * 2));
    GPIO_ODR(GPIOD_BASE) &= ~(1 << 4);
    delay(100000);
    GPIO_ODR(GPIOD_BASE) |= (1 << 4);
}

/* ===================== I2S CLOCK INIT ===================== */
void i2s_clock_init(void)
{
    /* Enable SPI3 clock */
    RCC_APB1ENR |= (1 << 15);

    /* PLLI2S for 44.1kHz approx */
    RCC_PLLI2SCFGR = (192 << 6) | (2 << 28);
}

/* ===================== I2S INIT ===================== */
void i2s_init(void)
{
    SPI_I2SCFGR = 0;

    /* I2S Mode | Master TX | Philips | 16-bit */
    SPI_I2SCFGR =
        (1 << 11) |   /* I2SMOD */
        (2 << 8)  |   /* Master Transmit */
        (0 << 4);     /* 16-bit */

    /* Prescaler */
    SPI_I2SPR = (2 << 0) | (1 << 9); /* MCK enabled */

    /* Enable I2S */
    SPI_I2SCFGR |= (1 << 10);
}

/* ===================== PLAY AUDIO ===================== */
void i2s_play(const unsigned char *data, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i += 2)
    {
        uint16_t sample = data[i] | (data[i + 1] << 8);

        while (!(SPI_SR & (1 << 1))); /* TXE */
        SPI_DR = sample;
    }
}

/* ===================== MAIN ===================== */
int main(void)
{
    gpio_init();
    i2s_clock_init();
    i2s_init();

    while (1)
    {
        i2s_play(&wav[WAV_DATA_OFFSET], wav_len - WAV_DATA_OFFSET);
    }
}
