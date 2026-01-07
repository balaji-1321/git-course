#include <stdint.h>

/* ================= RCC ================= */
#define RCC_BASE 0x40023800
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE+0x30))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE+0x40))
#define RCC_PLLI2SCFGR (*(volatile uint32_t *)(RCC_BASE+0x84))

/* ================= GPIO ================= */
#define GPIOA 0x40020000
#define GPIOB 0x40020400
#define GPIOC 0x40020800
#define GPIOD 0x40020C00

#define MODER(x) (*(volatile uint32_t *)(x+0x00))
#define OTYPER(x) (*(volatile uint32_t *)(x+0x04))
#define PUPDR(x) (*(volatile uint32_t *)(x+0x0C))
#define ODR(x) (*(volatile uint32_t *)(x+0x14))
#define AFRL(x) (*(volatile uint32_t *)(x+0x20))
#define AFRH(x) (*(volatile uint32_t *)(x+0x24))

/* ================= I2C1 ================= */
#define I2C1 0x40005400
#define CR1 (*(volatile uint32_t *)(I2C1+0x00))
#define CR2 (*(volatile uint32_t *)(I2C1+0x04))
#define DR  (*(volatile uint32_t *)(I2C1+0x10))
#define SR1 (*(volatile uint32_t *)(I2C1+0x14))
#define SR2 (*(volatile uint32_t *)(I2C1+0x18))
#define CCR (*(volatile uint32_t *)(I2C1+0x1C))
#define TRISE (*(volatile uint32_t *)(I2C1+0x20))

/* ================= SPI3 / I2S ================= */
#define SPI3 0x40003C00
#define SPI_DR (*(volatile uint32_t *)(SPI3+0x0C))
#define I2SCFGR (*(volatile uint32_t *)(SPI3+0x1C))
#define I2SPR (*(volatile uint32_t *)(SPI3+0x20))
#define SR (*(volatile uint32_t *)(SPI3+0x08))

#define CS43 0x94

void delay(volatile int d){while(d--);}

/* ================= I2C WRITE ================= */
void i2c_write(uint8_t reg, uint8_t val)
{
    while (SR2 & (1<<1));
    CR1 |= (1<<8);
    while (!(SR1 & 1));
    DR = CS43;
    while (!(SR1 & (1<<1)));
    (void)SR2;
    while (!(SR1 & (1<<7)));
    DR = reg;
    while (!(SR1 & (1<<7)));
    DR = val;
    while (!(SR1 & (1<<2)));
    CR1 |= (1<<9);
}

/* ================= MAIN ================= */
int main(void)
{
    /* Clocks */
    RCC_AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3);
    RCC_APB1ENR |= (1<<21)|(1<<15);

    /* ---- Codec RESET PD4 ---- */
    MODER(GPIOD) |= (1<<(4*2));
    ODR(GPIOD) &= ~(1<<4);
    delay(100000);
    ODR(GPIOD) |= (1<<4);

    /* ---- I2C PINS PB6 PB9 ---- */
    MODER(GPIOB) |= (2<<(6*2))|(2<<(9*2));
    AFRL(GPIOB) |= (4<<(6*4));
    AFRH(GPIOB) |= (4<<((9-8)*4));
    OTYPER(GPIOB) |= (1<<6)|(1<<9);
    PUPDR(GPIOB) |= (1<<(6*2))|(1<<(9*2));

    /* ---- I2S PINS ---- */
    MODER(GPIOA) |= (2<<(4*2));
    AFRL(GPIOA) |= (6<<(4*4));

    MODER(GPIOC) |= (2<<(7*2))|(2<<(10*2))|(2<<(12*2));
    AFRL(GPIOC) |= (6<<(7*4));
    AFRH(GPIOC) |= (6<<((10-8)*4))|(6<<((12-8)*4));

    /* ---- I2C INIT ---- */
    CR1 |= (1<<15); delay(1000); CR1 &= ~(1<<15);
    CR2 = 42; CCR = 210; TRISE = 43; CR1 |= 1;

    /* ---- CS43L22 POWER ON ---- */
    delay(100000);
    i2c_write(0x00,0x99);
    i2c_write(0x47,0x80);
    i2c_write(0x32,0x00);
    i2c_write(0x00,0x00);
    i2c_write(0x04,0xAF);
    i2c_write(0x05,0x81);
    i2c_write(0x06,0x7F);

    /* ---- I2S INIT ---- */
    RCC_PLLI2SCFGR = (192<<6)|(2<<28);
    I2SCFGR = (1<<11)|(2<<8);
    I2SPR = (2<<0)|(1<<9);
    I2SCFGR |= (1<<10);

    /* ---- CONSTANT TONE ---- */
    while (1)
    {
        while (!(SR & (1<<1)));
        SPI_DR = 0x7FFF;   /* LEFT */
        while (!(SR & (1<<1)));
        SPI_DR = 0x8000;   /* RIGHT */
    }
}
