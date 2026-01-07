#include <stdint.h>

/* ================= WAV DATA ================= */
extern const unsigned char wav[];
extern const unsigned int wav_len;
#define WAV_OFFSET 44   /* skip WAV header */

/* ================= RCC ================= */
#define RCC_BASE        0x40023800
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_PLLI2SCFGR  (*(volatile uint32_t *)(RCC_BASE + 0x84))

/* ================= GPIO ================= */
#define GPIOA_BASE  0x40020000
#define GPIOB_BASE  0x40020400
#define GPIOC_BASE  0x40020800
#define GPIOD_BASE  0x40020C00

#define GPIO_MODER(x)   (*(volatile uint32_t *)(x + 0x00))
#define GPIO_ODR(x)     (*(volatile uint32_t *)(x + 0x14))
#define GPIO_AFRL(x)    (*(volatile uint32_t *)(x + 0x20))
#define GPIO_AFRH(x)    (*(volatile uint32_t *)(x + 0x24))

/* ================= SPI3 / I2S ================= */
#define SPI3_BASE   0x40003C00
#define SPI_SR      (*(volatile uint32_t *)(SPI3_BASE + 0x08))
#define SPI_DR      (*(volatile uint32_t *)(SPI3_BASE + 0x0C))
#define SPI_I2SCFGR (*(volatile uint32_t *)(SPI3_BASE + 0x1C))
#define SPI_I2SPR   (*(volatile uint32_t *)(SPI3_BASE + 0x20))

/* ================= I2C1 ================= */
#define I2C1_BASE   0x40005400
#define I2C_CR1     (*(volatile uint32_t *)(I2C1_BASE + 0x00))
#define I2C_CR2     (*(volatile uint32_t *)(I2C1_BASE + 0x04))
#define I2C_DR      (*(volatile uint32_t *)(I2C1_BASE + 0x10))
#define I2C_SR1     (*(volatile uint32_t *)(I2C1_BASE + 0x14))
#define I2C_SR2     (*(volatile uint32_t *)(I2C1_BASE + 0x18))
#define I2C_CCR     (*(volatile uint32_t *)(I2C1_BASE + 0x1C))
#define I2C_TRISE   (*(volatile uint32_t *)(I2C1_BASE + 0x20))

#define CS43_ADDR   0x94

/* ================= DELAY ================= */
void delay(volatile uint32_t d)
{
    while (d--);
}

/* ================= GPIO INIT ================= */
void gpio_init(void)
{
    RCC_AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3);

    /* I2S pins */
    GPIO_MODER(GPIOA_BASE) |= (2<<(4*2));
    GPIO_AFRL(GPIOA_BASE) |= (6<<(4*4));

    GPIO_MODER(GPIOC_BASE) |= (2<<(7*2))|(2<<(10*2))|(2<<(12*2));
    GPIO_AFRL(GPIOC_BASE) |= (6<<(7*4));
    GPIO_AFRH(GPIOC_BASE) |= (6<<((10-8)*4))|(6<<((12-8)*4));

    /* Codec reset PD4 */
    GPIO_MODER(GPIOD_BASE) |= (1<<(4*2));
    GPIO_ODR(GPIOD_BASE) &= ~(1<<4);
    delay(100000);
    GPIO_ODR(GPIOD_BASE) |= (1<<4);

    /* I2C PB6 PB9 */
    GPIO_MODER(GPIOB_BASE) |= (2<<(6*2))|(2<<(9*2));
    GPIO_AFRL(GPIOB_BASE) |= (4<<(6*4));
    GPIO_AFRH(GPIOB_BASE) |= (4<<((9-8)*4));
}

/* ================= I2C ================= */
void i2c_init(void)
{
    RCC_APB1ENR |= (1<<21);

    I2C_CR2 = 42;
    I2C_CCR = 210;
    I2C_TRISE = 43;
    I2C_CR1 |= 1;
}

void i2c_write(uint8_t addr, uint8_t reg, uint8_t data)
{
    I2C_CR1 |= (1<<8);
    while(!(I2C_SR1 & 1));

    I2C_DR = addr;
    while(!(I2C_SR1 & (1<<1)));
    (void)I2C_SR2;

    I2C_DR = reg;
    while(!(I2C_SR1 & (1<<7)));

    I2C_DR = data;
    while(!(I2C_SR1 & (1<<7)));

    I2C_CR1 |= (1<<9);
}

/* ================= CS43L22 ================= */
void cs43_init(void)
{
    delay(100000);

    i2c_write(CS43_ADDR,0x00,0x99);
    i2c_write(CS43_ADDR,0x47,0x80);
    i2c_write(CS43_ADDR,0x32,0x80);
    i2c_write(CS43_ADDR,0x32,0x00);
    i2c_write(CS43_ADDR,0x00,0x00);

    i2c_write(CS43_ADDR,0x04,0xAF);
    i2c_write(CS43_ADDR,0x05,0x81);
    i2c_write(CS43_ADDR,0x06,0x7F);
}

/* ================= I2S ================= */
void i2s_init(void)
{
    RCC_APB1ENR |= (1<<15);

    RCC_PLLI2SCFGR = (192<<6)|(2<<28);

    SPI_I2SCFGR = 0;
    SPI_I2SCFGR = (1<<11)|(2<<8);
    SPI_I2SPR = (2<<0)|(1<<9);
    SPI_I2SCFGR |= (1<<10);
}

/* ================= PLAY AUDIO ================= */
void i2s_play(const unsigned char *data, uint32_t len)
{
    for(uint32_t i=0;i<len;i+=2)
    {
        uint16_t sample = data[i] | (data[i+1]<<8);
        while(!(SPI_SR & (1<<1)));
        SPI_DR = sample;
    }
}

/* ================= MAIN ================= */
int main(void)
{
    gpio_init();
    i2c_init();
    cs43_init();
    i2s_init();

    while(1)
    {
        i2s_play(&wav[WAV_OFFSET], wav_len - WAV_OFFSET);
    }
}
