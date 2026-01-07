
#include <stdint.h>

/* ================= WAV ================= */
extern const unsigned char wav[];
extern const unsigned int wav_len;
#define WAV_OFFSET 44

/* ================= RCC ================= */
#define RCC_BASE 0x40023800
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE+0x30))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE+0x40))
#define RCC_PLLI2SCFGR (*(volatile uint32_t *)(RCC_BASE+0x84))

/* ================= GPIO ================= */
#define GPIOA_BASE 0x40020000
#define GPIOB_BASE 0x40020400
#define GPIOC_BASE 0x40020800
#define GPIOD_BASE 0x40020C00

#define GPIO_MODER(x) (*(volatile uint32_t *)(x+0x00))
#define GPIO_OTYPER(x) (*(volatile uint32_t *)(x+0x04))
#define GPIO_PUPDR(x) (*(volatile uint32_t *)(x+0x0C))
#define GPIO_ODR(x) (*(volatile uint32_t *)(x+0x14))
#define GPIO_AFRL(x) (*(volatile uint32_t *)(x+0x20))
#define GPIO_AFRH(x) (*(volatile uint32_t *)(x+0x24))

/* ================= I2C ================= */
#define I2C1_BASE 0x40005400
#define I2C_CR1 (*(volatile uint32_t *)(I2C1_BASE+0x00))
#define I2C_CR2 (*(volatile uint32_t *)(I2C1_BASE+0x04))
#define I2C_DR  (*(volatile uint32_t *)(I2C1_BASE+0x10))
#define I2C_SR1 (*(volatile uint32_t *)(I2C1_BASE+0x14))
#define I2C_SR2 (*(volatile uint32_t *)(I2C1_BASE+0x18))
#define I2C_CCR (*(volatile uint32_t *)(I2C1_BASE+0x1C))
#define I2C_TRISE (*(volatile uint32_t *)(I2C1_BASE+0x20))
#define CS43_ADDR 0x94

/* ================= SPI3 / I2S ================= */
#define SPI3_BASE 0x40003C00
#define SPI_DR (*(volatile uint32_t *)(SPI3_BASE+0x0C))
#define SPI_I2SCFGR (*(volatile uint32_t *)(SPI3_BASE+0x1C))
#define SPI_I2SPR (*(volatile uint32_t *)(SPI3_BASE+0x20))

/* ================= DMA ================= */
#define DMA1_BASE 0x40026000
#define DMA1_S5CR   (*(volatile uint32_t *)(DMA1_BASE+0x0A8))
#define DMA1_S5NDTR (*(volatile uint32_t *)(DMA1_BASE+0x0AC))
#define DMA1_S5PAR  (*(volatile uint32_t *)(DMA1_BASE+0x0B0))
#define DMA1_S5M0AR (*(volatile uint32_t *)(DMA1_BASE+0x0B8))
#define DMA1_HIFCR  (*(volatile uint32_t *)(DMA1_BASE+0x00C))

/* ================= DELAY ================= */
void delay(volatile uint32_t d){ while(d--); }

/* ================= I2C WRITE ================= */
void i2c_write(uint8_t a,uint8_t r,uint8_t d)
{
    while(I2C_SR2&(1<<1));
    I2C_CR1|=(1<<8); while(!(I2C_SR1&1));
    I2C_DR=a; while(!(I2C_SR1&(1<<1))); (void)I2C_SR2;
    while(!(I2C_SR1&(1<<7))); I2C_DR=r;
    while(!(I2C_SR1&(1<<7))); I2C_DR=d;
    while(!(I2C_SR1&(1<<2)));
    I2C_CR1|=(1<<9);
}

/* ================= CS43 ================= */
void cs43_init(void)
{
    delay(100000);
    i2c_write(CS43_ADDR,0x00,0x99);
    i2c_write(CS43_ADDR,0x47,0x80);
    i2c_write(CS43_ADDR,0x32,0x00);
    i2c_write(CS43_ADDR,0x00,0x00);
    i2c_write(CS43_ADDR,0x04,0xAF);
    i2c_write(CS43_ADDR,0x05,0x81);
    i2c_write(CS43_ADDR,0x06,0x7F);
}

/* ================= DMA INIT ================= */
void dma_init(void)
{
    RCC_AHB1ENR |= (1<<21);   /* DMA1 */

    DMA1_S5CR = 0;
    DMA1_HIFCR = 0x0F400000;

    DMA1_S5PAR  = (uint32_t)&SPI_DR;
    DMA1_S5M0AR = (uint32_t)&wav[WAV_OFFSET];
    DMA1_S5NDTR = (wav_len-WAV_OFFSET)/2;

    DMA1_S5CR =
        (0<<25) |     /* CH0 */
        (1<<10) |     /* MINC */
        (1<<8)  |     /* CIRC */
        (1<<6)  |     /* DIR: mem→periph */
        (1<<11)| (1<<13); /* 16-bit */

    DMA1_S5CR |= 1;
}

/* ================= I2S INIT ================= */
void i2s_init(void)
{
    RCC_APB1ENR |= (1<<15);
    RCC_PLLI2SCFGR = (192<<6)|(2<<28);

    SPI_I2SCFGR = (1<<11)|(2<<8);
    SPI_I2SPR = (2<<0)|(1<<9);
    SPI_I2SCFGR |= (1<<10);
}

/* ================= MAIN ================= */
int main(void)
{
    RCC_AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3);

    /* GPIO + I2C setup omitted here for brevity —
       use SAME gpio_init() from previous file */

    cs43_init();
    dma_init();
    i2s_init();

    while(1); /* 🎧 Audio plays via DMA */
}
