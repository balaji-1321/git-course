/* bootloader_raw.c
   Raw-register STM32F407 USART3 bootloader (no CMSIS/Cube HAL).
   Protocol:
     1) Host sends ASCII '1'
     2) Host sends 4 bytes (uint32 little-endian) -> firmware size (N)
     3) Host sends N bytes firmware
     4) Host sends 4 bytes (uint32 little-endian) -> CRC32 of firmware
*/

#include <stdint.h>

#define  volatile_u32  (*(volatile uint32_t *)0U)  /* placeholder not used */

/* ---------- Base addresses ---------- */
#define PERIPH_BASE      0x40000000UL
#define AHB1PERIPH_BASE  (PERIPH_BASE + 0x00020000UL)
#define APB1PERIPH_BASE  (PERIPH_BASE + 0x00000000UL)
#define RCC_BASE         (0x40023800UL)
#define GPIOB_BASE       (AHB1PERIPH_BASE + 0x0400UL) /* 0x40020400 */
#define USART3_BASE      (APB1PERIPH_BASE + 0x4800UL) /* 0x40004800 */
#define FLASH_BASE       (0x40023C00UL)
#define SCB_AIRCR        (*(volatile uint32_t*)(0xE000ED0CUL))

/* ---------- RCC registers ---------- */
#define RCC_AHB1ENR      (*(volatile uint32_t*)(RCC_BASE + 0x30UL))
#define RCC_APB1ENR      (*(volatile uint32_t*)(RCC_BASE + 0x40UL))

/* ---------- GPIOB registers ---------- */
#define GPIOB_MODER      (*(volatile uint32_t*)(GPIOB_BASE + 0x00UL))
#define GPIOB_AFRL       (*(volatile uint32_t*)(GPIOB_BASE + 0x20UL))
#define GPIOB_AFRH       (*(volatile uint32_t*)(GPIOB_BASE + 0x24UL))

/* ---------- USART3 registers ---------- */
#define USART3_SR        (*(volatile uint32_t*)(USART3_BASE + 0x00UL))
#define USART3_DR        (*(volatile uint32_t*)(USART3_BASE + 0x04UL))
#define USART3_BRR       (*(volatile uint32_t*)(USART3_BASE + 0x08UL))
#define USART3_CR1       (*(volatile uint32_t*)(USART3_BASE + 0x0CUL))
#define USART3_CR2       (*(volatile uint32_t*)(USART3_BASE + 0x10UL))
#define USART3_CR3       (*(volatile uint32_t*)(USART3_BASE + 0x14UL))

/* ---------- FLASH registers ---------- */
#define FLASH_ACR        (*(volatile uint32_t*)(FLASH_BASE + 0x00UL))
#define FLASH_KEYR       (*(volatile uint32_t*)(FLASH_BASE + 0x04UL))
#define FLASH_OPTKEYR    (*(volatile uint32_t*)(FLASH_BASE + 0x08UL))
#define FLASH_SR         (*(volatile uint32_t*)(FLASH_BASE + 0x0CUL))
#define FLASH_CR         (*(volatile uint32_t*)(FLASH_BASE + 0x10UL))
#define FLASH_AR         (*(volatile uint32_t*)(FLASH_BASE + 0x14UL))

/* ---------- Useful bit definitions ---------- */
/* RCC bits */
#define RCC_AHB1ENR_GPIOBEN   (1U << 1)    /* GPIOB clock enable */
#define RCC_APB1ENR_USART3EN  (1U << 18)   /* USART3 clock enable */

/* USART SR bits */
#define USART_SR_TXE          (1U << 7)
#define USART_SR_RXNE         (1U << 5)

/* USART CR1 bits */
#define USART_CR1_UE          (1U << 13)
#define USART_CR1_TE          (1U << 3)
#define USART_CR1_RE          (1U << 2)

/* FLASH bits */
#define FLASH_KEY1            0x45670123U
#define FLASH_KEY2            0xCDEF89ABU

#define FLASH_CR_PG           (1U << 0)
#define FLASH_CR_SER          (1U << 1)
#define FLASH_CR_MER          (1U << 2)
#define FLASH_CR_SNB_Pos      3U
#define FLASH_CR_STRT         (1U << 16)
#define FLASH_CR_LOCK         (1U << 31)

#define FLASH_SR_EOP          (1U << 0)
#define FLASH_SR_BSY          (1U << 16)

/* System reset: write VECTKEY (0x5FA) << 16 and SYSRESETREQ (1<<2) */
#define AIRCR_VECTKEY_MASK    (0x05FAUL << 16)
#define AIRCR_SYSRESETREQ     (1U << 2)

/* ---------- Bootloader parameters ---------- */
#define APP_START_ADDR    0x08008000UL
#define MAX_FW_SIZE       (128 * 1024UL)   /* 128 KB max firmware size allowed */

/* ---------- USART3 configuration constants ---------- */
/* For APB1 = 42 MHz, desired baud 115200, BRR (oversamp16) = 42e6/115200 = 364.583
   mantissa = 364, fraction = round(0.583*16)=9 => BRR = (364<<4)|9 = 5833 (0x16D9) */
#define USART3_BRR_VAL    5833U

/* ---------- Simple helpers ---------- */
static inline void mm_delay(volatile uint32_t d) { while(d--) __asm__("nop"); }

/* ---------- UART I/O ---------- */
static void usart3_init(void)
{
    /* Enable clocks: GPIOB and USART3 */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC_APB1ENR |= RCC_APB1ENR_USART3EN;
    /* small delay to let peripheral clock start */
    mm_delay(1000);

    /* Configure PB10(TX) and PB11(RX) to AF7 (USART3)
       MODER: 10 = AF for each pin
    */
    /* Clear bits then set */
    GPIOB_MODER &= ~((3U << (10*2)) | (3U << (11*2)));
    GPIOB_MODER |=  ((2U << (10*2)) | (2U << (11*2)));

    /* AFRH holds pins 8..15. Set AF7 (value 7) for pin10 and pin11 */
    GPIOB_AFRH &= ~((0xFU << ((10-8)*4)) | (0xFU << ((11-8)*4)));
    GPIOB_AFRH |=  ((7U << ((10-8)*4)) | (7U << ((11-8)*4)));

    /* Configure USART3 basic: baud, enable TE/RE and UE */
    USART3_BRR = USART3_BRR_VAL;
    USART3_CR1 = USART_CR1_TE | USART_CR1_RE; /* set TE and RE */
    mm_delay(10);
    USART3_CR1 |= USART_CR1_UE; /* enable USART */
}

static void usend_byte(uint8_t b)
{
    while (!(USART3_SR & USART_SR_TXE));
    USART3_DR = b;
}

static uint8_t urecv_byte(void)
{
    while (!(USART3_SR & USART_SR_RXNE));
    return (uint8_t)(USART3_DR & 0xFFU);
}

static void usend_str(const char *s)
{
    while (*s) usend_byte((uint8_t)*s++);
}

/* ---------- CRC32 software (table-based) ---------- */
static uint32_t crc_table[256];

static void crc32_init(void)
{
    const uint32_t poly = 0xEDB88320U;
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (uint32_t j = 0; j < 8; ++j)
            c = (c & 1) ? (poly ^ (c >> 1)) : (c >> 1);
        crc_table[i] = c;
    }
}

static uint32_t crc32_compute(const uint8_t *buf, uint32_t len)
{
    uint32_t c = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t byte = buf[i];
        uint32_t idx = (c ^ byte) & 0xFFU;
        c = crc_table[idx] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFU;
}

/* ---------- FLASH helpers ---------- */
static void flash_unlock(void)
{
    if (FLASH_CR & FLASH_CR_LOCK) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
}

static void flash_lock(void)
{
    FLASH_CR |= FLASH_CR_LOCK;
}

static void flash_wait_ready(void)
{
    while (FLASH_SR & FLASH_SR_BSY) { /* wait */ }
}

static int flash_erase_sector(uint32_t sector_number)
{
    /* Unlock */
    flash_unlock();
    flash_wait_ready();

    /* Set SER and SNB and STRT */
    /* write SNB in bits [6:3] */
    FLASH_CR &= ~(0xF << FLASH_CR_SNB_Pos); /* clear SNB */
    FLASH_CR |= FLASH_CR_SER | (sector_number << FLASH_CR_SNB_Pos);
    FLASH_CR |= FLASH_CR_STRT;

    /* Wait */
    flash_wait_ready();

    /* Clear SER bit */
    FLASH_CR &= ~FLASH_CR_SER;

    /* Check EOP maybe clear it by reading/writing SR (EOP cleared by writing 1) */
    if (FLASH_SR & FLASH_SR_EOP) FLASH_SR = FLASH_SR_EOP;

    return 0;
}

static int flash_program_word(uint32_t addr, uint32_t data)
{
    flash_wait_ready();
    FLASH_CR |= FLASH_CR_PG;
    /* Program word: write to address */
    *(volatile uint32_t*)addr = data;
    flash_wait_ready();
    FLASH_CR &= ~FLASH_CR_PG;

    /* Check EOP and clear */
    if (FLASH_SR & FLASH_SR_EOP) FLASH_SR = FLASH_SR_EOP;

    return 0;
}

/* ---------- Jump to application ---------- */
static void jump_to_app(uint32_t app_addr)
{
    uint32_t sp = *(uint32_t*)(app_addr);
    uint32_t reset_handler = *(uint32_t*)(app_addr + 4);

    /* set main stack pointer and jump */
    __asm__ volatile (
        "msr msp, %0     \n"
        "bx %1           \n"
        : : "r" (sp), "r" (reset_handler) : "memory"
    );
}

/* ---------- Bootloader main logic ---------- */
void bootloader_run(void)
{
    uint8_t cmd;
    uint32_t fw_size;
    uint32_t remote_crc;
    uint8_t *fw_buf;

    /* Prepare CRC table */
    crc32_init();

    usend_str("BOOT: ready\r\n");
    cmd = urecv_byte();
    if (cmd != '1') {
        usend_str("BOOT: bad cmd\r\n");
        return;
    }
    usend_str("BOOT: recv size(4), data, crc(4)\r\n");

    /* receive 4 bytes of size (little endian) */
    uint8_t b0 = urecv_byte();
    uint8_t b1 = urecv_byte();
    uint8_t b2 = urecv_byte();
    uint8_t b3 = urecv_byte();
    fw_size = ((uint32_t)b0) | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);

    if (fw_size == 0 || fw_size > MAX_FW_SIZE) {
        usend_str("BOOT: bad size\r\n");
        return;
    }

    /* allocate buffer statically on stack? better static array */
    static uint8_t fw_static_buf[MAX_FW_SIZE];
    fw_buf = fw_static_buf;

    /* receive firmware bytes */
    for (uint32_t i = 0; i < fw_size; ++i) {
        fw_buf[i] = urecv_byte();
    }

    /* receive 4 bytes of CRC (little-endian) */
    b0 = urecv_byte(); b1 = urecv_byte(); b2 = urecv_byte(); b3 = urecv_byte();
    remote_crc = ((uint32_t)b0) | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);

    /* compute CRC */
    uint32_t calc_crc = crc32_compute(fw_buf, fw_size);

    if (calc_crc != remote_crc) {
        usend_str("BOOT: CRC FAIL\r\n");
        return;
    }

    usend_str("BOOT: CRC OK - erasing sector2\r\n");

    /* Erase Flash Sector 2 (0x08008000) */
    /* Sector numbering for STM32F4:
       Sector 0: 0x08000000 - 16KB
       Sector 1: 0x08004000 - 16KB
       Sector 2: 0x08008000 - 16KB  <-- we erase this
    */
    flash_erase_sector(2);

    usend_str("BOOT: programming...\r\n");

    /* Program word by word (4 bytes). fw_size is assumed divisible by 4 for simplicity;
       if not divisible, we pad the last word with 0xFF */
    uint32_t addr = APP_START_ADDR;
    uint32_t i = 0;
    while (i < fw_size) {
        uint32_t w = 0xFFFFFFFFU;
        uint8_t a = fw_buf[i++];
        w = (uint32_t)a;
        if (i < fw_size) { a = fw_buf[i++]; w |= ((uint32_t)a << 8); }
        else w |= (0xFFU << 8);
        if (i < fw_size) { a = fw_buf[i++]; w |= ((uint32_t)a << 16); }
        else w |= (0xFFU << 16);
        if (i < fw_size) { a = fw_buf[i++]; w |= ((uint32_t)a << 24); }
        else w |= (0xFFU << 24);

        flash_program_word(addr, w);
        addr += 4;
    }

    flash_lock();

    usend_str("BOOT: done - jumping to app\r\n");
    mm_delay(100000);

    /* Jump to application */
    jump_to_app(APP_START_ADDR);
}

/* ---------- main ---------- */
/* Provide a minimal main that calls init and bootloader */
int main(void)
{
    usart3_init();

    /* check if there's a valid application? simple approach: always run bootloader;
       you could add a timeout or check a pin to decide. */

    bootloader_run();

    /* if bootloader returns, try to jump to app (if valid) */
    uint32_t sp = *(uint32_t*)(APP_START_ADDR);
    uint32_t reset = *(uint32_t*)(APP_START_ADDR + 4);

    if ((sp != 0xFFFFFFFFU) && ((reset & 0xFF000000U) == 0x08000000U || (reset & 0xFF000000U) == 0x20000000U)) {
        jump_to_app(APP_START_ADDR);
    }

    /* otherwise loop forever */
    while (1) { __asm__("wfi"); }
    return 0;
}
