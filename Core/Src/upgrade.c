#include "upgrade.h"
#include "stm32f4xx_hal.h"

/* 底层 UART5 原始收发，由 boot_main.c 提供 */
extern void boot_uart_putc(uint8_t c);
extern int  boot_uart_getc(uint32_t timeout_ms);
extern void boot_uart_puts(const char *s);

#define MAX_FW_SIZE      0x18000U  /* 固件上限 96KB: 先收进RAM再写, 避免写入期间丢字节 */
#define IDLE_TIGHT_LOOPS 20000U    /* 紧密轮询窗口 (~1ms), 期间持续检查RXNE, 不丢字节 */
#define IDLE_END_CNT     3000U     /* 3000×~1ms = ~3s 无数据 = 传完 */

static uint8_t fw_buf[MAX_FW_SIZE];  /* 固件缓冲 (128KB SRAM 内) */

/* 紧密轮询读一个字节: ~1ms 窗口内持续检查 RXNE, 返回字节或 -1
 * 不用 delay_ms(1), 避免发送方微空隙期间 UART 1字节缓冲溢出丢字节 */
static int getc_idle(void)
{
    for (uint32_t i = 0; i < IDLE_TIGHT_LOOPS; i++) {
        if (UART5->SR & USART_SR_RXNE) return (int)(UART5->DR & 0xFF);
        if (UART5->SR & USART_SR_ORE)  { (void)UART5->DR; }
    }
    return -1;
}

int upgrade_receive(upgrade_ctx_t *ctx)
{
    uint32_t len = 0;
    uint32_t idle = 0;
    int c;

    boot_uart_puts("\r\nBOOT: waiting bin, XCOM send file now...\r\n");

    /* 裸协议无流控: 先把整个文件收进 RAM, 再擦除+写入, 避免写 Flash 期间 UART 字节丢失 */

    /* 等第一个字节: 紧密轮询不超时, 用户可从容点"发送文件" */
    do {
        c = getc_idle();
    } while (c < 0);
    if (len >= MAX_FW_SIZE || len >= ctx->max_size) return -4;  /* 固件超限 */
    fw_buf[len++] = (uint8_t)c;

    /* 收剩余: 紧密轮询, ~3s 无数据判定传完 */
    while (1) {
        c = getc_idle();
        if (c >= 0) {
            if (len >= MAX_FW_SIZE || len >= ctx->max_size) return -4;  /* 固件超限 */
            fw_buf[len++] = (uint8_t)c;
            idle = 0;
        } else {
            if (++idle >= IDLE_END_CNT) break;   /* ~3s 无数据 = 传完 */
        }
    }

    /* 收完: 擦除 APP 区 Sector4-6, 再顺序写入 */
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef f;
    uint32_t err = 0;
    f.TypeErase     = FLASH_TYPEERASE_SECTORS;
    f.Sector        = FLASH_SECTOR_4;
    f.NbSectors     = 3;
    f.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    if (HAL_FLASHEx_Erase(&f, &err) != HAL_OK) { HAL_FLASH_Lock(); return -2; }

    /* 4 字节对齐写入 (不足4字节尾部补 0, 不越界读) */
    for (uint32_t i = 0; i < len; i += 4U) {
        uint32_t w = 0U;
        for (uint32_t j = 0; j < 4U; j++) {
            if (i + j < len)
                w |= (uint32_t)fw_buf[i + j] << (8U * j);
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ctx->target_addr + i, w) != HAL_OK) {
            HAL_FLASH_Lock();
            return -3;
        }
    }
    HAL_FLASH_Lock();

    ctx->received = len;
    return 0;
}
