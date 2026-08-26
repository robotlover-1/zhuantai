#include "upgrade.h"
#include "stm32f4xx_hal.h"

/* 底层 UART5 原始收发，由 boot_main.c 提供 */
extern void boot_uart_putc(uint8_t c);
extern int  boot_uart_getc(uint32_t timeout_ms);
extern void boot_uart_puts(const char *s);

#define IDLE_TIMEOUT_MS  100U      /* 单次无数据检测窗口 */
#define IDLE_END_CNT     10U       /* 10×100ms=1s 无数据 = 传完 */
#define MAX_FW_SIZE      0x18000U  /* 固件上限 96KB: 先收进RAM再写, 避免写入期间丢字节 */

static uint8_t fw_buf[MAX_FW_SIZE];  /* 固件缓冲 (128KB SRAM 内) */

int upgrade_receive(upgrade_ctx_t *ctx)
{
    uint32_t len = 0;
    uint32_t idle = 0;
    int c;

    boot_uart_puts("\r\nBOOT: waiting bin, XCOM send file now...\r\n");

    /* 裸协议无流控: 先把整个文件收进 RAM, 再擦除+写入, 避免写 Flash 期间 UART 字节丢失 */

    /* 等第一个字节: 不设超时, 用户可从容点"发送文件" */
    do {
        c = boot_uart_getc(1000);
    } while (c < 0);
    if (len >= MAX_FW_SIZE || len >= ctx->max_size) return -4;  /* 固件超限 */
    fw_buf[len++] = (uint8_t)c;

    /* 后续: 空闲 1 秒判定传输结束 */
    while (1) {
        c = boot_uart_getc(IDLE_TIMEOUT_MS);
        if (c < 0) {
            if (++idle >= IDLE_END_CNT) break;   /* 1s 无数据 = 传完 */
            continue;
        }
        idle = 0;
        if (len >= MAX_FW_SIZE || len >= ctx->max_size) return -4;  /* 固件超限 */
        fw_buf[len++] = (uint8_t)c;
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
