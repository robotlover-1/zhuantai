#include "ymodem.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>

/* 底层 UART5 原始收发，由 boot_main.c 提供 */
extern void boot_uart_putc(uint8_t c);
extern int  boot_uart_getc(uint32_t timeout_ms);
extern void boot_uart_puts(const char *s);

#define C_CHR  0x43   /* 'C': 邀请/继续 (CRC模式) */
#define ACK    0x06
#define NAK    0x15
#define CAN    0x18
#define EOT    0x04
#define SOH    0x01
#define STX    0x02

#define RX_TIMEOUT   2000U  /* 单字节接收超时 ms */
#define INVITE_LIMIT 6      /* 无块0时'C'重邀上限, 6×2s≈12s 判超时 */

uint16_t crc16_xmodem(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* 写 len 字节（4字节对齐）到 Flash */
static void flash_write_words(uint32_t addr, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t w = (uint32_t)data[i]
                   | ((uint32_t)data[i+1] << 8)
                   | ((uint32_t)data[i+2] << 16)
                   | ((uint32_t)data[i+3] << 24);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, w);
    }
}

int ymodem_receive(ymodem_ctx_t *ctx)
{
    uint8_t data[1024];
    int     exp_seq = 1;        /* 数据块序号从1开始 */
    int     got_header = 0;
    int     invite = 0;
    int     eot_handshake = 1;  /* 1=未完成EOT握手, 0=已发NAK等第二次EOT */

    boot_uart_puts("\r\nBOOT: waiting YModem...\r\n");

    while (1) {
        int c = boot_uart_getc(RX_TIMEOUT);
        if (c < 0) {
            if (!got_header) {
                if (++invite > INVITE_LIMIT) return -1;  /* 约12s无块0 */
                boot_uart_putc(C_CHR);
            } else {
                boot_uart_putc(NAK);  /* 数据阶段超时: 请求重传 */
            }
            continue;
        }
        uint8_t hdr = (uint8_t)c;

        if (hdr == SOH || hdr == STX) {
            uint32_t dlen = (hdr == SOH) ? 128U : 1024U;
            int s  = boot_uart_getc(RX_TIMEOUT);
            int ns = boot_uart_getc(RX_TIMEOUT);
            if (s < 0 || ns < 0 || (uint8_t)~s != (uint8_t)ns) { boot_uart_putc(NAK); continue; }
            uint8_t seq = (uint8_t)s;

            int ok = 1;
            for (uint32_t i = 0; i < dlen; i++) {
                int b = boot_uart_getc(RX_TIMEOUT);
                if (b < 0) { ok = 0; break; }
                data[i] = (uint8_t)b;
            }
            if (!ok) { boot_uart_putc(NAK); continue; }

            int crc_hi = boot_uart_getc(RX_TIMEOUT);  /* YModem: CRC高字节在前 */
            int crc_lo = boot_uart_getc(RX_TIMEOUT);
            if (crc_hi < 0 || crc_lo < 0) { boot_uart_putc(NAK); continue; }
            uint16_t crc = (uint16_t)((crc_hi << 8) | crc_lo);
            if (crc16_xmodem(data, dlen) != crc) { boot_uart_putc(NAK); continue; }

            if (!got_header) {
                /* 块0: "文件名\0大小\0" */
                char *fname = (char *)data;
                char *fsize = fname;
                while (*fsize) fsize++;
                fsize++;
                ctx->total_size = 0;
                if (*fsize) ctx->total_size = (uint32_t)strtoul(fsize, NULL, 10);
                got_header = 1;
                exp_seq = 1;
                ctx->received = 0;

                /* 擦除 APP 区 Sector4-6 */
                HAL_FLASH_Unlock();
                FLASH_EraseInitTypeDef f;
                uint32_t err = 0;
                f.TypeErase     = FLASH_TYPEERASE_SECTORS;
                f.Sector        = FLASH_SECTOR_4;
                f.NbSectors     = 3;
                f.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
                if (HAL_FLASHEx_Erase(&f, &err) != HAL_OK) { HAL_FLASH_Lock(); return -2; }

                boot_uart_putc(ACK);
                boot_uart_putc(C_CHR);
            }
            else if (!eot_handshake) {
                /* 结束块: 空块0 → 完成 */
                if (seq == 0 && data[0] == 0) { boot_uart_putc(ACK); HAL_FLASH_Lock(); return 0; }
                boot_uart_putc(NAK);
            }
            else {
                /* 数据块 */
                if (seq != (uint8_t)exp_seq) { boot_uart_putc(NAK); continue; }
                if (ctx->received + dlen > ctx->max_size) { HAL_FLASH_Lock(); return -4; }  /* 固件超界 */
                flash_write_words(ctx->target_addr + ctx->received, data, dlen);
                ctx->received += dlen;
                exp_seq = (exp_seq + 1) & 0xFF;
                boot_uart_putc(ACK);
                boot_uart_putc(C_CHR);
            }
        }
        else if (hdr == EOT) {
            if (eot_handshake) { boot_uart_putc(NAK); eot_handshake = 0; }  /* 第一次EOT: NAK确认 */
            else               { boot_uart_putc(ACK); boot_uart_putc(C_CHR); } /* 第二次EOT: 等结束块 */
        }
        else if (hdr == CAN) {
            HAL_FLASH_Lock();
            return -3;  /* 发送方取消 */
        }
        /* 其他字节忽略 */
    }
}
