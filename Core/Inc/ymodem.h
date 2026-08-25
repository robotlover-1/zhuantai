#ifndef __YMODEM_H
#define __YMODEM_H

#include "stm32f4xx.h"

/* 单个 YModem 文件接收上下文 */
typedef struct {
    uint32_t total_size;    /* 块0声明的文件大小（可能为0） */
    uint32_t received;      /* 已写入字节数 */
    uint32_t target_addr;   /* 固件写入起始地址（APP_BASE） */
    uint32_t max_size;      /* 固件区最大可写字节数（由调用方设置） */
} ymodem_ctx_t;

uint16_t crc16_xmodem(const uint8_t *data, uint32_t len);
/* 返回: 0=成功, -1=超时(未收到块0), -2=擦除失败, -3=发送方取消(CAN), -4=数据超界 */
int ymodem_receive(ymodem_ctx_t *ctx);

#endif
