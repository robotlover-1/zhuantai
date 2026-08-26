#ifndef __UPGRADE_H
#define __UPGRADE_H

#include "stm32f4xx.h"

/* 固件接收上下文 */
typedef struct {
    uint32_t received;      /* 已接收字节数 */
    uint32_t target_addr;   /* 写入起始地址 (APP_BASE) */
    uint32_t max_size;      /* 最大可写字节数 (APP区大小) */
} upgrade_ctx_t;

/* 接收一个 .bin 文件到 Flash（XCOM"发送文件"裸字节, ~3s空闲判结束, 写后读回校验）
 * 返回: 0=成功, -1=未收到数据, -2=擦除失败, -3=写入失败, -4=固件超限, -5=写回校验失败 */
int upgrade_receive(upgrade_ctx_t *ctx);

#endif
