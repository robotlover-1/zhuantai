/**
 ****************************************************************************************************
 * @file        eth.h
 * @brief       以太网模块 (DM9000 + LwIP + TCP Server) 公共接口
 ****************************************************************************************************
 */
#ifndef __ETH_H
#define __ETH_H

#include "sys.h"

/* ---- 命令来源标记 ---- */
typedef enum {
    CMD_SRC_UART = 0,
    CMD_SRC_TCP  = 1,
} cmd_source_t;

extern cmd_source_t g_cmd_src;

/* ---- TCP 接收缓冲区 ---- */
#ifdef ETH_RX_BUF_SIZE
  #undef ETH_RX_BUF_SIZE
#endif
#define ETH_RX_BUF_SIZE  256
extern u8   g_eth_rx_buf[ETH_RX_BUF_SIZE];
extern u16  g_eth_rx_len;
extern u8   g_eth_rx_ready;
extern u8   g_tcp_client_connected;

/* ---- 公共接口 ---- */
int  eth_init(void);                        /* 初始化 DM9000 + LwIP + TCP Server */
void eth_tcp_send_byte(uint8_t ch);         /* 发送单字节到 TCP 客户端 (非阻塞) */
void MX_LWIP_Process(void);                 /* LwIP 协议栈周期处理 */

/* ---- DM9000 驱动层 (供 ethernetif.c 使用) ---- */
uint8_t dm9000_reg_read(uint16_t reg);
void    dm9000_reg_write(uint16_t reg, uint8_t val);
int     dm9000_hw_init(uint8_t *mac_addr);  /* 硬件复位 + VID/PID 校验，返回 MAC */
int     dm9000_recv_packet(uint8_t *buf, u16 *len);
int     dm9000_send_packet(uint8_t *buf, u16 len);

#endif /* __ETH_H */
