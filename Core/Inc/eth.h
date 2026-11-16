/**
 ****************************************************************************************************
 * @file        eth.h
 * @brief       以太网模块 (DM9000 + LwIP + TCP Server) 公共接口
 ****************************************************************************************************
 */
#ifndef __ETH_H
#define __ETH_H

#include "sys.h"
#include <stdint.h>

/* ==================================================================== */
/* DM9000 FSMC 硬件配置                                                  */
/* 板子引脚 (由硬件工程师提供):                                          */
/*   PG9 = FSMC_NE2,  PG10 = FSMC_NE3,  PD7 = DM9000-RST,  PG6 = DM9000-INT */
/*   FSMC 数据/地址总线 16 位全接 (D0-D15, A0-A18)                       */
/* 说明: eth.c 启动时 dm9000_probe() 会在 NE2/NE3 × A0..A18 中自动探测   */
/*   CS#/CMD 的实际接法 (原理图未标明), 下面两个宏仅作默认/参考值.       */
/*   探测失败时串口会打印 [ETH] DM9000 NOT found, 按提示查硬件接线.      */
/* ==================================================================== */
#define DM9000_NE           2            /* 2=NE2(0x64000000)  3=NE3(0x68000000) */
#if DM9000_NE == 3
#define DM9000_BASE         0x68000000UL
#else
#define DM9000_BASE         0x64000000UL
#endif
/* 16 位 FSMC 下: FSMC_A1 = HADDR[2] (偏移 0x4), FSMC_A2 = HADDR[3] (偏移 0x8), FSMC_A7 = HADDR[8] (偏移 0x100)
 * 真机实测 (2026-08-26): CMD 接 FSMC_A7 (PF13), 偏移 0x100 */
#define DM9000_CMD_OFFSET   0x100UL
#define DM9000_INDEX_ADDR   (*((volatile uint16_t *)(DM9000_BASE + 0x0UL)))
#define DM9000_DATA_ADDR    (*((volatile uint16_t *)(DM9000_BASE + DM9000_CMD_OFFSET)))

/* ==================================================================== */
/* 命令来源标记 (fputc 输出路由: TCP 来源时响应同步发到 TCP 客户端)      */
/* ==================================================================== */
typedef enum {
    CMD_SRC_UART = 0,
    CMD_SRC_TCP  = 1,
} cmd_source_t;

extern cmd_source_t g_cmd_src;

/* ---- TCP 接收缓冲区 ---- */
#define ETH_RX_BUF_SIZE  256
extern u8   g_eth_rx_buf[ETH_RX_BUF_SIZE];
extern u16  g_eth_rx_len;
extern u8   g_eth_rx_ready;
extern u8   g_tcp_client_connected;

/* ---- 公共接口 ---- */
int  eth_init(void);                        /* 初始化 FSMC + DM9000 + LwIP + TCP Server */
void eth_tcp_send_byte(uint8_t ch);         /* 发送单字节到 TCP 客户端 (非阻塞, 环形缓冲) */
void MX_LWIP_Process(void);                 /* LwIP 协议栈周期处理 (主循环调用) */

/* ---- DM9000 驱动层 (供 ethernetif.c 使用) ---- */
uint8_t dm9000_reg_read(uint16_t reg);
void    dm9000_reg_write(uint16_t reg, uint8_t val);
int     dm9000_hw_init(uint8_t *mac_addr);  /* 硬件复位 + VID/PID 校验 + 读 MAC */
int     dm9000_recv_packet(uint8_t *buf, u16 *len);
int     dm9000_send_packet(uint8_t *buf, u16 len);

#endif /* __ETH_H */
