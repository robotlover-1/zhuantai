/**
 ****************************************************************************************************
 * @file        eth.c
 * @brief       DM9000 以太网驱动 + FSMC 配置 + LwIP 集成 + TCP Server
 * @note        DM9000 挂 FSMC (NE2/NE3), 16 位数据总线.
 *              裸机轮询模式 (NO_SYS): 收包/定时器/TCP 发送都在主循环 MX_LWIP_Process() 中处理.
 *              TCP 发送用环形缓冲分块 tcp_write, 避免逐字节写入撑爆 MEMP_NUM_TCP_SEG.
 ****************************************************************************************************
 */
#include "eth.h"
#include "delay.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/def.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"

/* ---- DM9000 关键寄存器 ---- */
#define DM9000_NCR          0x00    /* 网络控制寄存器 */
#define DM9000_NSR          0x01    /* 网络状态寄存器 */
#define DM9000_TCR          0x02    /* 发送控制寄存器 */
#define DM9000_RCR          0x05    /* 接收控制寄存器 */
#define DM9000_ISR          0xFE    /* 中断状态寄存器 */
#define DM9000_IMR          0xFF    /* 中断屏蔽寄存器 */
#define DM9000_MRCMD        0xF8    /* 内存读命令 (自动递增) */
#define DM9000_MWCMD        0xF8    /* 内存写命令 (自动递增) */
#define DM9000_TXPLL        0xFC    /* 发送包长度低字节 */
#define DM9000_TXPLH        0xFD    /* 发送包长度高字节 */
#define DM9000_PAR          0x10    /* 物理地址(MAC) 寄存器 0x10~0x15 */
#define DM9000_VIDL         0x28
#define DM9000_VIDH         0x29
#define DM9000_PIDL         0x2A
#define DM9000_PIDH         0x2B
#define DM9000_FCR          0x2D    /* 流控制寄存器 */
#define DM9000_GPR          0x1F    /* 通用数据寄存器: bit0(GEPIO0) 输出值 */
#define DM9000_GPCR         0x1E    /* 通用控制寄存器: bit0(GEPIO0) 方向, 1=输出 */

#define DM9000_GPR_PHY_PD   (1 << 0)   /* GEPIO0: 1=PHY 省电, 0=PHY 工作 */

#define DM9000_NCR_RST      (1 << 0)
#define DM9000_ISR_PRS      (1 << 0)    /* 包接收中断 */

/* ==================================================================== */
/* 全局变量                                                             */
/* ==================================================================== */
cmd_source_t g_cmd_src = CMD_SRC_UART;
u8   g_eth_rx_buf[ETH_RX_BUF_SIZE];
u16  g_eth_rx_len = 0;
u8   g_eth_rx_ready = 0;
u8   g_tcp_client_connected = 0;

/* LwIP 结构 */
static struct netif g_netif;
static struct tcp_pcb *g_tcp_pcb = NULL;
static struct tcp_pcb *g_client_pcb = NULL;
static ip4_addr_t g_ipaddr, g_netmask, g_gw;
static u8 g_eth_ready = 0;      /* 仅在 LwIP 完整初始化后置 1; 诊断模式下保持 0 */
u8 g_mac_addr[6];                       /* MAC (由 dm9000_hw_init 填充, ethernetif.c extern) */

/* DM9000 数据缓冲 (静态分配, 供 ethernetif.c extern) */
u8 dm9000_tx_buf[1536];

/* 网络参数 (默认值; 由 flash_param.c 的 param_load_all 从 Flash 覆盖) */
u8  g_eth_ip[4]   = {192, 168, 1, 100};
u8  g_eth_mask[4] = {255, 255, 255, 0};
u8  g_eth_gw[4]   = {192, 168, 1, 1};
u16 g_eth_port    = 5000;

/* ---- TCP 发送环形缓冲 (fputc 逐字节写入, MX_LWIP_Process 分块发出) ---- */
#define ETH_TX_BUF_SIZE   2048
#define ETH_TX_CHUNK      512
static u8  g_eth_tx_buf[ETH_TX_BUF_SIZE];
static u16 g_eth_tx_head = 0;   /* 写计数 (单调递增, u16 自然回绕) */
static u16 g_eth_tx_tail = 0;   /* 读计数 */

/* ---- DM9000 访问指针 (启动时 dm9000_probe 自动确定基址+CMD偏移) ---- */
static volatile uint16_t *g_dm9000_index = (volatile uint16_t *)DM9000_BASE;
static volatile uint16_t *g_dm9000_data  = (volatile uint16_t *)(DM9000_BASE + DM9000_CMD_OFFSET);

/* ==================================================================== */
/* 转发声明                                                             */
/* ==================================================================== */
err_t ethernetif_init(struct netif *netif);
void  ethernetif_input(struct netif *netif);
static err_t tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void  tcp_err_cb(void *arg, err_t err);
static err_t tcp_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void  eth_fsmc_init(void);
static void  eth_tx_flush(void);
static uint16_t dm9000_phy_read(uint8_t phy_reg);   /* MX_LWIP_Process 链路监控用 */
static void dm9000_phy_status_poll(void);           /* MX_LWIP_Process 周期 PHY 状态检查用 */

/* ==================================================================== */
/* FSMC 配置: 总线引脚 AF12 + NOR/SRAM Bank + DM9000 时序                 */
/* ==================================================================== */
static void eth_fsmc_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* FSMC 总线引脚复用为 AF12 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_FSMC;

    /* PORT D: D0/D1/D2/D3/D4/D5(控制器)/D8-D15 */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
               GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
               GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* PORT E: E0/NBL0, E1/NBL1, E7-E15 */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 |
               GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
               GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* PORT F: F0-A0 .. F5-A5, F12-A6 .. F15-A9 */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
               GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 |
               GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &gpio);

    /* PORT G: G0-A10 .. G5-A15 + 片选 NE2/NE3 */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
               GPIO_PIN_4 | GPIO_PIN_5;
#if DM9000_NE == 3
    gpio.Pin |= GPIO_PIN_10;    /* NE3 */
#else
    gpio.Pin |= GPIO_PIN_9;     /* NE2 */
#endif
    HAL_GPIO_Init(GPIOG, &gpio);

    /* DM9000 RST: PD7 推挽输出, 上电保持高 */
    gpio.Mode   = GPIO_MODE_OUTPUT_PP;
    gpio.Pull   = GPIO_PULLUP;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0;
    gpio.Pin    = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOD, &gpio);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

    /* DM9000 INT: PG6 输入 (本驱动用轮询收包, 不启用中断) */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin  = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOG, &gpio);

    /* 使能 FSMC 时钟 */
    __HAL_RCC_FSMC_CLK_ENABLE();

    /* NOR/SRAM Bank: 16 位异步 SRAM, 写使能.
     * BCR  = 0x1011 = MBKEN | MWID=16bit | WREN
     * BTR  = 0x003E000F: ADDSET=15 (90ns), DATAST=62 (372ns), 满足 DM9000 ~100ns 周期
     * 同时使能 Bank2(NE2) 和 Bank3(NE3), 供 dm9000_probe 自动探测片选 */
    FSMC_Bank1->BTCR[2] = 0x00001011UL;     /* BCR2 (NE2) */
    FSMC_Bank1->BTCR[3] = 0x003E000FUL;     /* BTR2 (NE2) */
    FSMC_Bank1->BTCR[4] = 0x00001011UL;     /* BCR3 (NE3) */
    FSMC_Bank1->BTCR[5] = 0x003E000FUL;     /* BTR3 (NE3) */
}

/* ==================================================================== */
/* DM9000 自动探测: 在 NE2/NE3 × 各地址线(CMD) 中找能读到 0x0A46/0x9000 的组合 */
/* ==================================================================== */
static int dm9_probe_at(uint32_t base_addr, uint32_t off)
{
    volatile uint16_t *base = (volatile uint16_t *)base_addr;
    volatile uint16_t *data = base + (off >> 1);
    u16 vid, pid;

    base[0] = DM9000_VIDH;  vid = (u16)((data[0] & 0xFF) << 8);
    base[0] = DM9000_VIDL;  vid |= (u16)(data[0] & 0xFF);
    base[0] = DM9000_PIDH;  pid = (u16)((data[0] & 0xFF) << 8);
    base[0] = DM9000_PIDL;  pid |= (u16)(data[0] & 0xFF);

    if (vid == 0x0A46 && pid == 0x9000) {
        g_dm9000_index = base;
        g_dm9000_data  = data;
        // printf("[ETH] DM9000 @ %#08lX, CMD offset %#lX\r\n",
        //        (unsigned long)base_addr, (unsigned long)off);
        return 0;
    }
    return -1;
}

static int dm9000_probe(void)
{
    const uint32_t bases[2] = { 0x64000000UL, 0x68000000UL };   /* NE2 / NE3 */
    uint32_t offs[19];
    int b, n, i;

    /* 候选 CMD 偏移: 16 位 FSMC 下 FSMC_A0..A18 = HADDR[1..19] = 偏移 0x2..0x80000 */
    for (n = 0; n < 19; n++) offs[n] = (uint32_t)2 << n;

    /* 快速路径: 先试 eth.h 写死的组合 (真机已验证 NE2 + CMD=FSMC_A7) */
    if (dm9_probe_at(DM9000_BASE, DM9000_CMD_OFFSET) == 0) {
        return 0;
    }

    /* 兜底: 全组合扫描 (换板子/换线时仍可自愈) */
    for (b = 0; b < 2; b++) {
        for (i = 0; i < 19; i++) {
            if (dm9_probe_at(bases[b], offs[i]) == 0) {
                return 0;
            }
        }
    }
    return -1;
}

/* ==================================================================== */
/* DM9000 寄存器读写                                                     */
/* ==================================================================== */
uint8_t dm9000_reg_read(uint16_t reg)
{
    *g_dm9000_index = reg;
    return (uint8_t)(*g_dm9000_data & 0xFF);
}

void dm9000_reg_write(uint16_t reg, uint8_t val)
{
    *g_dm9000_index = reg;
    *g_dm9000_data  = val;
}

/* ==================================================================== */
/* DM9000 硬件初始化: 复位 + VID/PID 校验 + MAC                          */
/* ==================================================================== */
int dm9000_hw_init(uint8_t *mac_addr)
{
    u16 vid, pid;
    int  i, all_zero = 1, all_ff = 1;

    /* PD7 硬件复位: 拉低 10ms → 拉高 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    delay_ms(10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    delay_ms(10);

    /* 自动探测: 片选(NE2/NE3) × CMD 地址线, 找到能读到 VID/PID 的组合 */
    if (dm9000_probe() != 0) {
        // printf("[ETH] DM9000 NOT found! no base/CMD match (probed NE2/NE3 x A0..A18)\r\n");
        // printf("[ETH] Check FSMC wiring: /CS -> NE2 or NE3? CMD -> which address line?\r\n");
        return -1;
    }

    /* 软件复位 (此时 g_dm9000_index/data 已指向正确的基址+CMD) */
    dm9000_reg_write(DM9000_NCR, DM9000_NCR_RST);
    delay_ms(5);
    dm9000_reg_write(DM9000_NCR, 0x00);
    delay_ms(5);

    /* 唤醒内部 PHY (两步): 无 EEPROM 时 GPIO0 默认输入且为高 → PHY 省电.
     * ① GPCR(0x1E) bit0=1: 把 GPIO0 配成输出
     * ② GPR(0x1F)  bit0=0: GPIO0 输出低 → PHY 上电
     * 只写 GPR 不写 GPCR, GPIO0 仍是输入, 唤醒无效 */
    dm9000_reg_write(DM9000_GPCR, 0x01);        /* GPIO0 = 输出 */
    dm9000_reg_write(DM9000_GPR,  0x00);        /* GPIO0 = 低, PHY 上电 */
    /* printf("[ETH] GPCR=0x%02X GPR=0x%02X\r\n", ...); 调试打印已注释 */
    delay_ms(300);      /* 等 PHY 上电 + 自动协商 */

    /* 再读一次 VID/PID 用于日志 */
    vid = (u16)((dm9000_reg_read(DM9000_VIDH) << 8) | dm9000_reg_read(DM9000_VIDL));
    pid = (u16)((dm9000_reg_read(DM9000_PIDH) << 8) | dm9000_reg_read(DM9000_PIDL));
    // printf("[ETH] DM9000 detected VID=0x%04X PID=0x%04X\r\n", vid, pid);

    /* 读 MAC (PAR 0x10~0x15); 若全 0/全 F 说明无 EEPROM, 用默认 MAC 并写回
     * (DM9000 用 PAR 做单播接收过滤, 全 0 会导致收不到发给自己的帧) */
    for (i = 0; i < 6; i++) {
        mac_addr[i] = dm9000_reg_read(DM9000_PAR + i);
        if (mac_addr[i] != 0x00) all_zero = 0;
        if (mac_addr[i] != 0xFF) all_ff = 0;
    }
    if (all_zero || all_ff) {
        static const u8 def_mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
        // printf("[ETH] No valid MAC (no EEPROM), using default: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
        //        def_mac[0], def_mac[1], def_mac[2],
        //        def_mac[3], def_mac[4], def_mac[5]);
        for (i = 0; i < 6; i++) {
            mac_addr[i] = def_mac[i];
            dm9000_reg_write(DM9000_PAR + i, mac_addr[i]);
        }
    } else {
        /* printf("[ETH] MAC: ..."); 已注释 */
        (void)0;
    }

    /* 初始化组播过滤表 (MAR 0x16~0x1D): 前 7 字节清 0, bit63=1 收广播 */
    for (i = 0; i < 7; i++) dm9000_reg_write(0x16 + i, 0x00);
    dm9000_reg_write(0x16 + 7, 0x80);

    /* 正常模式, 清中断, 使能接收, 全双工流控 */
    dm9000_reg_write(DM9000_NCR, 0x00);
    dm9000_reg_write(DM9000_IMR, 0x80);         /* 使能包接收中断位 (轮询读 ISR 用) */
    dm9000_reg_write(DM9000_RCR, 0x01);         /* RXEN: 使能接收 */
    dm9000_reg_write(DM9000_FCR, 0x39);         /* 流控 */

    return 0;
}

/* ==================================================================== */
/* DM9000 帧收发                                                         */
/* ==================================================================== */

/**
 * @brief  从 DM9000 RX FIFO 读出一帧以太网数据
 * @note   DM9000 每帧前有 4 字节头: [RX Status][Len L][Len H][reserved].
 *         16 位读时字节序被交换: 第一个 16 位 = (Len_L << 8) | Status,
 *         第二个 16 位 = (reserved << 8) | Len_H.
 * @retval 0=成功, -1=无数据或错误帧
 */
int dm9000_recv_packet(uint8_t *buf, u16 *len)
{
    u8  isr;
    u16 w1, w2, status, pkt_len;
    u16 i;

    /* 查 PRS 中断位 */
    isr = dm9000_reg_read(DM9000_ISR);
    if (!(isr & DM9000_ISR_PRS)) {
        return -1;
    }
    dm9000_reg_write(DM9000_ISR, 0x01);     /* 清 PRS */

    /* 进入读数据: 设 MRCMD, 第一个读为 dummy */
    DM9000_INDEX_ADDR = DM9000_MRCMD;
    (void)DM9000_DATA_ADDR;

    w1 = DM9000_DATA_ADDR;                  /* (Len_L << 8) | Status */
    w2 = DM9000_DATA_ADDR;                  /* (reserved << 8) | Len_H */

    status  = w1 & 0xFF;
    pkt_len = (u16)(((w1 >> 8) & 0xFF) | ((w2 & 0xFF) << 8));

    /* 错误帧丢弃: RX Status 错误位 = bit0 RXEE | bit2 FOE | bit3 RFCE | bit4 RUNT | bit5 LATECOL | bit6 RXCP */
    if (status & 0x7D) {
        return -1;
    }
    if (pkt_len < 14 || pkt_len > 1536) {
        return -1;
    }

    *len = pkt_len;
    for (i = 0; i < pkt_len; i += 2) {
        u16 word = DM9000_DATA_ADDR;
        buf[i]     = (u8)(word & 0xFF);
        if (i + 1 < pkt_len) {
            buf[i + 1] = (u8)((word >> 8) & 0xFF);
        }
    }
    return 0;
}

/**
 * @brief  向 DM9000 TX FIFO 写入一帧并触发发送
 * @retval 0=成功, -1=超时
 */
int dm9000_send_packet(uint8_t *buf, u16 len)
{
    u16 i;
    u32 timeout;

    /* 等待上一次发送完成 (TXREQ 被硬件清 0) */
    timeout = 100000;
    while (dm9000_reg_read(DM9000_TCR) & 0x01) {
        if (--timeout == 0) return -1;
    }

    /* 写入发送长度 */
    dm9000_reg_write(DM9000_TXPLL, (u8)(len & 0xFF));
    dm9000_reg_write(DM9000_TXPLH, (u8)((len >> 8) & 0xFF));

    /* 写入数据 (16 位) */
    DM9000_INDEX_ADDR = DM9000_MWCMD;
    for (i = 0; i < len; i += 2) {
        u16 word = buf[i];
        if (i + 1 < len) {
            word |= ((u16)buf[i + 1] << 8);
        }
        DM9000_DATA_ADDR = word;
    }

    /* 触发发送 */
    dm9000_reg_write(DM9000_TCR, 0x01);

    /* 等待发送完成 (TX1END|TX2END) */
    timeout = 500000;
    while (!(dm9000_reg_read(DM9000_NSR) & 0x0C)) {
        if (--timeout == 0) return -1;
    }
    dm9000_reg_write(DM9000_NSR, 0x0C);     /* 清 TX 完成标志 */

    return 0;
}

/* ==================================================================== */
/* DM9000 内部 PHY 寄存器读取 (诊断链路用)                                */
/* ==================================================================== */
#define DM9000_EPCR        0x0B    /* EEPROM/PHY 控制寄存器 (0x0C=读, 0x0A=写) */
#define DM9000_EPAR        0x0C    /* EEPROM/PHY 地址寄存器: [7:6]=PHY地址, [4:0]=寄存器 */
#define DM9000_EPDRL       0x0D    /* EEPROM/PHY 数据低字节 */
#define DM9000_EPDRH       0x0E    /* EEPROM/PHY 数据高字节 */
#define DM9000_WCR         0x0F    /* 唤醒控制寄存器 (不是 EPCR!) */
#define DM9000_PHY_ADDR    0x40    /* 内部 PHY 地址 1 << 6 */

/* PHY 寄存器 (经 EPAR/EPCR 访问) */
#define PHY_BMCR           0x00    /* 基本模式控制 */
#define PHY_BMSR           0x01    /* 基本模式状态: bit2=link, bit5=AN完成 */
#define PHY_ANAR           0x04    /* 自动协商通告 */
#define PHY_ANLPAR         0x05    /* 自动协商对端能力 (0=没收到对端脉冲) */
#define BMCR_AN_ENABLE     0x1000  /* 自动协商使能 */
#define BMCR_AN_RESTART    0x0200  /* 重启自动协商 */
#define BMCR_POWER_DOWN    0x0800  /* 掉电 */
#define BMCR_ISOLATE       0x0400  /* 隔离 */

/* 最小固件测试开关: 1=只跑 PHY 复位+自动协商+无限轮询 (不起 LwIP), 0=正常 */
#define PHY_MINIMAL_TEST    0

/* ==================================================================== */
/* TCP Server 回调                                                       */
/* ==================================================================== */

static err_t tcp_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    struct pbuf *head;
    u16 total_len;
    (void)arg;
    (void)err;

    if (p == NULL) {
        /* 客户端关闭连接 */
        g_tcp_client_connected = 0;
        g_client_pcb = NULL;
        // printf("[ETH] client disconnected\r\n");
        return ERR_OK;
    }

    head = p;
    total_len = 0;
    while (p != NULL && total_len < ETH_RX_BUF_SIZE - 1) {
        u16 copy_len = p->len;
        if (total_len + copy_len > ETH_RX_BUF_SIZE - 1) {
            copy_len = ETH_RX_BUF_SIZE - 1 - total_len;
        }
        memcpy(&g_eth_rx_buf[total_len], p->payload, copy_len);
        total_len += copy_len;
        p = p->next;
    }
    g_eth_rx_buf[total_len] = '\0';
    g_eth_rx_len = total_len;
    g_eth_rx_ready = 1;

    tcp_recved(tpcb, total_len);
    pbuf_free(head);
    return ERR_OK;
}

static void tcp_err_cb(void *arg, err_t err)
{
    (void)arg;
    (void)err;
    g_tcp_client_connected = 0;
    g_client_pcb = NULL;
    // printf("[ETH] TCP error: %d\r\n", err);
}

static err_t tcp_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    (void)arg;
    (void)tpcb;
    (void)len;
    return ERR_OK;
}

static err_t tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg;
    (void)err;

    if (g_tcp_client_connected) {
        /* 单客户端: 拒绝第二个连接 */
        // printf("[ETH] reject new client (already connected)\r\n");
        tcp_abort(newpcb);
        return ERR_ABRT;
    }

    g_client_pcb = newpcb;
    g_tcp_client_connected = 1;
    g_cmd_src = CMD_SRC_UART;       /* 确保标志复位 */

    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, tcp_recv_cb);
    tcp_err(newpcb, tcp_err_cb);
    tcp_sent(newpcb, tcp_sent_cb);

    // printf("[ETH] client connected\r\n");
    return ERR_OK;
}

/* ==================================================================== */
/* TCP 发送: fputc 逐字节入环形缓冲, MX_LWIP_Process 分块发出             */
/* ==================================================================== */

/**
 * @brief  发送单字节到 TCP 客户端 (供 fputc 路由调用)
 * @note   非阻塞: 无客户端/缓冲满时静默丢弃, 不影响电机实时性
 */
void eth_tcp_send_byte(uint8_t ch)
{
    u16 used;

    if (!g_tcp_client_connected || g_client_pcb == NULL) {
        return;
    }
    used = (u16)(g_eth_tx_head - g_eth_tx_tail);
    if (used >= ETH_TX_BUF_SIZE) {
        return;     /* 缓冲满, 丢弃 */
    }
    g_eth_tx_buf[g_eth_tx_head % ETH_TX_BUF_SIZE] = ch;
    g_eth_tx_head++;
}

/**
 * @brief  将环形缓冲中的数据分块写入 LwIP 发送队列 (每次最多 ETH_TX_CHUNK)
 */
static void eth_tx_flush(void)
{
    if (!g_tcp_client_connected || g_client_pcb == NULL) {
        return;
    }

    while ((u16)(g_eth_tx_head - g_eth_tx_tail) > 0) {
        u16 used = (u16)(g_eth_tx_head - g_eth_tx_tail);
        u16 idx  = (u16)(g_eth_tx_tail % ETH_TX_BUF_SIZE);
        u16 n    = used;

        if (n > ETH_TX_CHUNK) n = ETH_TX_CHUNK;
        if (idx + n > ETH_TX_BUF_SIZE) n = (u16)(ETH_TX_BUF_SIZE - idx);   /* 不跨环尾 */
        if (tcp_sndbuf(g_client_pcb) < n) {
            n = tcp_sndbuf(g_client_pcb);
            if (n == 0) break;
        }
        if (tcp_write(g_client_pcb, &g_eth_tx_buf[idx], n, TCP_WRITE_FLAG_COPY) != ERR_OK) {
            break;
        }
        g_eth_tx_tail += n;
    }
}

/* ==================================================================== */
/* LwIP 周期处理 + 完整初始化                                             */
/* ==================================================================== */

/**
 * @brief  LwIP 协议栈周期处理 (主循环中调用)
 * @note   轮询收包 → 定时器 → 发送缓冲数据 → tcp_output
 */
/* 链路同步: 检测 PHY link, 变化时同步 netif 并打印 (约每秒一次) */
static void dm9000_link_poll(void)
{
    static u8 previous_link = 0xFF;
    u16 bmsr;
    u8  link;

    (void)dm9000_phy_read(PHY_BMSR);        /* 第一次读可能读到锁存的旧值 */
    bmsr = dm9000_phy_read(PHY_BMSR);
    link = (bmsr & 0x0004U) ? 1U : 0U;

    if (link != previous_link) {
        previous_link = link;
        if (link) {
            netif_set_link_up(&g_netif);
            // printf("[ETH] LINK UP\r\n");
        } else {
            netif_set_link_down(&g_netif);
            // printf("[ETH] LINK DOWN\r\n");
        }
    }
}

void MX_LWIP_Process(void)
{
    static u32 link_cnt = 0;

    if (!g_eth_ready) {
        return;     /* LwIP 未初始化, 不操作 netif */
    }

    /* 链路同步: 约每秒一次 */
    if ((++link_cnt % 20) == 0) {
        dm9000_link_poll();
    }
    /* PHY 状态周期检查 (调试用, 已注释; 需要时恢复) */
    /* if ((link_cnt % 100) == 0) { dm9000_phy_status_poll(); } */

    ethernetif_input(&g_netif);
    sys_check_timeouts();

    if (g_tcp_client_connected && g_client_pcb != NULL) {
        eth_tx_flush();
        tcp_output(g_client_pcb);
    }
}

/* ==================================================================== */
/* 最小诊断: 只验证寄存器接口 + EPAR/EPDRL/EPDRH/EPCR 流程, 不启动 LwIP   */
/* 关键: EPDRL=0x0D, EPDRH=0x0E, EPCR=0x0F                              */
/* 顺序: 写EPAR → 写EPCR=0x0C → 延时 → 写EPCR=0x00 → 读EPDRL → 读EPDRH  */
/* ==================================================================== */

/* 1. VID/PID 连续读取稳定性 */
static void dm9000_read_stability_test(void)
{
    int i, err = 0;
    for (i = 0; i < 1000; i++) {
        u16 vid = (u16)((dm9000_reg_read(DM9000_VIDH) << 8) | dm9000_reg_read(DM9000_VIDL));
        u16 pid = (u16)((dm9000_reg_read(DM9000_PIDH) << 8) | dm9000_reg_read(DM9000_PIDL));
        if (vid != 0x0A46 || pid != 0x9000) err++;
    }
    printf("[TEST] VID/PID errors=%d\r\n", err);
}

/* 2. 寄存器写回测试 (RCR) */
static void dm9000_register_write_test(void)
{
    dm9000_reg_write(DM9000_RCR, 0x00);
    printf("[TEST] RCR: write 00 -> read 0x%02X\r\n", dm9000_reg_read(DM9000_RCR));
    dm9000_reg_write(DM9000_RCR, 0x30);
    printf("[TEST] RCR: write 30 -> read 0x%02X\r\n", dm9000_reg_read(DM9000_RCR));
    dm9000_reg_write(DM9000_RCR, 0x00);
    printf("[TEST] RCR: write 00 -> read 0x%02X\r\n", dm9000_reg_read(DM9000_RCR));
}

/* 3. EPDR 写回测试 (验证 EPDRL/EPDRH 是否可读写) */
static void dm9000_epdr_test(void)
{
    uint8_t low, high;
    dm9000_reg_write(DM9000_EPCR, 0x00);
    dm9000_reg_write(DM9000_EPDRL, 0x5A);
    dm9000_reg_write(DM9000_EPDRH, 0xA5);
    low  = dm9000_reg_read(DM9000_EPDRL);
    high = dm9000_reg_read(DM9000_EPDRH);
    printf("[TEST] EPDR write A55A -> read %02X%02X %s\r\n",
           high, low, (low == 0x5A && high == 0xA5) ? "(OK)" : "(MISMATCH)");
}

/* 4. 干净 PHY 读取: 固定延时版 (EPCR=0x0B) */
static uint16_t dm9000_phy_read(uint8_t phy_reg)
{
    u8 low, high;

    dm9000_reg_write(DM9000_EPAR, (uint8_t)(DM9000_PHY_ADDR | (phy_reg & 0x1FU)));
    dm9000_reg_write(DM9000_EPCR, 0x0C);            /* EPOS=1, ERPRR=1: 发起 PHY 读 */
    delay_ms(1);
    dm9000_reg_write(DM9000_EPCR, 0x00);            /* 软件清除命令 */

    low  = dm9000_reg_read(DM9000_EPDRL);
    high = dm9000_reg_read(DM9000_EPDRH);
    return (uint16_t)((high << 8) | low);
}

/* 4b. PHY 写 */
static void dm9000_phy_write(uint8_t phy_reg, uint16_t value)
{
    dm9000_reg_write(DM9000_EPAR, (uint8_t)(DM9000_PHY_ADDR | (phy_reg & 0x1FU)));
    dm9000_reg_write(DM9000_EPDRL, (uint8_t)(value & 0xFFU));
    dm9000_reg_write(DM9000_EPDRH, (uint8_t)((value >> 8) & 0xFFU));
    dm9000_reg_write(DM9000_EPCR, 0x0A);            /* EPOS=1, ERPRW=1: 发起 PHY 写 */
    delay_ms(1);
    dm9000_reg_write(DM9000_EPCR, 0x00);
}

/* 4c. 哨兵值 PHY 读取 (哨兵未变返回 -1), 同样走 0x0B */
static int dm9000_phy_read_sentinel(uint8_t phy_reg, uint16_t sentinel, uint16_t *result)
{
    u8 low, high;

    dm9000_reg_write(DM9000_EPDRL, (uint8_t)(sentinel & 0xFFU));
    dm9000_reg_write(DM9000_EPDRH, (uint8_t)((sentinel >> 8) & 0xFFU));

    dm9000_reg_write(DM9000_EPAR, (uint8_t)(DM9000_PHY_ADDR | (phy_reg & 0x1FU)));
    dm9000_reg_write(DM9000_EPCR, 0x0C);
    delay_ms(1);
    dm9000_reg_write(DM9000_EPCR, 0x00);

    low  = dm9000_reg_read(DM9000_EPDRL);
    high = dm9000_reg_read(DM9000_EPDRH);

    {
        uint16_t value = (uint16_t)((high << 8) | low);
        printf("[PHY-SENT] reg=%u sent=%04X result=%04X %s\r\n",
               phy_reg, (unsigned)sentinel, (unsigned)value,
               (value == sentinel) ? "UNCHANGED" : "UPDATED");
        if (result) *result = value;
        return (value == sentinel) ? -1 : 0;
    }
}

/* 4d. PHY 基础读取测试 */
static void dm9000_phy_basic_test(void)
{
    u16 bmcr, bmsr1, bmsr2, id1, id2, anar;

    // printf("[PHY] EPCR address=0x%02X  WCR@0x0F=0x%02X\r\n",
    //        (unsigned)DM9000_EPCR, (unsigned)dm9000_reg_read(DM9000_WCR));

    bmcr  = dm9000_phy_read(0x00);
    bmsr1 = dm9000_phy_read(0x01);
    bmsr2 = dm9000_phy_read(0x01);
    id1   = dm9000_phy_read(0x02);
    id2   = dm9000_phy_read(0x03);
    anar  = dm9000_phy_read(0x04);

    // printf("[PHY] BMCR  =0x%04X\r\n", (unsigned)bmcr);
    // printf("[PHY] BMSR1 =0x%04X\r\n", (unsigned)bmsr1);
    // printf("[PHY] BMSR2 =0x%04X\r\n", (unsigned)bmsr2);
    // printf("[PHY] ID1   =0x%04X\r\n", (unsigned)id1);
    // printf("[PHY] ID2   =0x%04X\r\n", (unsigned)id2);
    // printf("[PHY] ANAR  =0x%04X\r\n", (unsigned)anar);
    (void)bmcr; (void)bmsr1; (void)bmsr2; (void)id1; (void)id2; (void)anar;
}

/* 4e. 重启 PHY 自动协商 */
static void dm9000_phy_restart_autoneg(void)
{
    u16 bmcr = dm9000_phy_read(PHY_BMCR);

    bmcr |= BMCR_AN_ENABLE | BMCR_AN_RESTART;
    bmcr &= (u16)~(BMCR_POWER_DOWN | BMCR_ISOLATE);

    dm9000_phy_write(PHY_BMCR, bmcr);
    // printf("[PHY] restart autoneg BMCR=0x%04X\r\n", (unsigned)dm9000_phy_read(PHY_BMCR));
}

/* 4f. 等待链路: 轮询 BMSR (双读防锁存) + ANLPAR, 最多 5 秒 */
static void dm9000_wait_link(void)
{
    u32 i;
    for (i = 0; i < 50; i++) {
        (void)dm9000_phy_read(PHY_BMSR);    /* 第一次读清锁存 */
        u16 bmsr2 = dm9000_phy_read(PHY_BMSR);

        /* 逐次打印已注释, 需要时恢复 */
        /* printf("[PHY] %lu BMSR=%04X LINK=%u AN_DONE=%u ANLPAR=%04X\r\n", ...); */

        if ((bmsr2 & 0x0004U) != 0U) {
            // printf("[PHY] LINK UP\r\n");
            return;
        }
        delay_ms(100);
    }
    // printf("[PHY] LINK TIMEOUT (ANLPAR=%04X -> no peer pulses, check cable/hardware)\r\n",
    //        (unsigned)dm9000_phy_read(PHY_ANLPAR));
}

/* 4g. 强制 10M 半/全双工 (交叉验证模拟收发电路, 电脑网卡需同步设置) */
static void dm9000_phy_force_10_half(void)
{
    dm9000_phy_write(PHY_BMCR, 0x0000);     /* 10M, 半双工, 关 AN */
    delay_ms(1000);
    // printf("[PHY-10H] BMCR=%04X BMSR=%04X LINK=%u\r\n",
    //        (unsigned)dm9000_phy_read(PHY_BMCR),
    //        (unsigned)dm9000_phy_read(PHY_BMSR),
    //        (dm9000_phy_read(PHY_BMSR) & 0x0004U) ? 1U : 0U);
}

static void dm9000_phy_force_10_full(void)
{
    dm9000_phy_write(PHY_BMCR, 0x0100);     /* 10M, 全双工, 关 AN */
    delay_ms(1000);
    // printf("[PHY-10F] BMCR=%04X BMSR=%04X LINK=%u\r\n",
    //        (unsigned)dm9000_phy_read(PHY_BMCR),
    //        (unsigned)dm9000_phy_read(PHY_BMSR),
    //        (dm9000_phy_read(PHY_BMSR) & 0x0004U) ? 1U : 0U);
}

/* 4h. PHY 软件复位 + 重新配置自动协商 */
static int dm9000_phy_reset_and_autoneg(void)
{
    u32 timeout;

    /* PHY 软件复位 */
    dm9000_phy_write(PHY_BMCR, 0x8000);
    for (timeout = 0; timeout < 100; timeout++) {
        if ((dm9000_phy_read(PHY_BMCR) & 0x8000U) == 0U) break;
        delay_ms(10);
    }
    if (timeout >= 100) {
        // printf("[PHY] reset timeout\r\n");
        return -1;
    }

    /* 发布 10M/100M、半/全双工能力 */
    dm9000_phy_write(PHY_ANAR, 0x01E1);
    /* 自动协商使能 + 重启, 清掉电/隔离 */
    dm9000_phy_write(PHY_BMCR, 0x1200);
    delay_ms(100);

    // printf("[PHY] reset complete BMCR=%04X ANAR=%04X\r\n",
    //        (unsigned)dm9000_phy_read(PHY_BMCR),
    //        (unsigned)dm9000_phy_read(PHY_ANAR));
    return 0;
}

/* 4i. 最小固件测试: 无限轮询 BMSR/ANLPAR, 不起 LwIP (彻底排除软件) */
static void dm9000_phy_minimal_test(void)
{
    printf("[TEST] PHY MINIMAL TEST mode (no LwIP)\r\n");
    dm9000_phy_reset_and_autoneg();

    while (1) {
        u16 bmcr = dm9000_phy_read(0x00);
        (void)dm9000_phy_read(0x01);
        u16 bmsr = dm9000_phy_read(0x01);
        u16 anlpar = dm9000_phy_read(0x05);
        printf("BMCR=%04X BMSR=%04X LINK=%u ANLPAR=%04X\r\n",
               (unsigned)bmcr, (unsigned)bmsr,
               (bmsr & 0x0004U) ? 1U : 0U, (unsigned)anlpar);
        delay_ms(500);
    }
}

/* 4j. 周期打印 PHY 状态 (检查是否被其他代码改坏 BMCR) */
static void dm9000_phy_status_poll(void)
{
    u16 bmcr = dm9000_phy_read(0x00);
    (void)dm9000_phy_read(0x01);
    u16 bmsr = dm9000_phy_read(0x01);
    u16 anar = dm9000_phy_read(0x04);
    u16 anlpar = dm9000_phy_read(0x05);
    // printf("[PHY] BMCR=%04X BMSR=%04X ANAR=%04X ANLPAR=%04X\r\n",
    //        (unsigned)bmcr, (unsigned)bmsr, (unsigned)anar, (unsigned)anlpar);
    (void)bmcr; (void)bmsr; (void)anar; (void)anlpar;
}

/* 5. PHY 掉电再唤醒, 排除上电状态机卡住 (只看 bit0) */
static void dm9000_phy_power_cycle(void)
{
    u8 gpcr, gpr;

    dm9000_reg_write(DM9000_GPCR, 0x01);        /* GPIO0 = 输出 */

    dm9000_reg_write(DM9000_GPR, 0x01);         /* GPIO0=1: PHY 掉电 */
    delay_ms(20);
    gpcr = dm9000_reg_read(DM9000_GPCR);
    gpr  = dm9000_reg_read(DM9000_GPR);
    printf("[PHY-PWR] OFF GPCR=%02X/%u GPR=%02X/%u\r\n", gpcr, gpcr & 1U, gpr, gpr & 1U);

    dm9000_reg_write(DM9000_GPR, 0x00);         /* GPIO0=0: PHY 唤醒 */
    delay_ms(100);
    gpcr = dm9000_reg_read(DM9000_GPCR);
    gpr  = dm9000_reg_read(DM9000_GPR);
    printf("[PHY-PWR] ON  GPCR=%02X/%u GPR=%02X/%u\r\n", gpcr, gpcr & 1U, gpr, gpr & 1U);
}

/* 6. GPR bit0 交替写回测试: 验证 GPR 能否写 1 */
static void dm9000_gpr_toggle_test(void)
{
    static const u8 values[] = { 0x00, 0x01, 0x00, 0x01, 0x00 };
    u32 i;

    printf("[GPR-TEST] start\r\n");
    dm9000_reg_write(DM9000_GPCR, 0x01);
    delay_ms(1);

    for (i = 0; i < sizeof(values); i++) {
        u8 wv = values[i];
        u8 rv;
        dm9000_reg_write(DM9000_GPR, wv);
        delay_ms(2);
        rv = dm9000_reg_read(DM9000_GPR);
        printf("[GPR-TEST] write=%02X read=%02X write_bit0=%u read_bit0=%u %s\r\n",
               wv, rv, wv & 1U, rv & 1U,
               ((wv ^ rv) & 1U) ? "FAIL" : "OK");
    }
}

/* 7. 写寄存器带读回日志 (针对 GPCR/GPR) */
static void dm9000_write_reg_debug(u8 reg, u8 value)
{
    dm9000_reg_write(reg, value);
    {
        u8 back = dm9000_reg_read(reg);
        printf("[REG-W] reg=%02X write=%02X read=%02X\r\n", reg, value, back);
    }
}

/* 8. 相邻寄存器 0x1C-0x20 测试, 检查寻址错位 */
static void dm9000_gpr_address_test(void)
{
    u8 reg;

    printf("[REG] 0x1C..0x20 before:\r\n");
    for (reg = 0x1C; reg <= 0x20; reg++) {
        printf("[REG] 0x%02X = 0x%02X\r\n", reg, dm9000_reg_read(reg));
    }
    dm9000_reg_write(0x1F, 0x01);
    delay_ms(1);
    printf("[REG] after GPR write 01:\r\n");
    for (reg = 0x1C; reg <= 0x20; reg++) {
        printf("[REG] 0x%02X = 0x%02X\r\n", reg, dm9000_reg_read(reg));
    }
}

/* 9. 索引锁存测试: 写索引后读索引端口 (DM9000 可能不支持读回, 验证一下) */
static void dm9000_index_latch_test(void)
{
    static const u8 idx[] = { 0x00, 0x05, 0x0C, 0x1E, 0x1F, 0x28, 0x29 };
    u32 i;

    printf("[INDEX-TEST] start\r\n");
    for (i = 0; i < sizeof(idx); i++) {
        *g_dm9000_index = idx[i];
        {
            u16 raw = *g_dm9000_index;
            printf("[INDEX-TEST] write=%02X index-read=%04X low=%02X %s\r\n",
                   idx[i], (unsigned)raw, (unsigned)(raw & 0xFFU),
                   ((raw & 0xFFU) == idx[i]) ? "OK" : "FAIL");
        }
    }
}

/* 10. 宏值打印 + CHIPR + 寄存器 0x00..0x2C 全地图 dump */
static void dm9000_chip_dump(void)
{
    u8 reg;

    printf("[MACRO] NCR=%02X RCR=%02X EPAR=%02X EPDRL=%02X EPDRH=%02X EPCR=%02X GPCR=%02X GPR=%02X\r\n",
           (unsigned)DM9000_NCR, (unsigned)DM9000_RCR,
           (unsigned)DM9000_EPAR, (unsigned)DM9000_EPDRL,
           (unsigned)DM9000_EPDRH, (unsigned)DM9000_EPCR,
           (unsigned)DM9000_GPCR, (unsigned)DM9000_GPR);

    printf("[CHIPR] 0x2C = 0x%02X\r\n", (unsigned)dm9000_reg_read(0x2C));

    printf("[MAP] 0x00..0x2C:\r\n");
    for (reg = 0x00; reg <= 0x2C; reg++) {
        u8 v = dm9000_reg_read(reg);
        printf("%02X=%02X%s", (unsigned)reg, (unsigned)v,
               ((reg & 0x07) == 0x07) ? "\r\n" : "  ");
    }
    printf("\r\n");
}

/* 最小诊断主流程 (按调整后的初始化顺序) */
static void dm9000_minimal_diag(void)
{
    u16 bmsr, id1, id2;
    u8 ncr;

    printf("[TEST] DM9000 minimal diagnosis\r\n");

    /* 保持 LwIP 符号引用, 避免诊断模式下 "declared but never referenced" 警告 */
    (void)g_tcp_pcb; (void)g_ipaddr; (void)g_netmask; (void)g_gw; (void)tcp_accept_cb;
    (void)dm9000_phy_write;   /* 诊断阶段暂不执行 PHY 写 */
    (void)dm9000_phy_force_10_half; (void)dm9000_phy_force_10_full;   /* 强制 10M 交叉验证备用 */
    (void)dm9000_phy_minimal_test;  /* 最小固件测试备用 (PHY_MINIMAL_TEST=1 时启用) */
    (void)dm9000_phy_status_poll;   /* PHY 状态周期检查备用 */

    /* 读/写稳定性 + EPDR 写回 */
    dm9000_read_stability_test();
    dm9000_register_write_test();
    dm9000_epdr_test();

    /* 1. 验证芯片 */
    printf("[TEST] chip: VID=%02X%02X PID=%02X%02X\r\n",
           (unsigned)dm9000_reg_read(DM9000_VIDH), (unsigned)dm9000_reg_read(DM9000_VIDL),
           (unsigned)dm9000_reg_read(DM9000_PIDH), (unsigned)dm9000_reg_read(DM9000_PIDL));

    /* 2. DM9000 软件复位 */
    dm9000_reg_write(DM9000_NCR, 0x01);
    delay_ms(2);

    /* 3. 确保内部 PHY (NCR bit7=0, 不清其他位) */
    ncr = dm9000_reg_read(DM9000_NCR);
    printf("[TEST] NCR raw=0x%02X EXT_PHY=%u\r\n", ncr, (ncr >> 7) & 0x01U);
    ncr &= (u8)~0x80U;
    dm9000_reg_write(DM9000_NCR, ncr);
    printf("[TEST] select internal PHY, NCR=0x%02X\r\n", (unsigned)dm9000_reg_read(DM9000_NCR));

    /* 4. GPR bit0 交替写回测试 (关键: 验证 GPR 能否写 1) */
    dm9000_gpr_toggle_test();

    /* 5. 带读回日志写 GPCR/GPR */
    printf("[REG-W-DBG] GPCR/GPR:\r\n");
    dm9000_write_reg_debug(0x1E, 0x00);
    dm9000_write_reg_debug(0x1E, 0x01);
    dm9000_write_reg_debug(0x1F, 0x00);
    dm9000_write_reg_debug(0x1F, 0x01);

    /* 6. 相邻寄存器 0x1C-0x20, 检查寻址错位 */
    dm9000_gpr_address_test();

    /* 6b. 索引锁存 + 宏值 + CHIPR + 全地图 dump */
    dm9000_index_latch_test();
    dm9000_chip_dump();

    /* 7. PHY 掉电再唤醒 */
    dm9000_phy_power_cycle();

    /* 8. 等待 PHY 模拟部分稳定 */
    delay_ms(100);

    /* 9. PHY 基础读取测试 (EPCR 修正到 0x0B 后) */
    dm9000_phy_basic_test();

    /* 10. 哨兵值复核 */
    {
        int r0, r1, r2, r3;
        r0 = dm9000_phy_read_sentinel(0x00, 0xA55A, NULL);  /* BMCR */
        r1 = dm9000_phy_read_sentinel(0x01, 0x5AA5, &bmsr); /* BMSR */
        r2 = dm9000_phy_read_sentinel(0x02, 0x1234, &id1);  /* PHYID1 */
        r3 = dm9000_phy_read_sentinel(0x03, 0x5678, &id2);  /* PHYID2 */

        if (r0 == 0 && r1 == 0 && r2 == 0 && r3 == 0) {
            /* printf("[PHY] BMSR=..."); 已注释 */
            (void)0;
        } else {
            /* printf("[PHY] internal PHY ..."); 已注释 */
            (void)0;
        }
        (void)bmsr; (void)id1; (void)id2;   /* 避免 set-but-unused 告警 */
    }

    printf("[TEST] minimal diagnosis done\r\n");
}

/**
 * @brief  以太网完整初始化: FSMC → DM9000 → LwIP → TCP Server
 * @retval 0=成功, -1=DM9000 未识别, -2=LwIP/TCP 失败
 */
int eth_init(void)
{
    /* 1. FSMC + GPIO 配置 */
    eth_fsmc_init();

    /* 2. DM9000 硬件初始化 (失败不 Error_Handler, 串口模式仍可用) */
    if (dm9000_hw_init(g_mac_addr) != 0) {
        return -1;
    }

    /* 开机 PHY 确认: 诊断打印已注释, 需要时恢复 dm9000_minimal_diag() 调用 */
    /* dm9000_minimal_diag(); */
    (void)dm9000_minimal_diag;   /* 保持符号引用, 避免未使用警告 */

#if PHY_MINIMAL_TEST
    /* 最小固件模式: 只做 PHY 复位+自动协商+无限轮询, 彻底排除软件影响 */
    dm9000_phy_minimal_test();      /* 无限循环, 不返回 */
#endif

    /* 重启自动协商并等待链路 (最多 5 秒, 打印 BMSR/ANLPAR) */
    dm9000_phy_restart_autoneg();
    dm9000_wait_link();

    /* 确保 NCR=0: 正常模式 + 内部 PHY + 无回环 (复位/协商流程可能改过该寄存器) */
    dm9000_reg_write(DM9000_NCR, 0x00);

    /* 3. LwIP 初始化 */
    lwip_init();
    // printf("[ETH] LwIP init done\r\n");

    /* 4. 配置 IP 地址 (来自 g_eth_ip, 由 param_load_all 从 Flash 加载) */
    IP4_ADDR(&g_ipaddr,
             g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);
    IP4_ADDR(&g_netmask,
             g_eth_mask[0], g_eth_mask[1], g_eth_mask[2], g_eth_mask[3]);
    IP4_ADDR(&g_gw,
             g_eth_gw[0], g_eth_gw[1], g_eth_gw[2], g_eth_gw[3]);

    /* 5. 添加网络接口 */
    if (netif_add(&g_netif, &g_ipaddr, &g_netmask, &g_gw,
                  NULL, ethernetif_init, netif_input) == NULL) {
        // printf("[ETH] netif_add failed!\r\n");
        return -2;
    }
    netif_set_default(&g_netif);
    netif_set_up(&g_netif);

    // printf("[ETH] netif up: %d.%d.%d.%d\r\n",
    //        g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);

    /* 6. 创建 TCP Server */
    g_tcp_pcb = tcp_new();
    if (g_tcp_pcb == NULL) {
        // printf("[ETH] TCP PCB create failed!\r\n");
        return -2;
    }
    if (tcp_bind(g_tcp_pcb, IP_ADDR_ANY, g_eth_port) != ERR_OK) {
        // printf("[ETH] TCP bind port %d failed!\r\n", (int)g_eth_port);
        return -2;
    }
    g_tcp_pcb = tcp_listen(g_tcp_pcb);
    tcp_accept(g_tcp_pcb, tcp_accept_cb);

    // printf("[ETH] TCP server listening on port %d\r\n", (int)g_eth_port);
    g_eth_ready = 1;
    return 0;
}

/* ==================================================================== */
/* LwIP 移植: NO_SYS 模式下需要提供的系统接口                             */
/* ==================================================================== */

/* lwip_init() 无条件调用 sys_init(), NO_SYS 模式下为空实现 */
void sys_init(void)
{
}

/* 毫秒时钟: 供 sys_check_timeouts() / TCP 时间戳使用 */
u32_t sys_now(void)
{
    return HAL_GetTick();
}
