/**
 ****************************************************************************************************
 * @file        eth.c
 * @brief       DM9000 以太网驱动 + LwIP 集成 + TCP Server
 * @note        DM9000 通过 FSMC Bank1 (NE1, 基址 0x60000000) 16位访问
 *              CMD 引脚接 A2 (HADDR[2]), 所以:
 *              数据端口 = 0x60000000 + (0 << 2) = 0x60000000
 *              索引端口 = 0x60000000 + (1 << 2) = 0x60000004
 ****************************************************************************************************
 */
#include "eth.h"
#include "delay.h"
#include "stm32f4xx_hal.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"

/* ---- DM9000 硬件地址 (FSMC Bank1, NE1) ---- */
#define DM9000_DATA_ADDR    (*((volatile uint16_t *)0x60000000))
#define DM9000_INDEX_ADDR   (*((volatile uint16_t *)0x60000004))

/* ---- DM9000 关键寄存器 ---- */
#define DM9000_NCR          0x00    /* 网络控制寄存器 */
#define DM9000_NSR          0x01    /* 网络状态寄存器 */
#define DM9000_TCR          0x02    /* 发送控制寄存器 */
#define DM9000_TSR_I        0x03    /* 发送状态寄存器1 */
#define DM9000_TSR_II       0x04    /* 发送状态寄存器2 */
#define DM9000_RCR          0x05    /* 接收控制寄存器 */
#define DM9000_RSR          0x06    /* 接收状态寄存器 */
#define DM9000_ISR          0xFE    /* 中断状态寄存器 */
#define DM9000_IMR          0xFF    /* 中断屏蔽寄存器 */
#define DM9000_MRCMD        0xF8    /* 内存读命令 (需先写 MWCMD 设地址) */
#define DM9000_MWCMD        0xF8    /* 内存写命令 */
#define DM9000_MRRL         0xF4    /* 内存地址低字节 */
#define DM9000_MRRH         0xF5    /* 内存地址高字节 */
#define DM9000_MWRL         0xF6    /* 内存地址低字节(写) */
#define DM9000_MWRH         0xF7    /* 内存地址高字节(写) */
#define DM9000_TXPLL        0xFC    /* 发送包长度低字节 */
#define DM9000_TXPLH        0xFD    /* 发送包长度高字节 */
#define DM9000_PAR          0x10    /* 物理地址(MAC) 寄存器起始(0x10~0x15) */
#define DM9000_VIDL         0x28    /* 厂商ID 低字节 */
#define DM9000_VIDH         0x29    /* 厂商ID 高字节 */
#define DM9000_PIDL         0x2A    /* 产品ID 低字节 */
#define DM9000_PIDH         0x2B    /* 产品ID 高字节 */
#define DM9000_GPCR         0x1E    /* GPIO 控制 */
#define DM9000_GPR          0x1F    /* GPIO 数据 */
#define DM9000_FCR          0x2D    /* 流控制寄存器 */

/* 发送控制位 */
#define DM9000_TCR_REQ      (1 << 0)
/* 网络控制位 */
#define DM9000_NCR_RST      (1 << 0)
#define DM9000_NCR_LBK      (1 << 2)  /* MAC 内部回环(第3种模式) */
/* 接收控制位 */
#define DM9000_RCR_ENABLE   (1 << 0)
#define DM9000_RCR_PASS_ALL (1 << 3)
/* 中断状态位 */
#define DM9000_ISR_PRS      (1 << 0)  /* 包接收 */
#define DM9000_ISR_PTS      (1 << 1)  /* 包发送完成 */

/* ---- 全局变量 ---- */
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
u8 g_mac_addr[6];              /* MAC 地址 (供 ethernetif.c extern) */

/* DM9000 数据缓冲区 (静态分配，避免 malloc, 供 ethernetif.c 共享) */
u8 dm9000_tx_buf[1536];         /* 最大以太网帧 */

/* ---- 网络参数 (默认值) ---- */
u8  g_eth_ip[4]   = {192, 168, 1, 100};
u8  g_eth_mask[4] = {255, 255, 255, 0};
u8  g_eth_gw[4]   = {192, 168, 1, 1};
u16 g_eth_port    = 5000;

/* ---- 转发声明 ---- */
static err_t tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void  tcp_err_cb(void *arg, err_t err);
static err_t tcp_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len);

/**
 * @brief  读 DM9000 寄存器
 * @param  reg: 寄存器地址
 * @retval 寄存器值
 */
uint8_t dm9000_reg_read(uint16_t reg)
{
    DM9000_INDEX_ADDR = reg;
    return (uint8_t)(DM9000_DATA_ADDR & 0xFF);
}

/**
 * @brief  写 DM9000 寄存器
 * @param  reg: 寄存器地址
 * @param  val: 要写入的值
 */
void dm9000_reg_write(uint16_t reg, uint8_t val)
{
    DM9000_INDEX_ADDR = reg;
    DM9000_DATA_ADDR = val;
}

/**
 * @brief  DM9000 硬件复位 + VID/PID 校验 + 读取 MAC 地址
 * @param  mac_addr: 输出 MAC 地址 (6 bytes)
 * @retval 0=成功, -1=DM9000 未识别
 */
int dm9000_hw_init(uint8_t *mac_addr)
{
    u16 vid, pid;
    int i;

    /* PD7 拉低 10ms 复位 DM9000 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    delay_ms(10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    delay_ms(10);

    /* 软件复位: NCR 写 RST 位 */
    dm9000_reg_write(DM9000_NCR, DM9000_NCR_RST);
    delay_ms(5);
    dm9000_reg_write(DM9000_NCR, 0x00);
    delay_ms(5);

    /* 读取 VID/PID */
    vid = (dm9000_reg_read(DM9000_VIDH) << 8) | dm9000_reg_read(DM9000_VIDL);
    pid = (dm9000_reg_read(DM9000_PIDH) << 8) | dm9000_reg_read(DM9000_PIDL);

    if (vid != 0x0A46 || pid != 0x9000) {
        printf("[ETH] DM9000 未识别! VID=0x%04X PID=0x%04X (期望 0x0A46/0x9000)\r\n",
               vid, pid);
        return -1;
    }
    printf("[ETH] DM9000 识别成功 VID=0x%04X PID=0x%04X\r\n", vid, pid);

    /* 读取 MAC 地址 (PAR 寄存器 0x10~0x15) */
    for (i = 0; i < 6; i++) {
        mac_addr[i] = dm9000_reg_read(DM9000_PAR + i);
    }
    printf("[ETH] MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);

    /* 配置: 正常模式, 清除所有中断 */
    dm9000_reg_write(DM9000_NCR, 0x00);
    dm9000_reg_write(DM9000_IMR, 0x80);  /* 使能包接收中断位 */
    /* 接收控制: 使能接收, 不过滤, 丢弃 < 64 字节的 runt 帧 */
    dm9000_reg_write(DM9000_RCR, DM9000_RCR_ENABLE);
    /* 流控制: 开启全双工流控 */
    dm9000_reg_write(DM9000_FCR, 0x39);

    return 0;
}
