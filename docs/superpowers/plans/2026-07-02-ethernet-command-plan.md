# 以太网口命令功能 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 UART5 串口命令基础上，新增以太网口（DM9000 + LwIP + TCP Server）作为并行命令通道。

**Architecture:** 新增 `eth.c/.h` 封装 DM9000 驱动 + LwIP + TCP Server。改造 `fputc()` 实现输出路由（UART5 始终输出，TCP 来源时同步发送到 TCP 客户端）。网络参数（IP/MASK/GW/PORT）存入 Flash，通过 `IP:`/`MASK:`/`GW:`/`PORT:` 命令配置。`cmd_dispatch()` 无需修改。

**Tech Stack:** STM32F407ZG, DM9000 Ethernet PHY (FSMC), LwIP 2.x TCP/IP stack, Keil MDK, C

## Global Constraints

- 所有命令处理函数（40+ 个）**不得修改** — `cmd_dispatch()` 签名不变
- TCP 发送失败时**不阻塞、不重试** — 保证电机控制实时性
- DM9000 初始化失败**不触发 Error_Handler()** — 串口模式正常可用
- 网络参数修改后需**复位生效**（发送 `RES` 重启）
- 串口输出**始终保留** — `fputc()` 不管来源都写入 UART5
- 接收模式使用**轮询** — 不注册 PG6 外部中断，避免干扰定时器
- LwIP 静态 IP — 无 UDP/DHCP/DNS
- 单 TCP 客户端 — 第二个连接请求被拒绝
- Flash 参数继续使用 Sector 7，网络参数从 offset 0x4C 开始

---

## File Structure

| 文件 | 操作 | 职责 |
|------|:----:|------|
| `Core/Inc/eth.h` | **新增** | 以太网模块公共接口：`eth_init()`, `eth_tcp_send_byte()`, `g_cmd_src` 类型定义, `g_eth_rx_ready`, `g_eth_rx_buf`, `g_eth_rx_len`, `g_tcp_client_connected` |
| `Core/Src/eth.c` | **新增** | DM9000 驱动（寄存器 R/W + 帧收发）、LwIP netif 创建、TCP Server（bind/listen/accept/recv/err）、`MX_LWIP_Process()` |
| `Core/Src/ethernetif.c` | **新增** | LwIP 移植对接层：`low_level_init()`, `low_level_output()`, `low_level_input()`, `ethernetif_input()` |
| `Core/Inc/lwipopts.h` | **新增** | LwIP 配置裁剪：TCP only, 静态 IP, 无 UDP/DHCP/DNS |
| `Core/Src/usart.c` | 修改 | `fputc()` 增加 TCP 路由分支（约 3 行） |
| `Core/Inc/flash_param.h` | 修改 | 新增 4 个网络参数 offset 定义 |
| `Core/Src/flash_param.c` | 修改 | `param_load_all()`/`param_save_all()` 增加网络参数读写 |
| `Core/Inc/cmd_handler.h` | 修改 | 新增 `cmd_set_ip/mask/gw/port` 声明 |
| `Core/Src/cmd_handler.c` | 修改 | 新增 4 个处理函数 + `cmd_dispatch()` 增加 4 个分支 + `PARA` 扩展输出 |
| `Core/Src/main.c` | 修改 | 初始化调 `eth_init()`，主循环增加 `MX_LWIP_Process()` + TCP RX 处理 |

---

### Task 1: LwIP 源码引入 + `lwipopts.h` 配置

**Files:**
- Create: `Core/Inc/lwipopts.h`
- 添加 LwIP 源码到 `Middlewares/LwIP/`（通过 Keil RTE 或手动导入）

**Interfaces:**
- Produces: LwIP 编译所需的全部宏定义（供 Task 2/3/4 使用）

**说明:** LwIP 源码建议通过 Keil MDK RTE 添加（CMSIS Driver → Network → LwIP），避免手动管理源文件。若 RTE 不可用，从 STM32CubeF4 `Middlewares/Third_Party/LwIP/` 复制以下目录到项目：
- `src/core/` — 全部 .c
- `src/netif/` — `ethernet.c`
- `src/api/` — `tcp.c`, `api_msg.c`, `api_lib.c`, `err.c`, `netbuf.c`, `netdb.c`, `sockets.c`
- `include/` — 全部头文件路径加入 include path
- `port/` — 由本项目 `ethernetif.c` 提供（Task 3）

- [ ] **Step 1: 创建 `Core/Inc/lwipopts.h`**

```c
/**
 ****************************************************************************************************
 * @file        lwipopts.h
 * @brief       LwIP 配置裁剪: TCP Server only, 静态 IP, 最小 RAM/Flash 占用
 ****************************************************************************************************
 */
#ifndef __LWIPOPTS_H
#define __LWIPOPTS_H

/* ---- 功能裁剪 ---- */
#define LWIP_TCP                1
#define LWIP_UDP                0
#define LWIP_DHCP               0
#define LWIP_DNS                0
#define LWIP_SNMP               0
#define LWIP_IGMP               0
#define LWIP_ICMP               1       /* ping 可用 */
#define LWIP_RAW                0
#define LWIP_NETCONN            0       /* 使用 raw API，不使用 netconn/socket */
#define LWIP_SOCKET             0
#define LWIP_STATS              0
#define LWIP_STATS_DISPLAY      0

/* ---- 内存池 ---- */
#define MEMP_NUM_TCP_PCB        4
#define MEMP_NUM_TCP_SEG        16
#define MEMP_NUM_NETBUF         8
#define MEMP_NUM_NETCONN        0
#define MEMP_NUM_TCPIP_MSG_API  8
#define MEMP_NUM_TCPIP_MSG_INPKT 8
#define MEMP_NUM_SYS_TIMEOUT    6

/* ---- TCP 参数 ---- */
#define TCP_MSS                 1460
#define TCP_SND_BUF             (4 * TCP_MSS)
#define TCP_WND                 (2 * TCP_MSS)
#define TCP_TTL                 255
#define TCP_SNDLOWAT            0
#define TCP_QUEUE_OOSEQ         1

/* ---- 内存 ---- */
#define MEM_ALIGNMENT           4
#define MEM_SIZE                (8 * 1024)      /* 8KB heap */
#define MEMP_MEM_MALLOC         0
#define MEMP_MEM_INIT           0

/* ---- 网络接口 ---- */
#define LWIP_NETIF_HOSTNAME     1
#define LWIP_NETIF_API          0
#define LWIP_NETIF_STATUS_CALLBACK 0
#define LWIP_NETIF_LINK_CALLBACK   0
#define LWIP_NETIF_REMOVE_CALLBACK 0

/* ---- 杂项 ---- */
#define LWIP_NO_REDIRECT_ASSERT 1
#define LWIP_DONT_PROVIDE_BYTEON_FUNCTIONS 1
#define CHECKSUM_BY_HARDWARE    0       /* DM9000 无硬件校验 */
#define LWIP_CHECKSUM_ON_COPY   0
#define TCPIP_THREAD_STACKSIZE  1024
#define TCPIP_THREAD_PRIO       2
#define TCPIP_MBOX_SIZE         10
#define DEFAULT_TCP_RECVMBOX_SIZE 6
#define DEFAULT_ACCEPTMBOX_SIZE   6
#define DEFAULT_RAW_RECVMBOX_SIZE 0
#define DEFAULT_UDP_RECVMBOX_SIZE 0
#define LWIP_HOOK_FILENAME      "lwipopts.h"

/* ---- 调试 (release 关闭) ---- */
#define LWIP_DEBUG              0
#define TCP_DEBUG               LWIP_DBG_OFF
#define TCP_INPUT_DEBUG          LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG         LWIP_DBG_OFF
#define ETHARP_DEBUG             LWIP_DBG_OFF
#define NETIF_DEBUG              LWIP_DBG_OFF

#endif /* __LWIPOPTS_H */
```

- [ ] **Step 2: 添加 LwIP 源码**

**方式 A (推荐): Keil RTE**
在 Keil MDK 中打开 RTE 管理器 → CMSIS → Network → 勾选 LwIP，自动添加源文件并设置 include paths。

**方式 B: 手动导入**
从 `STM32CubeF4/Middlewares/Third_Party/LwIP/` 复制以下文件:
```
Middlewares/LwIP/src/core/ipv4/icmp.c
Middlewares/LwIP/src/core/ipv4/ip4.c
Middlewares/LwIP/src/core/ipv4/ip4_addr.c
Middlewares/LwIP/src/core/ipv4/ip4_frag.c
Middlewares/LwIP/src/core/ipv4/etharp.c
Middlewares/LwIP/src/core/tcp.c
Middlewares/LwIP/src/core/tcp_in.c
Middlewares/LwIP/src/core/tcp_out.c
Middlewares/LwIP/src/core/init.c
Middlewares/LwIP/src/core/def.c
Middlewares/LwIP/src/core/dns.c     (LwIP 需要 dns.c 编译, 即使 DNS=0)
Middlewares/LwIP/src/core/mem.c
Middlewares/LwIP/src/core/memp.c
Middlewares/LwIP/src/core/netif.c
Middlewares/LwIP/src/core/pbuf.c
Middlewares/LwIP/src/core/stats.c
Middlewares/LwIP/src/core/sys.c
Middlewares/LwIP/src/core/timeouts.c
Middlewares/LwIP/src/core/udp.c    (LwIP 编译需要, 即使 UDP=0)
Middlewares/LwIP/src/api/api_lib.c
Middlewares/LwIP/src/api/api_msg.c
Middlewares/LwIP/src/api/err.c
Middlewares/LwIP/src/api/tcpip.c
Middlewares/LwIP/src/netif/ethernet.c
```
Include paths 添加 `Middlewares/LwIP/src/include/`。

- [ ] **Step 3: 编译验证** — 确保 LwIP 源码编译通过（会有未定义引用，因为 `ethernetif.c` 还未添加）

- [ ] **Step 4: Commit**

```bash
git add Core/Inc/lwipopts.h Middlewares/LwIP/
git commit -m "feat: add LwIP source and lwipopts.h (TCP only, static IP)"
```

---

### Task 2: DM9000 驱动 — FSMC 配置 + 寄存器读写 + 初始化

**Files:**
- Create: `Core/Inc/eth.h`
- Create: `Core/Src/eth.c`

**Interfaces:**
- Produces:
  - `uint8_t dm9000_reg_read(uint16_t reg)` — 读 DM9000 寄存器
  - `void dm9000_reg_write(uint16_t reg, uint8_t val)` — 写 DM9000 寄存器
  - `int dm9000_init(uint8_t *mac_addr)` — 初始化 DM9000，读取 MAC 地址，返回 0=成功

- [ ] **Step 1: 创建 `Core/Inc/eth.h` — 驱动层接口声明**

```c
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
```

- [ ] **Step 2: 创建 `Core/Src/eth.c` — DM9000 寄存器定义 + 全局变量**

```c
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
```

- [ ] **Step 3: 实现 DM9000 寄存器读写函数**

```c
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
```

- [ ] **Step 4: 实现 DM9000 硬件初始化**

```c
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
```

- [ ] **Step 5: Commit**

```bash
git add Core/Inc/eth.h Core/Src/eth.c
git commit -m "feat: add DM9000 driver (FSMC register R/W, init, VID/PID check)"
```

---

### Task 3: DM9000 驱动 — 帧收发

**Files:**
- Modify: `Core/Src/eth.c` — 追加 `dm9000_recv_packet()` / `dm9000_send_packet()`

**Interfaces:**
- Produces:
  - `int dm9000_recv_packet(uint8_t *buf, u16 *len)` — 读取一帧，返回 -1=无数据, 0=成功
  - `int dm9000_send_packet(uint8_t *buf, u16 len)` — 发送一帧，返回 0=成功, -1=超时

- [ ] **Step 1: 追加 `dm9000_recv_packet()` 到 `eth.c`**

```c
/**
 * @brief  从 DM9000 接收 FIFO 读出一帧以太网数据
 * @param  buf: 输出缓冲区
 * @param  len: [out] 实际帧长度
 * @retval 0=成功, -1=无数据或错误
 */
int dm9000_recv_packet(uint8_t *buf, u16 *len)
{
    u8  rx_ready;
    u16 status, pkt_len;
    u16 i;

    /* 读取 ISR，清除 PRS 位 */
    dm9000_reg_write(DM9000_ISR, 0x01);  /* 写1清零 */
    rx_ready = dm9000_reg_read(DM9000_ISR);
    if (!(rx_ready & DM9000_ISR_PRS)) {
        return -1;  /* 无数据 */
    }

    /* 读取数据 (先读 MRCMD 中的 dummy byte) */
    DM9000_INDEX_ADDR = DM9000_MRCMD;
    (void)DM9000_DATA_ADDR;  /* dummy read */

    /* 读取状态和长度 (DM9000 在每帧前插入 4 字节头部) */
    status   = DM9000_DATA_ADDR;
    pkt_len  = DM9000_DATA_ADDR;

    /* 检查接收状态 */
    if (!(status & 0x3F00)) {
        /* 错误帧: 丢弃 */
        return -1;
    }

    if (pkt_len > 1536 || pkt_len < 64) {
        return -1;  /* 非法长度 */
    }

    *len = (u16)(pkt_len);

    /* 读取帧数据 (16-bit 读取) */
    for (i = 0; i < pkt_len; i += 2) {
        u16 word = DM9000_DATA_ADDR;
        buf[i]     = (u8)(word & 0xFF);
        if (i + 1 < pkt_len) {
            buf[i + 1] = (u8)((word >> 8) & 0xFF);
        }
    }

    return 0;
}
```

- [ ] **Step 2: 追加 `dm9000_send_packet()` 到 `eth.c`**

```c
/**
 * @brief  向 DM9000 发送 FIFO 写入一帧并触发发送
 * @param  buf: 数据缓冲区
 * @param  len: 帧长度
 * @retval 0=成功, -1=超时
 */
int dm9000_send_packet(uint8_t *buf, u16 len)
{
    u16 i;
    u32 timeout;

    /* 等待上次发送完成 */
    timeout = 100000;
    while (!(dm9000_reg_read(DM9000_TCR) & 0x01)) {
        if (--timeout == 0) return -1;
    }

    /* 写入发送长度 */
    dm9000_reg_write(DM9000_TXPLL, (u8)(len & 0xFF));
    dm9000_reg_write(DM9000_TXPLH, (u8)((len >> 8) & 0xFF));

    /* 写入发送数据 */
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

    /* 等待发送完成 */
    timeout = 500000;
    while (!(dm9000_reg_read(DM9000_NSR) & 0x0C)) {
        if (--timeout == 0) return -1;
    }

    /* 清除发送完成标志 */
    dm9000_reg_write(DM9000_NSR, 0x0C);

    return 0;
}
```

- [ ] **Step 3: Commit**

```bash
git add Core/Src/eth.c
git commit -m "feat: add DM9000 packet RX/TX functions"
```

---

### Task 4: LwIP 移植对接层 — `ethernetif.c`

**Files:**
- Create: `Core/Src/ethernetif.c`

**Interfaces:**
- Consumes: `dm9000_hw_init()`, `dm9000_recv_packet()`, `dm9000_send_packet()` from Task 2/3
- Produces:
  - `err_t ethernetif_init(struct netif *netif)` — 供 LwIP `netif_add()` 使用的 init 回调
  - 内部函数: `low_level_init()`, `low_level_output()`, `low_level_input()`, `ethernetif_input()`

- [ ] **Step 1: 创建 `Core/Src/ethernetif.c`**

```c
/**
 ****************************************************************************************************
 * @file        ethernetif.c
 * @brief       LwIP 与 DM9000 的移植对接层
 * @note        实现 LwIP netif 接口: init / output / input
 ****************************************************************************************************
 */
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "eth.h"

/* 由 eth.c 中的 dm9000_hw_init 获取的 MAC 地址，存在外部 */
extern u8 g_mac_addr[6];

/* ---- 内部辅助 ---- */
static struct pbuf *low_level_input(struct netif *netif);

/**
 * @brief  底层初始化 — 调用 DM9000 硬件初始化，设置 MAC
 */
static void low_level_init(struct netif *netif)
{
    /* 从 g_mac_addr 获取 MAC (dm9000_hw_init 已读取) */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    netif->hwaddr[0] = g_mac_addr[0];
    netif->hwaddr[1] = g_mac_addr[1];
    netif->hwaddr[2] = g_mac_addr[2];
    netif->hwaddr[3] = g_mac_addr[3];
    netif->hwaddr[4] = g_mac_addr[4];
    netif->hwaddr[5] = g_mac_addr[5];

    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    printf("[ETH] netif init OK, MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           g_mac_addr[0], g_mac_addr[1], g_mac_addr[2],
           g_mac_addr[3], g_mac_addr[4], g_mac_addr[5]);
}

/**
 * @brief  LwIP → DM9000 发包
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;
    u8  *tx_buf;
    u16  offset = 0;
    extern u8 dm9000_tx_buf[1536];  /* eth.c 中定义 */

    (void)netif;
    tx_buf = dm9000_tx_buf;

    /* 将 pbuf 链复制到连续缓冲区 */
    for (q = p; q != NULL; q = q->next) {
        MEMCPY(&tx_buf[offset], q->payload, q->len);
        offset += q->len;
    }

    if (dm9000_send_packet(tx_buf, offset) != 0) {
        return ERR_IF;
    }

#if LINK_STATS_ENABLED
    LINK_STATS_INC(link.xmit);
#endif
    return ERR_OK;
}

/**
 * @brief  DM9000 → LwIP 收包 (内部)
 */
static struct pbuf *low_level_input(struct netif *netif)
{
    struct pbuf *p = NULL;
    u16 len = 0;
    extern u8 dm9000_tx_buf[1536];

    if (dm9000_recv_packet(dm9000_tx_buf, &len) != 0) {
        return NULL;
    }

    if (len == 0) return NULL;

    /* 分配到 pbuf */
    p = pbuf_alloc(PBUF_RAW, (u16_t)len, PBUF_POOL);
    if (p != NULL) {
        pbuf_take(p, dm9000_tx_buf, len);
    }

#if LINK_STATS_ENABLED
    if (p != NULL) LINK_STATS_INC(link.recv);
    else          LINK_STATS_INC(link.memerr);
#endif
    (void)netif;
    return p;
}

/**
 * @brief  主循环调用的收包入口 — 从 DM9000 收包投喂给 LwIP
 */
void ethernetif_input(struct netif *netif)
{
    struct pbuf *p;
    struct eth_hdr *ethhdr;

    /* 轮询收包 */
    while (1) {
        p = low_level_input(netif);
        if (p == NULL) break;

        ethhdr = (struct eth_hdr *)p->payload;

        switch (htons(ethhdr->type)) {
        case ETHTYPE_IP:
        case ETHTYPE_ARP:
            if (netif->input(p, netif) != ERR_OK) {
                pbuf_free(p);
            }
            break;
        default:
            pbuf_free(p);
            break;
        }
    }
}

/**
 * @brief  LwIP netif_add 回调 — 注册 init 函数
 */
err_t ethernetif_init(struct netif *netif)
{
    netif->name[0] = 'e';
    netif->name[1] = 't';

    netif->output     = etharp_output;
    netif->linkoutput = low_level_output;

    low_level_init(netif);

    return ERR_OK;
}
```

- [ ] **Step 2: Commit**

```bash
git add Core/Src/ethernetif.c
git commit -m "feat: add LwIP ethernetif移植层 (DM9000 ↔ LwIP)"
```

---

### Task 5: TCP Server — 连接管理 + 命令接收 + 逐字节发送

**Files:**
- Modify: `Core/Src/eth.c` — 追加 TCP Server 实现
- Modify: `Core/Inc/eth.h` — 无需修改（接口已在 Task 2 声明）

**Interfaces:**
- Consumes: `g_eth_ip[]`, `g_eth_mask[]`, `g_eth_gw[]`, `g_eth_port` from Task 2
- Produces: `eth_init()`, `MX_LWIP_Process()`, `eth_tcp_send_byte()`

- [ ] **Step 1: 追加 TCP 回调函数到 `eth.c`**

```c
/* ================================================================ */
/* TCP Server 回调                                                   */
/* ================================================================ */

/**
 * @brief  TCP 接收回调 — LwIP 收到数据时调用
 */
static err_t tcp_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    u16 total_len;
    (void)arg;
    (void)err;

    if (p == NULL) {
        /* 客户端关闭连接 */
        g_tcp_client_connected = 0;
        g_client_pcb = NULL;
        printf("[ETH] 客户端断开\r\n");
        return ERR_OK;
    }

    /* 将数据拷贝到接收缓冲区 */
    total_len = 0;
    while (p != NULL && total_len < ETH_RX_BUF_SIZE - 1) {
        u16 copy_len = p->len;
        if (total_len + copy_len > ETH_RX_BUF_SIZE - 1) {
            copy_len = ETH_RX_BUF_SIZE - 1 - total_len;
        }
        MEMCPY(&g_eth_rx_buf[total_len], p->payload, copy_len);
        total_len += copy_len;
        p = p->next;
    }
    g_eth_rx_buf[total_len] = '\0';
    g_eth_rx_len = total_len;
    g_eth_rx_ready = 1;

    /* 通知 LwIP 已接收 */
    tcp_recved(tpcb, total_len);

    pbuf_free(p);
    return ERR_OK;
}

/**
 * @brief  TCP 错误回调 — 连接异常断开
 */
static void tcp_err_cb(void *arg, err_t err)
{
    (void)arg;
    (void)err;
    g_tcp_client_connected = 0;
    g_client_pcb = NULL;
    printf("[ETH] TCP 错误: %d\r\n", err);
}

/**
 * @brief  TCP 发送完成回调
 */
static err_t tcp_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    (void)arg;
    (void)tpcb;
    (void)len;
    return ERR_OK;
}

/**
 * @brief  TCP 接受连接回调 — 单客户端，拒绝多连接
 */
static err_t tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg;
    (void)err;

    if (g_tcp_client_connected) {
        /* 已有客户端，拒绝新连接 */
        printf("[ETH] 拒绝新客户端 (已有连接)\r\n");
        tcp_abort(newpcb);
        return ERR_ABRT;
    }

    g_client_pcb = newpcb;
    g_tcp_client_connected = 1;
    g_cmd_src = CMD_SRC_UART;  /* 确保标志复位 */

    /* 注册回调 */
    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, tcp_recv_cb);
    tcp_err(newpcb, tcp_err_cb);
    tcp_sent(newpcb, tcp_sent_cb);

    printf("[ETH] 客户端已连接\r\n");
    return ERR_OK;
}
```

- [ ] **Step 2: 实现 `eth_tcp_send_byte()`**

```c
/**
 * @brief  发送单字节到 TCP 客户端 (供 fputc 路由调用)
 * @note   非阻塞：发送失败/无客户端 静默返回
 *         LwIP 内部会对 tcp_write() 的数据进行缓冲，
 *         由 MX_LWIP_Process() 中的 tcp_output() 周期发送。
 */
void eth_tcp_send_byte(uint8_t ch)
{
    err_t werr;

    if (!g_tcp_client_connected || g_client_pcb == NULL) {
        return;
    }

    /* 仅当发送缓冲区有空间时才写入 */
    if (tcp_sndbuf(g_client_pcb) > 0) {
        werr = tcp_write(g_client_pcb, &ch, 1, TCP_WRITE_FLAG_COPY);
        if (werr != ERR_OK) {
            /* 失败静默丢弃，不阻塞 */
        }
    }
}
```

- [ ] **Step 3: 实现 `MX_LWIP_Process()`**

```c
/**
 * @brief  LwIP 协议栈周期处理 (主循环中调用)
 * @note   轮询 DM9000 收包 → 投喂 LwIP → 处理 TCP 定时器 → tcp_output 发送
 */
void MX_LWIP_Process(void)
{
    /* 1. 收包: 从 DM9000 读取并投喂 LwIP */
    ethernetif_input(&g_netif);

    /* 2. LwIP 定时器处理 (ARP 超时、TCP 重传等) */
    sys_check_timeouts();

    /* 3. 如果有客户端，触发 TCP 发送 (将缓冲数据发出) */
    if (g_tcp_client_connected && g_client_pcb != NULL) {
        tcp_output(g_client_pcb);
    }
}
```

- [ ] **Step 4: 实现 `eth_init()` — 完整初始化流程**

```c
/**
 * @brief  以太网完整初始化: DM9000 → LwIP → TCP Server
 * @retval 0=成功, -1=DM9000 失败, -2=LwIP 失败
 */
int eth_init(void)
{
    /* 0. FSMC 已在 MX_GPIO_Init 中配置，无需重复 */
    /* 为安全，DM9000 RST 脚保持高 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

    /* 1. DM9000 硬件初始化 */
    if (dm9000_hw_init(g_mac_addr) != 0) {
        return -1;
    }

    /* 2. LwIP 初始化 */
    lwip_init();
    printf("[ETH] LwIP 初始化完成\r\n");

    /* 3. 配置 IP 地址 */
    IP4_ADDR(&g_ipaddr,
             g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);
    IP4_ADDR(&g_netmask,
             g_eth_mask[0], g_eth_mask[1], g_eth_mask[2], g_eth_mask[3]);
    IP4_ADDR(&g_gw,
             g_eth_gw[0], g_eth_gw[1], g_eth_gw[2], g_eth_gw[3]);

    /* 4. 添加网络接口 */
    if (netif_add(&g_netif, &g_ipaddr, &g_netmask, &g_gw,
                  NULL, ethernetif_init, netif_input) == NULL) {
        printf("[ETH] netif_add 失败!\r\n");
        return -2;
    }

    netif_set_default(&g_netif);
    netif_set_up(&g_netif);

    printf("[ETH] 网络接口已建立: %d.%d.%d.%d\r\n",
           g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);

    /* 5. 创建 TCP Server */
    g_tcp_pcb = tcp_new();
    if (g_tcp_pcb == NULL) {
        printf("[ETH] TCP PCB 创建失败!\r\n");
        return -2;
    }

    if (tcp_bind(g_tcp_pcb, IP_ADDR_ANY, g_eth_port) != ERR_OK) {
        printf("[ETH] TCP bind 端口 %d 失败!\r\n", g_eth_port);
        return -2;
    }

    g_tcp_pcb = tcp_listen(g_tcp_pcb);
    tcp_accept(g_tcp_pcb, tcp_accept_cb);

    printf("[ETH] TCP Server 监听端口 %d\r\n", g_eth_port);

    return 0;
}
```

- [ ] **Step 5: Commit**

```bash
git add Core/Src/eth.c
git commit -m "feat: add TCP Server (single client, recv/send/accept)"
```

---

### Task 6: `fputc()` 输出路由改造

**Files:**
- Modify: `Core/Src/usart.c` — `fputc()` 增加 TCP 路由

**Interfaces:**
- Consumes: `g_cmd_src`, `g_tcp_client_connected`, `eth_tcp_send_byte()` from eth.h

- [ ] **Step 1: 修改 `Core/Src/usart.c` — 头文件增加 eth.h 引用 + 修改 `fputc()`**

在文件顶部 `#include` 区增加一行：
```c
#include "eth.h"
```

修改 `fputc()` 函数（当前位于 usart.c 第 20-32 行）：

将现有代码：
```c
int fputc(int ch, FILE *f)
{
    uint32_t timeout = 500000;
    while (!(UART5->SR & USART_SR_TC))
    {
        if (--timeout == 0)
        {
            break;
        }
    }
    UART5->DR = (uint8_t)ch;
    return ch;
}
```

替换为：
```c
int fputc(int ch, FILE *f)
{
    uint32_t timeout = 500000;
    while (!(UART5->SR & USART_SR_TC))
    {
        if (--timeout == 0)
        {
            break;
        }
    }
    UART5->DR = (uint8_t)ch;

    /* 如果当前命令来源是 TCP，同步发送到 TCP 客户端 */
    if (g_cmd_src == CMD_SRC_TCP && g_tcp_client_connected)
    {
        eth_tcp_send_byte((uint8_t)ch);
    }
    return ch;
}
```

- [ ] **Step 2: 编译验证** — 确认 `usart.c` 编译通过，无未定义引用

- [ ] **Step 3: Commit**

```bash
git add Core/Src/usart.c
git commit -m "feat: add TCP output routing to fputc()"
```

---

### Task 7: 网络参数 Flash 存储

**Files:**
- Modify: `Core/Inc/flash_param.h` — 增加 4 个 offset 定义 + extern 声明
- Modify: `Core/Src/flash_param.c` — `param_load_all()` 和 `param_save_all()` 扩展

**Interfaces:**
- Consumes: Flash Sector 7 base `0x08060000`
- Produces: `g_eth_ip`, `g_eth_mask`, `g_eth_gw`, `g_eth_port` 持久化 (extern 供 cmd_handler / eth.c 使用)

- [ ] **Step 1: 修改 `Core/Inc/flash_param.h`**

在 `PARAM_OFFSET_PASSHOLE_CNT` (0x48) 之后追加：
```c
#define PARAM_OFFSET_ETH_IP         0x4C    /* 本机IP (4 bytes packed) */
#define PARAM_OFFSET_ETH_MASK       0x50    /* 子网掩码 (4 bytes packed) */
#define PARAM_OFFSET_ETH_GW         0x54    /* 默认网关 (4 bytes packed) */
#define PARAM_OFFSET_ETH_PORT       0x58    /* TCP 监听端口 */
```

在 `extern` 声明区追加：
```c
extern u8  g_eth_ip[4];
extern u8  g_eth_mask[4];
extern u8  g_eth_gw[4];
extern u16 g_eth_port;
```

- [ ] **Step 2: 修改 `Core/Src/flash_param.c` — `param_load_all()`**

在函数末尾 `writeFlashData = PARAM_READ(PARAM_OFFSET_UPDATE);` 之后追加：
```c
    /* ---- 网络参数 ---- */
    /* IP 地址 */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_IP);
        if (raw != 0xFFFFFFFF && raw != 0) {  /* 有效值，非擦除态 */
            g_eth_ip[0] = (u8)((raw >> 0)  & 0xFF);
            g_eth_ip[1] = (u8)((raw >> 8)  & 0xFF);
            g_eth_ip[2] = (u8)((raw >> 16) & 0xFF);
            g_eth_ip[3] = (u8)((raw >> 24) & 0xFF);
        }
    }
    printf("IP: %d.%d.%d.%d\r\n",
           g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);

    /* 子网掩码 */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_MASK);
        if (raw != 0xFFFFFFFF && raw != 0) {
            g_eth_mask[0] = (u8)((raw >> 0)  & 0xFF);
            g_eth_mask[1] = (u8)((raw >> 8)  & 0xFF);
            g_eth_mask[2] = (u8)((raw >> 16) & 0xFF);
            g_eth_mask[3] = (u8)((raw >> 24) & 0xFF);
        }
    }
    printf("MASK: %d.%d.%d.%d\r\n",
           g_eth_mask[0], g_eth_mask[1], g_eth_mask[2], g_eth_mask[3]);

    /* 网关 */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_GW);
        if (raw != 0xFFFFFFFF && raw != 0) {
            g_eth_gw[0] = (u8)((raw >> 0)  & 0xFF);
            g_eth_gw[1] = (u8)((raw >> 8)  & 0xFF);
            g_eth_gw[2] = (u8)((raw >> 16) & 0xFF);
            g_eth_gw[3] = (u8)((raw >> 24) & 0xFF);
        }
    }
    printf("GW: %d.%d.%d.%d\r\n",
           g_eth_gw[0], g_eth_gw[1], g_eth_gw[2], g_eth_gw[3]);

    /* TCP 端口 */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_PORT);
        if (raw > 0 && raw <= 65535) {
            g_eth_port = (u16)raw;
        }
    }
    printf("PORT: %d\r\n", g_eth_port);
```

- [ ] **Step 3: 修改 `Core/Src/flash_param.c` — `param_save_all()`**

在 `HAL_FLASH_Program(..., PARAM_OFFSET_PASSHOLE_CNT, ...)` 之后、`HAL_FLASH_Lock()` 之前追加：
```c
    /* 保存网络参数 */
    {
        uint32_t ip_packed = ((uint32_t)g_eth_ip[3] << 24) |
                             ((uint32_t)g_eth_ip[2] << 16) |
                             ((uint32_t)g_eth_ip[1] << 8)  |
                             ((uint32_t)g_eth_ip[0]);
        uint32_t mask_packed = ((uint32_t)g_eth_mask[3] << 24) |
                               ((uint32_t)g_eth_mask[2] << 16) |
                               ((uint32_t)g_eth_mask[1] << 8)  |
                               ((uint32_t)g_eth_mask[0]);
        uint32_t gw_packed = ((uint32_t)g_eth_gw[3] << 24) |
                             ((uint32_t)g_eth_gw[2] << 16) |
                             ((uint32_t)g_eth_gw[1] << 8)  |
                             ((uint32_t)g_eth_gw[0]);

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_IP,   ip_packed);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_MASK, mask_packed);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_GW,   gw_packed);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_PORT, (uint32_t)g_eth_port);
    }
```

- [ ] **Step 4: 修改 `Core/Src/flash_param.c` — 顶部追加 extern**

在文件顶部的 extern 声明区追加：
```c
extern u8  g_eth_ip[4];
extern u8  g_eth_mask[4];
extern u8  g_eth_gw[4];
extern u16 g_eth_port;
```

- [ ] **Step 5: 编译验证** — 确保 `flash_param.c` 编译通过

- [ ] **Step 6: Commit**

```bash
git add Core/Inc/flash_param.h Core/Src/flash_param.c
git commit -m "feat: add network params (IP/MASK/GW/PORT) to Flash storage"
```

---

### Task 8: 网络参数命令 + PARA 扩展

**Files:**
- Modify: `Core/Inc/cmd_handler.h` — 新增 4 个声明
- Modify: `Core/Src/cmd_handler.c` — 新增 4 个处理函数 + dispatch 分支 + PARA 输出

**Interfaces:**
- Consumes: `g_eth_ip`, `g_eth_mask`, `g_eth_gw`, `g_eth_port` from eth.c
- Produces: `cmd_set_ip()`, `cmd_set_mask()`, `cmd_set_gw()`, `cmd_set_port()`

- [ ] **Step 1: 修改 `Core/Inc/cmd_handler.h`**

在现有声明区末尾（`cmd_dispatch` 声明之前）追加：
```c
void cmd_set_ip(u8 *buf, int len);        /* IP: 设置本机IP地址 */
void cmd_set_mask(u8 *buf, int len);      /* MASK: 设置子网掩码 */
void cmd_set_gw(u8 *buf, int len);        /* GW: 设置默认网关 */
void cmd_set_port(u8 *buf, int len);      /* PORT: 设置TCP监听端口 */
```

- [ ] **Step 2: 修改 `Core/Src/cmd_handler.c` — 顶部追加 extern 声明**

在现有 extern 声明区追加：
```c
extern u8  g_eth_ip[4];
extern u8  g_eth_mask[4];
extern u8  g_eth_gw[4];
extern u16 g_eth_port;
```

- [ ] **Step 3: 新增 4 个处理函数**

在 `cmd_handler.c` 末尾（`cmd_dispatch` 之前）追加：
```c
/**
 * @brief  IP: 设置本机IP地址
 *         格式: IP:192.168.1.100
 */
void cmd_set_ip(u8 *buf, int len)
{
    int a = 192, b = 168, c = 1, d = 100;
    (void)len;
    sscanf((char *)buf, "IP:%d.%d.%d.%d", &a, &b, &c, &d);
    if (a < 0 || a > 255) a = 192;
    if (b < 0 || b > 255) b = 168;
    if (c < 0 || c > 255) c = 1;
    if (d < 0 || d > 255) d = 100;
    g_eth_ip[0] = (u8)a;
    g_eth_ip[1] = (u8)b;
    g_eth_ip[2] = (u8)c;
    g_eth_ip[3] = (u8)d;
    printf("IP 已设置为: %d.%d.%d.%d (需发送RES复位生效)\r\n", a, b, c, d);
    param_save_all();
}

/**
 * @brief  MASK: 设置子网掩码
 *         格式: MASK:255.255.255.0
 */
void cmd_set_mask(u8 *buf, int len)
{
    int a = 255, b = 255, c = 255, d = 0;
    (void)len;
    sscanf((char *)buf, "MASK:%d.%d.%d.%d", &a, &b, &c, &d);
    g_eth_mask[0] = (u8)a;
    g_eth_mask[1] = (u8)b;
    g_eth_mask[2] = (u8)c;
    g_eth_mask[3] = (u8)d;
    printf("MASK 已设置为: %d.%d.%d.%d (需发送RES复位生效)\r\n", a, b, c, d);
    param_save_all();
}

/**
 * @brief  GW: 设置默认网关
 *         格式: GW:192.168.1.1
 */
void cmd_set_gw(u8 *buf, int len)
{
    int a = 192, b = 168, c = 1, d = 1;
    (void)len;
    sscanf((char *)buf, "GW:%d.%d.%d.%d", &a, &b, &c, &d);
    g_eth_gw[0] = (u8)a;
    g_eth_gw[1] = (u8)b;
    g_eth_gw[2] = (u8)c;
    g_eth_gw[3] = (u8)d;
    printf("GW 已设置为: %d.%d.%d.%d (需发送RES复位生效)\r\n", a, b, c, d);
    param_save_all();
}

/**
 * @brief  PORT: 设置TCP监听端口
 *         格式: PORT:5000
 */
void cmd_set_port(u8 *buf, int len)
{
    int port = 5000;
    (void)len;
    sscanf((char *)buf, "PORT:%d", &port);
    if (port < 1 || port > 65535) port = 5000;
    g_eth_port = (u16)port;
    printf("PORT 已设置为: %d (需发送RES复位生效)\r\n", port);
    param_save_all();
}
```

- [ ] **Step 4: 修改 `PARA` 输出 — `cmd_show_params()`**

在 `cmd_show_params()` 函数末尾 `printf("====================\r\n");` 之前追加：
```c
    printf("--- 网络配置 ---\r\n");
    printf("IP: %d.%d.%d.%d\r\n",
           g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);
    printf("MASK: %d.%d.%d.%d\r\n",
           g_eth_mask[0], g_eth_mask[1], g_eth_mask[2], g_eth_mask[3]);
    printf("GW: %d.%d.%d.%d\r\n",
           g_eth_gw[0], g_eth_gw[1], g_eth_gw[2], g_eth_gw[3]);
    printf("PORT: %d\r\n", g_eth_port);
```

- [ ] **Step 5: 修改 `cmd_dispatch()` — 增加 4 个解析分支**

在 `cmd_dispatch()` 函数中，在其他命令分支附近（如 `PARA` 分支之后）追加：
```c
    /* 网络参数设置 (IP/MASK/GW/PORT) */
    if (buf[0] == 'I' && buf[1] == 'P' && buf[2] == ':')
    {
        cmd_set_ip(buf, len);
        return 1;
    }
    if (buf[0] == 'M' && buf[1] == 'A' && buf[2] == 'S' && buf[3] == 'K' && buf[4] == ':')
    {
        cmd_set_mask(buf, len);
        return 1;
    }
    if (buf[0] == 'G' && buf[1] == 'W' && buf[2] == ':')
    {
        cmd_set_gw(buf, len);
        return 1;
    }
    if (buf[0] == 'P' && buf[1] == 'O' && buf[2] == 'R' && buf[3] == 'T' && buf[4] == ':')
    {
        cmd_set_port(buf, len);
        return 1;
    }
```

- [ ] **Step 6: 编译验证**

- [ ] **Step 7: Commit**

```bash
git add Core/Inc/cmd_handler.h Core/Src/cmd_handler.c
git commit -m "feat: add IP/MASK/GW/PORT commands and PARA network display"
```

---

### Task 9: `main.c` 集成

**Files:**
- Modify: `Core/Src/main.c` — 初始化 + 主循环

**Interfaces:**
- Consumes: `eth_init()`, `MX_LWIP_Process()`, `g_eth_rx_ready`, `g_eth_rx_buf`, `g_eth_rx_len`, `g_cmd_src` from eth.h

- [ ] **Step 1: 修改 `Core/Src/main.c` — 增加 include**

在现有 `#include` 区追加：
```c
#include "eth.h"
```

- [ ] **Step 2: 在 `main()` 初始化区增加 `eth_init()` 调用**

在 `uart_init(115200);` 之后、`__enable_irq();` 之前插入：
```c
    /* 以太网初始化 (DM9000 + LwIP + TCP Server) */
    /* 放在 uart_init 之后: printf 可用, 可输出初始化日志 */
    /* 放在 __enable_irq 之前: 避免初始化未完成时中断干扰 */
    eth_init();
```

- [ ] **Step 3: 在主循环 `while(1)` 中增加两段**

在 `while(1)` 循环体内，`printf_flag` 处理块之前追加：

```c
        /* ---- 以太网处理 ---- */
        /* 1. LwIP 协议栈周期处理 (收包、定时器、发送) */
        MX_LWIP_Process();

        /* 2. TCP 接收处理 (镜像 UART5 的处理逻辑) */
        if (g_eth_rx_ready)
        {
            u16 eth_len = g_eth_rx_len;
            g_eth_rx_ready = 0;
            g_eth_rx_len = 0;
            g_cmd_src = CMD_SRC_TCP;
            printf("ETH_RX: %s\r\n", g_eth_rx_buf);
            cmd_dispatch(g_eth_rx_buf, eth_len);
            g_cmd_src = CMD_SRC_UART;
            /* 清理接收缓冲区 */
            {
                u16 i;
                for (i = 0; i < ETH_RX_BUF_SIZE; i++) g_eth_rx_buf[i] = 0;
            }
        }
```

- [ ] **Step 4: 编译验证** — 确认 `main.c` 编译通过

- [ ] **Step 5: Commit**

```bash
git add Core/Src/main.c
git commit -m "feat: integrate Ethernet (DM9000+LwIP+TCP) into main loop"
```

---

### Task 10: 编译 + 烧录 + 功能验证

**说明:** 本任务为手动验证，非代码任务。按照以下步骤逐项验证。

- [ ] **Step 1: 完整编译**

在 Keil MDK 中 Rebuild All，确保 0 Error, 0 Warning。

- [ ] **Step 2: 烧录 + 上电检查**

烧录后连接串口终端（115200），观察上电输出应包含：
```
[ETH] DM9000 识别成功 VID=0x0A46 PID=0x9000
[ETH] MAC: XX:XX:XX:XX:XX:XX
[ETH] LwIP 初始化完成
[ETH] 网络接口已建立: 192.168.1.100
[ETH] TCP Server 监听端口 5000
```

- [ ] **Step 3: ping 测试**

PC 端：`ping 192.168.1.100`（确保 PC 与设备在同一子网）
期望：有 ICMP 响应

- [ ] **Step 4: TCP 连接 + PARA 命令**

PC 端用 `nc`（或 Putty Raw 模式）连接 `192.168.1.100:5000`：
```bash
nc 192.168.1.100 5000
```
发送 `PARA`，期望返回与串口 `PARA` 相同的参数列表（含网络配置段）。
串口终端同步显示 `ETH_RX: PARA`。

- [ ] **Step 5: 串口命令验证**

串口发送 `PARA` / `HAL` / `L1` 等命令，确认行为与之前完全一致。

- [ ] **Step 6: 网络参数持久化**

串口发送 `IP:192.168.1.200` → 返回含"需发送RES复位生效" → 发送 `RES` → 重启后发送 `PARA`，确认 IP 变为 `192.168.1.200`。

- [ ] **Step 7: 断连重连**

TCP 客户端断开连接 → 观察串口输出 `[ETH] 客户端断开` → 重新 `nc` 连接 → 再次发 `PARA`，确认正常。

- [ ] **Step 8: 网线未插**

拔掉网线 → 串口发 `PARA` 正常 → TCP 连接不上 → 插回网线 → 重新 `nc` 连接恢复。

- [ ] **Step 9: Commit (如有调整)**

```bash
git add -A
git commit -m "feat: verify Ethernet command feature — all tests pass"
```
