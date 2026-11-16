/**
 ****************************************************************************************************
 * @file        lwipopts.h
 * @brief       LwIP 配置裁剪: 裸机轮询 (NO_SYS) + TCP Server + 静态 IP, 最小 RAM/Flash 占用
 * @note        与 DM9000 驱动配套, 无 UDP/DHCP/DNS, 保留 ICMP (ping 可用)
 ****************************************************************************************************
 */
#ifndef __LWIPOPTS_H
#define __LWIPOPTS_H

/* ---- 平台模式 ---- */
#define NO_SYS                      1       /* 裸机轮询模式, 无 RTOS */
#define SYS_LIGHTWEIGHT_PROT        0       /* 单线程轮询, 无临界区竞争 */

/* ---- 功能裁剪 ---- */
#define LWIP_TCP                    1
#define LWIP_UDP                    0
#define LWIP_DHCP                   0
#define LWIP_DNS                    0
#define LWIP_SNMP                   0
#define LWIP_IGMP                   0
#define LWIP_ICMP                   1       /* ping 可用 */
#define LWIP_RAW                    0
#define LWIP_NETCONN                0       /* 使用 raw API, 不使用 netconn/socket */
#define LWIP_SOCKET                 0
#define LWIP_STATS                  0
#define LWIP_STATS_DISPLAY          0

/* ---- IP 版本 ---- */
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_ARP                    1

/* ---- 内存池 ---- */
#define MEM_SIZE                    (16 * 1024)     /* 堆: tcp_write(COPY) 帧数据 */
#define MEM_ALIGNMENT               4
#define MEMP_NUM_PBUF               16
#define PBUF_POOL_SIZE              8               /* 收包池, 轮询单帧处理足够 */
#define MEMP_NUM_TCP_PCB            4
#define MEMP_NUM_TCP_SEG            16              /* 发送分段, 配合 TX 缓冲分块发送 */
#define MEMP_NUM_TCPIP_MSG_API      8
#define MEMP_NUM_TCPIP_MSG_INPKT    8
#define MEMP_NUM_SYS_TIMEOUT        6

/* ---- TCP 参数 ---- */
#define TCP_MSS                     1460
#define TCP_SND_BUF                 (4 * TCP_MSS)
#define TCP_WND                     (2 * TCP_MSS)
#define TCP_TTL                     255
#define TCP_SNDLOWAT                0
#define TCP_QUEUE_OOSEQ             1

/* ---- 校验 ---- */
#define CHECKSUM_BY_HARDWARE        0       /* DM9000 无硬件校验, 软件校验 */
#define LWIP_CHECKSUM_ON_COPY       0

/* ---- 网络接口 ---- */
#define LWIP_NETIF_HOSTNAME         0
#define LWIP_NETIF_API              0
#define LWIP_NETIF_STATUS_CALLBACK  0
#define LWIP_NETIF_LINK_CALLBACK    0
#define LWIP_NETIF_REMOVE_CALLBACK  0

/* ---- 调试 (release 关闭) ---- */
#define LWIP_DEBUG                  0
#define TCP_DEBUG                   LWIP_DBG_OFF
#define TCP_INPUT_DEBUG             LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG            LWIP_DBG_OFF
#define ETHARP_DEBUG                LWIP_DBG_OFF
#define NETIF_DEBUG                 LWIP_DBG_OFF

#endif /* __LWIPOPTS_H */
