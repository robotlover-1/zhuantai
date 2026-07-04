/**
 ****************************************************************************************************
 * @file        lwipopts.h
 * @brief       LwIP 配置裁剪: TCP Server only, 静态 IP, 最小 RAM/Flash 占用
 ****************************************************************************************************
 */
#ifndef __LWIPOPTS_H
#define __LWIPOPTS_H

/* ---- 操作系统抽象层 (裸机) ---- */
#define NO_SYS                      1
#define LWIP_TCPIP_CORE_LOCKING     1
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1
#define SYS_LIGHTWEIGHT_PROT        0       /* bare-metal: no OS-level critical section protection */

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
