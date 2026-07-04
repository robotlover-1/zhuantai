/**
 ****************************************************************************************************
 * @file        ethernetif.c
 * @brief       LwIP 与 DM9000 的移植对接层
 * @note        实现 LwIP netif 接口: init / output / input
 ****************************************************************************************************
 */
#include <string.h>
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
