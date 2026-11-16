/**
 ****************************************************************************************************
 * @file        ethernetif.c
 * @brief       LwIP 与 DM9000 的移植对接层
 * @note        实现 netif 接口: init / linkoutput / input
 *              收包由主循环 MX_LWIP_Process() → ethernetif_input() 轮询驱动
 ****************************************************************************************************
 */
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/def.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "eth.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* eth.c 中定义 */
extern u8 g_mac_addr[6];
extern u8 dm9000_tx_buf[1536];

static struct pbuf *low_level_input(struct netif *netif);

/* ---- 底层初始化: 设置 MAC / MTU / flags ---- */
static void low_level_init(struct netif *netif)
{
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    memcpy(netif->hwaddr, g_mac_addr, 6);

    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    // printf("[ETH] netif init OK, MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
    //        g_mac_addr[0], g_mac_addr[1], g_mac_addr[2],
    //        g_mac_addr[3], g_mac_addr[4], g_mac_addr[5]);
}

/* ---- LwIP → DM9000 发包: 把 pbuf 链拷贝到连续缓冲后发出 ---- */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;
    u8  *tx_buf = dm9000_tx_buf;
    u16  offset = 0;
    (void)netif;

    for (q = p; q != NULL; q = q->next) {
        memcpy(&tx_buf[offset], q->payload, q->len);
        offset += q->len;
    }

    if (dm9000_send_packet(tx_buf, offset) != 0) {
        return ERR_IF;
    }
    return ERR_OK;
}

/* ---- DM9000 → LwIP 收包 (内部): 读一帧并转成 pbuf ---- */
static struct pbuf *low_level_input(struct netif *netif)
{
    struct pbuf *p;
    u16 len = 0;
    (void)netif;

    if (dm9000_recv_packet(dm9000_tx_buf, &len) != 0) {
        return NULL;
    }
    if (len == 0) {
        return NULL;
    }

    p = pbuf_alloc(PBUF_RAW, (u16_t)len, PBUF_POOL);
    if (p != NULL) {
        pbuf_take(p, dm9000_tx_buf, len);
    }
    return p;
}

/* ---- 主循环轮询收包入口: 从 DM9000 收包投喂给 LwIP ---- */
void ethernetif_input(struct netif *netif)
{
    struct pbuf *p;
    struct eth_hdr *ethhdr;

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

/* ---- netif_add 的 init 回调 ---- */
err_t ethernetif_init(struct netif *netif)
{
    netif->name[0] = 'e';
    netif->name[1] = 't';

    netif->output     = etharp_output;
    netif->linkoutput = low_level_output;

    low_level_init(netif);

    return ERR_OK;
}
