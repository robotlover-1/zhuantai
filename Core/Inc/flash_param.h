#ifndef __FLASH_PARAM_H
#define __FLASH_PARAM_H

#include "sys.h"

/* Flash Sector: Sector 7 (0x08060000, 128KB) */
#define PARAM_SECTOR                FLASH_SECTOR_7
#define PARAM_SECTOR_BASE           0x08060000U

/* F1 参数偏移 (每个4字节) */
#define PARAM_OFFSET_SET_SPEED      0x00    /* g_set_speed */
#define PARAM_OFFSET_STEP_ACCEL     0x04    /* g_step_accel */
#define PARAM_OFFSET_STEP_DECEL     0x08    /* g_step_decel */
#define PARAM_OFFSET_STEP1          0x0C    /* g_step1 */
#define PARAM_OFFSET_STEP2          0x10    /* g_step2 */
#define PARAM_OFFSET_STEP_PASS      0x14    /* step_pass */
#define PARAM_OFFSET_SPEED_RCC      0x18    /* speed_rcc (原始值, 转/s) */
#define PARAM_OFFSET_PULSE_CUT      0x1C    /* pulse_cut */
#define PARAM_OFFSET_PULSE_COMP     0x20    /* pulse_compensate */
#define PARAM_OFFSET_TIME_DELAY     0x24    /* time_delay_compensate */
#define PARAM_OFFSET_LOOP1          0x28    /* loop1 */
#define PARAM_OFFSET_LOOP2          0x2C    /* loop2 */
#define PARAM_OFFSET_CYCLE_CNT      0x30    /* total_cycles */
#define PARAM_OFFSET_UPDATE         0x34    /* 参数更新标志 */
#define PARAM_OFFSET_RTC_YMD        0x38    /* RTC时间备份: 年月日 */
#define PARAM_OFFSET_RTC_HMS        0x3C    /* RTC时间备份: 时分秒 */
#define PARAM_OFFSET_NO_COMP        0x40    /* 补偿开关参数偏移 (B开关已移除, 宏保留未用) */
#define PARAM_OFFSET_PL_T           0x44    /* PL_T PL疲劳测试大循环 */
#define PARAM_OFFSET_PL_H           0x48    /* PL_H PL疲劳测试小循环 */
#define PARAM_OFFSET_CS_T           0x4C    /* CS_T CS自动测试大循环 */
#define PARAM_OFFSET_CS_H           0x50    /* CS_H CS自动测试小循环 */
#define PARAM_OFFSET_ETH_IP         0x54    /* 本机IP (4 bytes packed) */
#define PARAM_OFFSET_ETH_MASK       0x58    /* 子网掩码 (4 bytes packed) */
#define PARAM_OFFSET_ETH_GW         0x5C    /* 默认网关 (4 bytes packed) */
#define PARAM_OFFSET_ETH_PORT       0x60    /* TCP 监听端口 */

#define PARAM_READ(offset)          (*(__IO uint32_t *)(PARAM_SECTOR_BASE + (offset)))

/* 固件升级标志 (bootloader握手地址, 对齐F1 0x0807C000)
 * 注: 落在参数区Sector7(0x08060000)内, 写1依赖cmd_res先param_save_all整扇区擦除 */
#define UPDATE_FLAG_ADDR            0x0807C000U

/* 以太网参数 (定义在 eth.c, 默认值 192.168.1.100:5000) */
extern u8  g_eth_ip[4];
extern u8  g_eth_mask[4];
extern u8  g_eth_gw[4];
extern u16 g_eth_port;

void param_load_all(void);
void param_save_all(void);
void write_update_flag(uint32_t val);   /* 固件升级标志: RES写1复位后bootloader进入升级, APP启动写0清除 */
#endif
