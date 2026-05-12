/**
 ****************************************************************************************************
 * @file        flash_param.h
 * @brief       F407 Flash 参数存储管理
 * @note        所有参数集中在 Sector 7 (0x08060000, 128KB) 统一管理
 ****************************************************************************************************
 */
#ifndef __FLASH_PARAM_H
#define __FLASH_PARAM_H

#include "sys.h"

/******************************************************************************************/
/* Flash Sector 定义 (STM32F407ZG, 1MB Flash) */
/******************************************************************************************/

/* 参数存储区：Sector 7 (128KB) */
#define PARAM_SECTOR                FLASH_SECTOR_7
#define PARAM_SECTOR_BASE           0x08060000U

/* 参数偏移量定义 (每个参数占 4 字节 = 1 Word) */
#define PARAM_OFFSET_FORCE          0x00    /* 运行最大速度 */
#define PARAM_OFFSET_FORCE_F        0x04    /* 最大占空比 */
#define PARAM_OFFSET_TIME_C         0x08    /* 进孔速度 */
#define PARAM_OFFSET_PULSE_LOW      0x0C    /* 进孔脉冲数 */
#define PARAM_OFFSET_PLUSE_ELE      0x10    /* 光电脉冲检测限制 */
#define PARAM_OFFSET_UPDATE         0x14    /* 参数更新标志 */

/* 从Flash指定偏移读取一个32位值 */
#define PARAM_READ(offset)          (*(__IO uint32_t *)(PARAM_SECTOR_BASE + (offset)))

/******************************************************************************************/
/* 外部接口 */
/******************************************************************************************/
void param_load_all(void);          /* 从Flash加载所有参数（含有效性校验） */
void param_save_all(void);          /* 一次性保存所有参数到Flash */

#endif
