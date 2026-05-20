/**
 ****************************************************************************************************
 * @file        stmflash.h
 * @brief       STM32F407 Flash 读写驱动
 ****************************************************************************************************
 */
#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "sys.h"

/******************************************************************************************/
/* 用户配置 */
/* STM32F407ZG: 1MB Flash */
#define STM32_FLASH_SIZE        1024        /* Flash大小(单位: KB) */
#define STM32_FLASH_WREN        1           /* Flash写使能(0:禁止, 1:允许) */

/* Flash起始地址 */
#define STM32_FLASH_BASE        0x08000000  /* STM32 Flash起始地址 */

/******************************************************************************************/
/* 外部接口 */
u16 STMFLASH_ReadHalfWord(u32 faddr);

void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);

void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);

void Test_Write(u32 WriteAddr, u16 WriteData);
#endif
