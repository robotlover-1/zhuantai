/**
 ****************************************************************************************************
 * @file        stmflash.c
 * @brief       STM32F407 Flash 读写驱动（适配自 F103 版本）
 * @note        使用 F4 HAL 库 Flash API，扇区映射适用于 STM32F407ZG 1MB Flash
 ****************************************************************************************************
 */
#include "stmflash.h"
#include "delay.h"

/******************************************************************************************/
/* F407 Flash 扇区地址映射 (1MB) */
/******************************************************************************************/
static const uint32_t g_flash_sectors[] = {
    0x08000000, 0x08004000, 0x08008000, 0x0800C000,  /* Sector 0~3: 16KB */
    0x08010000,                                         /* Sector 4:   64KB */
    0x08020000, 0x08040000, 0x08060000, 0x08080000,  /* Sector 5~8:  128KB */
    0x080A0000, 0x080C0000, 0x080E0000,               /* Sector 9~11: 128KB */
};

/**
 * @brief       根据Flash地址获取扇区编号
 * @param       addr: Flash绝对地址
 * @retval      扇区编号 0~11，非法地址返回 -1
 */
static int stmflash_get_sector(uint32_t addr)
{
    for (int i = 0; i < 11; i++)
    {
        if (addr >= g_flash_sectors[i] && addr < g_flash_sectors[i + 1])
            return i;
    }
    if (addr >= g_flash_sectors[11] && addr < (STM32_FLASH_BASE + STM32_FLASH_SIZE * 1024))
        return 11;
    return -1;
}

/**
 * @brief       擦除指定Flash扇区
 * @param       sector: 扇区编号 (0~11)
 * @retval      0=成功, 非0=失败
 */
static int __attribute__((unused)) stmflash_erase_sector(uint32_t sector)
{
    FLASH_EraseInitTypeDef f = {0};
    uint32_t SectorError = 0;

    f.TypeErase = FLASH_TYPEERASE_SECTORS;
    f.Sector = sector;
    f.NbSectors = 1;
    f.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&f, &SectorError);
    HAL_FLASH_Lock();

    return SectorError;
}

/**
 * @brief       读取指定地址的半字(16位)
 */
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16 *)faddr;
}

#if STM32_FLASH_WREN

/**
 * @brief       不检查写入（直接编程半字）
 */
void STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    u16 i;
    for (i = 0; i < NumToWrite; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]);
        WriteAddr += 2;
    }
}

/**
 * @brief       从指定地址开始写入指定长度的数据（自动处理扇区擦除）
 * @param       WriteAddr: 起始地址（必须是2字节对齐）
 * @param       pBuffer: 数据指针
 * @param       NumToWrite: 半字(16位)数量
 */
void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    int sector;

    if (WriteAddr < STM32_FLASH_BASE ||
        (WriteAddr >= (STM32_FLASH_BASE + STM32_FLASH_SIZE * 1024)))
        return;

    sector = stmflash_get_sector(WriteAddr);
    if (sector < 0)
        return;

    HAL_FLASH_Unlock();

    /* 擦除目标扇区 */
    {
        FLASH_EraseInitTypeDef f = {0};
        uint32_t SectorError = 0;
        f.TypeErase = FLASH_TYPEERASE_SECTORS;
        f.Sector = (uint32_t)sector;
        f.NbSectors = 1;
        f.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        HAL_FLASHEx_Erase(&f, &SectorError);
    }

    /* 逐半字写入 */
    {
        u16 i;
        for (i = 0; i < NumToWrite; i++)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]);
            WriteAddr += 2;
        }
    }

    HAL_FLASH_Lock();
}

#endif /* STM32_FLASH_WREN */

/**
 * @brief       从指定地址读取指定数量的半字
 */
void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead)
{
    u16 i;
    for (i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);
        ReadAddr += 2;
    }
}

/**
 * @brief       测试写入一个半字
 */
void Test_Write(u32 WriteAddr, u16 WriteData)
{
    STMFLASH_Write(WriteAddr, &WriteData, 1);
}













