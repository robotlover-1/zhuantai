/**
 ****************************************************************************************************
 * @file        flash_param.c
 * @brief       F407 Flash 参数存储管理实现
 * @note        所有参数集中在 Sector 7 统一管理，避免逐一擦除互相影响
 ****************************************************************************************************
 */
#include "flash_param.h"
#include "delay.h"
#include "usart.h"

/******************************************************************************************/
/* 外部变量声明 */
/******************************************************************************************/
extern int32_t force;
extern int32_t force_F;
extern int time_c;
extern int32_t pulse_low;
extern int32_t pluse_ele;
extern int32_t writeFlashData;

/******************************************************************************************/
/* 公共实现 */
/******************************************************************************************/

/**
 * @brief       从Flash加载所有参数（含有效性校验）
 * @note        首次使用或Flash为空时会自动写入默认值
 */
void param_load_all(void)
{
    int32_t val;

    /* force - 运行最大速度 */
    val = PARAM_READ(PARAM_OFFSET_FORCE);
    printf("从Flash读取运行最大速度: %d\r\n", val);
    if (val >= 0 && val <= 1500)
        force = val;
    else
        force = 1500;
    printf("运行最大速度设置F: %d\r\n", force);
    //printf("设置F: %d\r\n", force);
    /* force_F - 最大占空比 */
    val = PARAM_READ(PARAM_OFFSET_FORCE_F);
        printf("从Flash读取最大占空比: %d\r\n", val);
    if (val >= 0 && val <= 3599)
        force_F = val;
    else
        force_F = 3500;
    printf("最大占空比设置R: %d\r\n", force_F);
    //printf("R: %d\r\n", force_F);
    /* time_c - 进孔速度 */
    val = PARAM_READ(PARAM_OFFSET_TIME_C);
    printf("从Flash读取进孔速度: %d\r\n", val);
    if (val >= 0 && val <= 1000)
        time_c = val;
    else
        time_c = 200;
    printf("进孔速度V: %d\r\n", time_c);
    //printf("V: %d\r\n", time_c);

    /* pulse_low - 进孔脉冲数 */
    val = PARAM_READ(PARAM_OFFSET_PULSE_LOW);
    printf("从Flash读取进孔脉冲数: %d\r\n", val);
    if (val >= 0 && val <= 20000)
        pulse_low = val;
    else
        pulse_low = 14000;
    printf("进孔脉冲数S: %d\r\n", pulse_low);
    //printf("S: %d\r\n", pulse_low);

    /* pluse_ele - 光电脉冲检测限制 */
    val = PARAM_READ(PARAM_OFFSET_PLUSE_ELE);
    printf("从Flash读取光电脉冲检测限制: %d\r\n", val);
    if (val >= 0 && val <= 39000)
        pluse_ele = val;
    else
        pluse_ele = 38000;
    printf("光电脉冲限制E: %d\r\n", pluse_ele);
    //printf("E: %d\r\n", pluse_ele);

    /* 更新标志 */
    writeFlashData = PARAM_READ(PARAM_OFFSET_UPDATE);
}

/**
 * @brief       一次性保存所有参数到Flash
 * @note        仅擦除Sector 7一次，然后连续写入所有参数
 *              相比原来每存一个就擦整个扇区，大幅减少擦写次数
 */
void param_save_all(void)
{
    uint32_t PageError = 0;
    printf("解锁FLASH\r\n");
    /* 解锁Flash */
    HAL_FLASH_Unlock();

    /* 擦除整个Sector 7 (128KB) */
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_SECTORS;
    f.Sector = PARAM_SECTOR;
    f.NbSectors = 1;
    f.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&f, &PageError);

    /* 连续写入所有参数 */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_FORCE, (uint32_t)force);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_FORCE_F, (uint32_t)force_F);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_TIME_C, (uint32_t)time_c);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PULSE_LOW, (uint32_t)pulse_low);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PLUSE_ELE, (uint32_t)pluse_ele);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_UPDATE, 0);

    /* 锁定Flash */
    HAL_FLASH_Lock();

    printf("所有参数已保存到Flash\r\n");
}
