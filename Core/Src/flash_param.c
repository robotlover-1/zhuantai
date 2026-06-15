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
#include "pid.h"

/******************************************************************************************/
/* 外部变量声明 */
/******************************************************************************************/
extern int32_t force;
extern int32_t force_F;
extern int time_c;
extern int32_t pulse_low;
extern int32_t pulse_buf;
extern int time_buf;
extern int32_t pluse_ele;
extern int32_t writeFlashData;
extern float g_pid_kp;
extern float g_pid_ki;
extern float g_pid_kd;
extern float g_pid_skp;
extern float g_pid_ski;
extern float g_pid_skd;
extern int   g_pid_period_ms;

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
        pulse_low = 20000;
    printf("进孔脉冲数S: %d\r\n", pulse_low);
    //printf("S: %d\r\n", pulse_low);

    /* pulse_buf - 缓冲段起始脉冲数 */
    val = PARAM_READ(PARAM_OFFSET_PULSE_BUF);
    printf("从Flash读取缓冲段脉冲数: %d\r\n", val);
    if (val >= 0 && val <= 50000)
        pulse_buf = val;
    else
        pulse_buf = 30000;
    printf("缓冲段脉冲数BS: %d\r\n", pulse_buf);

    /* time_buf - 缓冲段速度 */
    val = PARAM_READ(PARAM_OFFSET_TIME_BUF);
    printf("从Flash读取缓冲段速度: %d\r\n", val);
    if (val >= 0 && val <= 1000)
        time_buf = (int)val;
    else
        time_buf = 300;
    printf("缓冲段速度BV: %d\r\n", time_buf);

    /* pluse_ele - 光电脉冲检测限制 */
    val = PARAM_READ(PARAM_OFFSET_PLUSE_ELE);
    printf("从Flash读取光电脉冲检测限制: %d\r\n", val);
    if (val >= 0 && val <= 39000)
        pluse_ele = val;
    else
        pluse_ele = 38000;
    printf("光电脉冲限制E: %d\r\n", pluse_ele);
    //printf("E: %d\r\n", pluse_ele);

    /* PID 位置环 KP (×1000) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_KP);
        if (raw >= 0 && raw <= 1000)
            g_pid_kp = raw * 0.001f;
        else
            g_pid_kp = KP;
    }
    printf("位置环KP: %.3f\r\n", g_pid_kp);

    /* PID 位置环 KI (×1000) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_KI);
        if (raw >= 0 && raw <= 1000)
            g_pid_ki = raw * 0.001f;
        else
            g_pid_ki = KI;
    }
    printf("位置环KI: %.3f\r\n", g_pid_ki);

    /* PID 位置环 KD (×1000) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_KD);
        if (raw >= 0 && raw <= 1000)
            g_pid_kd = raw * 0.001f;
        else
            g_pid_kd = KD;
    }
    printf("位置环KD: %.3f\r\n", g_pid_kd);

    /* PID 速度环 S_KP (×100) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_SKP);
        if (raw >= 0 && raw <= 5000)
            g_pid_skp = raw * 0.01f;
        else
            g_pid_skp = S_KP;
    }
    printf("速度环S_KP: %.2f\r\n", g_pid_skp);

    /* PID 速度环 S_KI (×100) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_SKI);
        if (raw >= 0 && raw <= 5000)
            g_pid_ski = raw * 0.01f;
        else
            g_pid_ski = S_KI;
    }
    printf("速度环S_KI: %.2f\r\n", g_pid_ski);

    /* PID 速度环 S_KD (×100) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_SKD);
        if (raw >= 0 && raw <= 5000)
            g_pid_skd = raw * 0.01f;
        else
            g_pid_skd = S_KD;
    }
    printf("速度环S_KD: %.2f\r\n", g_pid_skd);

    /* PID采样周期 */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_PERIOD);
        if (raw >= 5 && raw <= 100)
            g_pid_period_ms = (int)raw;
        else
            g_pid_period_ms = SMAPLSE_PID_SPEED;
    }
    printf("PID采样周期: %dms\r\n", g_pid_period_ms);

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
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PULSE_BUF, (uint32_t)pulse_buf);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_TIME_BUF, (uint32_t)time_buf);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PLUSE_ELE, (uint32_t)pluse_ele);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_UPDATE, 0);

    /* 保存PID参数（整数化存储，避免float直接写Flash） */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_KP,  (uint32_t)(g_pid_kp  * 1000.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_KI,  (uint32_t)(g_pid_ki  * 1000.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_KD,  (uint32_t)(g_pid_kd  * 1000.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_SKP, (uint32_t)(g_pid_skp * 100.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_SKI, (uint32_t)(g_pid_ski * 100.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_SKD, (uint32_t)(g_pid_skd * 100.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_PERIOD, (uint32_t)g_pid_period_ms);

    /* 锁定Flash */
    HAL_FLASH_Lock();

    printf("所有参数已保存到Flash\r\n");
}
