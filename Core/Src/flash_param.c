#include "flash_param.h"
#include "delay.h"
#include "usart.h"
#include <stdio.h>

extern __IO uint32_t g_set_speed;
extern __IO uint32_t g_step_accel;
extern __IO uint32_t g_step_decel;
extern __IO uint16_t g_step1;
extern __IO uint16_t g_step2;
extern int step_pass;
extern int speed_rcc;          /* 注意: 存储的是原始转/s值, 不是rcc单位 */
extern int pulse_cut;
extern int32_t pulse_compensate;
extern int time_delay_compensate;
extern int loop1;
extern int loop2;
extern int32_t total_cycles;
extern int32_t writeFlashData;

void param_load_all(void)
{
    int32_t val;

    val = PARAM_READ(PARAM_OFFSET_SET_SPEED);
    if (val >= 0 && val <= 8000) g_set_speed = val;
    else g_set_speed = 72;
    printf("g_set_speed(D:): %d\r\n", (int)g_set_speed);

    val = PARAM_READ(PARAM_OFFSET_STEP_ACCEL);
    if (val >= 0) g_step_accel = val;
    else g_step_accel = 8;
    printf("g_step_accel(J:): %d\r\n", (int)g_step_accel);

    val = PARAM_READ(PARAM_OFFSET_STEP_DECEL);
    if (val >= 0) g_step_decel = val;
    else g_step_decel = 8;
    printf("g_step_decel(L:): %d\r\n", (int)g_step_decel);

    val = PARAM_READ(PARAM_OFFSET_STEP1);
    if (val >= 0) g_step1 = val;
    else g_step1 = 300;
    printf("g_step1(M:): %d\r\n", (int)g_step1);

    val = PARAM_READ(PARAM_OFFSET_STEP2);
    if (val >= 0) g_step2 = val;
    else g_step2 = 300;
    printf("g_step2(N:): %d\r\n", (int)g_step2);

    val = PARAM_READ(PARAM_OFFSET_STEP_PASS);
    if (val >= 0) step_pass = val;
    else step_pass = 1333;
    printf("step_pass(X:): %d\r\n", step_pass);

    val = PARAM_READ(PARAM_OFFSET_SPEED_RCC);
    if (val >= 0 && val <= 200) speed_rcc = val;
    else speed_rcc = 18;
    printf("speed_rcc(Z:): %d\r\n", speed_rcc);

    val = PARAM_READ(PARAM_OFFSET_PULSE_CUT);
    if (val >= 0 && val <= 8000) pulse_cut = val;
    else pulse_cut = 7125;
    printf("pulse_cut(P:): %d\r\n", pulse_cut);

    val = PARAM_READ(PARAM_OFFSET_PULSE_COMP);
    if (val >= 0 && val <= 8000) pulse_compensate = val;
    else pulse_compensate = 1000;
    printf("pulse_compensate(C:): %d\r\n", (int)pulse_compensate);

    val = PARAM_READ(PARAM_OFFSET_TIME_DELAY);
    if (val >= 0) time_delay_compensate = val;
    else time_delay_compensate = 100;
    printf("time_delay(Q:): %d\r\n", time_delay_compensate);

    val = PARAM_READ(PARAM_OFFSET_LOOP1);
    if (val >= 0) loop1 = val; else loop1 = 0;
    printf("loop1(T:): %d\r\n", loop1);

    val = PARAM_READ(PARAM_OFFSET_LOOP2);
    if (val >= 0) loop2 = val; else loop2 = 0;
    printf("loop2(H:): %d\r\n", loop2);

    val = PARAM_READ(PARAM_OFFSET_CYCLE_CNT);
    if (val >= 0) total_cycles = val; else total_cycles = 0;
    printf("total_cycles: %d\r\n", total_cycles);

    writeFlashData = PARAM_READ(PARAM_OFFSET_UPDATE);
}

void param_save_all(void)
{
    uint32_t PageError = 0;
    FLASH_EraseInitTypeDef f;

    HAL_FLASH_Unlock();

    f.TypeErase = FLASH_TYPEERASE_SECTORS;
    f.Sector = PARAM_SECTOR;
    f.NbSectors = 1;
    f.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&f, &PageError);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_SET_SPEED,  (uint32_t)g_set_speed);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_STEP_ACCEL, (uint32_t)g_step_accel);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_STEP_DECEL, (uint32_t)g_step_decel);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_STEP1,      (uint32_t)g_step1);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_STEP2,      (uint32_t)g_step2);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_STEP_PASS,  (uint32_t)step_pass);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_SPEED_RCC,  (uint32_t)speed_rcc);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_PULSE_CUT,  (uint32_t)pulse_cut);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_PULSE_COMP, (uint32_t)pulse_compensate);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_TIME_DELAY, (uint32_t)time_delay_compensate);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_LOOP1,      (uint32_t)loop1);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_LOOP2,      (uint32_t)loop2);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_CYCLE_CNT,  (uint32_t)total_cycles);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_UPDATE,     0);

    /* RTC 时间备份 */
    {
        uint32_t _dr = RTC->DR;
        uint32_t _tr = RTC->TR;
        int _y, _mon, _d, _h, _min, _s;
#define B2D(b) ((((b) >> 4) * 10) + ((b) & 0x0F))
        _y   = 2000 + B2D((_dr >> 16) & 0xFF);
        _mon = B2D((_dr >> 8) & 0x1F);
        _d   = B2D(_dr & 0x3F);
        _h   = B2D((_tr >> 16) & 0x3F);
        _min = B2D((_tr >> 8) & 0x7F);
        _s   = B2D(_tr & 0x7F);
#undef B2D
        if (_y >= 2025 && _y <= 2099 && _mon >= 1 && _mon <= 12
            && _d >= 1 && _d <= 31 && _h <= 23 && _min <= 59 && _s <= 59)
        {
            uint32_t _ymd = ((uint32_t)_y << 16) | ((uint32_t)_mon << 8) | (uint32_t)_d;
            uint32_t _hms = ((uint32_t)_h << 16) | ((uint32_t)_min << 8) | (uint32_t)_s;
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_RTC_YMD, _ymd);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_RTC_HMS, _hms);
        }
    }

    HAL_FLASH_Lock();
    printf("Params saved\r\n");
}
