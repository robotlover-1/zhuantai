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
extern int speed_rcc;          /* 存储的是 delay 值 (12000/rpm), 与运动代码一致 */
extern int pulse_cut;
extern int32_t pulse_compensate;
extern int time_delay_compensate;
extern int loop1;
extern int loop2;
extern int PL_T;
extern int PL_H;
extern int CS_T;
extern int CS_H;
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
    if (val >= 60 && val <= 12000) speed_rcc = val;     /* delay 值 (12000/rpm) */
    else if (val >= 1 && val <= 200) speed_rcc = 12000 / val;  /* 兼容旧 rpm 格式 */
    else speed_rcc = 666;                                /* 默认 12000/18 rpm */
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

    val = PARAM_READ(PARAM_OFFSET_PL_T);
    if (val >= 0) PL_T = val; else PL_T = 60;
    printf("PL_T: %d\r\n", PL_T);

    val = PARAM_READ(PARAM_OFFSET_PL_H);
    if (val >= 0) PL_H = val; else PL_H = 30;
    printf("PL_H: %d\r\n", PL_H);

    val = PARAM_READ(PARAM_OFFSET_CS_T);
    if (val >= 0) CS_T = val; else CS_T = 40;
    printf("CS_T: %d\r\n", CS_T);

    val = PARAM_READ(PARAM_OFFSET_CS_H);
    if (val >= 0) CS_H = val; else CS_H = 30;
    printf("CS_H: %d\r\n", CS_H);

    val = PARAM_READ(PARAM_OFFSET_CYCLE_CNT);
    if (val >= 0) total_cycles = val; else total_cycles = 0;
    printf("total_cycles: %d\r\n", total_cycles);

    writeFlashData = PARAM_READ(PARAM_OFFSET_UPDATE);

    /* ---- 网络参数 ---- */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_IP);
        if (raw != 0xFFFFFFFF && raw != 0) {   /* 有效值, 非擦除态 */
            g_eth_ip[0] = (u8)((raw >> 0)  & 0xFF);
            g_eth_ip[1] = (u8)((raw >> 8)  & 0xFF);
            g_eth_ip[2] = (u8)((raw >> 16) & 0xFF);
            g_eth_ip[3] = (u8)((raw >> 24) & 0xFF);
        }
    }
    //printf("IP: %d.%d.%d.%d\r\n",
    //       g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);

    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_MASK);
        if (raw != 0xFFFFFFFF && raw != 0) {
            g_eth_mask[0] = (u8)((raw >> 0)  & 0xFF);
            g_eth_mask[1] = (u8)((raw >> 8)  & 0xFF);
            g_eth_mask[2] = (u8)((raw >> 16) & 0xFF);
            g_eth_mask[3] = (u8)((raw >> 24) & 0xFF);
        }
    }
    //printf("MASK: %d.%d.%d.%d\r\n",
    //       g_eth_mask[0], g_eth_mask[1], g_eth_mask[2], g_eth_mask[3]);

    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_GW);
        if (raw != 0xFFFFFFFF && raw != 0) {
            g_eth_gw[0] = (u8)((raw >> 0)  & 0xFF);
            g_eth_gw[1] = (u8)((raw >> 8)  & 0xFF);
            g_eth_gw[2] = (u8)((raw >> 16) & 0xFF);
            g_eth_gw[3] = (u8)((raw >> 24) & 0xFF);
        }
    }
    //printf("GW: %d.%d.%d.%d\r\n",
    //       g_eth_gw[0], g_eth_gw[1], g_eth_gw[2], g_eth_gw[3]);

    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_PORT);
        if (raw > 0 && raw <= 65535) {
            g_eth_port = (u16)raw;
        }
    }
    //printf("PORT: %d\r\n", (int)g_eth_port);
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
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_PL_T,       (uint32_t)PL_T);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_PL_H,       (uint32_t)PL_H);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_CS_T,       (uint32_t)CS_T);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_CS_H,       (uint32_t)CS_H);
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

    /* 保存网络参数 (4 字节 packed) */
    {
        uint32_t ip_packed   = ((uint32_t)g_eth_ip[3] << 24) |
                               ((uint32_t)g_eth_ip[2] << 16) |
                               ((uint32_t)g_eth_ip[1] << 8)  |
                               ((uint32_t)g_eth_ip[0]);
        uint32_t mask_packed = ((uint32_t)g_eth_mask[3] << 24) |
                               ((uint32_t)g_eth_mask[2] << 16) |
                               ((uint32_t)g_eth_mask[1] << 8)  |
                               ((uint32_t)g_eth_mask[0]);
        uint32_t gw_packed   = ((uint32_t)g_eth_gw[3] << 24) |
                               ((uint32_t)g_eth_gw[2] << 16) |
                               ((uint32_t)g_eth_gw[1] << 8)  |
                               ((uint32_t)g_eth_gw[0]);

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_IP,   ip_packed);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_MASK, mask_packed);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_GW,   gw_packed);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_PORT, (uint32_t)g_eth_port);
    }

    HAL_FLASH_Lock();
    printf("Params saved\r\n");
}

/* 固件升级标志写入 (对齐F1 0x0807C000, bootloader握手)
 * val=1: RES命令设置后复位, bootloader检测到即进入固件升级模式
 * val=0: APP正常启动清除, 防止下次复位误进升级模式
 * 只编程不擦除: 写1依赖cmd_res先param_save_all整扇区擦除(该地址在Sector7内);
 * 写0由1或0xFF直接编程得到; 值相同则跳过, 减少Flash磨损 */
void write_update_flag(uint32_t val) {
    if (*(__IO uint32_t *)UPDATE_FLAG_ADDR == val) return;

    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UPDATE_FLAG_ADDR, val);
    HAL_FLASH_Lock();
}
