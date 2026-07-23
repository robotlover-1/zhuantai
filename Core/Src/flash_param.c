/**
 ****************************************************************************************************
 * @file        flash_param.c
 * @brief       F407 Flash parameter storage management
 * @note        All parameters centralized in Sector 7, avoids erase conflicts
 ****************************************************************************************************
 */
#include "flash_param.h"
#include "delay.h"
#include "usart.h"
#include "pid.h"

/******************************************************************************************/
/* External variable declarations */
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
extern int32_t total_cycles;
extern int undershoot_cnt;
extern int overshoot_cnt;
extern int passhole_cnt;
extern u8  g_eth_ip[4];
extern u8  g_eth_mask[4];
extern u8  g_eth_gw[4];
extern u16 g_eth_port;
extern int gravity_ff_amp;
extern int gravity_ff_phase;

/******************************************************************************************/
/* Common implementation */
/******************************************************************************************/

/**
 * @brief       Load all parameters from Flash (with validity check)
 * @note        On first use or empty Flash, default values are written automatically
 */
void param_load_all(void)
{
    int32_t val;

    /* force - max speed */
    val = PARAM_READ(PARAM_OFFSET_FORCE);
    printf("Flash read - force: %d\r\n", val);
    if (val >= 0 && val <= 1500)
        force = val;
    else
        force = 1500;
    printf("Force(F): %d\r\n", force);
    //printf("设置F: %d\r\n", force);
    /* force_F - max duty */
    val = PARAM_READ(PARAM_OFFSET_FORCE_F);
        printf("Flash read - duty: %d\r\n", val);
    if (val >= 0 && val <= 3599)
        force_F = val;
    else
        force_F = 3500;
    printf("Duty(R): %d\r\n", force_F);
    //printf("R: %d\r\n", force_F);
    /* time_c - hole entry speed */
    val = PARAM_READ(PARAM_OFFSET_TIME_C);
    printf("Flash read - hole speed: %d\r\n", val);
    if (val >= 0 && val <= 1000)
        time_c = val;
    else
        time_c = 200;
    printf("Hole speed(V): %d\r\n", time_c);
    //printf("V: %d\r\n", time_c);

    /* pulse_low - 进孔脉冲数 */
    val = PARAM_READ(PARAM_OFFSET_PULSE_LOW);
    printf("Flash read - pulse_low: %d\r\n", val);
    if (val >= 0 && val <= 20000)
        pulse_low = val;
    else
        pulse_low = 20000;
    printf("Pulse low(S): %d\r\n", pulse_low);
    //printf("S: %d\r\n", pulse_low);

    /* pulse_buf - buffer segment start pulse count */
    val = PARAM_READ(PARAM_OFFSET_PULSE_BUF);
    printf("Flash read - pulse_buf: %d\r\n", val);
    if (val >= 0 && val <= 50000)
        pulse_buf = val;
    else
        pulse_buf = 30000;
    printf("Pulse buf(BS): %d\r\n", pulse_buf);

    /* time_buf - buffer segment speed */
    val = PARAM_READ(PARAM_OFFSET_TIME_BUF);
    printf("Flash read - time_buf: %d\r\n", val);
    if (val >= 0 && val <= 1000)
        time_buf = (int)val;
    else
        time_buf = 300;
    printf("Time buf(BV): %d\r\n", time_buf);

    /* gravity_ff_amp - gravity feedforward: sine amplitude */
    val = PARAM_READ(PARAM_OFFSET_FF_AMP);
    printf("Flash read - FF amp: %d\r\n", val);
    if (val >= 0 && val <= 500)
        gravity_ff_amp = (int)val;
    else
        gravity_ff_amp = 0;
    printf("FF amp(FA): %d\r\n", gravity_ff_amp);

    /* gravity_ff_phase - gravity feedforward: phase offset */
    val = PARAM_READ(PARAM_OFFSET_FF_PHASE);
    printf("Flash read - FF phase: %d\r\n", val);
    if (val >= 0 && val <= 195999)
        gravity_ff_phase = (int)val;
    else
        gravity_ff_phase = 0;
    printf("FF phase(FP): %d\r\n", gravity_ff_phase);

    /* pluse_ele - photo sensor pulse limit */
    val = PARAM_READ(PARAM_OFFSET_PLUSE_ELE);
    printf("Flash read - pluse_ele: %d\r\n", val);
    if (val >= 0 && val <= 39000)
        pluse_ele = val;
    else
        pluse_ele = 38000;
    printf("Pluse ele(E): %d\r\n", pluse_ele);
    //printf("E: %d\r\n", pluse_ele);

    /* PID 位置环 KP (×1000) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_KP);
        if (raw >= 0 && raw <= 3000)
            g_pid_kp = raw * 0.001f;
        else
            g_pid_kp = KP;
    }
    printf("Pos KP: %.3f\r\n", g_pid_kp);

    /* PID position loop KI (x1000) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_KI);
        if (raw >= 0 && raw <= 3000)
            g_pid_ki = raw * 0.001f;
        else
            g_pid_ki = KI;
    }
    printf("Pos KI: %.3f\r\n", g_pid_ki);

    /* PID position loop KD (x1000) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_KD);
        if (raw >= 0 && raw <= 1000)
            g_pid_kd = raw * 0.001f;
        else
            g_pid_kd = KD;
    }
    printf("Pos KD: %.3f\r\n", g_pid_kd);

    /* PID speed loop S_KP (x1000) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_SKP);
        if (raw >= 0 && raw <= 50000)
            g_pid_skp = raw * 0.001f;
        else
            g_pid_skp = S_KP;
    }
    printf("Spd S_KP: %.3f\r\n", g_pid_skp);

    /* PID speed loop S_KI (x1000) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_SKI);
        if (raw >= 0 && raw <= 50000)
            g_pid_ski = raw * 0.001f;
        else
            g_pid_ski = S_KI;
    }
    printf("Spd S_KI: %.3f\r\n", g_pid_ski);

    /* PID speed loop S_KD (x1000) */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_SKD);
        if (raw >= 0 && raw <= 50000)
            g_pid_skd = raw * 0.001f;
        else
            g_pid_skd = S_KD;
    }
    printf("Spd S_KD: %.3f\r\n", g_pid_skd);

    /* PID sampling period */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_PID_PERIOD);
        if (raw >= 5 && raw <= 100)
            g_pid_period_ms = (int)raw;
        else
            g_pid_period_ms = SMAPLSE_PID_SPEED;
    }
    printf("PID period: %dms\r\n", g_pid_period_ms);

    /* total_cycles - 总转动孔位计数 */
    val = PARAM_READ(PARAM_OFFSET_CYCLE_CNT);
    if (val >= 0)
        total_cycles = val;
    else
        total_cycles = 0;
    printf("total_cycles: %d\r\n", total_cycles);

    /* undershoot_cnt - 欠冲累计计数 */
    val = PARAM_READ(PARAM_OFFSET_UNDER_CNT);
    if (val >= 0)
        undershoot_cnt = (int)val;
    else
        undershoot_cnt = 0;
    printf("Undershoot: %d\r\n", undershoot_cnt);

    /* overshoot_cnt */
    val = PARAM_READ(PARAM_OFFSET_OVER_CNT);
    if (val >= 0)
        overshoot_cnt = (int)val;
    else
        overshoot_cnt = 0;
    printf("Overshoot: %d\r\n", overshoot_cnt);

    /* passhole_cnt - 过孔位累计计数 */
    val = PARAM_READ(PARAM_OFFSET_PASSHOLE_CNT);
    if (val >= 0)
        passhole_cnt = (int)val;
    else
        passhole_cnt = 0;
    printf("passhole_cnt: %d\r\n", passhole_cnt);

    /* Update flag */
    writeFlashData = PARAM_READ(PARAM_OFFSET_UPDATE);

    /* ---- Network parameters ---- */
    /* IP address */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_IP);
        if (raw != 0xFFFFFFFF && raw != 0) {  /* 有效值，非擦除态 */
            g_eth_ip[0] = (u8)((raw >> 0)  & 0xFF);
            g_eth_ip[1] = (u8)((raw >> 8)  & 0xFF);
            g_eth_ip[2] = (u8)((raw >> 16) & 0xFF);
            g_eth_ip[3] = (u8)((raw >> 24) & 0xFF);
        }
    }
    printf("IP: %d.%d.%d.%d\r\n",
           g_eth_ip[0], g_eth_ip[1], g_eth_ip[2], g_eth_ip[3]);

    /* Subnet mask */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_MASK);
        if (raw != 0xFFFFFFFF && raw != 0) {
            g_eth_mask[0] = (u8)((raw >> 0)  & 0xFF);
            g_eth_mask[1] = (u8)((raw >> 8)  & 0xFF);
            g_eth_mask[2] = (u8)((raw >> 16) & 0xFF);
            g_eth_mask[3] = (u8)((raw >> 24) & 0xFF);
        }
    }
    printf("MASK: %d.%d.%d.%d\r\n",
           g_eth_mask[0], g_eth_mask[1], g_eth_mask[2], g_eth_mask[3]);

    /* Gateway */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_GW);
        if (raw != 0xFFFFFFFF && raw != 0) {
            g_eth_gw[0] = (u8)((raw >> 0)  & 0xFF);
            g_eth_gw[1] = (u8)((raw >> 8)  & 0xFF);
            g_eth_gw[2] = (u8)((raw >> 16) & 0xFF);
            g_eth_gw[3] = (u8)((raw >> 24) & 0xFF);
        }
    }
    printf("GW: %d.%d.%d.%d\r\n",
           g_eth_gw[0], g_eth_gw[1], g_eth_gw[2], g_eth_gw[3]);

    /* TCP port */
    {
        int32_t raw = PARAM_READ(PARAM_OFFSET_ETH_PORT);
        if (raw > 0 && raw <= 65535) {
            g_eth_port = (u16)raw;
        }
    }
    printf("PORT: %d\r\n", g_eth_port);
}

/**
 * @brief       Save all parameters to Flash in one operation
 * @note        Erases Sector 7 once, then writes all parameters sequentially
 *              Much fewer erase cycles than erasing per-parameter
 */
void param_save_all(void)
{
    uint32_t PageError = 0;
    printf("Unlock FLASH\r\n");
    /* Unlock Flash */
    HAL_FLASH_Unlock();

    /* 擦除整个Sector 7 (128KB) */
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_SECTORS;
    f.Sector = PARAM_SECTOR;
    f.NbSectors = 1;
    f.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&f, &PageError);

    /* Write all parameters sequentially */
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
                      PARAM_SECTOR_BASE + PARAM_OFFSET_FF_AMP, (uint32_t)gravity_ff_amp);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_FF_PHASE, (uint32_t)gravity_ff_phase);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PLUSE_ELE, (uint32_t)pluse_ele);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_UPDATE, 0);

    /* Save PID params (integer storage, avoid float in Flash) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_KP,  (uint32_t)(g_pid_kp  * 1000.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_KI,  (uint32_t)(g_pid_ki  * 1000.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_KD,  (uint32_t)(g_pid_kd  * 1000.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_SKP, (uint32_t)(g_pid_skp * 1000.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_SKI, (uint32_t)(g_pid_ski * 1000.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_SKD, (uint32_t)(g_pid_skd * 1000.0f + 0.5f));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PID_PERIOD, (uint32_t)g_pid_period_ms);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_CYCLE_CNT, (uint32_t)total_cycles);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_UNDER_CNT, (uint32_t)undershoot_cnt);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_OVER_CNT, (uint32_t)overshoot_cnt);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_PASSHOLE_CNT, (uint32_t)passhole_cnt);

    /* Save RTC time backup (read RTC registers directly, BCD->DEC then pack) */
    {
        uint32_t _dr = RTC->DR;
        uint32_t _tr = RTC->TR;
        uint32_t _ymd, _hms;
        int _y, _mon, _d, _h, _min, _s;
        /* Inline BCD to decimal: high nibble*10 + low nibble */
#define B2D(b) ((((b) >> 4) * 10) + ((b) & 0x0F))
        _y   = 2000 + B2D((_dr >> 16) & 0xFF);
        _mon = B2D((_dr >> 8) & 0x1F);
        _d   = B2D(_dr & 0x3F);
        _h   = B2D((_tr >> 16) & 0x3F);
        _min = B2D((_tr >> 8) & 0x7F);
        _s   = B2D(_tr & 0x7F);
#undef B2D
        /* Basic validation: skip invalid values (RTC may not be initialized) */
        if (_y >= 2025 && _y <= 2099 && _mon >= 1 && _mon <= 12
            && _d >= 1 && _d <= 31 && _h <= 23 && _min <= 59 && _s <= 59)
        {
            _ymd = ((uint32_t)_y << 16) | ((uint32_t)_mon << 8) | (uint32_t)_d;
            _hms = ((uint32_t)_h << 16) | ((uint32_t)_min << 8) | (uint32_t)_s;
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              PARAM_SECTOR_BASE + PARAM_OFFSET_RTC_YMD, _ymd);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              PARAM_SECTOR_BASE + PARAM_OFFSET_RTC_HMS, _hms);
        }
    }

    /* Save network parameters */
    {
        uint32_t ip_packed = ((uint32_t)g_eth_ip[3] << 24) |
                             ((uint32_t)g_eth_ip[2] << 16) |
                             ((uint32_t)g_eth_ip[1] << 8)  |
                             ((uint32_t)g_eth_ip[0]);
        uint32_t mask_packed = ((uint32_t)g_eth_mask[3] << 24) |
                               ((uint32_t)g_eth_mask[2] << 16) |
                               ((uint32_t)g_eth_mask[1] << 8)  |
                               ((uint32_t)g_eth_mask[0]);
        uint32_t gw_packed = ((uint32_t)g_eth_gw[3] << 24) |
                             ((uint32_t)g_eth_gw[2] << 16) |
                             ((uint32_t)g_eth_gw[1] << 8)  |
                             ((uint32_t)g_eth_gw[0]);

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_IP,   ip_packed);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_MASK, mask_packed);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_GW,   gw_packed);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          PARAM_SECTOR_BASE + PARAM_OFFSET_ETH_PORT, (uint32_t)g_eth_port);
    }

    /* 锁定Flash */
    HAL_FLASH_Lock();

    printf("All params saved to Flash\r\n");
}
