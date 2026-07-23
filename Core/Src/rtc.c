#include "rtc.h"
#include "flash_param.h"
#include "stdio.h"
#include "string.h"

extern volatile uint32_t g_tick_ms;  /* 来自atim.c的1ms系统滴答 */

static inline uint8_t rtc_bcd2dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static inline uint8_t rtc_dec2bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

/**
 * @brief       从__DATE__/__TIME__解析年月日时分秒
 */
static void rtc_parse_time(int *y, int *mon, int *d, int *hour, int *min, int *sec)
{
    const char *months[] = {"Jan","Feb","Mar","Apr","May","Jun",
                            "Jul","Aug","Sep","Oct","Nov","Dec"};
    char mon_str[4] = {0};
    *y = 2026; *mon = 1; *d = 1; *hour = 0; *min = 0; *sec = 0;
    sscanf(__DATE__, "%3s %d %d", mon_str, d, y);
    for (int i = 0; i < 12; i++)
        if (strncmp(mon_str, months[i], 3) == 0) { *mon = i + 1; break; }
    sscanf(__TIME__, "%d:%d:%d", hour, min, sec);
}

static int rtc_restore_from_flash(int *y, int *mon, int *d,
                                   int *h, int *min, int *s);

void rtc_init(void)
{
    uint32_t tick;
    int y, mon, d, hour, min, sec;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;

    /* 开启 LSI */
    RCC->CSR |= RCC_CSR_LSION;
    tick = HAL_GetTick();
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0)
        if ((HAL_GetTick() - tick) > 500) break;
    if ((RCC->CSR & RCC_CSR_LSIRDY) == 0) return;

    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    if ((RTC->ISR & RTC_ISR_INITS) == 0
        || (RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_BDCR_RTCSEL_1)
    {
        /* 首次初始化: 选择LSI, 配置分频和时间 */
        RCC->BDCR &= ~RCC_BDCR_RTCSEL;
        RCC->BDCR |= RCC_BDCR_RTCSEL_1;
        RCC->BDCR |= RCC_BDCR_RTCEN;

        tick = HAL_GetTick();
        RTC->ISR |= RTC_ISR_INIT;
        while ((RTC->ISR & RTC_ISR_INITF) == 0)
            if ((HAL_GetTick() - tick) > 100) break;
        RTC->PRER = (127 << 16) | 249;

        /* 优先从Flash备份恢复, 失败则用编译时间 */
        if (!rtc_restore_from_flash(&y, &mon, &d, &hour, &min, &sec))
        {
            rtc_parse_time(&y, &mon, &d, &hour, &min, &sec);
        }
        RTC->DR = (rtc_dec2bcd(y - 2000) << 16)
                | (rtc_dec2bcd(mon) << 8)
                | (rtc_dec2bcd(d) << 0);
        RTC->TR = (rtc_dec2bcd(hour) << 16)
                | (rtc_dec2bcd(min) << 8)
                | (rtc_dec2bcd(sec) << 0);
        RTC->ISR &= ~RTC_ISR_INIT;
    }
    else
    {
        /* RTC已有值, 仅修正异常年份 */
        uint32_t dr = RTC->DR;
        y = 2000 + rtc_bcd2dec((dr >> 16) & 0xFF);
        if (y < 2025)
        {
            tick = HAL_GetTick();
            RTC->ISR |= RTC_ISR_INIT;
            while ((RTC->ISR & RTC_ISR_INITF) == 0)
                if ((HAL_GetTick() - tick) > 100) break;
            rtc_parse_time(&y, &mon, &d, &hour, &min, &sec);
            RTC->DR = (rtc_dec2bcd(y - 2000) << 16)
                    | (rtc_dec2bcd(mon) << 8)
                    | (rtc_dec2bcd(d) << 0);
            RTC->TR = (rtc_dec2bcd(hour) << 16)
                    | (rtc_dec2bcd(min) << 8)
                    | (rtc_dec2bcd(sec) << 0);
            RTC->ISR &= ~RTC_ISR_INIT;
        }
    }

    RTC->WPR = 0xFF;
}

void rtc_get_time_str(char *buf, int len)
{
    if (buf == NULL || len < 32) return;

    /* 先清除RSF（需解除写保护），然后等待RTC影子寄存器同步完成 */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    RTC->ISR &= ~RTC_ISR_RSF;
    {
        uint32_t _tick = HAL_GetTick();
        while ((RTC->ISR & RTC_ISR_RSF) == 0) {
            if ((HAL_GetTick() - _tick) > 200) break;
        }
    }
    if ((RTC->ISR & RTC_ISR_RSF) == 0) {
        RTC->WPR = 0xFF;
        snprintf(buf, len, "---- -- -- --:--:--");
        return;
    }

    uint32_t dr = RTC->DR;
    uint32_t tr = RTC->TR;
    RTC->WPR = 0xFF;

    /* 亚秒: g_tick_ms % 1000, 1kHz系统滴答, 相对计时准确 */
    {
        uint32_t _ms = g_tick_ms % 1000;
        snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03lu",
                 2000 + rtc_bcd2dec((dr >> 16) & 0xFF),
                 rtc_bcd2dec((dr >> 8) & 0x1F),
                 rtc_bcd2dec(dr & 0x3F),
                 rtc_bcd2dec((tr >> 16) & 0x3F),
                 rtc_bcd2dec((tr >> 8) & 0x7F),
                 rtc_bcd2dec(tr & 0x7F),
                 (unsigned long)_ms);
    }
}

/**
 * @brief       设置RTC时间
 * @param       y: 年(2000~2099), mon: 月(1~12), d: 日(1~31)
 * @param       h: 时(0~23), min: 分(0~59), s: 秒(0~59)
 */
void rtc_set_time(int y, int mon, int d, int h, int min, int s)
{
    uint32_t tick;

    /* 解除写保护 */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* 进入初始化模式 */
    tick = HAL_GetTick();
    RTC->ISR |= RTC_ISR_INIT;
    while ((RTC->ISR & RTC_ISR_INITF) == 0)
        if ((HAL_GetTick() - tick) > 100) break;

    /* 写入日期和时间 */
    RTC->DR = (rtc_dec2bcd(y - 2000) << 16)
            | (rtc_dec2bcd(mon) << 8)
            | (rtc_dec2bcd(d) << 0);
    RTC->TR = (rtc_dec2bcd(h) << 16)
            | (rtc_dec2bcd(min) << 8)
            | (rtc_dec2bcd(s) << 0);

    /* 退出初始化模式 */
    RTC->ISR &= ~RTC_ISR_INIT;

    /* 恢复写保护 */
    RTC->WPR = 0xFF;
}

/**
 * @brief       从Flash读取RTC时间备份
 * @param       y/mon/d/h/min/s: 输出参数
 * @retval      1-成功恢复, 0-无有效备份
 */
static int rtc_restore_from_flash(int *y, int *mon, int *d,
                                   int *h, int *min, int *s)
{
    uint32_t ymd = PARAM_READ(PARAM_OFFSET_RTC_YMD);
    uint32_t hms = PARAM_READ(PARAM_OFFSET_RTC_HMS);

    /* 检查是否为擦除态(0xFFFFFFFF = 未写过) */
    if (ymd == 0xFFFFFFFF || hms == 0xFFFFFFFF)
        return 0;

    *y   = (int)((ymd >> 16) & 0xFFFF);
    *mon = (int)((ymd >> 8)  & 0xFF);
    *d   = (int)((ymd >> 0)  & 0xFF);
    *h   = (int)((hms >> 16) & 0xFF);
    *min = (int)((hms >> 8)  & 0xFF);
    *s   = (int)((hms >> 0)  & 0xFF);

    /* 基本合法性检查 */
    if (*y < 2025 || *y > 2099 || *mon < 1 || *mon > 12
        || *d < 1 || *d > 31 || *h > 23 || *min > 59 || *s > 59)
        return 0;

    return 1;
}

/**
 * @brief       备份当前RTC时间到Flash(2个Word)
 * @note        调用前需确保Flash已解锁，调用后锁定
 *              仅编程2个Word，不擦除扇区 (配合param_save_all使用)
 */
void rtc_backup_to_flash(void)
{
    uint32_t dr, tr;
    int y, mon, d, h, min, s;
    uint32_t ymd, hms;

    /* 等待RTC影子寄存器同步 */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    RTC->ISR &= ~RTC_ISR_RSF;
    {
        uint32_t _tick = HAL_GetTick();
        while ((RTC->ISR & RTC_ISR_RSF) == 0) {
            if ((HAL_GetTick() - _tick) > 200) break;
        }
    }

    dr = RTC->DR;
    tr = RTC->TR;
    RTC->WPR = 0xFF;

    if ((RTC->ISR & RTC_ISR_RSF) == 0) return;  /* 同步失败, 不保存 */

    y   = 2000 + rtc_bcd2dec((dr >> 16) & 0xFF);
    mon = rtc_bcd2dec((dr >> 8) & 0x1F);
    d   = rtc_bcd2dec(dr & 0x3F);
    h   = rtc_bcd2dec((tr >> 16) & 0x3F);
    min = rtc_bcd2dec((tr >> 8) & 0x7F);
    s   = rtc_bcd2dec(tr & 0x7F);

    ymd = ((uint32_t)y << 16) | ((uint32_t)mon << 8) | (uint32_t)d;
    hms = ((uint32_t)h << 16) | ((uint32_t)min << 8) | (uint32_t)s;

    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_RTC_YMD, ymd);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      PARAM_SECTOR_BASE + PARAM_OFFSET_RTC_HMS, hms);
    HAL_FLASH_Lock();
}
