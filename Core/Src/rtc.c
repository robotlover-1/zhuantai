#include "rtc.h"
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

void rtc_init(void)
{
    uint32_t tick;
    int y, mon, d, hour, min, sec;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;

    /* 解除RTC写保护：必须先写0xCA再写0x53，否则PRER/DR/TR写入无效 */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* 若RTC尚未初始化，配置时钟和分频 */
    if ((RTC->ISR & RTC_ISR_INITS) == 0)
    {
        RCC->CSR |= RCC_CSR_LSION;
        tick = HAL_GetTick();
        while ((RCC->CSR & RCC_CSR_LSIRDY) == 0)
	        if ((HAL_GetTick() - tick) > 500) break;
        if ((RCC->CSR & RCC_CSR_LSIRDY) == 0) return;

        RCC->BDCR |= RCC_BDCR_RTCSEL_1;
        RCC->BDCR |= RCC_BDCR_RTCEN;

        /* 先设置INIT再等待INITF，顺序不能反 */
        tick = HAL_GetTick();
        RTC->ISR |= RTC_ISR_INIT;
	    while ((RTC->ISR & RTC_ISR_INITF) == 0)
	        if ((HAL_GetTick() - tick) > 100) break;
        RTC->PRER = (127 << 16) | 249;

        rtc_parse_time(&y, &mon, &d, &hour, &min, &sec);
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
        /* RTC已有值但年份明显不对(如2000)，则修正 */
        uint32_t dr = RTC->DR;
        y = 2000 + rtc_bcd2dec((dr >> 16) & 0xFF);
        if (y < 2025)
        {
            /* 先设置INIT再等待INITF，顺序不能反；加超时避免死等 */
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

    /* 恢复RTC写保护 */
    RTC->WPR = 0xFF;
}

void rtc_get_time_str(char *buf, int len)
{
    if (buf == NULL || len < 32) return;

	{
	    uint32_t _tick = HAL_GetTick();
	    while ((RTC->ISR & RTC_ISR_RSF) == 0) {
	        if ((HAL_GetTick() - _tick) > 50) break;
		}
	}
	if ((RTC->ISR & RTC_ISR_RSF) == 0) {
	    snprintf(buf, len, "---- -- -- --:--:--");
	    return;
	}
    RTC->ISR &= ~RTC_ISR_RSF;

    uint32_t dr = RTC->DR;
    uint32_t tr = RTC->TR;

    snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03u",
             2000 + rtc_bcd2dec((dr >> 16) & 0xFF),
             rtc_bcd2dec((dr >> 8) & 0x1F),
             rtc_bcd2dec(dr & 0x3F),
             rtc_bcd2dec((tr >> 16) & 0x3F),
             rtc_bcd2dec((tr >> 8) & 0x7F),
             rtc_bcd2dec(tr & 0x7F),
             g_tick_ms % 1000);
}
