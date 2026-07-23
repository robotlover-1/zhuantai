#ifndef __FLASH_PARAM_H
#define __FLASH_PARAM_H

#include "sys.h"

/* Flash Sector: Sector 7 (0x08060000, 128KB) */
#define PARAM_SECTOR                FLASH_SECTOR_7
#define PARAM_SECTOR_BASE           0x08060000U

/* F1 参数偏移 (每个4字节) */
#define PARAM_OFFSET_SET_SPEED      0x00    /* g_set_speed */
#define PARAM_OFFSET_STEP_ACCEL     0x04    /* g_step_accel */
#define PARAM_OFFSET_STEP_DECEL     0x08    /* g_step_decel */
#define PARAM_OFFSET_STEP1          0x0C    /* g_step1 */
#define PARAM_OFFSET_STEP2          0x10    /* g_step2 */
#define PARAM_OFFSET_STEP_PASS      0x14    /* step_pass */
#define PARAM_OFFSET_SPEED_RCC      0x18    /* speed_rcc (原始值, 转/s) */
#define PARAM_OFFSET_PULSE_CUT      0x1C    /* pulse_cut */
#define PARAM_OFFSET_PULSE_COMP     0x20    /* pulse_compensate */
#define PARAM_OFFSET_TIME_DELAY     0x24    /* time_delay_compensate */
#define PARAM_OFFSET_LOOP1          0x28    /* loop1 */
#define PARAM_OFFSET_LOOP2          0x2C    /* loop2 */
#define PARAM_OFFSET_CYCLE_CNT      0x30    /* total_cycles */
#define PARAM_OFFSET_UPDATE         0x34    /* 参数更新标志 */
#define PARAM_OFFSET_RTC_YMD        0x38    /* RTC时间备份: 年月日 */
#define PARAM_OFFSET_RTC_HMS        0x3C    /* RTC时间备份: 时分秒 */

#define PARAM_READ(offset)          (*(__IO uint32_t *)(PARAM_SECTOR_BASE + (offset)))

void param_load_all(void);
void param_save_all(void);
#endif
