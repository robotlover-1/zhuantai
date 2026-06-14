#ifndef __RTC_H
#define __RTC_H

#include "sys.h"

void rtc_init(void);                        /* 初始化RTC */
void rtc_get_time_str(char *buf, int len);  /* 获取时间字符串(含毫秒), buf至少32字节 */

#endif
