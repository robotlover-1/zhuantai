#ifndef __PID_H
#define __PID_H

#include "sys.h"

/******************************************************************************************/
/* PID相关参数 */

#define  INCR_LOCT_SELECT  1         /* 0：位置式 ，1：增量式 */

#if INCR_LOCT_SELECT

/* 增量式PID参数相关宏 */
#define  KP      0.02f               /* P参数*/  //0.3   0.4
#define  KI      0.00f               /* I参数*/
#define  KD      0.00f               /* D参数*/
#define  SMAPLSE_PID_SPEED  25       /* 采样周期 单位ms*/

/* 定义速度环（内环）PID参数相关宏 */
#define  S_KP      0.8f             /* P参数 */  // 50合适 60
#define  S_KI      1.5f            /* I参数 */
#define  S_KD      0.0f             /* D参数 */


#else

/* 位置式PID参数相关宏 */
#define  KP      10.0f               /* P参数*/
#define  KI      0.00f               /* I参数*/
#define  KD      0.0f                /* D参数*/
#define  SMAPLSE_PID_SPEED  50       /* 采样周期 单位ms*/
/* 定义速度环（内环）PID参数相关宏 */
#define  S_KP      20.0f             /* P参数 */
#define  S_KI      0.00f            /* I参数 */
#define  S_KD      0.00f             /* D参数 */
#endif

/* PID参数结构体 */
typedef struct
{
    __IO float  SetPoint;            /* 设定目标 */
    __IO float  ActualValue;         /* 期望输出值 */
    __IO float  SumError;            /* 误差累计 */
    __IO float  Proportion;          /* 比例常数 P */
    __IO float  Integral;            /* 积分常数 I */
    __IO float  Derivative;          /* 微分常数 D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
} PID_TypeDef;

extern PID_TypeDef  g_location_pid;     /*速度环PID参数结构体*/
extern PID_TypeDef  g_speed_pid;     /* 速度环PID参数结构体 */
/******************************************************************************************/

void pid_init(void);                 /* pid初始化 */

int32_t increment_pid_speed(float Feedback_value);     /* pid闭环控制 */
int32_t position_pid_speed(float Feedback_value);

#endif
