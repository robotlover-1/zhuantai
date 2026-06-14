#include "pid.h"
#include "step_motor.h"

PID_TypeDef  g_location_pid;           /* 速度环PID参数结构体 */
PID_TypeDef  g_speed_pid;                /* 速度环PID参数结构体 */

/* 运行时PID增益（可通过串口指令调整，Flash持久化） */
float g_pid_kp  = KP;                   /* 位置环 P 增益 */
float g_pid_ki  = KI;                   /* 位置环 I 增益 */
float g_pid_kd  = KD;                   /* 位置环 D 增益 */
float g_pid_skp = S_KP;                 /* 速度环 P 增益 */
float g_pid_ski = S_KI;                 /* 速度环 I 增益 */
float g_pid_skd = S_KD;                 /* 速度环 D 增益 */
int   g_pid_period_ms = SMAPLSE_PID_SPEED; /* PID采样周期(ms) */

/**
 * @brief       pid初始化
 * @param       无
 * @retval      无
 */

extern float  SetPoint_S;            /* 设定目标 */
extern float  ActualValue_S;         /* 期望输出值 */
extern float  SumError_S;            /* 误差累计 */
extern float  Proportion_S;          /* 比例常数 P */
extern float  Integral_S;            /* 积分常数 I */
extern float  Derivative_S;          /* 微分常数 D */
extern float  Error_S;               /* Error[1] */
extern float  LastError_S;           /* Error[-1] */
extern float  PrevError_S;           /* Error[-2] */
extern float  SetPoint_P;            /* 设定目标 */
extern float  ActualValue_P;         /* 期望输出值 */
extern float  SumError_P;            /* 误差累计 */
extern float  Proportion_P;          /* 比例常数 P */
extern float  Integral_P;            /* 积分常数 I */
extern float  Derivative_P;          /* 微分常数 D */
extern float  Error_P;               /* Error[1] */
extern float  LastError_P;           /* Error[-1] */
extern float  PrevError_P;           /* Error[-2] */

void pid_init(void)
{
  // SetPoint_S=0;            /* 设定目标 */
   ActualValue_S=0;         /* 期望输出值 */
   SumError_S=0;            /* 误差累计 */
   Proportion_S=g_pid_skp;  /* 比例常数 P (运行时可调) */
   Integral_S=g_pid_ski;    /* 积分常数 I (运行时可调) */
   Derivative_S=g_pid_skd;  /* 微分常数 D (运行时可调) */
   Error_S=0;               /* Error[1] */
   LastError_S=0;           /* Error[-1] */
   PrevError_S=0;           /* Error[-2] */
  // SetPoint_S=0;            /* 设定目标 */
   ActualValue_P=0;         /* 期望输出值 */
   SumError_P=0;            /* 误差累计 */
   Proportion_P=g_pid_kp;   /* 比例常数 P (运行时可调) */
   Integral_P=g_pid_ki;     /* 积分常数 I (运行时可调) */
   Derivative_P=g_pid_kd;   /* 微分常数 D (运行时可调) */
   Error_P=0;               /* Error[1] */
   LastError_P=0;           /* Error[-1] */
   PrevError_P=0;           /* Error[-2] */
}

/**
 * @brief       增量式PID速度控制计算
 * @param       Feedback_value：当前反馈速度值
 * @retval      PID输出值(PWM占空比)
 */

extern int pulse_add; 

int32_t increment_pid_speed(float Feedback_value)
{	  
    Error_S = (float)(SetPoint_S - Feedback_value);                   /* 计算偏差 */ 
    ActualValue_S += (Proportion_S * (Error_S - LastError_S))                          /* 比例环节 */
                        + (Integral_S * Error_S)                                             /* 积分环节 */
                        + (Derivative_S * (Error_S - 2 *LastError_S + PrevError_S));  /* 微分环节 */
    PrevError_S = LastError_S;                                        /* 存储偏差，用于下次计算 */
    LastError_S = Error_S;
    return ((int32_t)(ActualValue_S )); 
//	  SumError_S += Error_S;
//    ActualValue_S = (Proportion_S * Error_S)                       /* 比例环节 */
//                       + (Integral_S * SumError_S )                    /* 积分环节 */
//                       + (Derivative_S * (Error_S - 2 *LastError_S + PrevError_S));  /* 微分环节 */
//    LastError_S = Error_S;
//    
//    return ((int32_t)(ActualValue_S )); 
}

int32_t position_pid_speed(float Feedback_value)
{	  
    Error_P = (float)(SetPoint_P - Feedback_value);                   /* 计算偏差 */ 
	  SumError_P += Error_P;
	   if(SumError_P>40000)
			{
				SumError_P=40000;
			}

			 if(SumError_P<-40000)
			{
				SumError_P=-40000;
			}

    ActualValue_P = (Proportion_P * Error_P)                       /* 比例环节 */
                       + (Integral_P * SumError_P )                    /* 积分环节 */
                       + (Derivative_P * (Error_P - 2 *LastError_P + PrevError_P));  /* 微分环节 */
    LastError_P = Error_P;
    return ((int32_t)(ActualValue_P )); 
}
