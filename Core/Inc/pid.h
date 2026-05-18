#ifndef __PID_H
#define __PID_H

#include "sys.h"

/******************************************************************************************/
/* PID��ز��� */

#define  INCR_LOCT_SELECT  1         /* 0��λ��ʽ ��1������ʽ */

#if INCR_LOCT_SELECT

/* ����ʽPID������غ� */
#define  KP      0.02f               /* P����*/  //0.3   0.4
#define  KI      0.00f               /* I����*/
#define  KD      0.00f               /* D����*/
#define  SMAPLSE_PID_SPEED  25       /* �������� ��λms*/

/* �����ٶȻ����ڻ���PID������غ� */
#define  S_KP      0.8f             /* P���� */  // 50���� 60
#define  S_KI      1.5f            /* I���� */
#define  S_KD      0.0f             /* D���� */


#else

/* λ��ʽPID������غ� */
#define  KP      10.0f               /* P����*/
#define  KI      0.00f               /* I����*/
#define  KD      0.0f                /* D����*/
#define  SMAPLSE_PID_SPEED  50       /* �������� ��λms*/
/* �����ٶȻ����ڻ���PID������غ� */
#define  S_KP      20.0f             /* P���� */
#define  S_KI      0.00f            /* I���� */
#define  S_KD      0.00f             /* D���� */
#endif

/* PID�����ṹ�� */
typedef struct
{
    __IO float  SetPoint;            /* 设定目标 */
    __IO float  ActualValue;         /* �������ֵ */
    __IO float  SumError;            /* ����ۼ� */
    __IO float  Proportion;          /* �������� P */
    __IO float  Integral;            /* ���ֳ��� I */
    __IO float  Derivative;          /* ΢�ֳ��� D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
} PID_TypeDef;

extern PID_TypeDef  g_location_pid;     /*�ٶȻ�PID�����ṹ��*/
extern PID_TypeDef  g_speed_pid;     /* �ٶȻ�PID�����ṹ�� */
/******************************************************************************************/

void pid_init(void);                 /* pid��ʼ�� */

int32_t increment_pid_speed(float Feedback_value);     /* pid�ջ����� */
int32_t position_pid_speed(float Feedback_value);

#endif
