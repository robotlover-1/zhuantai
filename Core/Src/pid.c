#include "pid.h"
#include "step_motor.h"

PID_TypeDef  g_location_pid;           /* �ٶȻ�PID�����ṹ�� */
PID_TypeDef  g_speed_pid;                /* �ٶȻ�PID�����ṹ�� */
/**
 * @brief       pid��ʼ��
 * @param       ��
 * @retval      ��
 */
extern float  SetPoint_S;            /* �趨Ŀ�� */
extern float  ActualValue_S;         /* �������ֵ */
extern float  SumError_S;            /* ����ۼ� */
extern float  Proportion_S;          /* �������� P */
extern float  Integral_S;            /* ���ֳ��� I */
extern float  Derivative_S;          /* ΢�ֳ��� D */
extern float  Error_S;               /* Error[1] */
extern float  LastError_S;           /* Error[-1] */
extern float  PrevError_S;           /* Error[-2] */

extern float  SetPoint_P;            /* �趨Ŀ�� */
extern float  ActualValue_P;         /* �������ֵ */
extern float  SumError_P;            /* ����ۼ� */
extern float  Proportion_P;          /* �������� P */
extern float  Integral_P;            /* ���ֳ��� I */
extern float  Derivative_P;          /* ΢�ֳ��� D */
extern float  Error_P;               /* Error[1] */
extern float  LastError_P;           /* Error[-1] */
extern float  PrevError_P;           /* Error[-2] */

void pid_init(void)
{
  // SetPoint_S=0;            /* �趨Ŀ�� */
   ActualValue_S=0;         /* �������ֵ */
   SumError_S=0;            /* ����ۼ� */
   Proportion_S=S_KP;          /* �������� P */
   Integral_S=S_KI;            /* ���ֳ��� I */
   Derivative_S=S_KD;          /* ΢�ֳ��� D */
   Error_S=0;               /* Error[1] */
   LastError_S=0;           /* Error[-1] */
   PrevError_S=0;           /* Error[-2] */
	
	// SetPoint_P=0;            /* �趨Ŀ�� */
   ActualValue_P=0;         /* �������ֵ */
   SumError_P=0;            /* ����ۼ� */
   Proportion_P=KP;          /* �������� P */
   Integral_P=KI;            /* ���ֳ��� I */
   Derivative_P=KD;          /* ΢�ֳ��� D */
   Error_P=0;               /* Error[1] */
   LastError_P=0;           /* Error[-1] */
   PrevError_P=0;           /* Error[-2] */
}
/**
 * @brief       pid�ջ�����
 * @param       *PID��PID�ṹ�������ַ
 * @param       Feedback_value����ǰʵ��ֵ
 * @retval      �������ֵ
 */
extern int pulse_add; 
int32_t increment_pid_speed(float Feedback_value)
{	  
    Error_S = (float)(SetPoint_S - Feedback_value);                   /* ����ƫ�� */ 
    ActualValue_S += (Proportion_S * (Error_S - LastError_S))                          /* �������� */
                        + (Integral_S * Error_S)                                             /* ���ֻ��� */
                        + (Derivative_S * (Error_S - 2 *LastError_S + PrevError_S));  /* ΢�ֻ��� */
    
    PrevError_S = LastError_S;                                        /* �洢ƫ������´μ��� */
    LastError_S = Error_S;
    return ((int32_t)(ActualValue_S )); 
//	  SumError_S += Error_S;
//    ActualValue_S = (Proportion_S * Error_S)                       /* �������� */
//                       + (Integral_S * SumError_S )                    /* ���ֻ��� */
//                       + (Derivative_S * (Error_S - LastError_S)); /* ΢�ֻ��� */
//    LastError_S = Error_S;
//    
//    return ((int32_t)(ActualValue_S )); 
}

int32_t position_pid_speed(float Feedback_value)
{	  
     Error_P = (float)(SetPoint_P - Feedback_value);                   /* ����ƫ�� */ 

	  SumError_P += Error_P;
	   if(SumError_P>40000)
			{
				SumError_P=40000;
			}
			 if(SumError_P<-40000)
			{
				SumError_P=-40000;
			}
    ActualValue_P = (Proportion_P * Error_P)                       /* �������� */
                       + (Integral_P * SumError_P )                    /* ���ֻ��� */
                       + (Derivative_P * (Error_P - LastError_P)); /* ΢�ֻ��� */
    LastError_P = Error_P;
    
    return ((int32_t)(ActualValue_P )); 
}
