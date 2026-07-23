#include "step_motor.h"
#include "atim.h"
#include "rtc.h"
#include "math.h"
#include "delay.h"

 STEPPER_MOTOR step = {0};  

extern u8 receive_flag;   //���Խ���ͨѶ ���н�����־

void step_motor_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    ST1_DIR_GPIO_CLK_ENABLE();           /* PA11 方向 */
    ST1_EN_GPIO_CLK_ENABLE();            /* PC1 使能 */
    ST1_ZERO_GPIO_CLK_ENABLE();          /* PC0 回零气缸 */
    ST1_ZERO1_GPIO_CLK_ENABLE();         /* PF8 回零检测 */
    ST1_END_GPIO_CLK_ENABLE();           /* PF9 光电到位 */
    ST1_IO_GPIO_CLK_ENABLE();            /* PC5 IO/拍照 */
    ST1_DONE_GPIO_CLK_ENABLE();          /* PC4 到位脉冲 */
    ST1_AUX_GPIO_CLK_ENABLE();           /* PC2 辅助输出 */
    ST1_HAL1_GPIO_CLK_ENABLE();          /* PE5 霍尔1 */
    ST1_HAL2_GPIO_CLK_ENABLE();          /* PE6 霍尔2 */
    ST1_HAL3_GPIO_CLK_ENABLE();          /* PC13 霍尔3 */

    /* PA11: 方向输出 (推挽) */
    gpio_init_struct.Pin = ST1_DIR_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ST1_DIR_GPIO_PORT, &gpio_init_struct);

    /* PC1: 电机使能 (开漏) */
    gpio_init_struct.Pin = ST1_EN_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ST1_EN_GPIO_PORT, &gpio_init_struct);
    HAL_GPIO_WritePin(ST1_EN_GPIO_PORT, ST1_EN_GPIO_PIN, GPIO_PIN_RESET);

    /* PC0: 回零气缸 (开漏) */
    gpio_init_struct.Pin = ST1_ZERO_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ST1_ZERO_GPIO_PORT, &gpio_init_struct);
    HAL_GPIO_WritePin(ST1_ZERO_GPIO_PORT, ST1_ZERO_GPIO_PIN, GPIO_PIN_RESET);

    /* PF8: 回零完成检测 (输入) */
    gpio_init_struct.Pin = ST1_ZERO1_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(ST1_ZERO1_GPIO_PORT, &gpio_init_struct);

    /* PF9: 光电到位检测 (输入) */
    gpio_init_struct.Pin = ST1_END_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ST1_END_GPIO_PORT, &gpio_init_struct);

    /* PC5: IO/拍照输出 (推挽) */
    gpio_init_struct.Pin = ST1_IO_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ST1_IO_GPIO_PORT, &gpio_init_struct);

    /* PC4: 到位脉冲输出 (推挽) */
    gpio_init_struct.Pin = ST1_DONE_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ST1_DONE_GPIO_PORT, &gpio_init_struct);

    /* PC2: 辅助输出 (开漏) */
    gpio_init_struct.Pin = ST1_AUX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ST1_AUX_GPIO_PORT, &gpio_init_struct);
    HAL_GPIO_WritePin(ST1_AUX_GPIO_PORT, ST1_AUX_GPIO_PIN, GPIO_PIN_RESET);

    /* 24V 输入 IO */
    __HAL_RCC_GPIOF_CLK_ENABLE();   /* PF6, PF7: 24V输入 */
    __HAL_RCC_GPIOE_CLK_ENABLE();   /* PE2: 复位输入 */
    gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_2;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);

    /* 霍尔传感器 (输入, 仅调试用) */
    gpio_init_struct.Pin = ST1_HAL1_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ST1_HAL1_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ST1_HAL2_GPIO_PIN;
    HAL_GPIO_Init(ST1_HAL2_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ST1_HAL3_GPIO_PIN;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ST1_HAL3_GPIO_PORT, &gpio_init_struct);

    /* PC5: 拍照信号 (初始化已在上面作为ST1_IO) */
    ST1_EN(0);  /* 初始释放使能 */
}


/********************************************���μӼ���***********************************************/
speedRampData g_srd               = {STOP,CW,0,0,0,0,0};  /* �Ӽ��ٱ��� */
__IO int32_t  g_step_position     = 0;                    /* ��ǰλ�� */
__IO uint8_t  g_motion_sta        = 0;                    /* �Ƿ����˶���0��ֹͣ��1���˶� */
__IO uint32_t g_add_pulse_count   = 0;                    /* ��������ۼ� */
       
extern int32_t step_all;	    //�����ܲ���
extern int32_t step_decel_2;  //���ٽ��׽׶Σ���һ�μ���
extern int32_t step_accel_2;  //���ټ��ٽ׶Σ��ڶ��μ���
extern	int32_t step_max;     //��������ٶ�����
extern	int32_t step_max_1;     //��������ٶ�����

//�������������ٶȣ����ٶȣ�����Ӽ��ٵĲ�����С step,accel,decel,speed
//step:�ܲ���
//step1:���������������ײ���
//step2�����������������ײ���
//accel�����׼��ٶ�
//decel�����׼��ٶ�
//speed����������ٶ�
	void create_t_ctrl_param(int32_t step,int32_t step1, int32_t accel, int32_t step2,int32_t decel, int32_t speed)
{
	if(step==0)
		{
			return;
		}
	if(g_motion_sta != STOP)        /* ֻ�������������ֹͣ��ʱ��ż���*/
        return;
	//***************�������*********************************//
	  //������������10000������������λ20000
//	   if(step>=10000 && step>0)   //������������Ϊ16000����������λ
//		 {
//			 step=10000;
//		 }
//		 else if(step<=-10000 && step<0) //������������Ϊ-16000��������λ
//		 {
//			 step=-10000;
//		 }		    
	   if(step<0)
			{
				step_all=-step;
				g_srd.step_decel1=step_all-step2-step_decel_2;            //����1����
        g_srd.step_decel2=step_all-step2;        //����2����
			}else
			{
				step_all=step;
				g_srd.step_decel1=step_all-step2-step_decel_2;            //����1����
        g_srd.step_decel2=step_all-step2;        //����2����
			}
			
		 step_max=12000/speed;  //��������ٶ�ʱ��delay�ļ���
	   g_srd.step_accel1=0;           //��ʼ����1���� 
     g_srd.step_accel2=step1;        //����2����
	   g_srd.step_run=step1+step_accel_2;               //�������в���
					      
    if(step < 0)                    /* ����Ϊ���� */
    {   
        g_srd.dir = CCW;            /* ��ʱ�뷽����ת 0*/
        ST1_DIR(CCW);
        step = -step-1;               /* ��ȡ��������ֵ */
    }
    else
    {
        g_srd.dir = CW;             /* ˳ʱ�뷽����ת 1*/
        ST1_DIR(CW);
			  step = step-1;
    }
   if(step != 0)              /* ���Ŀ���˶�������Ϊ0*/
    {
         g_srd.step_delay=6220;   //���ٶ�4.5��ÿ��
		}
    g_motion_sta = 1;                                           /* ���Ϊ�˶�״̬ */
	//	receive_flag=1;
		
		/*****������ټӼ���ʱ�ļӼ���******/
	
		g_srd.step_accel1_delay=accel;    //���׼��ٶ�
		g_srd.step_decel1_delay=decel;    //���׼��ٶ�
		
		g_srd.step_accel2_delay=(6220-step_max-step1*accel)/step_accel_2;  
		g_srd.step_decel2_delay=(6220-step_max-step2*decel)/step_decel_2;
		
		//�������ƣ���ֹΪ��ֵ
		if(6220-step_max-step1*accel<0)
		{
			g_srd.step_accel2_delay=10;
		}
		
		if(6220-step_max-step2*decel<0)
		{
			g_srd.step_decel2_delay=10;
		}
		
		g_srd.run_state = ACCEL;
		
     HAL_TIM_OC_Start_IT(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY);                                 /* ʹ�ܶ�ʱ��ͨ�� */
}



/**
  * @brief  ��ʱ���Ƚ��ж�
  * @param  htim����ʱ�����ָ��
  * @note   ��
  * @retval ��
  */

extern  int32_t step_count; 
extern  u8 printf_flag;
extern  int time_out;
extern int step_pass;                //�����ٶ�
extern int32_t pulse_compensate;     //����������
extern int speed_rcc;                //�����ٶ�
extern int level_speed;              //����leve�µ��������
//���й켣ѡ��level=0��1��2��3��4��5��6��7
//level=0��18��ÿ�������˶�
//level=1������8000����  �����ֶμӼ���
//level=2����16000���壬�м���ٹ��ף���������������   ��pluse_compensate   **************
//level=3�� �������岹�� һ����λ����  ������8000+pulse_compensate  ������-pulse_compensate;  **********
//level=4�� ������������������λ2      ����16000+pulse_compensate   ������-pulse_compensate;  **********
//level=5�� ���׶�������������100���٣�100����
//level=6�� ��8000����������������  ��pluse_compensate    *********
//level=7:  �Խ��׶��ٶ���������
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{    
    uint32_t tim_count = 0;
    uint32_t tmp = 0;
    static uint8_t i = 0;                              /* ��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ���������� */
	  static uint8_t j = 0;  
	    
    if(htim->Instance==TIM8)
    {		
			if(level_speed==0)
			{
				tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + 1000/2;      //18��ÿ��
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
				j++;                                                /* ��ʱ���жϴ�������ֵ */
        if(j == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 
         step_count++;
					if(step_count >=step_all) 
					{
						   step_count = 0;                             						
							HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
							g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
							//receive_flag=0;
						 // printf_flag=1;
							g_add_pulse_count=0;
							step_count=0;
						 // HAL_TIM_Base_Stop_IT(&TIM5_Handler);
             // time_out=0;	
					}
					j=0;
				}		
			}			
			else if	(level_speed==1)	  //һ����λ������ͣ�������� ��������12000���� T��
			{
		    tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;    //���ٶ�4.5��ÿ��           
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* ��ʱ���жϴ�������ֵ */
        if(i == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 				
					  switch(g_srd.run_state)                         /* �Ӽ������߽׶� */
            {
							case STOP:
                step_count = 0;                                        
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
							  //receive_flag=0;
							 // printf_flag=1;
							  step_count=0;	
						  //	HAL_TIM_Base_Stop_IT(&TIM5_Handler);
              // time_out=0;
                break;
							
            case ACCEL:
                step_count++;                               					  
								if(0<step_count & step_count<=g_srd.step_accel2)   //��һ�μ�������	
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //��һ�μ��� delay�ı�
									 if(g_srd.step_delay<step_max)  //��������ٶ�����
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)//�ڶ��μ���
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay; //�ڶ��μ��� delay�ı�
									
									 if(g_srd.step_delay<step_max)  //��������ٶ�����
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)          //������������
                {
                    g_srd.run_state = RUN;                  /* ����Ϊ��������״̬ */
                }
                break;
								
            case RUN:             
                step_count++;                               /* ������1 */              
								if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)  //��������
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( g_srd.step_decel1<step_count)         /* ��Ҫ��ʼ���� */
                {
                    g_srd.run_state = DECEL;                /* ״̬�ı�Ϊ���� */
                }
                break;

            case DECEL:
                step_count++;                               /* ������1 */                          
								if( g_srd.step_decel1<step_count & step_count<=g_srd.step_decel2)         //��һ�μ���
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;         
								}
								else if( g_srd.step_decel2<step_count & step_count<=step_all-2)           //�ڶ��μ���
								{    
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_accel1_delay;
								}              
								else if(step_count >step_all-2)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.run_state = STOP;
                }
                break;
            }
						 i = 0;                                          /* ���㶨ʱ���жϴ�������ֵ */
        }				
			}
			else if	(level_speed==2)//������ʱ���м���ٹ��ף�������  ������24000���壬���������������
			{
		    tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* ��ʱ���жϴ�������ֵ */
        if(i == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 				
					  switch(g_srd.run_state)                         /* �Ӽ������߽׶� */
            {
							case STOP:
                step_count = 0;                             /* ���㲽�������� */
                /* �ر�ͨ��*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
							//  receive_flag=0;
							 // printf_flag=1;
							  step_count=0;
              //  HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;							
                break;
            case ACCEL:     //��һ�μ��ٽ׶�                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //���е�һ�μ���
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //��������ٶ�����
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //���еڶ��μ���
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                    g_srd.run_state = RUN;                                              /* ����Ϊ��������״̬ */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //��������
								{
									g_srd.step_delay =g_srd.step_delay ;
									                                 //��¼��һ�����е�����ٶ�
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* ��Ҫ��ʼ���� */
                {
                    g_srd.run_state = DECEL;                                             /* ״̬�ı�Ϊ���� */
                }   
                break; 
						case DECEL:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����               /* ������׵����壬��֤�����ٶȽϵ�*/
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN2;                  /* ����Ϊ��������״̬ */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL2;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
					case DECEL2:
                step_count++;                                      
							if( step_all-pulse_compensate-step_decel_2<step_count & step_count<=step_all-pulse_compensate)         /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									if(g_srd.step_delay>speed_rcc)
									 {
										 g_srd.step_delay=speed_rcc;
									 }	
								}															 
								else if(step_count >step_all-pulse_compensate)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.run_state = RUN3;
                }
                break;	
					case RUN3:
               step_count++;
					    if(step_all-pulse_compensate<step_count&step_count<step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.step_delay=speed_rcc;
                }
							else if(step_count >=step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
									  g_srd.run_state = STOP;
                }
							break;		
            }
						 i = 0;                                          /* ���㶨ʱ���жϴ�������ֵ */
        }				
			}
			 else if	(level_speed==3)	  //�������岹�� һ����λ����  ������12000+pulse_compensate
			{	
				tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* ��ʱ���жϴ�������ֵ */
        if(i == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 				
					  switch(g_srd.run_state)                        
            {
							case STOP:
                step_count = 0;                           
                /* �ر�ͨ��*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                          
							 // receive_flag=0;
							 // printf_flag=1;
							  step_count=0;	
             //  HAL_TIM_Base_Stop_IT(&TIM5_Handler);
             // time_out=0;							
                break;
            case ACCEL:                                        
                step_count++;                             					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                        //��һ�μ��ٽ׶�   
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)        //�ڶ��μ���
                {
									  g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN;                   /* ����Ϊ��������״̬ */
                }
                break;
            case RUN:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1) 
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( g_srd.step_decel1<step_count)         /* ��Ҫ��ʼ���� */
                {
                  g_srd.run_state = DECEL;                      /* ״̬�ı�Ϊ���� */
                }   
                break; 
						case DECEL:
                step_count++;                             
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)         /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										  g_srd.step_delay =step_pass;   
									 }
								}												              
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.run_state = ACCEL2;
									  step_count=0;									  
                }
                break;									
						case ACCEL2:
                step_count++;                            					 
                if(0<step_count & step_count<=pulse_compensate-2) 	
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									  if(g_srd.step_delay<speed_rcc)
										{
											g_srd.step_delay=speed_rcc;
										}
								}								
								else if(pulse_compensate-2<step_count)
                {
                  g_srd.run_state = RUN3; 
									 step_count=0;
									if(g_srd.dir == CCW )
									{        
                    ST1_DIR(CW);
									}else
									{
										ST1_DIR(CCW);
									}		
								}														
                break;								
					case RUN2:     
                step_count++;                                        
               if( 100<step_count & step_count<=pulse_compensate-100) 
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( pulse_compensate-100<step_count)         /* ��Ҫ��ʼ���� */
                {
                  g_srd.run_state = DECEL2;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
					case DECEL2:
                step_count++;                                          
							if( pulse_compensate-100<step_count & step_count<=pulse_compensate-2)         /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
								}
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >pulse_compensate-2)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                   g_srd.run_state = RUN3;
									 step_count=0;
									if(g_srd.dir == CCW )
									{        
                    ST1_DIR(CW);
									}else
									{
										ST1_DIR(CCW);
									}									
                }
                break;	
          case RUN3:
						    step_count++;
					      g_srd.step_delay =speed_rcc;
						    if(step_count >pulse_compensate-3)
								{
									g_srd.run_state = STOP;
								}					      
						    break;
            }
						 i = 0;                                          /* ���㶨ʱ���жϴ�������ֵ */
        }							 
			 }
			else if	(level_speed==4)	  //������������������λ2    ����24000+pulse_compensate
			{
       tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* ��ʱ���жϴ�������ֵ */
        if(i == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 				
					  switch(g_srd.run_state)                         /* �Ӽ������߽׶� */
            {
							case STOP:
                step_count = 0;                             /* ���㲽�������� */
                /* �ر�ͨ��*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
							//  receive_flag=0;
							//  printf_flag=1;
							  step_count=0;	
              //  HAL_TIM_Base_Stop_IT(&TIM5_Handler);
              //  time_out=0;														
                break;
            case ACCEL:     //��һ�μ��ٽ׶�                       
                step_count++;                             					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //��������ٶ�����
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
									  g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)          /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                    g_srd.run_state = RUN;                  /* ����Ϊ��������״̬ */
                }
                break;
            case RUN:
                step_count++;  
						     g_srd.step_decel1=step_all-300;  //����500�ļ��ٲ�������
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1) 
								{
									g_srd.step_delay =g_srd.step_delay ;
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL;                    /* ״̬�ı�Ϊ���� */
                }   
                break; 
						case DECEL:
                step_count++;
                g_srd.step_decel1=step_all-300;  //����500�ļ��ٲ�������						
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)         /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										  g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.run_state = ACCEL2;
									 // g_srd.step_delay =4000;
									  step_count=0;									  
                }
                break;									
						case ACCEL2:        
                step_count++; 
						   if(0<step_count & step_count<=100) //��֤�ȶ�����      /*����������壬��֤���ٽ���*/
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
								else if(100<step_count & step_count<=g_srd.step_run) 	  //ֱ�ӿ����
								{									
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}
										if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)           /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                    g_srd.run_state = RUN2;                  /* ����Ϊ��������״̬ */
                }
								 
                break;								
					case RUN2:
                step_count++;                                       
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1) 
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( g_srd.step_decel1<step_count)         /* ��Ҫ��ʼ���� */
                {
                  g_srd.run_state = DECEL2;                    /* ״̬�ı�Ϊ���� */
                }   
                break; 
					case DECEL2:
                step_count++;                                         
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)         //ֱ�ӿ����
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										  g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.run_state = ACCEL3;
									  step_count=0;									  
                }
                break;
					case ACCEL3:
                step_count++;                               					 
					        if(0<step_count & step_count<=pulse_compensate-2) 	
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									  if(g_srd.step_delay<speed_rcc)
										{
											g_srd.step_delay=speed_rcc;
										}
								}								
								else if(pulse_compensate-2<step_count)
                {
                 g_srd.run_state = RUN3;
									 step_count=0;
									if(g_srd.dir == CCW )
									{        
                    ST1_DIR(CW);
									}else
									{
										ST1_DIR(CCW);
									}									
                }	
                break;													
          case RUN3:
						    step_count++;
					      g_srd.step_delay =speed_rcc;
						    if(step_count >pulse_compensate-3)
								{
									g_srd.run_state = STOP;
								}					      
						    break;
            }
						 i = 0;                                          /* ���㶨ʱ���жϴ�������ֵ */
        }								 			 
		 }				 		 
			else if	(level_speed==6)	  //��12000���壬�����������
			{
				 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;               /* ����Cֵ�������Ҫ��ת���ε�������Ҫ����2 */
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* ��ʱ���жϴ�������ֵ */
        if(i == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 				
					  switch(g_srd.run_state)                         /* �Ӽ������߽׶� */
            {
						case STOP:
                step_count = 0;                             /* ���㲽�������� */
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
							 // receive_flag=0;
							  step_count=0;							
							 // printf_flag=1;
						   // HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;
                break;

            case ACCEL:
                step_count++;                          						 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                        //��һ�μ���
								{
									 g_srd.step_delay = g_srd.step_delay-g_srd.step_accel1_delay;
									if(g_srd.step_delay<step_max)																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																													
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
									  g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;          //�ڶ��μ���
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)          /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                    g_srd.run_state = RUN;                  /* ����Ϊ��������״̬ */
                }
                break;

            case RUN:    
                step_count++;                                        
								if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2)   //�������м��ٶ�Ϊ100
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)                           /* ��Ҫ��ʼ���� */
                {
                    g_srd.run_state = DECEL;                                /* ״̬�ı�Ϊ���� */
                }
                break;

           case DECEL:
                step_count++;                                      
							if( step_all-pulse_compensate-step_decel_2<step_count & step_count<=step_all-pulse_compensate)         /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									if(g_srd.step_delay>speed_rcc)
									 {
										 g_srd.step_delay=speed_rcc;
									 }	
								}															 
								else if(step_count >step_all-pulse_compensate)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.run_state = RUN2;
                }
                break;	
					 case RUN2:
               step_count++;				
					    if(step_all-pulse_compensate<step_count & step_count<=step_all-2)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.step_delay=speed_rcc;
                }
							else if(step_count >step_all-2)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
									  g_srd.run_state = STOP;
                }
							break;
            }
						 i = 0;                                          /* ���㶨ʱ���жϴ�������ֵ */
        }				 
			 }
			else if(level_speed==7)     //�Խ��׶��ٶ���������
			{
				tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + speed_rcc/2;               /* ����Cֵ�������Ҫ��ת���ε�������Ҫ����2 */
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
				j++;                                                /* ��ʱ���жϴ�������ֵ */
        if(j == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 
         step_count++;
					if(step_count >=step_all) 
					{
					 step_count = 0;                             /* ���㲽�������� */
						/* �ر�ͨ��*/
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
						//receive_flag=0;
					//	printf_flag=1;						
						g_add_pulse_count=0;
						step_count=0;
						//HAL_TIM_Base_Stop_IT(&TIM5_Handler);
           // time_out=0;
					}
					j=0;
				}		
			}
			else if(level_speed==8)     //�����ף��м���ٹ��ף�������������36000���壬�����������
			{
				 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* ��ʱ���жϴ�������ֵ */
        if(i == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 				
					  switch(g_srd.run_state)                         /* �Ӽ������߽׶� */
            {
							case STOP:
                step_count = 0;                             /* ���㲽�������� */
                /* �ر�ͨ��*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
							 // receive_flag=0;
							 // printf_flag=1;
							  step_count=0;
              //  HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;							
                break;
            case ACCEL:     //��һ�μ��ٽ׶�                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //���е�һ�μ���
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //��������ٶ�����
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //���еڶ��μ���
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                    g_srd.run_state = RUN;                                              /* ����Ϊ��������״̬ */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //��������
								{
									g_srd.step_delay =g_srd.step_delay ;
									//��¼��һ�����е�����ٶ�
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* ��Ҫ��ʼ���� */
                {
                    g_srd.run_state = DECEL;                                             /* ״̬�ı�Ϊ���� */
                }   
                break; 
						case DECEL:    //��һ�μ���
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN2;                  /* ����Ϊ��������״̬ */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL2;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
					case DECEL2:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL3;								
									step_count=0;	                 								  
                }
                break;		
					case ACCEL3:	 //�����μ���
						 step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN4;                  /* ����Ϊ��������״̬ */
                }
						
                break;
						case RUN3:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL3;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
					case DECEL3:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = RUN4;								
									step_count=0;	                 								  
                }
                break;									
					case RUN4:
               step_count++;
					    if(step_all-pulse_compensate<step_count&step_count<step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.step_delay=speed_rcc;
                }
							else if(step_count >=step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
									  g_srd.run_state = STOP;
                }
							break;		
            }
						 i = 0;                                          /* ���㶨ʱ���жϴ�������ֵ */
        }
			}
			else if(level_speed==9)     //���Ŀף��м���ٹ��ף�������������36000���壬�����������
			{
					 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* ��ʱ���жϴ�������ֵ */
        if(i == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 				
					  switch(g_srd.run_state)                         /* �Ӽ������߽׶� */
            {
							case STOP:
                step_count = 0;                             /* ���㲽�������� */
                /* �ر�ͨ��*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
							//  receive_flag=0;
							//  printf_flag=1;
							  step_count=0;
               // HAL_TIM_Base_Stop_IT(&TIM5_Handler);
              //  time_out=0;							
                break;
            case ACCEL:     //��һ�μ��ٽ׶�                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //���е�һ�μ���
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //��������ٶ�����
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //���еڶ��μ���
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                    g_srd.run_state = RUN;                                              /* ����Ϊ��������״̬ */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //��������
								{
									g_srd.step_delay =g_srd.step_delay ;
									//��¼��һ�����е�����ٶ�
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* ��Ҫ��ʼ���� */
                {
                    g_srd.run_state = DECEL;                                             /* ״̬�ı�Ϊ���� */
                }   
                break; 
						case DECEL:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN2;                  /* ����Ϊ��������״̬ */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL2;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
					case DECEL2:
               step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL3;								
									step_count=0;	                 								  
                }
                break;		
					case ACCEL3:	 //�����μ���
						 step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN3;                  /* ����Ϊ��������״̬ */
                }
						
                break;
						case RUN3:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL3;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
						case DECEL3:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL4;								
									step_count=0;	                 								  
                }
                break;	
					case ACCEL4:
						 step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN5;                  /* ����Ϊ��������״̬ */
                }
						
                break;
					case RUN4:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL4;                /* ״̬�ı�Ϊ���� */
                }   
                break; 			
					case DECEL4:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = RUN5;								
									step_count=0;	                 								  
                }
                break;									
					case RUN5:
               step_count++;
					    if(step_all-pulse_compensate<step_count&step_count<step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.step_delay=speed_rcc;
                }
							else if(step_count >=step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
									  g_srd.run_state = STOP;
                }
							break;		
            }
						 i = 0;                                          /* ���㶨ʱ���жϴ�������ֵ */
        }
			}
			else if(level_speed==10)     //�����ף��м���ٹ��ף�����������36000���壬�����������
			{
				 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* ��ʱ���жϴ�������ֵ */
        if(i == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 				
					  switch(g_srd.run_state)                         /* �Ӽ������߽׶� */
            {
							case STOP:
                step_count = 0;                             /* ���㲽�������� */
                /* �ر�ͨ��*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
							//  receive_flag=0;
							 // printf_flag=1;
							  step_count=0;
               // HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;							
                break;
            case ACCEL:     //��һ�μ��ٽ׶�                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //���е�һ�μ���
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //��������ٶ�����
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //���еڶ��μ���
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                    g_srd.run_state = RUN;                                              /* ����Ϊ��������״̬ */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //��������
								{
									g_srd.step_delay =g_srd.step_delay ;
									//��¼��һ�����е�����ٶ�
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* ��Ҫ��ʼ���� */
                {
                    g_srd.run_state = DECEL;                                             /* ״̬�ı�Ϊ���� */
                }   
                break; 
						case DECEL:    //��һ�μ���
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN2;                  /* ����Ϊ��������״̬ */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL2;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
					case DECEL2:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL3;								
									step_count=0;	                 								  
                }
                break;		
					case ACCEL3:	 //�����μ���
						 step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN3;                  /* ����Ϊ��������״̬ */
                }
						
                break;
						case RUN3:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL3;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
					case DECEL3:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL4;								
									step_count=0;	                 								  
                }
                break;									
					
						case ACCEL4:
                step_count++;                               					 
					        if(0<step_count & step_count<=pulse_compensate-2) 	
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									  if(g_srd.step_delay<speed_rcc)
										{
											g_srd.step_delay=speed_rcc;
										}
								}								
								else if(pulse_compensate-2<step_count)
                {
                 g_srd.run_state = RUN4;
									 step_count=0;
									if(g_srd.dir == CCW )
									{        
                    ST1_DIR(CW);
									}else
									{
										ST1_DIR(CCW);
									}									
                }	
                break;													
          case RUN4:
						    step_count++;
					      g_srd.step_delay =speed_rcc;
						    if(step_count >pulse_compensate-3)
								{
									g_srd.run_state = STOP;
								}					      
						    break;		
            }
						 i = 0;                                          /* ���㶨ʱ���жϴ�������ֵ */
        }
			}
			else if(level_speed==11)     //���Ŀף��м���ٹ��ף�����������36000���壬�����������
			{
					 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* ��ʱ���жϴ�������ֵ */
        if(i == 2)                                          /* 2�Σ�˵���Ѿ����һ���������� */
        { 				
					  switch(g_srd.run_state)                         /* �Ӽ������߽׶� */
            {
							case STOP:
                step_count = 0;                             /* ���㲽�������� */
                /* �ر�ͨ��*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
							 // receive_flag=0;
							 // printf_flag=1;
							  step_count=0;
               // HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;							
                break;
            case ACCEL:     //��һ�μ��ٽ׶�                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //���е�һ�μ���
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //��������ٶ�����
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //���еڶ��μ���
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                    g_srd.run_state = RUN;                                              /* ����Ϊ��������״̬ */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //��������
								{
									g_srd.step_delay =g_srd.step_delay ;
									//��¼��һ�����е�����ٶ�
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* ��Ҫ��ʼ���� */
                {
                    g_srd.run_state = DECEL;                                             /* ״̬�ı�Ϊ���� */
                }   
                break; 
						case DECEL:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN2;                  /* ����Ϊ��������״̬ */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL2;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
					case DECEL2:
               step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL3;								
									step_count=0;	                 								  
                }
                break;		
					case ACCEL3:	 //�����μ���
						 step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN3;                  /* ����Ϊ��������״̬ */
                }
						
                break;
						case RUN3:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL3;                /* ״̬�ı�Ϊ���� */
                }   
                break; 
						case DECEL3:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //����300�ļ��ٲ�������
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* ��Ҫ��ʼ���� */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                  g_srd.run_state = ACCEL4;								
									step_count=0;	                 								  
                }
                break;	
					case ACCEL4:
						 step_count++;                               /* ������1*/						 
								 if(0<step_count & step_count<=100) //��֤�ȶ�����
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //���ټ���
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //����ٶ����һ�α���һ��
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* ����Ƿ�����Ҫ���ٵĲ��� */
                {
                  g_srd.run_state = RUN4;                  /* ����Ϊ��������״̬ */
                }
						
                break;
					case RUN4:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //�ܲ���-����������-����ٲ���
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* ��Ҫ��ʼ���� */
                {
                   g_srd.run_state = DECEL4;                /* ״̬�ı�Ϊ���� */
                }   
                break; 			
					case DECEL4:
                step_count++;                                         
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)         //ֱ�ӿ����
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //���Ƶ��ٹ��׵��ٶ�
									 {
										  g_srd.step_delay =step_pass;   
									 }
								}												
                /* ����Ƿ�Ϊ���һ�� */
								else if(step_count >step_all-1)                  /* �жϼ��ٲ����Ƿ�Ӹ�ֵ�ӵ�0�ǵĻ� ������� */
                {	
                    g_srd.run_state = ACCEL5;
									  step_count=0;									  
                }
                break;	
					case ACCEL5:
					step_count++;                               					 
					        if(0<step_count & step_count<=pulse_compensate-2) 	
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									  if(g_srd.step_delay<speed_rcc)
										{
											g_srd.step_delay=speed_rcc;
										}
								}								
								else if(pulse_compensate-2<step_count)
                {
                 g_srd.run_state = RUN5;
									 step_count=0;
									if(g_srd.dir == CCW )
									{        
                    ST1_DIR(CW);
									}else
									{
										ST1_DIR(CCW);
									}									
                }	
                break;													
          case RUN5:
						    step_count++;
					      g_srd.step_delay =speed_rcc;
						    if(step_count >pulse_compensate-3)
								{
									g_srd.run_state = STOP;
								}					      
						    break;	
            }
						 i = 0;                                          /* ���㶨ʱ���жϴ�������ֵ */
        }
			}
         __HAL_TIM_CLEAR_IT(&g_timx_pwm_chy_handle, TIM_IT_UPDATE);      /* �����ʱ������жϱ�־λ */
		}
	}

 //****************************************��С�׼��������
extern u8 zero_flag;
extern u8 zero_run;
u32 time_pass=0;
extern int time_out;
extern TIM_HandleTypeDef TIM5_Handler;      //��ʱ����� 2
extern u8 constant;
extern int level_speed;	
extern int pulse_cut;
		
 u8 CheckRtrig(uint16_t GPIO_Pin)  //�����ؼ��
	{
    static u32 Rtrig = 0;	
    if(HAL_GPIO_ReadPin(ST1_ZERO_GPIO_PORT,GPIO_Pin ) == 0)
			{
        delay_ms(20);
        if(HAL_GPIO_ReadPin(ST1_ZERO_GPIO_PORT,GPIO_Pin ) == 1)
					{
            if(Rtrig == 0 )
							{
                Rtrig = 1;
								return 1;
              }else
							{
                Rtrig = 0;
								return 0;
              }
        }else
			  {
           Rtrig = 0;
					return 0;
         }    
     }else
	    {
				Rtrig = 0;
					return 0;
			}
		}
 u8 CheckPtrig(uint16_t GPIO_Pin)  //�½��ؼ��
	{
    static u32 Rtrig = 0;	
    if(HAL_GPIO_ReadPin(ST1_ZERO_GPIO_PORT,GPIO_Pin ) == 1)
			{
        delay_ms(2);
        if(HAL_GPIO_ReadPin(ST1_ZERO_GPIO_PORT,GPIO_Pin ) == 0)
					{
            if(Rtrig == 0 )
							{
                Rtrig = 1;
								return 1;
              }else
							{
                Rtrig = 0;
								return 0;
              }
        }else
			  {
           Rtrig = 0;
					return 0;
         }    
     }else
	    {
				Rtrig = 0;
					return 0;
			}
		}	
		
	
extern u8 run_flag;
		
extern __IO uint32_t g_set_speed ;          /* ����ٶ� ��λΪ0.1rad/sec */
extern __IO uint32_t g_step_accel;            /* ���ٶ� ��λΪ0.1rad/sec^2    100*/
extern __IO uint32_t g_step_decel;            /* ���ٶ� ��λΪ0.1rad/sec^2 */
extern __IO uint16_t g_step_angle;             /* ���õĲ���*/
extern  __IO uint16_t g_step1;             /* ���õĲ���1*/
extern __IO uint16_t g_step2;             /* ���õĲ���2*/		

extern u8 time_delay_pass;	
extern int pulse_compensate;	
extern int time_delay_compensate;	
extern  uint8_t g_zero_flag;
extern  uint8_t reset_flag;	
extern u8 receive_flag;    //���Խ���ͨѶ ���н�����־
extern u8 printf_flag; 

		
extern int32_t cycles_72;		
	
//ע��level=1��������8000�����壬������
//    level=6����8000���壬����������
//		level=7�����׶�����
//    level=2����16000�� ����������
//    level=5�����׶��������������Ӽ��ٺ�����ٶ�
//    level=3����8000+����������
//    level=4����16000+����������		
void step_motor_motion_compensate(int num,int dir)   
{		
  int i=0;	
	if (dir==1)  //��������
	{
	   if(num==1)
		 {
			  level_speed=3;
			  create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
			  while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
					//	printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
						//����ת������¼���洢����
						 cycles_72=cycles_72+1;											
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
					{
				// printf("E4\r\n");   //��ͣ
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
						time_out=0;
						g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
						step_count = 0;                             /* ���㲽�������� */
						receive_flag=0;
						g_zero_flag=0;
						printf_flag=0;
						reset_flag=1;
					}
				}
		 }
		 else if(num==2)
			{
//				level_speed=4;
//			  create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
				level_speed=6;
			  create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				     while(1)
              {
								if(g_motion_sta==0)
								{
								cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								  //printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;								
									break;
								}	
              }
				delay_ms(100);
				level_speed=3;
			  create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				 while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
					//	printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
						cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
					{
				// printf("E4\r\n");   //��ͣ
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
						time_out=0;
						g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
						step_count = 0;                             /* ���㲽�������� */
						receive_flag=0;
						g_zero_flag=0;
						printf_flag=0;
						reset_flag=1;
					}
				}			
			}
			else if(num==3)
			{
//				level_speed=10;
//			  create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);				
			    level_speed=6;
			    create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				     while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;									
									break;
								}	
              }
						delay_ms(100);
						level_speed=6;
			      create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				     while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								 //printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;										
									break;
								}	
              }
							delay_ms(100);
				level_speed=3;
			  create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				 while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
					//	printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
						cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
					{
				// printf("E4\r\n");   //��ͣ
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
						time_out=0;
						g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
						step_count = 0;                             /* ���㲽�������� */
						receive_flag=0;
						g_zero_flag=0;
						printf_flag=0;
						reset_flag=1;
					}
				}			
			}
			else if(num==4)
			{
//				level_speed=11;
//			  create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
						level_speed=6;
			      create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				     while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}	
              }
							delay_ms(100);			  			
			   level_speed=6;
			    create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				     while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}	
              }
							delay_ms(100);
						level_speed=6;
			      create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				     while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}	
              }
							delay_ms(100);					
				     level_speed=3;
			      create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
							 while(1)
				      {
								if(g_motion_sta==0)
								{
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
								//	printf("time_out: %d\r\n",time_out);
									time_out=0;
									printf_flag=1;
									receive_flag=0;
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
							// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}
				}
			}
		  run_flag=0;
			
	}
	else if(dir==0)  //��������
	{		
			run_flag=0;
	}	
}	


void step_motor_motion(int num,int dir)  
{			
	int i=0;		
	if (dir==1)  //��������
	{	
		  if(num==1)
		 {
			level_speed=6;
			create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
			 while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
				//		printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
					{
				// printf("E4\r\n");   //��ͣ
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
						time_out=0;
						g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
						step_count = 0;                             /* ���㲽�������� */
						receive_flag=0;
						g_zero_flag=0;
						printf_flag=0;
						reset_flag=1;
					}
				} 
		 }
    else if(num==2)
    {
			level_speed=2;
			create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
			 while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
				//		printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+2;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
					{
				// printf("E4\r\n");   //��ͣ
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
						time_out=0;
						g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
						step_count = 0;                             /* ���㲽�������� */
						receive_flag=0;
						g_zero_flag=0;
						printf_flag=0;
						reset_flag=1;
					}
				}
		}
//		 else if(num==3)
//    {
//			level_speed=8;
//			create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
//		}
//		 else if(num==4)
//    {
//			level_speed=9;
//			create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
//		}
		  run_flag=0;
	}
	else if(dir==0)  //��������
	{
		if(num==1)
		 {
			level_speed=6;
			create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);		
			  while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
				//		printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
					{
				// printf("E4\r\n");   //��ͣ
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
						time_out=0;
						g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
						step_count = 0;                             /* ���㲽�������� */
						receive_flag=0;
						g_zero_flag=0;
						printf_flag=0;
						reset_flag=1;
					}
				}								
		 }
    else if(num==2)
    {
//			level_speed=2;
//			create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				level_speed=6;
				create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
		         while(1)
              {
								if(g_motion_sta==0)
								{	
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
							// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}	
              }
         delay_ms(100);							
		     level_speed=6;
		     create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
					 while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
			//			printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
					{
				// printf("E4\r\n");   //��ͣ
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
						time_out=0;
						g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
						step_count = 0;                             /* ���㲽�������� */
						receive_flag=0;
						g_zero_flag=0;
						printf_flag=0;
						reset_flag=1;
					}
				}		
		}
		else if(num==3)
    {
//			level_speed=8;
//			create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
					level_speed=6;
					create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
		         while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}	
              }	
							delay_ms(100);
					level_speed=6;
					create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
						while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}	
              }	
							delay_ms(100);
							level_speed=6;
							create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
							 while(1)
							{
								if(g_motion_sta==0)
								{
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
							//		printf("time_out: %d\r\n",time_out);
									time_out=0;
									printf_flag=1;
									receive_flag=0;
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
							// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}
							}
		}
		 else if(num==4)
    {
//			level_speed=9;
//			create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
				level_speed=6;
	   	   create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
		         while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}	
              }
       delay_ms(100);							
		  level_speed=6;
		  create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
						while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
									break;
								}	
              }	
							delay_ms(100);
						level_speed=6;
						create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
				   while(1)
              {
								if(g_motion_sta==0)
								{
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
								{
								// printf("E4\r\n");   //��ͣ
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
									time_out=0;
									g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
									step_count = 0;                             /* ���㲽�������� */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
									break;
								}	
              }	
							delay_ms(100);
		      level_speed=6;
		      create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
					 while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
					//	printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==1)  //Ӳ��24V����IO���ɹ���ת��ģ�齫24Vת����3.3V���뵽��Ƭ��IO��
					{
				// printf("E4\r\n");   //��ͣ
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //ֹͣ���м�ʱ
						time_out=0;
						g_motion_sta = 0;                           /* ���Ϊֹͣ״̬  */
						step_count = 0;                             /* ���㲽�������� */
						receive_flag=0;
						g_zero_flag=0;
						printf_flag=0;
						reset_flag=1;
					}
				}		
		}
		 run_flag=0;
	}	
}	


extern u8 run_flag;
extern int station_x;
void step_motor_L1(int station_y,int station_z)    //���·������
{
	switch (station_y)
	{
		/**************************����1�Ź�λ**************************/
		case 1:   //����һ�Ź�λ										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //��1 
										step_motor_motion(1,1);		
										break;							 
							 case 1:
										 //��ǰ��λ������
									run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 2:   //˳ 1													
							     step_motor_motion(1,0);
										break;							 
							 case 3:  //˳ 2									
							     step_motor_motion(2,0);
										break;							 
							 case 4:   //��ʱ����ת������λ
										step_motor_motion(2,1);
							 break; 
							}					
						}
					 station_x=1;
					 break;
	/**************************����2�Ź�λ**************************/					
			case 2:   //����2�Ź�λ					
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //��ʱ����ת2
										step_motor_motion(2,1);
										break;							 
							 case 1:  //��ʱ����ת1
									  step_motor_motion(1,1);			
										break;																	 
							 case 2:   //������			
										run_flag=0;
							       receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;						 
							 case 3:  //˳ 1
							      step_motor_motion(1,0);
										break;							 
							 case 4:   //˳ 2
							     step_motor_motion(2,0);
							 break; 
							}					
						}
					 station_x=2;
					 break;
	/**************************����3�Ź�λ**************************/	
			case 3:   //���е�3�Ź�λ										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //˳  2
							      step_motor_motion(2,0);
										break;							 
							 case 1: //��  2
										step_motor_motion(2,1);
										break;							 
							 case 2:   //��1 			
										step_motor_motion(1,1);
										break;							 
							 case 3:  //��ǰ��λ������
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 4:   //˳ʱ����ת1
							      step_motor_motion(1,0);
							 break; 
							}					
						}
					 station_x=3;
					 break;
		/**************************����4�Ź�λ**************************/						
			case 4:   //����4�Ź�λ										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //˳ 1
							     step_motor_motion(1,0);
										break;							 
							 case 1://˳2
							     step_motor_motion(2,0);
										break;							 
							 case 2:   //�� 2				
										step_motor_motion(2,1);
										break;							 
							 case 3:  //�� 1
										step_motor_motion(1,1);
										break;							 
							 case 4:   //������
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
							      break; 
							}					
						}
					 station_x=4;
					 break;
	/**************************����5�Ź�λ**************************/							
			case 5:   //����5�Ź�λ										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //������
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 1://˳1
							     step_motor_motion(1,0);
										break;							 
							 case 2:   //˳2				
							     step_motor_motion(2,0);
										break;							 
							 case 3:  //�� 2 
										step_motor_motion(2,1);
										break;							 
							 case 4:   //�� 1
										step_motor_motion(1,1);
							 break; 
							}					
						}
					 station_x=5;
					 break;
		}
}
 
void step_motor_L2(int station_y,int station_z)   //������������    ����м�ͣ��100ms  ��˳����
{
	receive_flag=1;
	switch (station_y)
	{
		/**************************����1�Ź�λ**************************/
		case 1:   //����һ�Ź�λ										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //˳ʱ����תһ����λ
										step_motor_motion(4,0);
										break;							 
							 case 1:
										 //��ǰ��λ������
									run_flag=0;
							     receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 2:   //˳ʱ����ת4				
										//step_motor_motion(1,1);
							     step_motor_motion(1,0);
										break;							 
							 case 3:  //˳ 3 
										//step_motor_motion(2,1);
							     step_motor_motion(2,0);
										break;							 
							 case 4:   //˳ʱ����ת������λ
										step_motor_motion(3,0);
							 break; 
							}					
						}
					 station_x=1;
					 break;
	/**************************����2�Ź�λ**************************/					
			case 2:   //����2�Ź�λ	
       // printf("����CASE2 \r\n");				
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //˳3
										step_motor_motion(3,0);
										break;							 
							 case 1:  //��ʱ��1
									//printf("����CASE 1 to 2 \r\n");				
							      step_motor_motion_compensate(1,1);
										break;																	 
							 case 2:   //������			
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;						 
							 case 3:  //˳ 1
										//step_motor_motion(1,1);
							      step_motor_motion(1,0);
										break;							 
							 case 4:   //˳ 2
										//step_motor_motion(2,1);
							     step_motor_motion(2,0);
							 break; 
							}					
						}
					  station_x=2;
					 break;
	/**************************����3�Ź�λ**************************/	
			case 3:   //���е�3�Ź�λ										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //˳ 2
										//step_motor_motion(2,1);
							      step_motor_motion(2,0);
										break;							 
							 case 1: //��  2
										step_motor_motion_compensate(2,1);
										break;							 
							 case 2:   //�� 1				
										step_motor_motion_compensate(1,1);
										break;							 
							 case 3:  //��ǰ��λ������
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 4:   //˳1 
										//step_motor_motion(1,1);
							      step_motor_motion(1,0);
							 break; 
							}					
						}
					  station_x=3;
					 break;
		/**************************����4�Ź�λ**************************/						
			case 4:   //����4�Ź�λ										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //˳1
										//step_motor_motion(1,1);
							     step_motor_motion(1,0);
										break;							 
							 case 1://��ʱ����ת3
										// step_motor_motion(2,1);
							     step_motor_motion_compensate(3,1);
										break;							 
							 case 2:   //��2			
										step_motor_motion_compensate(2,1);
										break;							 
							 case 3:  //��1
										step_motor_motion_compensate(1,1);
										break;							 
							 case 4:   //������
										run_flag=0;
							       receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
							      break; 
							}					
						}
					  station_x=4;
					 break;
	/**************************����5�Ź�λ**************************/							
			case 5:   //����5�Ź�λ										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //������
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 1://��ʱ��1
										//step_motor_motion(1,1);
							     step_motor_motion_compensate(4,1);
										break;							 
							 case 2:   //��3				
										//step_motor_motion(2,1);
							    step_motor_motion_compensate(3,1);
										break;							 
							 case 3:  //��2
										step_motor_motion_compensate(2,1);
										break;							 
							 case 4:   //��1
										step_motor_motion_compensate(1,1);
							 break; 
							}					
						}
					  station_x=5;
					 break;
		}
}

void motor_run_all(int loop,int station_to) 
{
	  int j4=0;	           
		int  j5=1;						 
		 for(j5=1;j5<=5;j5++)   //��λѭ��
		 {
			 if(station_to==j5)
			 {
				 //����ڵ�ǰ��λ������ִ��
			 }else
			 {
			   for ( j4=0; j4<=loop;j4++)  //��λ����
			   {							 
				 //�Զ����Գ���
				   step_motor_L1(j5,station_to);  //
				 //�ȴ����н���
					 while(1)
					{
						if(g_motion_sta==0)
						{
							break;
						}
					}
				 delay_ms(500);
				 step_motor_L1(station_to,j5);
				//�ȴ����н���
					 while(1)
					{
						if(g_motion_sta==0)
						{
							break;
						}
					}
				 delay_ms(1000);
			 }
		  }
	 }	
}

	


/* 霍尔读取 (5次采样多数表决, 调试用) */
static int read_single_hal(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, int weight)
{
    int sum = 0;
    int i;
    for (i = 0; i < 5; i++) {
        sum += HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
        delay_ms(10);
    }
    return (sum >= 3) ? weight : 0;
}

int read_hal(void)
{
    int hal = 0;
    hal += read_single_hal(ST1_HAL1_GPIO_PORT, ST1_HAL1_GPIO_PIN, 1);
    hal += read_single_hal(ST1_HAL2_GPIO_PORT, ST1_HAL2_GPIO_PIN, 2);
    hal += read_single_hal(ST1_HAL3_GPIO_PORT, ST1_HAL3_GPIO_PIN, 4);
    switch (hal) {
        case 2: return 1;
        case 4: return 2;
        case 3: return 3;
        case 0: return 4;
        case 5: case 1: return 5;
        default: return -1;
    }
}

/* 含RTC时间戳的运动完成打印 (替代原 printf_flag 分支中的简单打印) */
void motion_done_with_ts(int st)
{
    char ts[32];
    rtc_get_time_str(ts, sizeof(ts));
    printf("[%s] P%d cycles=%d\r\n", ts, st, total_cycles);
}
