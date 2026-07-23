#include "step_motor.h"
#include "atim.h"
#include "rtc.h"
#include <stdio.h>
#include "math.h"
#include "delay.h"

 STEPPER_MOTOR step = {0};  

extern u8 receive_flag;   //可以接收通讯 运行结束标志


void step_motor_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    ST1_DIR_GPIO_CLK_ENABLE();           /* PA11 方向 */
    ST1_EN_GPIO_CLK_ENABLE();            /* PC1 使能 */
    ST1_ZERO_GPIO_CLK_ENABLE();          /* PC0 回零气缸 */
    ST1_ZERO1_GPIO_CLK_ENABLE();         /* PF8 回零完成检测 */
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

    ST1_EN(0);  /* 初始释放使能 */
}
speedRampData g_srd               = {STOP,CW,0,0,0,0,0};  /* 加减速变量 */
__IO int32_t  g_step_position     = 0;                    /* 当前位置 */
__IO uint8_t  g_motion_sta        = 0;                    /* 是否在运动？0：停止，1：运动 */
__IO uint32_t g_add_pulse_count   = 0;                    /* 脉冲个数累计 */
       
extern int32_t step_all;	    //运行总步数
extern int32_t step_decel_2;  //减速进孔阶段，第一次减速
extern int32_t step_accel_2;  //快速加速阶段，第二次加速
extern	int32_t step_max;     //最大运行速度限制
extern	int32_t step_max_1;     //最大运行速度限制

//给定步数，加速度，减速度，计算加减速的步数大小 step,accel,decel,speed
//step:总步数
//step1:出孔脉冲数，出孔步数
//step2：进孔脉冲数，进孔步数
//accel：出孔加速度
//decel：进孔加速度
//speed：最高运行速度
	void create_t_ctrl_param(int32_t step,int32_t step1, int32_t accel, int32_t step2,int32_t decel, int32_t speed)
{
	if(step==0)
		{
			return;
		}
	if(g_motion_sta != STOP)        /* 只允许步进电机在停止的时候才继续*/
        return;
	//***************计算参数*********************************//
	  //当脉冲数大于10000，即走两个工位20000
//	   if(step>=10000 && step>0)   //当运行脉冲数为16000，即两个工位
//		 {
//			 step=10000;
//		 }
//		 else if(step<=-10000 && step<0) //当运行脉冲数为-16000，两个工位
//		 {
//			 step=-10000;
//		 }		    
	   if(step<0)
			{
				step_all=-step;
				g_srd.step_decel1=step_all-step2-step_decel_2;            //减速1步数
        g_srd.step_decel2=step_all-step2;        //减速2步数
			}else
			{
				step_all=step;
				g_srd.step_decel1=step_all-step2-step_decel_2;            //减速1步数
        g_srd.step_decel2=step_all-step2;        //减速2步数
			}
			
		 step_max=12000/speed;  //最大运行速度时，delay的计算
	   g_srd.step_accel1=0;           //开始加速1步数 
     g_srd.step_accel2=step1;        //加速2步数
	   g_srd.step_run=step1+step_accel_2;               //匀速运行步数
					      
    if(step < 0)                    /* 步数为负数 */
    {   
        g_srd.dir = CCW;            /* 逆时针方向旋转 0*/
        ST1_DIR(CCW);
        step = -step-1;               /* 获取步数绝对值 */
    }
    else
    {
        g_srd.dir = CW;             /* 顺时针方向旋转 1*/
        ST1_DIR(CW);
			  step = step-1;
    }
   if(step != 0)              /* 如果目标运动步数不为0*/
    {
         g_srd.step_delay=6220;   //初速度4.5度每秒
		}
    g_motion_sta = 1;                                           /* 电机为运动状态 */
	//	receive_flag=1;
		
		/*****计算快速加减速时的加减速******/
	
		g_srd.step_accel1_delay=accel;    //出孔加速度
		g_srd.step_decel1_delay=decel;    //进孔加速度
		
		g_srd.step_accel2_delay=(2666-step_max-step1*accel)/step_accel_2;  
		g_srd.step_decel2_delay=(2666-step_max-step2*decel)/step_decel_2;
		
		//计算限制，防止为负值
		if(2666-step_max-step1*accel<0)
		{
			g_srd.step_accel2_delay=10;
		}
		
		if(2666-step_max-step2*decel<0)
		{
			g_srd.step_decel2_delay=10;
		}
		
		g_srd.run_state = ACCEL;
		
     HAL_TIM_OC_Start_IT(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY);                                 /* 使能定时器通道 */
}



/**
  * @brief  定时器比较中断
  * @param  htim：定时器句柄指针
  * @note   无
  * @retval 无
  */

extern  int32_t step_count; 
extern  u8 printf_flag;
extern  int time_out;
extern int step_pass;                //过孔速度
extern int32_t pulse_compensate;     //进孔脉冲数
extern int speed_rcc;                //进孔速度
extern int level_speed;              //不用leve下的运行情况
//运行轨迹选择，level=0、1、2、3、4、5、6、7
//level=0：18度每秒匀速运动
//level=1：运行8000脉冲  完整分段加减速
//level=2：走16000脉冲，中间低速过孔，最后进孔匀速运行   走pluse_compensate   **************
//level=3： 反向脉冲补偿 一个孔位运行  即运行8000+pulse_compensate  再运行-pulse_compensate;  **********
//level=4： 补偿脉冲运行两个工位2      运行16000+pulse_compensate   再运行-pulse_compensate;  **********
//level=5： 进孔段脉冲数，包含100加速，100减速
//level=6： 走8000，最后进孔匀速运行  走pluse_compensate    *********
//level=7:  以进孔段速度匀速运行
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{    
    uint32_t tim_count = 0;
    uint32_t tmp = 0;
    static uint8_t i = 0;                              /* 定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲 */
	  static uint8_t j = 0;  
	    
    if(htim->Instance==TIM8)
    {		
			if(level_speed==0)
			{
				tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + 1000/2;      //18度每秒
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
				j++;                                                /* 定时器中断次数计数值 */
        if(j == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 
         step_count++;
					if(step_count >=step_all) 
					{
						   step_count = 0;                             						
							HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
							g_motion_sta = 0;                           /* 电机为停止状态  */
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
			else if	(level_speed==1)	  //一个工位，到孔停，不补偿 ，即运行12000脉冲 T型
			{
		    tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;    //初速度4.5度每秒           
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 				
					  switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
							case STOP:
                step_count = 0;                                        
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* 电机为停止状态  */
							  //receive_flag=0;
							 // printf_flag=1;
							  step_count=0;	
						  //	HAL_TIM_Base_Stop_IT(&TIM5_Handler);
              // time_out=0;
                break;
							
            case ACCEL:
                step_count++;                               					  
								if(0<step_count & step_count<=g_srd.step_accel2)   //第一段加速运行	
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //第一次加速 delay改变
									 if(g_srd.step_delay<step_max)  //设置最大速度限制
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)//第二次加速
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay; //第二次加速 delay改变
									
									 if(g_srd.step_delay<step_max)  //设置最大速度限制
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)          //进入匀速运行
                {
                    g_srd.run_state = RUN;                  /* 设置为匀速运行状态 */
                }
                break;
								
            case RUN:             
                step_count++;                               /* 步数加1 */              
								if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)  //匀速运行
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( g_srd.step_decel1<step_count)         /* 需要开始减速 */
                {
                    g_srd.run_state = DECEL;                /* 状态改变为减速 */
                }
                break;

            case DECEL:
                step_count++;                               /* 步数加1 */                          
								if( g_srd.step_decel1<step_count & step_count<=g_srd.step_decel2)         //第一次减速
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;         
								}
								else if( g_srd.step_decel2<step_count & step_count<=step_all-2)           //第二次减速
								{    
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_accel1_delay;
								}              
								else if(step_count >step_all-2)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                    g_srd.run_state = STOP;
                }
                break;
            }
						 i = 0;                                          /* 清零定时器中断次数计数值 */
        }				
			}
			else if	(level_speed==2)//跨两孔时，中间低速过孔，不补偿  即运行24000脉冲，最后脉冲匀速运行
			{
		    tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 				
					  switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
							case STOP:
                step_count = 0;                             /* 清零步数计数器 */
                /* 关闭通道*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* 电机为停止状态  */
							//  receive_flag=0;
							 // printf_flag=1;
							  step_count=0;
              //  HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;							
                break;
            case ACCEL:     //第一次加速阶段                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //运行第一段加速
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //设置最大速度限制
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //运行第二段加速
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* 检查是否到了需要匀速的步数 */
                {
                    g_srd.run_state = RUN;                                              /* 设置为匀速运行状态 */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //匀速运行
								{
									g_srd.step_delay =g_srd.step_delay ;
									                                 //记录第一段运行的最大速度
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* 需要开始减速 */
                {
                    g_srd.run_state = DECEL;                                             /* 状态改变为减速 */
                }   
                break; 
						case DECEL:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔               /* 增大过孔的脉冲，保证过孔速度较低*/
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN2;                  /* 设置为匀速运行状态 */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL2;                /* 状态改变为减速 */
                }   
                break; 
					case DECEL2:
                step_count++;                                      
							if( step_all-pulse_compensate-step_decel_2<step_count & step_count<=step_all-pulse_compensate)         /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									if(g_srd.step_delay>speed_rcc)
									 {
										 g_srd.step_delay=speed_rcc;
									 }	
								}															 
								else if(step_count >step_all-pulse_compensate)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                    g_srd.run_state = RUN3;
                }
                break;	
					case RUN3:
               step_count++;
					    if(step_all-pulse_compensate<step_count&step_count<step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                    g_srd.step_delay=speed_rcc;
                }
							else if(step_count >=step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
									  g_srd.run_state = STOP;
                }
							break;		
            }
						 i = 0;                                          /* 清零定时器中断次数计数值 */
        }				
			}
			 else if	(level_speed==3)	  //反向脉冲补偿 一个孔位运行  即运行12000+pulse_compensate
			{	
				tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 				
					  switch(g_srd.run_state)                        
            {
							case STOP:
                step_count = 0;                           
                /* 关闭通道*/
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
								if(0<step_count & step_count<=g_srd.step_accel2) 	                        //第一次加速阶段   
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)        //第二段加速
                {
									  g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN;                   /* 设置为匀速运行状态 */
                }
                break;
            case RUN:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1) 
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( g_srd.step_decel1<step_count)         /* 需要开始减速 */
                {
                  g_srd.run_state = DECEL;                      /* 状态改变为减速 */
                }   
                break; 
						case DECEL:
                step_count++;                             
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)         /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										  g_srd.step_delay =step_pass;   
									 }
								}												              
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
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
								else if( pulse_compensate-100<step_count)         /* 需要开始减速 */
                {
                  g_srd.run_state = DECEL2;                /* 状态改变为减速 */
                }   
                break; 
					case DECEL2:
                step_count++;                                          
							if( pulse_compensate-100<step_count & step_count<=pulse_compensate-2)         /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
								}
                /* 检查是否为最后一步 */
								else if(step_count >pulse_compensate-2)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
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
						 i = 0;                                          /* 清零定时器中断次数计数值 */
        }							 
			 }
			else if	(level_speed==4)	  //补偿脉冲运行两个工位2    运行24000+pulse_compensate
			{
       tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 				
					  switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
							case STOP:
                step_count = 0;                             /* 清零步数计数器 */
                /* 关闭通道*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* 电机为停止状态  */
							//  receive_flag=0;
							//  printf_flag=1;
							  step_count=0;	
              //  HAL_TIM_Base_Stop_IT(&TIM5_Handler);
              //  time_out=0;														
                break;
            case ACCEL:     //第一次加速阶段                       
                step_count++;                             					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //设置最大速度限制
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)         /* 检查是否到了需要匀速的步数 */
                {
									  g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)          /* 检查是否到了需要匀速的步数 */
                {
                    g_srd.run_state = RUN;                  /* 设置为匀速运行状态 */
                }
                break;
            case RUN:
                step_count++;  
						     g_srd.step_decel1=step_all-300;  //设置500的减速步数过孔
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1) 
								{
									g_srd.step_delay =g_srd.step_delay ;
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL;                    /* 状态改变为减速 */
                }   
                break; 
						case DECEL:
                step_count++;
                g_srd.step_decel1=step_all-300;  //设置500的减速步数过孔						
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)         /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										  g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                    g_srd.run_state = ACCEL2;
									 // g_srd.step_delay =4000;
									  step_count=0;									  
                }
                break;									
						case ACCEL2:        
                step_count++; 
						   if(0<step_count & step_count<=100) //保证稳定过孔      /*增大过孔脉冲，保证低速进孔*/
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
								else if(100<step_count & step_count<=g_srd.step_run) 	  //直接快加速
								{									
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}
										if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)           /* 检查是否到了需要匀速的步数 */
                {
                    g_srd.run_state = RUN2;                  /* 设置为匀速运行状态 */
                }
								 
                break;								
					case RUN2:
                step_count++;                                       
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1) 
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( g_srd.step_decel1<step_count)         /* 需要开始减速 */
                {
                  g_srd.run_state = DECEL2;                    /* 状态改变为减速 */
                }   
                break; 
					case DECEL2:
                step_count++;                                         
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)         //直接快减速
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										  g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
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
						 i = 0;                                          /* 清零定时器中断次数计数值 */
        }								 			 
		 }				 		 
			else if	(level_speed==6)	  //走12000脉冲，最后匀速运行
			{
				 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;               /* 整个C值里边是需要翻转两次的所以需要除以2 */
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 				
					  switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
						case STOP:
                step_count = 0;                             /* 清零步数计数器 */
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* 电机为停止状态  */
							 // receive_flag=0;
							  step_count=0;							
							 // printf_flag=1;
						   // HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;
                break;

            case ACCEL:
                step_count++;                          						 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                        //第一段加速
								{
									 g_srd.step_delay = g_srd.step_delay-g_srd.step_accel1_delay;
									if(g_srd.step_delay<step_max)																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																													
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)         /* 检查是否到了需要匀速的步数 */
                {
									  g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;          //第二段加速
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)          /* 检查是否到了需要匀速的步数 */
                {
                    g_srd.run_state = RUN;                  /* 设置为匀速运行状态 */
                }
                break;

            case RUN:    
                step_count++;                                        
								if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2)   //设置运行减速段为100
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)                           /* 需要开始减速 */
                {
                    g_srd.run_state = DECEL;                                /* 状态改变为减速 */
                }
                break;

           case DECEL:
                step_count++;                                      
							if( step_all-pulse_compensate-step_decel_2<step_count & step_count<=step_all-pulse_compensate)         /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									if(g_srd.step_delay>speed_rcc)
									 {
										 g_srd.step_delay=speed_rcc;
									 }	
								}															 
								else if(step_count >step_all-pulse_compensate)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                    g_srd.run_state = RUN2;
                }
                break;	
					 case RUN2:
               step_count++;				
					    if(step_all-pulse_compensate<step_count & step_count<=step_all-2)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                    g_srd.step_delay=speed_rcc;
                }
							else if(step_count >step_all-2)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
									  g_srd.run_state = STOP;
                }
							break;
            }
						 i = 0;                                          /* 清零定时器中断次数计数值 */
        }				 
			 }
			else if(level_speed==7)     //以进孔段速度匀速运行
			{
				tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + speed_rcc/2;               /* 整个C值里边是需要翻转两次的所以需要除以2 */
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
				j++;                                                /* 定时器中断次数计数值 */
        if(j == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 
         step_count++;
					if(step_count >=step_all) 
					{
					 step_count = 0;                             /* 清零步数计数器 */
						/* 关闭通道*/
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						g_motion_sta = 0;                           /* 电机为停止状态  */
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
			else if(level_speed==8)     //跨三孔，中间低速过孔，不补偿，运行36000脉冲，最后匀速运行
			{
				 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 				
					  switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
							case STOP:
                step_count = 0;                             /* 清零步数计数器 */
                /* 关闭通道*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* 电机为停止状态  */
							 // receive_flag=0;
							 // printf_flag=1;
							  step_count=0;
              //  HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;							
                break;
            case ACCEL:     //第一次加速阶段                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //运行第一段加速
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //设置最大速度限制
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //运行第二段加速
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* 检查是否到了需要匀速的步数 */
                {
                    g_srd.run_state = RUN;                                              /* 设置为匀速运行状态 */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //匀速运行
								{
									g_srd.step_delay =g_srd.step_delay ;
									//记录第一段运行的最大速度
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* 需要开始减速 */
                {
                    g_srd.run_state = DECEL;                                             /* 状态改变为减速 */
                }   
                break; 
						case DECEL:    //第一次减速
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN2;                  /* 设置为匀速运行状态 */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL2;                /* 状态改变为减速 */
                }   
                break; 
					case DECEL2:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL3;								
									step_count=0;	                 								  
                }
                break;		
					case ACCEL3:	 //第三段加速
						 step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN4;                  /* 设置为匀速运行状态 */
                }
						
                break;
						case RUN3:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL3;                /* 状态改变为减速 */
                }   
                break; 
					case DECEL3:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = RUN4;								
									step_count=0;	                 								  
                }
                break;									
					case RUN4:
               step_count++;
					    if(step_all-pulse_compensate<step_count&step_count<step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                    g_srd.step_delay=speed_rcc;
                }
							else if(step_count >=step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
									  g_srd.run_state = STOP;
                }
							break;		
            }
						 i = 0;                                          /* 清零定时器中断次数计数值 */
        }
			}
			else if(level_speed==9)     //跨四孔，中间低速过孔，不补偿，运行36000脉冲，最后匀速运行
			{
					 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 				
					  switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
							case STOP:
                step_count = 0;                             /* 清零步数计数器 */
                /* 关闭通道*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* 电机为停止状态  */
							//  receive_flag=0;
							//  printf_flag=1;
							  step_count=0;
               // HAL_TIM_Base_Stop_IT(&TIM5_Handler);
              //  time_out=0;							
                break;
            case ACCEL:     //第一次加速阶段                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //运行第一段加速
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //设置最大速度限制
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //运行第二段加速
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* 检查是否到了需要匀速的步数 */
                {
                    g_srd.run_state = RUN;                                              /* 设置为匀速运行状态 */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //匀速运行
								{
									g_srd.step_delay =g_srd.step_delay ;
									//记录第一段运行的最大速度
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* 需要开始减速 */
                {
                    g_srd.run_state = DECEL;                                             /* 状态改变为减速 */
                }   
                break; 
						case DECEL:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN2;                  /* 设置为匀速运行状态 */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL2;                /* 状态改变为减速 */
                }   
                break; 
					case DECEL2:
               step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL3;								
									step_count=0;	                 								  
                }
                break;		
					case ACCEL3:	 //第三段加速
						 step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN3;                  /* 设置为匀速运行状态 */
                }
						
                break;
						case RUN3:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL3;                /* 状态改变为减速 */
                }   
                break; 
						case DECEL3:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL4;								
									step_count=0;	                 								  
                }
                break;	
					case ACCEL4:
						 step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN5;                  /* 设置为匀速运行状态 */
                }
						
                break;
					case RUN4:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL4;                /* 状态改变为减速 */
                }   
                break; 			
					case DECEL4:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = RUN5;								
									step_count=0;	                 								  
                }
                break;									
					case RUN5:
               step_count++;
					    if(step_all-pulse_compensate<step_count&step_count<step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                    g_srd.step_delay=speed_rcc;
                }
							else if(step_count >=step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
									  g_srd.run_state = STOP;
                }
							break;		
            }
						 i = 0;                                          /* 清零定时器中断次数计数值 */
        }
			}
			else if(level_speed==10)     //跨三孔，中间低速过孔，补偿，运行36000脉冲，最后匀速运行
			{
				 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 				
					  switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
							case STOP:
                step_count = 0;                             /* 清零步数计数器 */
                /* 关闭通道*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* 电机为停止状态  */
							//  receive_flag=0;
							 // printf_flag=1;
							  step_count=0;
               // HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;							
                break;
            case ACCEL:     //第一次加速阶段                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //运行第一段加速
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //设置最大速度限制
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //运行第二段加速
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* 检查是否到了需要匀速的步数 */
                {
                    g_srd.run_state = RUN;                                              /* 设置为匀速运行状态 */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //匀速运行
								{
									g_srd.step_delay =g_srd.step_delay ;
									//记录第一段运行的最大速度
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* 需要开始减速 */
                {
                    g_srd.run_state = DECEL;                                             /* 状态改变为减速 */
                }   
                break; 
						case DECEL:    //第一次减速
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN2;                  /* 设置为匀速运行状态 */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL2;                /* 状态改变为减速 */
                }   
                break; 
					case DECEL2:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL3;								
									step_count=0;	                 								  
                }
                break;		
					case ACCEL3:	 //第三段加速
						 step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN3;                  /* 设置为匀速运行状态 */
                }
						
                break;
						case RUN3:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL3;                /* 状态改变为减速 */
                }   
                break; 
					case DECEL3:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
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
						 i = 0;                                          /* 清零定时器中断次数计数值 */
        }
			}
			else if(level_speed==11)     //跨四孔，中间低速过孔，补偿，运行36000脉冲，最后匀速运行
			{
					 tim_count = __HAL_TIM_GET_COUNTER(&g_timx_pwm_chy_handle);
			  tmp = tim_count + g_srd.step_delay/2;             
       __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle,ATIM_TIMX_PWM_CHY,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        { 				
					  switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
							case STOP:
                step_count = 0;                             /* 清零步数计数器 */
                /* 关闭通道*/
                HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
                g_motion_sta = 0;                           /* 电机为停止状态  */
							 // receive_flag=0;
							 // printf_flag=1;
							  step_count=0;
               // HAL_TIM_Base_Stop_IT(&TIM5_Handler);
               // time_out=0;							
                break;
            case ACCEL:     //第一次加速阶段                      
                step_count++;                               					 
								if(0<step_count & step_count<=g_srd.step_accel2) 	                     //运行第一段加速
								{
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay;
									 if(g_srd.step_delay<step_max)  //设置最大速度限制
										{
											g_srd.step_delay=step_max;
										}
								}
								else if(g_srd.step_accel2<step_count & step_count<=g_srd.step_run)     //运行第二段加速
                {
									 g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel2_delay;
									if(g_srd.step_delay<step_max)
										{
											g_srd.step_delay=step_max;
										}
                }
								else if(g_srd.step_run<step_count)                                      /* 检查是否到了需要匀速的步数 */
                {
                    g_srd.run_state = RUN;                                              /* 设置为匀速运行状态 */	
                }
                break;
            case RUN:
                step_count++;  
						    g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
               if( g_srd.step_run<step_count & step_count<=g_srd.step_decel1)            //匀速运行
								{
									g_srd.step_delay =g_srd.step_delay ;
									//记录第一段运行的最大速度
									step_max_1=g_srd.step_delay;
								}
								else if( g_srd.step_decel1<step_count)                                   /* 需要开始减速 */
                {
                    g_srd.run_state = DECEL;                                             /* 状态改变为减速 */
                }   
                break; 
						case DECEL:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL2;								
									step_count=0;	                 								  
                }
                break;									
						case ACCEL2:
                step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN2;                  /* 设置为匀速运行状态 */
                }
						
                break;								
					 case RUN2:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL2;                /* 状态改变为减速 */
                }   
                break; 
					case DECEL2:
               step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL3;								
									step_count=0;	                 								  
                }
                break;		
					case ACCEL3:	 //第三段加速
						 step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN3;                  /* 设置为匀速运行状态 */
                }
						
                break;
						case RUN3:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL3;                /* 状态改变为减速 */
                }   
                break; 
						case DECEL3:
                step_count++;
						   g_srd.step_decel1=step_all-300;  //设置300的减速步数过孔
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)                 /* 需要开始减速 */
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										 g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {	
                  g_srd.run_state = ACCEL4;								
									step_count=0;	                 								  
                }
                break;	
					case ACCEL4:
						 step_count++;                               /* 步数加1*/						 
								 if(0<step_count & step_count<=100) //保证稳定过孔
						     {
									 g_srd.step_delay = g_srd.step_delay;
								 }
						    else if(100<step_count & step_count<=g_srd.step_run) 	
								{
									g_srd.step_delay =  g_srd.step_delay-g_srd.step_accel1_delay; //快速加速
//									if(g_srd.step_delay<step_max)
//										{
//											g_srd.step_delay=step_max;
//										}									
									if(g_srd.step_delay<step_max_1)  //最大速度与第一段保持一致
										{
											g_srd.step_delay=step_max_1;
									  }
								}								
								else if(g_srd.step_run<step_count)         /* 检查是否到了需要匀速的步数 */
                {
                  g_srd.run_state = RUN4;                  /* 设置为匀速运行状态 */
                }
						
                break;
					case RUN4:
                step_count++;                                          
               if( g_srd.step_run<step_count & step_count<=step_all-pulse_compensate-step_decel_2) //总步数-进孔脉冲数-快减速步数
								{
									g_srd.step_delay =g_srd.step_delay ;
								}
								else if( step_all-pulse_compensate-step_decel_2<step_count)         /* 需要开始减速 */
                {
                   g_srd.run_state = DECEL4;                /* 状态改变为减速 */
                }   
                break; 			
					case DECEL4:
                step_count++;                                         
							if( g_srd.step_decel1<step_count & step_count<=step_all-1)         //直接快减速
								{
									 g_srd.step_delay =  g_srd.step_delay+g_srd.step_decel2_delay;
									 if( g_srd.step_delay>step_pass)  //限制低速过孔的速度
									 {
										  g_srd.step_delay =step_pass;   
									 }
								}												
                /* 检查是否为最后一步 */
								else if(step_count >step_all-1)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
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
						 i = 0;                                          /* 清零定时器中断次数计数值 */
        }
			}
         __HAL_TIM_CLEAR_IT(&g_timx_pwm_chy_handle, TIM_IT_UPDATE);      /* 清除定时器溢出中断标志位 */
		}
	}

 //****************************************大小孔检测回零程序
extern u8 zero_flag;
extern u8 zero_run;
u32 time_pass=0;
extern int time_out;
extern TIM_HandleTypeDef TIM5_Handler;      //定时器句柄 2
extern u8 constant;
extern int level_speed;	
extern int pulse_cut;
		
 u8 CheckRtrig(uint16_t GPIO_Pin)  //上升沿检测
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
 u8 CheckPtrig(uint16_t GPIO_Pin)  //下降沿检测
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
		
extern __IO uint32_t g_set_speed ;          /* 最大速度 单位为0.1rad/sec */
extern __IO uint32_t g_step_accel;            /* 加速度 单位为0.1rad/sec^2    100*/
extern __IO uint32_t g_step_decel;            /* 减速度 单位为0.1rad/sec^2 */
extern __IO uint16_t g_step_angle;             /* 设置的步数*/
extern  __IO uint16_t g_step1;             /* 设置的步数1*/
extern __IO uint16_t g_step2;             /* 设置的步数2*/		

extern u8 time_delay_pass;	
extern int pulse_compensate;	
extern int time_delay_compensate;	
extern  uint8_t g_zero_flag;
extern  uint8_t reset_flag;	
extern u8 receive_flag;    //可以接收通讯 运行结束标志
extern u8 printf_flag; 

		
extern int32_t cycles_72;		
	
//注：level=1，正常走8000个脉冲，不补偿
//    level=6，走8000脉冲，最后进孔匀速
//		level=7，进孔段脉冲
//    level=2，走16000， 最后进孔匀速
//    level=5，进孔段脉冲数，包含加减速和最高速度
//    level=3，走8000+进孔脉冲数
//    level=4，走16000+进孔脉冲数		
void step_motor_motion_compensate(int num,int dir)   
{		
	if (dir==1)  //反向运行
	{
	   if(num==1)
		 {
			  level_speed=3;
			  create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);	
			  while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
					//	printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
						//将旋转次数记录并存储下来
						 cycles_72=cycles_72+1;											
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
					{
				// printf("E4\r\n");   //急停
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
						time_out=0;
						g_motion_sta = 0;                           /* 电机为停止状态  */
						step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								  //printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
					//	printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
						cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
					{
				// printf("E4\r\n");   //急停
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
						time_out=0;
						g_motion_sta = 0;                           /* 电机为停止状态  */
						step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								 //printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
					//	printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
						cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
					{
				// printf("E4\r\n");   //急停
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
						time_out=0;
						g_motion_sta = 0;                           /* 电机为停止状态  */
						step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
								//	printf("time_out: %d\r\n",time_out);
									time_out=0;
									printf_flag=1;
									receive_flag=0;
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
							// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
									receive_flag=0;
									g_zero_flag=0;
									printf_flag=0;
									reset_flag=1;
								}
				}
			}
		  run_flag=0;
			
	}
	else if(dir==0)  //反向运行
	{		
			run_flag=0;
	}	
}	


void step_motor_motion(int num,int dir)  
{			
	if (dir==1)  //反向运行
	{	
		  if(num==1)
		 {
			level_speed=6;
			create_t_ctrl_param(g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);
			 while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
				//		printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
					{
				// printf("E4\r\n");   //急停
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
						time_out=0;
						g_motion_sta = 0;                           /* 电机为停止状态  */
						step_count = 0;                             /* 清零步数计数器 */
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
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
				//		printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+2;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
					{
				// printf("E4\r\n");   //急停
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
						time_out=0;
						g_motion_sta = 0;                           /* 电机为停止状态  */
						step_count = 0;                             /* 清零步数计数器 */
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
	else if(dir==0)  //正向运行
	{
		if(num==1)
		 {
			level_speed=6;
			create_t_ctrl_param(-g_step_angle,g_step1,g_step_accel,g_step2,g_step_decel,g_set_speed);		
			  while(1)
				{
					if(g_motion_sta==0)
					{
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
				//		printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
					{
				// printf("E4\r\n");   //急停
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
						time_out=0;
						g_motion_sta = 0;                           /* 电机为停止状态  */
						step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
							// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
			//			printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
					{
				// printf("E4\r\n");   //急停
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
						time_out=0;
						g_motion_sta = 0;                           /* 电机为停止状态  */
						step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
							//		printf("time_out: %d\r\n",time_out);
									time_out=0;
									printf_flag=1;
									receive_flag=0;
										cycles_72=cycles_72+1;	
									break;
								}
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
							// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
								if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
								{
								// printf("E4\r\n");   //急停
									HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
									HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
									time_out=0;
									g_motion_sta = 0;                           /* 电机为停止状态  */
									step_count = 0;                             /* 清零步数计数器 */
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
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
					//	printf("time_out: %d\r\n",time_out);
						time_out=0;
						printf_flag=1;
						receive_flag=0;
							cycles_72=cycles_72+1;	
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1)  //硬件24V接收IO，由光耦转换模块将24V转换成3.3V输入到单片机IO中
					{
				// printf("E4\r\n");   //急停
						HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
						HAL_TIM_Base_Stop_IT(&TIM5_Handler);   //停止运行计时
						time_out=0;
						g_motion_sta = 0;                           /* 电机为停止状态  */
						step_count = 0;                             /* 清零步数计数器 */
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
void step_motor_L1(int station_y,int station_z)    //最短路径运行
{
	switch (station_y)
	{
		/**************************运行1号工位**************************/
		case 1:   //运行一号工位										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //逆1 
										step_motor_motion(1,1);		
										break;							 
							 case 1:
										 //当前工位不运行
									run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 2:   //顺 1													
							     step_motor_motion(1,0);
										break;							 
							 case 3:  //顺 2									
							     step_motor_motion(2,0);
										break;							 
							 case 4:   //逆时针旋转二个工位
										step_motor_motion(2,1);
							 break; 
							}					
						}
					 station_x=1;
					 break;
	/**************************运行2号工位**************************/					
			case 2:   //运行2号工位					
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //逆时针旋转2
										step_motor_motion(2,1);
										break;							 
							 case 1:  //逆时针旋转1
									  step_motor_motion(1,1);			
										break;																	 
							 case 2:   //不运行			
										run_flag=0;
							       receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;						 
							 case 3:  //顺 1
							      step_motor_motion(1,0);
										break;							 
							 case 4:   //顺 2
							     step_motor_motion(2,0);
							 break; 
							}					
						}
					 station_x=2;
					 break;
	/**************************运行3号工位**************************/	
			case 3:   //运行到3号工位										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //顺  2
							      step_motor_motion(2,0);
										break;							 
							 case 1: //逆  2
										step_motor_motion(2,1);
										break;							 
							 case 2:   //逆1 			
										step_motor_motion(1,1);
										break;							 
							 case 3:  //当前工位不运行
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 4:   //顺时针旋转1
							      step_motor_motion(1,0);
							 break; 
							}					
						}
					 station_x=3;
					 break;
		/**************************运行4号工位**************************/						
			case 4:   //运行4号工位										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //顺 1
							     step_motor_motion(1,0);
										break;							 
							 case 1://顺2
							     step_motor_motion(2,0);
										break;							 
							 case 2:   //逆 2				
										step_motor_motion(2,1);
										break;							 
							 case 3:  //逆 1
										step_motor_motion(1,1);
										break;							 
							 case 4:   //不运行
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
	/**************************运行5号工位**************************/							
			case 5:   //运行5号工位										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //不运行
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 1://顺1
							     step_motor_motion(1,0);
										break;							 
							 case 2:   //顺2				
							     step_motor_motion(2,0);
										break;							 
							 case 3:  //逆 2 
										step_motor_motion(2,1);
										break;							 
							 case 4:   //逆 1
										step_motor_motion(1,1);
							 break; 
							}					
						}
					 station_x=5;
					 break;
		}
}
 
void step_motor_L2(int station_y,int station_z)   //补偿脉冲运行    跨孔中间停顿100ms  高顺低逆
{
	receive_flag=1;
	switch (station_y)
	{
		/**************************运行1号工位**************************/
		case 1:   //运行一号工位										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //顺时针旋转一个工位
										step_motor_motion(4,0);
										break;							 
							 case 1:
										 //当前工位不运行
									run_flag=0;
							     receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 2:   //顺时针旋转4				
										//step_motor_motion(1,1);
							     step_motor_motion(1,0);
										break;							 
							 case 3:  //顺 3 
										//step_motor_motion(2,1);
							     step_motor_motion(2,0);
										break;							 
							 case 4:   //顺时针旋转二个工位
										step_motor_motion(3,0);
							 break; 
							}					
						}
					 station_x=1;
					 break;
	/**************************运行2号工位**************************/					
			case 2:   //运行2号工位	
       // printf("进入CASE2 \r\n");				
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //顺3
										step_motor_motion(3,0);
										break;							 
							 case 1:  //逆时针1
									//printf("进入CASE 1 to 2 \r\n");				
							      step_motor_motion_compensate(1,1);
										break;																	 
							 case 2:   //不运行			
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;						 
							 case 3:  //顺 1
										//step_motor_motion(1,1);
							      step_motor_motion(1,0);
										break;							 
							 case 4:   //顺 2
										//step_motor_motion(2,1);
							     step_motor_motion(2,0);
							 break; 
							}					
						}
					  station_x=2;
					 break;
	/**************************运行3号工位**************************/	
			case 3:   //运行到3号工位										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //顺 2
										//step_motor_motion(2,1);
							      step_motor_motion(2,0);
										break;							 
							 case 1: //逆  2
										step_motor_motion_compensate(2,1);
										break;							 
							 case 2:   //逆 1				
										step_motor_motion_compensate(1,1);
										break;							 
							 case 3:  //当前工位不运行
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 4:   //顺1 
										//step_motor_motion(1,1);
							      step_motor_motion(1,0);
							 break; 
							}					
						}
					  station_x=3;
					 break;
		/**************************运行4号工位**************************/						
			case 4:   //运行4号工位										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //顺1
										//step_motor_motion(1,1);
							     step_motor_motion(1,0);
										break;							 
							 case 1://逆时针旋转3
										// step_motor_motion(2,1);
							     step_motor_motion_compensate(3,1);
										break;							 
							 case 2:   //逆2			
										step_motor_motion_compensate(2,1);
										break;							 
							 case 3:  //逆1
										step_motor_motion_compensate(1,1);
										break;							 
							 case 4:   //不运行
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
	/**************************运行5号工位**************************/							
			case 5:   //运行5号工位										
				if(run_flag==0)
					 {						 
						 run_flag=1;
							 switch (station_z)
						 {
							 case 5:   //不运行
										run_flag=0;
							      receive_flag=0;
							      printf_flag=1;
							      step_count=0;
                    HAL_TIM_Base_Stop_IT(&TIM5_Handler);
                    time_out=0;	
										break;							 
							 case 1://逆时针1
										//step_motor_motion(1,1);
							     step_motor_motion_compensate(4,1);
										break;							 
							 case 2:   //逆3				
										//step_motor_motion(2,1);
							    step_motor_motion_compensate(3,1);
										break;							 
							 case 3:  //逆2
										step_motor_motion_compensate(2,1);
										break;							 
							 case 4:   //逆1
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
		 for(j5=1;j5<=5;j5++)   //工位循环
		 {
			 if(station_to==j5)
			 {
				 //如果在当前工位，程序不执行
			 }else
			 {
			   for ( j4=0; j4<=loop;j4++)  //工位往返
			   {							 
				 //自动测试程序
				   step_motor_L1(j5,station_to);  //
				 //等待运行接收
					 while(1)
					{
						if(g_motion_sta==0)
						{
							break;
						}
					}
				 delay_ms(500);
				 step_motor_L1(station_to,j5);
				//等待运行接收
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

/* 含RTC时间戳的运动完成打印 */
void motion_done_with_ts(int st)
{
    char ts[32];
    rtc_get_time_str(ts, sizeof(ts));
    printf("[%s] P%d cycles=%d\r\n", ts, st, total_cycles);
}
