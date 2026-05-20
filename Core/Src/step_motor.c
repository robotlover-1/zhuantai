#include "step_motor.h"
#include "atim.h"
#include "app_util.h"
#include "math.h"
#include "delay.h"
#include "pid.h"
#include "exti.h"
#include "key.h"
#include "stdio.h"

extern TIM_HandleTypeDef g_timx_pwm_chy_handle;
extern TIM_HandleTypeDef g_timx_encode_chy_handle;
extern float speed_m;	  
extern float location;	  
extern int32_t motor_pwm; /* 电机PWM值 */
extern int32_t pluse_ele;

void step_motor_init(void)
{
	GPIO_InitTypeDef gpio_init_struct;
	ST1_SHUTDOWN_GPIO_CLK_ENABLE();
	ST1_ELE_GPIO_CLK_ENABLE();
	ST1_IO_GPIO_CLK_ENABLE();
	ST1_HAL1_GPIO_CLK_ENABLE();
	ST1_HAL2_GPIO_CLK_ENABLE();
	ST1_HAL3_GPIO_CLK_ENABLE();
	ST1_IN1_GPIO_CLK_ENABLE();
	ST1_IN2_GPIO_CLK_ENABLE();
	ST1_HOME_GPIO_CLK_ENABLE();
	ST1_DONE_GPIO_CLK_ENABLE();
	ST1_ALARM_GPIO_CLK_ENABLE();
	gpio_init_struct.Pin = ST1_SHUTDOWN_GPIO_PIN; 
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;  /* 开漏输出(接24V) */
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;									  
	HAL_GPIO_Init(ST1_SHUTDOWN_GPIO_PORT, &gpio_init_struct);						  
	HAL_GPIO_WritePin(ST1_SHUTDOWN_GPIO_PORT, ST1_SHUTDOWN_GPIO_PIN, GPIO_PIN_RESET); 
	gpio_init_struct.Pin = ST1_ELE_GPIO_PIN;			 
	gpio_init_struct.Mode = GPIO_MODE_INPUT;			 
	gpio_init_struct.Pull = GPIO_PULLDOWN;				 
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;		 
	HAL_GPIO_Init(ST1_ELE_GPIO_PORT, &gpio_init_struct); 
	gpio_init_struct.Pin = ST1_HAL1_GPIO_PIN;			  
	gpio_init_struct.Mode = GPIO_MODE_INPUT;			  
	gpio_init_struct.Pull = GPIO_PULLDOWN;				  
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;		  
	HAL_GPIO_Init(ST1_HAL1_GPIO_PORT, &gpio_init_struct); 
	gpio_init_struct.Pin = ST1_HAL2_GPIO_PIN;			  
	gpio_init_struct.Mode = GPIO_MODE_INPUT;			  
	gpio_init_struct.Pull = GPIO_PULLDOWN;				  
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;		  
	HAL_GPIO_Init(ST1_HAL2_GPIO_PORT, &gpio_init_struct); 
	gpio_init_struct.Pin = ST1_HAL3_GPIO_PIN;			  
	gpio_init_struct.Mode = GPIO_MODE_INPUT;			  
	gpio_init_struct.Pull = GPIO_NOPULL;				  
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;		  
	HAL_GPIO_Init(ST1_HAL3_GPIO_PORT, &gpio_init_struct); 

	/* PF9: 平式光电信号输入(双向光耦) - 输入模式 */
	gpio_init_struct.Pin = ST1_IO_GPIO_PIN;		
	gpio_init_struct.Mode = GPIO_MODE_INPUT;		
	gpio_init_struct.Pull = GPIO_PULLDOWN;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;		
	HAL_GPIO_Init(ST1_IO_GPIO_PORT, &gpio_init_struct); 
	dcmotor_stop(); 
	gpio_init_struct.Pin = ST1_IN1_GPIO_PIN | ST1_IN2_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_INPUT;
	gpio_init_struct.Pull = GPIO_PULLDOWN;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(ST1_IN1_GPIO_PORT, &gpio_init_struct);
	// 平式步进回零完成信号输入(双向光耦) PF8
	gpio_init_struct.Pin = ST1_HOME_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_INPUT;
	gpio_init_struct.Pull = GPIO_PULLDOWN;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(ST1_HOME_GPIO_PORT, &gpio_init_struct);
	gpio_init_struct.Pin = ST1_DONE_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;  /* 推挽输出(3.3V) */
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(ST1_DONE_GPIO_PORT, &gpio_init_struct);
	gpio_init_struct.Pin = ST1_ALARM_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;  /* 推挽输出(3.3V) */
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(ST1_ALARM_GPIO_PORT, &gpio_init_struct);

	/* PC5: 拍照信号输出(3.3V推挽) */
	ST1_PHOTO_GPIO_CLK_ENABLE();
	gpio_init_struct.Pin = ST1_PHOTO_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(ST1_PHOTO_GPIO_PORT, &gpio_init_struct);
}

extern u8 run_printf_flag;
extern u8 run_flag;
extern u8 printf_flag;
extern int time_delay; 
extern int station_x;  
extern int error;
extern int DIR;	  
extern int start; 
extern u8 time_out;
extern TIM_HandleTypeDef TIM5_Handler; 
extern int32_t Encode_now;
extern int out;
extern int Ring;
extern int Ding;
extern int pulse_add;
extern int32_t force;
extern float SetPoint_P; 
extern float SetPoint_C; 
extern int duzhuan_flag;
float SetPluse;

extern int alarm;						 

void step_motor_motion(int num, int dir) 
{
	int hal_x = 0;
	int hal_y = 0;
	hal_x = read_hal(); 
	if (error == 1)
	{
		printf("电机初始化失败!\r\n");
		return;
	}

	if (hal_x == 1 || hal_x == 2 || hal_x == 3 || hal_x == 4 || hal_x == 5)
	{
		printf("当前位置:%d\r\n", hal_x);
	}

	if (run_printf_flag == 0 && error == 0 && run_flag == 0) 
	{
		if (ele_is_triggered()) 
		{
			if (dir == 0)  /* 正转 */
			{
				DIR = 0;
				motor_start_motion();
				SetPoint_C = Encode_now;
				SetPoint_P = Encode_now + 39200;
				run_flag = 1;
				delay_ms(500);
				while (1)
				{
					if (Encode_now > SetPoint_C + pluse_ele)
					{
						if (ele_is_triggered())
						{
							Ring = 0;
							SetPluse = Encode_now - SetPoint_C;
							motor_stop_normal();
							delay_ms(500);
							if (Encode_now > SetPoint_C + 1000)
								error = 0;
							else
							{
								error = 1;
								alarm = 3;
								printf("光电检测异常!\r\n");
								alarm_pulse();
							}

							break;
						}
					}

					if (Encode_now > SetPoint_P + 5000)
					{
						motor_safe_stop();
						error = 1;
						alarm = 10;
						printf("堵转超时!\r\n");
						SetPluse = Encode_now - SetPoint_C;
						printf("脉冲数2:%f\r\n", SetPluse);
						alarm_pulse();
						break;
					}

					if (check_motion_timeout(4, "电机超限!")) break;
					if (error == 1) { motor_safe_stop(); error = 1; break; }
					if (check_24v_loss()) break;
				}

				if (error == 1) return;

				/* 光电到达后验证工位 */
				if (ele_is_triggered())
				{
					printf_flag = 1;
					run_flag = 0;
					run_printf_flag = 0;
					hal_y = read_hal();
					handle_station_verify(hal_x, hal_y);
				}

				else
				{

					/* 堵转到达但光电未触发，需调整 */
					if (Encode_now < SetPoint_P - 300)
					{
						printf("当前位置:%d\r\n", Encode_now);
						delay_ms(100);
						motor_start_motion();
						run_flag = 2;
						Ring = 0;
						while (1)
						{
							if (Encode_now > SetPoint_C + pluse_ele)
							{
								if (ele_is_triggered())
								{
									Ring = 0;
									SetPluse = Encode_now - SetPoint_C;
									motor_stop_normal();
									break;
								}
							}

							if (check_motion_timeout(5, "运行超时!")) break;
							if (error == 1) { motor_safe_stop(); error = 1; break; }
							if (check_24v_loss()) break;
						}
					}

					else if (Encode_now > SetPoint_P + 300)
					{
						printf("当前位置:%d\r\n", Encode_now);
						delay_ms(100);
						motor_start_motion();
						run_flag = 3;
						Ring = 0;
						while (1)
						{
							if (ele_is_triggered())
							{
								Ring = 0;
								SetPluse = Encode_now - SetPoint_C;
								motor_stop_normal();
								break;
							}

							if (check_motion_timeout(6, "运行超时!")) break;
							if (error == 1) { motor_safe_stop(); error = 1; break; }
							if (check_24v_loss()) break;
						}
					}

					if (error == 1) return;
					delay_ms(500);
					if (ele_is_triggered())
					{
						printf_flag = 1;
						run_flag = 0;
						run_printf_flag = 0;
						printf("到达位置!\r\n");
						hal_y = read_hal();
						handle_station_verify(hal_x, hal_y);
					}

					else
					{
						error = 1;
						alarm = 7;
						printf("工位到达失败!\r\n");
						alarm_pulse();
					}
				}
			}

			else if (dir == 1)  /* 反转 */
			{
				DIR = 1;
				motor_start_motion();
				SetPoint_C = Encode_now;
				SetPoint_P = Encode_now - 39200;
				run_flag = 1;
				delay_ms(500);
				while (1)
				{
					if (Encode_now < SetPoint_C - pluse_ele)
					{
						if (ele_is_triggered())
						{
							Ring = 0;
							SetPluse = Encode_now - SetPoint_C;
							motor_stop_normal();
							delay_ms(500);
							if (Encode_now < SetPoint_C - 1000)
								error = 0;
							else
							{
								error = 1;
								alarm = 3;
								printf("光电检测异常!\r\n");
								alarm_pulse();
							}

							break;
						}
					}

					if (Encode_now < SetPoint_P - 5000)
					{
						motor_safe_stop();
						error = 1;
						alarm = 10;
						printf("反转堵转超时!\r\n");
						SetPluse = Encode_now - SetPoint_C;
						printf("脉冲数3:%f\r\n", SetPluse);
						alarm_pulse();
						break;
					}

					if (check_motion_timeout(4, "电机超限!")) break;
					if (error == 1) { motor_safe_stop(); error = 1; break; }
					if (check_24v_loss()) break;
				}

				if (error == 1) return;

				/* 光电到达后验证工位 */
				if (ele_is_triggered())
				{
					printf_flag = 1;
					run_flag = 0;
					run_printf_flag = 0;
					hal_y = read_hal();
					handle_station_verify(hal_x, hal_y);
				}

				else
				{

					/* 堵转到达但光电未触发，需调整 */
					if (Encode_now > SetPoint_P + 300)
					{
						printf("当前位置:%d\r\n", Encode_now);
						delay_ms(100);
						motor_start_motion();
						run_flag = 3;
						Ring = 0;
						while (1)
						{
							if (Encode_now < SetPoint_C - pluse_ele)
							{
								if (ele_is_triggered())
								{
									Ring = 0;
									SetPluse = Encode_now - SetPoint_C;
									motor_stop_normal();
									break;
								}
							}

							if (check_motion_timeout(5, "运行超时!")) break;
							if (error == 1) { motor_safe_stop(); error = 1; break; }
							if (check_24v_loss()) break;
						}
					}

					else if (Encode_now < SetPoint_P - 300)
					{
						printf("当前位置:%d\r\n", Encode_now);
						delay_ms(100);
						motor_start_motion();
						run_flag = 2;
						Ring = 0;
						while (1)
						{
							if (ele_is_triggered())
							{
								Ring = 0;
								SetPluse = Encode_now - SetPoint_C;
								motor_stop_normal();
								break;
							}

							if (check_motion_timeout(6, "运行超时!")) break;
							if (error == 1) { motor_safe_stop(); error = 1; break; }
							if (check_24v_loss()) break;
						}
					}

					if (error == 1) return;
					delay_ms(500);
					if (ele_is_triggered())
					{
						printf_flag = 1;
						run_flag = 0;
						run_printf_flag = 0;
						printf("到达位置!\r\n");
						hal_y = read_hal();
						handle_station_verify(hal_x, hal_y);
					}

					else
					{
						printf_flag = 0;
						error = 1;
						alarm = 7;
						printf("工位到达失败!\r\n");
						alarm_pulse();
					}
				}
			}
		}

		else
		{
			printf("光电异常,工位检测失败!\r\n");
			alarm_pulse();
		} 
	}
}

void motor_init(void)
{
	if (HAL_GPIO_ReadPin(ST1_ELE_GPIO_PORT, ST1_ELE_GPIO_PIN) == 0) 
	{
		if (error == 1)
		{
			printf("电机初始化中,请稍候!\r\n");
			return;
		}

		else if (error == 0)
		{
			if (run_printf_flag == 0) 
			{
				motor_start_motion();
				run_flag = 2;
				delay_ms(20);
				while (1) 
				{

				/* 找光电下降沿: 从不在孔内(1)变为在孔内(0) */
					if (HAL_GPIO_ReadPin(ST1_ELE_GPIO_PORT, ST1_ELE_GPIO_PIN) == 1)
					{
						delay_ms(10);
						if (HAL_GPIO_ReadPin(ST1_ELE_GPIO_PORT, ST1_ELE_GPIO_PIN) == 0)
						{
							Ring = 0;
							motor_stop_normal();
							break;
						}
					}

					if (error == 1) break;
					if (check_24v_loss()) break;
				}

				if (error == 1) return;
				delay_ms(300);
				if (ele_is_triggered())
				{
					printf_flag = 1;
					run_flag = 0;
					run_printf_flag = 0;
				}

				else
				{

					/* 第一次未找到, 再试一次 */
					motor_start_motion();
					run_flag = 2;
					delay_ms(20);
					while (1) 
					{
						if (ele_is_triggered())
						{
							Ring = 0;
							motor_stop_normal();
							break;
						}

						if (error == 1) break;
						if (check_24v_loss()) break;
					}

					if (error == 1) return;
					delay_ms(300);
					if (ele_is_triggered())
					{
						printf_flag = 1;
						run_flag = 0;
						run_printf_flag = 0;
					}

					else
					{
						error = 1;
						run_flag = 0;
						run_printf_flag = 0;
						alarm = 8;
						printf("电机初始化失败,请检查硬件!\r\n");
						alarm_pulse();
					}
				}
			}
		}
	}

	else if (HAL_GPIO_ReadPin(ST1_ELE_GPIO_PORT, ST1_ELE_GPIO_PIN) == 1)
	{
		printf_flag = 1;
		run_flag = 0;
		run_printf_flag = 0;
	}
}

void set_speed(int32_t compare)
{
	__HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY, compare); 
}

/**
 * @brief       读取单个霍尔引脚（3次采样多数表决消抖）
 * @param       GPIOx: 端口基址
 * @param       GPIO_Pin: 引脚号
 * @param       weight: 权重值(1/2/4)
 * @retval      0-引脚为低, weight-引脚为高
 * @note        连续采样3次取多数, 抵抗PC13等弱驱动引脚的浮动
 */
static int read_single_hal(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, int weight)
{
    int sum = 0;
    int i;
    for (i = 0; i < 5; i++) {
        sum += HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
        delay_ms(10);
    }
    /* 5次中至少3次为高才认定为高电平 */
    return (sum >= 3) ? weight : 0;
}

/**
 * @brief       读取霍尔工位
 * @retval      1-5 有效工位, -1 错误
 * @note        霍尔信号组合: HAL1(weight=1) + HAL2(weight=2) + HAL3(weight=4)
 *              7->5映射, 其他无效值返回-1
 */

int read_hal(void)
{
    int hal = 0;
    int retry = 0;

    /* 先检测光电传感器是否触发 */
    if (HAL_GPIO_ReadPin(ST1_ELE_GPIO_PORT, ST1_ELE_GPIO_PIN) == 1)
    {
        error = 1;
        alarm = 1;
        printf("光电传感器未触发 \r\n");
        alarm_pulse();
        return -1;
    }
    /* 最多重试2次读取霍尔 */
    for (retry = 0; retry < 2; retry++)
    {
        hal = 0;
        hal += read_single_hal(ST1_HAL1_GPIO_PORT, ST1_HAL1_GPIO_PIN, 1);
        hal += read_single_hal(ST1_HAL2_GPIO_PORT, ST1_HAL2_GPIO_PIN, 2);
        hal += read_single_hal(ST1_HAL3_GPIO_PORT, ST1_HAL3_GPIO_PIN, 4);
        /* 映射表: 实测权重 -> 工位号
         *   权重2 (0,1,0) → 工位1
         *   权重4 (0,0,1) → 工位2
         *   权重3 (1,1,0) → 工位3
         *   权重5 (1,0,1) → 工位4
         *   权重1 (1,0,0) → 工位5
         *   权重7 (1,1,1) → 工位5 (兼容旧项目 7→5)
         */
        switch (hal)
        {
			case 0: return 4;
            case 2: return 1;
            case 4: return 2;
            case 3: return 3;
            case 5: return 5;
			case 1: return 5;
            //case 7: return 5;
        }
        /* 无效值，延时后重试 */
        delay_ms(50);
    }
    /* 重试2次仍无效 */
    alarm = 2;
    printf("霍尔读取错误:%d\r\n", hal);
    alarm_pulse();
    return hal;
}

extern int DIR;								 

int dir_jud(int target_num, int current_num) 
{

    /* 5站圆形排列，计算正转步数(递增方向) */
    int fwd = (target_num - current_num + 5) % 5;
    if (fwd <= 2) {
        DIR = 0;  /* 正转步数 ≤ 2，走正转 */
        return 0;
    } else {
        DIR = 1;  /* 正转步数 > 2，走反转才最短 */
        return 1;
    }
}

void step_motor_LX(int target_num, int dir)
{
	int i = 0;
	// for(i=0;i<5;i++)
	while (1)
	{
		i++;
		if (i > 5)
		{
			break;
		}

		if (error == 0)
		{
			step_motor_motion(1, dir);
			while (1)
			{
				if (run_printf_flag == 0)
				{
					break;
				}
			}

			delay_ms(50);
			station_x = read_hal();
			if (target_num == station_x) 
			{
				break;
			}
		}

		else
		{
			printf("堵转超时!\r\n");
			break;
		}
	}
}

void dcmotor_start(void)
{
	HAL_GPIO_WritePin(ST1_SHUTDOWN_GPIO_PORT, ST1_SHUTDOWN_GPIO_PIN, GPIO_PIN_SET); 
}

/**
 * @brief       停止电机
 * @param       无
 * @retval      无
 */

void dcmotor_stop(void)
{

    /* 先设0%，再停通道，最后关闭主输出 */
	/* Start两个通道设0%，确保引脚被主动拉低而非悬浮 */
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, TIM_CHANNEL_4, 0);
    // HAL_TIM_PWM_Stop(&g_timx_pwm_chy_handle, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Stop(&g_timx_pwm_chy_handle, TIM_CHANNEL_4);
}

/**
 * @brief       设置电机方向
 * @param       para: 方向 0=正转(PA8 CH1) 1=反转(PA11 CH4)
 */

void dcmotor_dir(uint8_t para)
{

    /* 两个通道都Start确保引脚被主动驱动 */
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, TIM_CHANNEL_4, 0);
}

/**
 * @brief       设置PWM占空比
 * @param       para: PWM比较值, 正转时设置正转通道,反转时设置反转通道
 * @note        正数正转,负数反转,PWM值为绝对值
 * @retval      无
 */

void motor_pwm_set(int32_t para)
{
    if (para >= 0)
    {
        __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, TIM_CHANNEL_1, para);
        __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, TIM_CHANNEL_4, 0);
    }

    else if (para < 0)
    {
        para = -para;
        __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, TIM_CHANNEL_4, para);
    }
}
/*************************************    编码器测速    速度计算    ****************************************************/
ENCODE_TypeDef g_encode; 

extern float speed_m;

void speed_computer(int32_t encode_now, uint8_t ms)
{
	uint8_t i = 0, j = 0;
	float temp = 0.0;
	static uint8_t sp_count = 0, k = 0;
	static float speed_arr[10] = {0.0}; 
	if (sp_count == ms) 
	{
		/* 速度计算说明
		 * 编码器: 编码器线数 * 减速比 = 每圈脉冲数
		 * 速度: 1min内的脉冲数 = g_encode.speed * ((1000 / ms) * 60)
		 * 单位: 脉冲数/分钟 (RPM换算需要除以编码器分辨率)
		 * 滤波: 冒泡排序后取中间6个值做平均
		 */
		g_encode.encode_now = encode_now;							  
		g_encode.speed = (g_encode.encode_now - g_encode.encode_old); 
		speed_arr[k++] = g_encode.speed;
		speed_m = g_encode.speed;
		g_encode.encode_old = g_encode.encode_now; 
		if (k == 10)
		{
			for (i = 10; i >= 1; i--) 
			{
				for (j = 0; j < (i - 1); j++)
				{
					if (speed_arr[j] > speed_arr[j + 1]) 
					{
						temp = speed_arr[j]; 
						speed_arr[j] = speed_arr[j + 1];
						speed_arr[j + 1] = temp;
					}
				}
			}

			temp = 0.0;
			for (i = 2; i < 8; i++) 
			{
				temp += speed_arr[i]; 
			}

			temp = (float)(temp / 6); 
			/* 一阶低通滤波
			 * 公式:Y(n)= qX(n) + (1-q)Y(n-1)
			 * 其中X(n)为本次采样值,Y(n-1)为上次滤波输出,Y(n)为本次滤波输出,q为滤波系数
			 * q越小滤波效果越强,但响应越慢;q越大响应越快,但滤波效果越弱
			 */
			speed_m = (float)(((float)0.48 * temp) + (speed_m * (float)0.52));
			k = 0;
		}

		sp_count = 0;
	}

	sp_count++;
}
