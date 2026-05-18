/**
 ****************************************************************************************************
 
 * @brief       步进电机控制相关
 ****************************************************************************************************
 * @attention
 ****************************************************************************************************
 */
#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include "sys.h"

/******************************************************************************************/
/* GPIO端口宏（兼容旧代码） */
#define PC  GPIOC
#define PE  GPIOE
#define PF  GPIOF

/******************************************************************************************/
/* STM32F407 引脚定义 */

/* 平式步进回零信号输出(开漏) */
#define ST1_SHUTDOWN_GPIO_PORT                  PC
#define ST1_SHUTDOWN_GPIO_PIN                   GPIO_PIN_1
#define ST1_SHUTDOWN_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* 球式光电信号输入 */
#define ST1_ELE_GPIO_PORT                       PE
#define ST1_ELE_GPIO_PIN                        GPIO_PIN_4
#define ST1_ELE_GPIO_CLK_ENABLE()               do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

/* 平式光电信号输入(双向光耦) */
#define ST1_IO_GPIO_PORT                        PF
#define ST1_IO_GPIO_PIN                         GPIO_PIN_9
#define ST1_IO_GPIO_CLK_ENABLE()                do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* 球式霍尔信号输入 1/2/3 */
#define ST1_HAL1_GPIO_PORT                      PE
#define ST1_HAL1_GPIO_PIN                       GPIO_PIN_5
#define ST1_HAL1_GPIO_CLK_ENABLE()              do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

#define ST1_HAL2_GPIO_PORT                      PE
#define ST1_HAL2_GPIO_PIN                       GPIO_PIN_6
#define ST1_HAL2_GPIO_CLK_ENABLE()              do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

#define ST1_HAL3_GPIO_PORT                      PC
#define ST1_HAL3_GPIO_PIN                       GPIO_PIN_13
#define ST1_HAL3_GPIO_CLK_ENABLE()              do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* 24V信号输入1(双向光耦) */
#define ST1_IN1_GPIO_PORT                       PF
#define ST1_IN1_GPIO_PIN                        GPIO_PIN_6
#define ST1_IN1_GPIO_CLK_ENABLE()               do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* 24V信号输入2(双向光耦) */
#define ST1_IN2_GPIO_PORT                       PF
#define ST1_IN2_GPIO_PIN                        GPIO_PIN_7
#define ST1_IN2_GPIO_CLK_ENABLE()               do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* 平式步进回零完成信号输入(双向光耦) */
#define ST1_HOME_GPIO_PORT                      PF
#define ST1_HOME_GPIO_PIN                       GPIO_PIN_8
#define ST1_HOME_GPIO_CLK_ENABLE()              do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* 到位信号输出(3.3V推挽) */
#define ST1_DONE_GPIO_PORT                      PC
#define ST1_DONE_GPIO_PIN                       GPIO_PIN_3
#define ST1_DONE_GPIO_CLK_ENABLE()              do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* 报警信号输出(3.3V推挽) */
#define ST1_ALARM_GPIO_PORT                     PC
#define ST1_ALARM_GPIO_PIN                      GPIO_PIN_4
#define ST1_ALARM_GPIO_CLK_ENABLE()             do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* 拍照信号输出(3.3V推挽) */
#define ST1_PHOTO_GPIO_PORT                     PC
#define ST1_PHOTO_GPIO_PIN                      GPIO_PIN_5
#define ST1_PHOTO_GPIO_CLK_ENABLE()             do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)


#define ROTO_RATIO      44  /* 电机*编码系数，11*4=44 */
#define REDUCTION_RATIO 30  /* 减速比30:1 */


/******************************************************************************************/

/* 外部接口函数*/
void step_motor_init(void);
void dcmotor_start(void);               /* 启动电机 */
void dcmotor_stop(void);                /* 关闭电机 */  
void dcmotor_dir(uint8_t para);         /* 设置电机方向 */
void motor_pwm_set(int32_t para);         /* 设置PWM */
void step_motor_motion(int num, int dir);
void motor_init(void);
void step_motor_LX(int target_num,int dir);

void set_speed(int32_t compare);
int read_hal(void);
int dir_jud(int target_num,int current_num);   //判断转动方向
void speed_computer(int32_t encode_now, uint8_t ms);/* 电机速度计算 */
#endif
