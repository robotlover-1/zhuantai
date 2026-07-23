/**
 ****************************************************************************************************
 * @file        atim.h
 * @brief       定时器配置头文件 (平式步进电机模式)
 *
 * 定时器汇总（STM32F407, HSE=8MHz, SYSCLK=168MHz, APB1=42MHz×2=84MHz, APB2=84MHz×2=168MHz）
 *
 * TIM1 (APB2: 168MHz) - PA8(CH1 步进脉冲OC), PA11 方向GPIO
 *                        atim_timx_pwm_chy_init(0xFFFF, 36-1) -> 168M/36=4.67MHz 计数基准
 *
 * TIM4 (APB1: 84MHz)  - 磨合计时(time_run)
 *                        TIM4_Init(5000-1, 16800-1) -> 84MHz/16800/5000 = 1Hz(1s)
 *
 * TIM5 (APB1: 84MHz)  - 超时/堵转检测(time_out)
 *                        TIM5_Init(5000-1, 8400-1) -> 84MHz/8400/5000 = 2Hz(0.5s)
 ****************************************************************************************************
 */
#ifndef __ATIM_H
#define __ATIM_H

#include "sys.h"
#include "stm32f4xx_hal_tim.h"

/* TIM1 步进脉冲输出配置 (PA8, CH1, Output Compare) */
#define ATIM_TIMX_PWM_CHY_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CHY_GPIO_PIN             GPIO_PIN_8
#define ATIM_TIMX_PWM_CHY_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define ATIM_TIMX_PWM                          TIM1
#define ATIM_TIMX_PWM_CHY                      TIM_CHANNEL_1
#define ATIM_TIMX_INT_IRQn                     TIM1_CC_IRQn
#define ATIM_TIMX_INT_IRQHandler               TIM1_CC_IRQHandler
#define ATIM_TIMX_PWM_CHY_CLK_ENABLE()         do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)

extern TIM_HandleTypeDef g_timx_pwm_chy_handle;
extern TIM_HandleTypeDef TIM4_Handler;
extern TIM_HandleTypeDef TIM5_Handler;

/* 外部变量 */
extern u8 time_out;
extern u8 time_run;

/* 函数声明 */
void atim_timx_pwm_chy_init(uint16_t arr, uint16_t psc);  /* TIM1_CH1 OC 步进脉冲 */
void TIM4_Init(u16 arr, u16 psc);                         /* 磨合计时 */
void TIM5_Init(u16 arr, u16 psc);                         /* 超时检测 */
#endif
