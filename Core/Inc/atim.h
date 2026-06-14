/**
 ****************************************************************************************************
 * @file        atim.h
 * @brief       定时器相关头文件
 *
 * 定时器汇总（STM32F407, HSE=8MHz, SYSCLK=168MHz, APB1=42MHz×2=84MHz, APB2=84MHz×2=168MHz）
 *
 * TIM1 (APB2: 168MHz) - PA8(CH1正转PWM), PA11(CH4反转PWM)
 *                       atim_timx_pwm_chy_init(3600-1, 10) -> ~4.24kHz 电机PWM驱动
 *
 * TIM3 (APB1: 84MHz)  - PA6(CH1编码器A相), PA7(CH2编码器B相)
 *                       gtim_timx_encoder_chy_init(0xFFFF, 0) -> 84MHz 编码器位置计数
 *
 * TIM4 (APB1: 84MHz)  - 磨合计时(time_run)
 *                       TIM4_Init(5000-1, 16800-1) -> 84MHz/16800/5000 = 1Hz(1s中断)
 *
 * TIM5 (APB1: 84MHz)  - 超时/堵转检测(time_out)
 *                       TIM5_Init(5000-1, 8400-1) -> 84MHz/8400/5000 = 2Hz(0.5s中断)
 *
 * TIM6 (APB1: 84MHz)  - PID控制周期
 *                       btim_timx_int_init(1000-1, 84-1) -> 84MHz/84/1000 = 1ms
 ****************************************************************************************************
 */
#ifndef __ATIM_H
#define __ATIM_H

#include "sys.h"
#include "stm32f4xx_hal_tim.h"

#define ATIM_TIMX_PWM_CHY_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CHY_GPIO_PIN             GPIO_PIN_8
#define ATIM_TIMX_PWM_CHY_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define ATIM_TIMX_PWM                          TIM1
#define ATIM_TIMX_PWM_CHY                      TIM_CHANNEL_1
#define ATIM_TIMX_INT_IRQn                     TIM1_UP_TIM10_IRQn
#define ATIM_TIMX_INT_IRQHandler               TIM1_UP_TIM10_IRQHandler
#define ATIM_TIMX_PWM_CHY_CLK_ENABLE()         do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)
#define GTIM_TIMX_ENCODER                      TIM3
#define GTIM_TIMX_ENCODER_INT_IRQn             TIM3_IRQn
#define GTIM_TIMX_ENCODER_INT_IRQHandler       TIM3_IRQHandler
#define GTIM_TIMX_ENCODER_CH1                  TIM_CHANNEL_1
#define GTIM_TIMX_ENCODER_CH1_CLK_ENABLE()     do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)
#define GTIM_TIMX_ENCODER_CH2                  TIM_CHANNEL_2
#define GTIM_TIMX_ENCODER_CH2_CLK_ENABLE()     do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)
#define BTIM_TIMX_INT                          TIM6
#define BTIM_TIMX_INT_IRQn                     TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler               TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()             do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)

typedef struct
{
    int32_t encode_old;
    int32_t encode_now;
    float speed;
} ENCODE_TypeDef;

extern ENCODE_TypeDef g_encode;
extern TIM_HandleTypeDef g_timx_pwm_chy_handle;
extern TIM_HandleTypeDef TIM4_Handler;

void TIM5_Init(u16 arr, u16 psc);

void TIM4_Init(u16 arr, u16 psc);

void atim_timx_pwm_chy_init(uint16_t arr, uint16_t psc);

void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc);

void btim_timx_int_init(uint16_t arr, uint16_t psc);

int gtim_get_encode(void);
#endif
