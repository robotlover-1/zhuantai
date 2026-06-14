#include "atim.h"
#include "step_motor.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "delay.h"
#include "pid.h"
#include "debug.h"
#include "usart2.h"
#include "exti.h"

TIM_HandleTypeDef g_timx_pwm_chy_handle;
TIM_OC_InitTypeDef g_timx_oc_pwm_chy = {0};

void atim_timx_pwm_chy_init(uint16_t arr, uint16_t psc)
{
    g_timx_pwm_chy_handle.Instance = ATIM_TIMX_PWM;
    g_timx_pwm_chy_handle.Init.Prescaler = psc;
    g_timx_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_timx_pwm_chy_handle.Init.Period = arr;
    g_timx_pwm_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_timx_pwm_chy_handle.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&g_timx_pwm_chy_handle);

    /* 配置CH1(PA8) - 正转PWM */
    g_timx_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;
    g_timx_oc_pwm_chy.Pulse = 0;
    g_timx_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_HIGH;
    g_timx_oc_pwm_chy.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&g_timx_pwm_chy_handle, &g_timx_oc_pwm_chy, TIM_CHANNEL_1);

    /* 配置CH4(PA11) - 反转PWM */
    HAL_TIM_PWM_ConfigChannel(&g_timx_pwm_chy_handle, &g_timx_oc_pwm_chy, TIM_CHANNEL_4);

    HAL_NVIC_SetPriority(ATIM_TIMX_INT_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ATIM_TIMX_INT_IRQn);
    HAL_TIM_Base_Start_IT(&g_timx_pwm_chy_handle);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct = {0};
        ATIM_TIMX_PWM_CHY_GPIO_CLK_ENABLE();   /* GPIOA时钟已使能(PA8, PA11) */
        ATIM_TIMX_PWM_CHY_CLK_ENABLE();         /* TIM1时钟使能 */

        /* TIM1_CH1 - PA8: 球式正转PWM/平式步进脉冲复用 */
        gpio_init_struct.Pin = GPIO_PIN_8;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init_struct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);

        /* TIM1_CH4 - PA11: 球式反转PWM/平式步进方向复用 */
        gpio_init_struct.Pin = GPIO_PIN_11;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init_struct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    }
}

void ATIM_TIMX_INT_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&g_timx_pwm_chy_handle, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&g_timx_pwm_chy_handle, TIM_IT_UPDATE);
    }
}

u8 time_out = 0;
TIM_HandleTypeDef TIM5_Handler;

void TIM5_Init(u16 arr, u16 psc)
{
    TIM5_Handler.Instance = TIM5;
    TIM5_Handler.Init.Prescaler = psc;
    TIM5_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM5_Handler.Init.Period = arr;
    TIM5_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM5_Handler);
}

void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM5_Handler);
}

u8 time_run = 0;
TIM_HandleTypeDef TIM4_Handler;

void TIM4_Init(u16 arr, u16 psc)
{
    TIM4_Handler.Instance = TIM4;
    TIM4_Handler.Init.Prescaler = psc;
    TIM4_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM4_Handler.Init.Period = arr;
    TIM4_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM4_Handler);
}

void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM4_Handler);
}

TIM_HandleTypeDef g_timx_encode_chy_handle;
TIM_Encoder_InitTypeDef g_timx_encoder_chy_handle;

void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef gpio_init_struct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    g_timx_encode_chy_handle.Instance = TIM3;
    g_timx_encode_chy_handle.Init.Prescaler = psc;
    g_timx_encode_chy_handle.Init.Period = arr;
    g_timx_encode_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    g_timx_encoder_chy_handle.EncoderMode = TIM_ENCODERMODE_TI12;
    g_timx_encoder_chy_handle.IC1Polarity = TIM_ICPOLARITY_RISING;
    g_timx_encoder_chy_handle.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    g_timx_encoder_chy_handle.IC1Prescaler = TIM_ICPSC_DIV1;
    g_timx_encoder_chy_handle.IC1Filter = 10;
    g_timx_encoder_chy_handle.IC2Polarity = TIM_ICPOLARITY_RISING;
    g_timx_encoder_chy_handle.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    g_timx_encoder_chy_handle.IC2Prescaler = TIM_ICPSC_DIV1;
    g_timx_encoder_chy_handle.IC2Filter = 10;
    HAL_TIM_Encoder_Init(&g_timx_encode_chy_handle, &g_timx_encoder_chy_handle);

    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle, GTIM_TIMX_ENCODER_CH1);
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle, GTIM_TIMX_ENCODER_CH2);

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    __HAL_TIM_CLEAR_FLAG(&g_timx_encode_chy_handle, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&g_timx_encode_chy_handle, TIM_IT_UPDATE);
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_encode_chy_handle);
}

TIM_HandleTypeDef timx_handler;
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    timx_handler.Instance = BTIM_TIMX_INT;
    timx_handler.Init.Prescaler = psc;
    timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    timx_handler.Init.Period = arr;
    timx_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&timx_handler);

    HAL_TIM_Base_Start_IT(&timx_handler);
    __HAL_TIM_CLEAR_IT(&timx_handler, TIM_IT_UPDATE);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        BTIM_TIMX_INT_CLK_ENABLE();
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 0, 4);
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);
    }
    if (htim->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM5_IRQn, 1, 3);
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
    }
    if (htim->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM4_IRQn, 1, 4);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }
}

void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&timx_handler);
}

int32_t g_timx_encode_count = 0;
extern int decel_i;
extern u8 run_flag;
extern int32_t Encode_now;
extern float SetPoint_P;
extern float SetPoint_S;
extern float speed_m;
extern float location;
extern float ActualValue_S;      /* 速度环累加输出，run_flag切换时需复位 */
extern float LastError_S;
extern float PrevError_S;
extern int   g_pid_period_ms;    /* PID采样周期(ms), 运行时可调 */
extern int32_t motor_pwm;
extern int32_t force;
extern int32_t force_F;
extern int error;
float location_last = 0;
int k_loop = 0;
volatile uint32_t g_tick_ms = 0;   /* 系统滴答时钟(ms) */
extern int time_c;
extern int duzhuan_flag;
extern u8 run_printf_flag;
extern int DIR;
extern int32_t pulse_low;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t _pid_cnt = 0;
    if (htim->Instance == TIM6)
    {
        g_tick_ms++;                         /* 1ms递增系统滴答 */
        Encode_now = gtim_get_encode();
        speed_computer(Encode_now, 5);
        location = Encode_now;
        //printf("TIM6:\r\n");            /* ISR中严禁printf，会与主循环争抢UART5 */
        if (run_flag == 1)
        {
            if (DIR == 0)
            {
                if (Encode_now >= SetPoint_P - pulse_low)
                {
                    run_flag = 2;
                    //ActualValue_S = 0;  /* 复位速度环，从零开始重新积累 */
                    //LastError_S = 0;
                    //PrevError_S = 0;
                }
            }
            else if (DIR == 1)
            {
                if (Encode_now <= SetPoint_P + pulse_low)
                {
                    run_flag = 3;
                    //ActualValue_S = 0;
                    //LastError_S = 0;
                    //PrevError_S = 0;
                }
            }
        }
        if (++_pid_cnt >= (uint8_t)g_pid_period_ms)
        {
            _pid_cnt = 0;
            k_loop++;
            //printf("k_loop%d\r\n", k_loop);        /* ISR中严禁printf */
            if (run_flag == 1)
            {
                motor_pwm = position_pid_speed(location);
                if (motor_pwm >= 0)
                {
                    if (motor_pwm >= force)
                        motor_pwm = force;
                    else if (motor_pwm < 100)
                        motor_pwm = 100;
                }
                else
                {
                    if (motor_pwm <= -force)
                        motor_pwm = -force;
                    else if (motor_pwm > -100)
                        motor_pwm = -100;
                }
                SetPoint_S = motor_pwm;
                motor_pwm = increment_pid_speed(speed_m);
                if (motor_pwm >= force_F)
                    motor_pwm = force_F;
                else if (motor_pwm <= -force_F)
                    motor_pwm = -force_F;
                motor_pwm_set(motor_pwm);
            }
            else if (run_flag == 2)
            {
                SetPoint_S = time_c;
                motor_pwm = increment_pid_speed(speed_m);
                if (motor_pwm >= force_F)
                    motor_pwm = force_F;
                else if (motor_pwm <= -force_F)
                    motor_pwm = -force_F;
                motor_pwm_set(motor_pwm);
            }
            else if (run_flag == 3)
            {
                SetPoint_S = -time_c;
                motor_pwm = increment_pid_speed(speed_m);
                if (motor_pwm >= force_F)
                    motor_pwm = force_F;
                else if (motor_pwm <= -force_F)
                    motor_pwm = -force_F;
                motor_pwm_set(motor_pwm);
            }

            /* 调试打印: DP命令设置间隔(ms), 仅在运动中(run_flag!=0)输出 */
            {
                extern int dbg_print_ms;
                if (dbg_print_ms > 0 && run_flag != 0)
                {
                    static uint32_t _dbg_tick = 0;
                    if ((g_tick_ms - _dbg_tick) >= (uint32_t)dbg_print_ms)
                    {
                        _dbg_tick = g_tick_ms;
                        printf("DBG|rf=%d|pwm=%d|SpdS=%d|spd=%.0f|enc=%d\r\n",
                               run_flag, (int)motor_pwm, (int)SetPoint_S, speed_m, (int)Encode_now);
                    }
                }
            }

        }
    }
    else if (htim->Instance == TIM5)
    {
        time_out++;
        //printf("TIM5 timeout: %d\r\n", time_out);
    }
    else if (htim->Instance == TIM4)
    {
        time_run++;
        //printf("TIM4 time_run: %d\r\n", time_run);
    }
}

int32_t Encode_Now = 0;
int32_t Encode_Old = 0;
int gtim_get_encode(void)
{
    Encode_Now = __HAL_TIM_GET_COUNTER(&g_timx_encode_chy_handle);
    if (Encode_Now - Encode_Old < -40000)
    {
        g_timx_encode_count++;
    }
    else if (Encode_Now - Encode_Old > 40000)
    {
        g_timx_encode_count--;
    }
    Encode_Old = Encode_Now;
    return (int32_t)__HAL_TIM_GET_COUNTER(&g_timx_encode_chy_handle) + g_timx_encode_count * 65536;
}
