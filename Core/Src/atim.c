#include "atim.h"
#include "delay.h"
#include "step_motor.h"
#include <stdio.h>

TIM_HandleTypeDef g_timx_pwm_chy_handle;
TIM_OC_InitTypeDef g_timx_oc_pwm_chy = {0};

/* TIM1_CH1 OC 初始化：步进脉冲输出 (PA8) */
void atim_timx_pwm_chy_init(uint16_t arr, uint16_t psc) {
  g_timx_pwm_chy_handle.Instance = ATIM_TIMX_PWM;
  g_timx_pwm_chy_handle.Init.Prescaler = psc;
  g_timx_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  g_timx_pwm_chy_handle.Init.Period = arr;
  g_timx_pwm_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  g_timx_pwm_chy_handle.Init.RepetitionCounter = 0;
  HAL_TIM_OC_Init(&g_timx_pwm_chy_handle);

  g_timx_oc_pwm_chy.OCMode = TIM_OCMODE_TOGGLE;       /* 翻转模式产生脉冲 */
  g_timx_oc_pwm_chy.Pulse = 0;
  g_timx_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_HIGH;
  g_timx_oc_pwm_chy.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&g_timx_pwm_chy_handle, &g_timx_oc_pwm_chy,
                            ATIM_TIMX_PWM_CHY);

  HAL_NVIC_SetPriority(ATIM_TIMX_INT_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ATIM_TIMX_INT_IRQn);

  __HAL_TIM_ENABLE_IT(&g_timx_pwm_chy_handle, TIM_IT_CC1);
}

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim) {
  if (htim->Instance == ATIM_TIMX_PWM) {
    GPIO_InitTypeDef gpio_init_struct = {0};
    ATIM_TIMX_PWM_CHY_GPIO_CLK_ENABLE();
    ATIM_TIMX_PWM_CHY_CLK_ENABLE();

    /* PA8: TIM1_CH1 步进脉冲输出 */
    gpio_init_struct.Pin = GPIO_PIN_8;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
  }
}

/* ===== 步进电机状态机在 TIM1_CC_IRQHandler 中实现 ===== */
/* 此中断函数放在 step_motor.c 中，通过 extern 声明相互引用 */

/* ===== 系统滴答 (供 RTC 亚秒使用) ===== */
volatile uint32_t g_tick_ms = 0;

/* ===== TIM4: 磨合计时 ===== */
u8 time_run = 0;
TIM_HandleTypeDef TIM4_Handler;

void TIM4_Init(u16 arr, u16 psc) {
  TIM4_Handler.Instance = TIM4;
  TIM4_Handler.Init.Prescaler = psc;
  TIM4_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM4_Handler.Init.Period = arr;
  TIM4_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&TIM4_Handler);
  HAL_TIM_Base_Start_IT(&TIM4_Handler);
}

void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(&TIM4_Handler); }

/* ===== TIM5: 超时/堵转检测 ===== */
int time_out = 0;
TIM_HandleTypeDef TIM5_Handler;

void TIM5_Init(u16 arr, u16 psc) {
  TIM5_Handler.Instance = TIM5;
  TIM5_Handler.Init.Prescaler = psc;
  TIM5_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM5_Handler.Init.Period = arr;
  TIM5_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&TIM5_Handler);
  HAL_TIM_Base_Start_IT(&TIM5_Handler);
}

void TIM5_IRQHandler(void) { HAL_TIM_IRQHandler(&TIM5_Handler); }

/* ===== HAL MSP 回调 ===== */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 3);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  }
  if (htim->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 4);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}

/* ===== TIM4/TIM5 周期回调 ===== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM5) {
    time_out++;
  } else if (htim->Instance == TIM4) {
    time_run++;
  }
}
