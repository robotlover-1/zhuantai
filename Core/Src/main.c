/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usart2.h"
#include "led.h"
#include "atim.h"
#include "step_motor.h"
#include "pid.h"
#include "flash_param.h"
#include "exti.h"
#include "cmd_handler.h"
#include "rtc.h"

/************************************************/
/************************************************/
extern TIM_HandleTypeDef g_timx_encode_chy_handle;
extern TIM_HandleTypeDef timx_handler;      /* TIM6句柄，atim.c中定义 */
extern u8 time_run;
extern volatile u32 rx_debug_cnt;
u8 printf_flag = 0;
u8 run_flag = 0;
u8 run_printf_flag = 0;
u8 write_flag = 0;
u8 direction = 2;
int station_x = 0, station_k1 = 0;
int error = 0;
int32_t Encode_now;
int start = 0;
int DIR = 0; // 电机方向 
int out;
int Ring = 0;
int Ding = 0;
int pulse_add;
int32_t force = 1000;
int32_t force_F = 2500;
int time_c = 100;
int duzhuan_flag;
int alarm = 0;
float SetPoint_C;

/*********** PID参数 ************/
float SetPoint_P;       /* 目标值 */
float ActualValue_P;    /* 实际值 */
float SumError_P;       /* 累计误差 */
float Proportion_P;     /* 比例 P */
float Integral_P;       /* 积分 I */
float Derivative_P;     /* 微分 D */
float Error_P;          /* Error[1] */
float LastError_P;      /* Error[-1] */
float PrevError_P;      /* Error[-2] */
float SetPoint_S;       /* 目标值 */
float ActualValue_S;    /* 实际值 */
float SumError_S;       /* 累计误差 */
float Proportion_S;     /* 比例 P */
float Integral_S;       /* 积分 I */
float Derivative_S;     /* 微分 D */
float Error_S;          /* Error[1] */
float LastError_S;      /* Error[-1] */
float PrevError_S;      /* Error[-2] */
float speed_m;          /* 电机速度 */
float location;         /* 位置 */
int32_t motor_pwm;      /* 电机PWM值 */
int32_t pulse_low = 14000;  // 进孔脉冲数
int32_t pulse_out = 39000;  // 出孔脉冲数
int32_t pluse_ele = 38000;  // 光电检测脉冲限制  
int loop1 = 0;
int loop2 = 0;
int PhotoPos = 1;
int32_t addr;
int32_t writeFlashData;
int dbg_print_ms = 0;                   /* 调试打印间隔(ms), 0=关闭 */

/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{

  /* USER CODE BEGIN 1 */
	u8 len;
    int32_t i = 0;
    //SCB->VTOR = FLASH_BASE | 0x10000; 
  SCB->VTOR = FLASH_BASE; 

  /* USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    Stm32_Clock_Init();
    delay_init(168);
    rtc_init();                          /* 初始化RTC(从编译时间设初始值) */                   
    uart4_init(9600);                  
    USART_RX_STA = 0;                    /* 清除上电噪声导致的误接收 */
    for (i = 0; i < USART_REC_LEN; i++) USART_RX_BUF[i] = 0;
    //LED_Init();                        
    // atim_timx_pwm_chy_init moved after MX_GPIO_Init to avoid pin conflict
    TIM5_Init(5000 - 1, 8400 - 1);         
    TIM4_Init(5000 - 1, 16800 - 1);        
    step_motor_init();
    gtim_timx_encoder_chy_init(0XFFFF, 0); 
    btim_timx_int_init(1000 - 1, 84 - 1);   /* 1ms中断，用于PID控制 */
    pid_init();

  /* USER CODE END Init */
  /* Configure the system clock - skipped: Stm32_Clock_Init() already configures 168MHz */
  /* SystemClock_Config(); */
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Re-restore TIM1 pins overwritten by MX_GPIO_Init (PA8/PA11 need AF_PP for PWM) */
  atim_timx_pwm_chy_init(3600 - 1, 10);
  uart_init(115200);                     /* 直接寄存器配置，无需HAL */

  /* USER CODE BEGIN 2 */
    __enable_irq();                                            /* 开启全局中断，TIM6使用硬件中断 */
    USART_RX_STA = 0;                                          /* 清除开中断瞬间可能收到的噪声 */
	param_load_all();
  if (writeFlashData != 0) param_save_all();
  dcmotor_stop();
    // TIM4->CR1 |= TIM_CR1_CEN;                                  /* 启动TIM4(1s) */
    // TIM5->CR1 |= TIM_CR1_CEN;                                  /* 启动TIM5(500ms) */

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {

        /* TIM6使用硬件中断，无需轮询 */
        // /* TIM5: 500ms周期 */
        // if (TIM5->SR & TIM_SR_UIF) { TIM5->SR = ~TIM_SR_UIF; printf("T5:500ms\r\n"); }
        // /* TIM4: 1s周期 */
        // if (TIM4->SR & TIM_SR_UIF) { TIM4->SR = ~TIM_SR_UIF; printf("T4:1s\r\n"); }
        static u8 in1_last = 0xFF;  /* 0xFF=未初始化 */
        static u8 in2_last = 0xFF;
        u8 in1_now, in2_now;

        /* 首次运行，读取当前引脚值作为初始状态，避免误触发 */
        if (in1_last == 0xFF) {
            in1_last = HAL_GPIO_ReadPin(ST1_IN1_GPIO_PORT, ST1_IN1_GPIO_PIN);
            in2_last = HAL_GPIO_ReadPin(ST1_IN2_GPIO_PORT, ST1_IN2_GPIO_PIN);
        }

        /* UART5接收由UART5_IRQHandler硬件中断完成，主循环无需轮询 */
        if (USART_RX_STA_4 & 0x8000)
        {
            len = USART_RX_STA_4 & 0x3fff;
            USART_RX_STA_4 = 0;
            HAL_UART_Transmit(&UART4_Handler, (uint8_t *)USART_RX_BUF_4, len, 1000);
        }

        //printf("USART_RX_STA: 0x%04X\r\n", USART_RX_STA);
        if (USART_RX_STA & 0x8000)
        {
            len = USART_RX_STA & 0x3fff;
            USART_RX_STA = 0;

            /* 直接回显，绕过HAL */
            // for (i = 0; i < len; i++) {
            //     uint32_t timeout = 500000;
            //     while (!(UART5->SR & USART_SR_TC)) {
            //         if (--timeout == 0) break;
            //     }
            //     UART5->DR = USART_RX_BUF[i];
            // }
            //printf("\r\n");
            printf("USART_RX_BUF: %s\r\n", USART_RX_BUF);
            cmd_dispatch(USART_RX_BUF, len);
            for (i = 0; i < len; i++)
                USART_RX_BUF[i] = 0;
        }

        in1_now = HAL_GPIO_ReadPin(ST1_IN1_GPIO_PORT, ST1_IN1_GPIO_PIN);
        if (in1_now == 1 && in1_last == 0)
        {
            printf("复位信号,报警清除!\r\n");
            error = 0;
        }

        in1_last = in1_now;
        in2_now = HAL_GPIO_ReadPin(ST1_IN2_GPIO_PORT, ST1_IN2_GPIO_PIN);
        if (in2_now == 0 && in2_last == 1)  /* 下降沿检测：24V正常时IN2=1，断电后IN2=0 */
        {
            printf("急停信号!\r\n");
            dcmotor_stop();
            motor_pwm_set(0);
            run_flag = 0;
            run_printf_flag = 0;
            error = 1;
        }

        in2_last = in2_now;
        if (printf_flag == 1)
        {
            printf_flag = 0;
        }
    }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /* --- 以下引脚由 step_motor_init() / atim_timx_pwm_chy_init() 管理，此处不再重复 --- */
  /* --- FSMC 引脚 (PF0~PF5, PF12~PF15) --- */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* --- PF6/PF7/PF8/PF9 由 step_motor_init() 管理 --- */
  /* --- PC0/PC1/PC2 由 step_motor_init() 管理 --- */
  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*Configure GPIO pins : PG0 PG1 PG2 PG3
                           PG4 PG5 PG9 PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  /*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE11 PE12 PE13 PE14
                           PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD4 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 (DM9000-INT) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* --- PA8/PA11 由 atim_timx_pwm_chy_init() (TIM1 PWM) 管理 --- */
  /* --- PC12(TX)/PD2(RX) 由 uart_init (UART5) 管理 --- */
  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 (DM9000-RST) */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void Error_Handler(void)
{

  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */

void assert_failed(uint8_t *file, uint32_t line)
{

  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
