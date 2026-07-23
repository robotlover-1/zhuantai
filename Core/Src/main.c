#include "main.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usart2.h"
#include "led.h"
#include "atim.h"
#include "step_motor.h"
#include "flash_param.h"
#include "cmd_handler.h"
#include "rtc.h"
#include <stdio.h>

/******* F1 全局变量 *******/
extern TIM_HandleTypeDef g_timx_pwm_chy_handle;
extern TIM_HandleTypeDef TIM5_Handler;
int32_t step_count;
u8 receive_flag = 0;
u8 printf_flag = 0;
u8 zero_run = 0;
u8 zero_flag = 1;
u8 error = 0;

int pulse_cut = 1000;
int32_t pulse_compensate = 1000;
int32_t step_decel_2 = 100;
int32_t step_accel_2 = 100;
int level_speed = 0;

int32_t step_max = 130;
int32_t step_max_1;
int step_pass = 1333;
int speed_rcc = 666;
int time_delay_compensate = 100;
u8 run_flag = 0;
int32_t step_all;

extern u8 time_out;
u8 time_run;

int station_x = 0;
uint8_t g_zero_flag = 0;
uint8_t reset_flag = 0;
int32_t cycles_72 = 0;
int32_t total_cycles = 0;

__IO uint32_t g_set_speed  = 72;
__IO uint32_t g_step_accel = 8;
__IO uint32_t g_step_decel = 8;
__IO uint16_t g_step_angle = 12000;
__IO uint16_t g_step1 = 300;
__IO uint16_t g_step2 = 300;

int32_t writeFlashData = 0;
int loop1 = 0;
int loop2 = 0;

extern __IO int32_t  g_step_position;
extern __IO uint8_t  g_motion_sta;
extern __IO uint32_t g_add_pulse_count;

void SystemClock_Config(void);

int main(void)
{
    u8 len;
    int32_t i = 0;

    SCB->VTOR = FLASH_BASE;

    HAL_Init();
    Stm32_Clock_Init();            /* 168MHz */
    delay_init(168);
    rtc_init();                    /* RTC 从编译时间初始化 */

    uart4_init(9600);              /* UART4 上位机 */
    USART_RX_STA_4 = 0;
    for (i = 0; i < USART_REC_LEN_4; i++) USART_RX_BUF_4[i] = 0;

    step_motor_init();             /* GPIO 初始化 */
    atim_timx_pwm_chy_init(0xFFFF, 36 - 1);  /* TIM1_CH1 步进脉冲 */
    TIM5_Init(5000-1, 8400-1);    /* 4Hz 超时 */
    TIM4_Init(5000-1, 16800-1);   /* 2Hz 用时计时 */

    uart_init(115200);            /* USART1 调试 */

    __enable_irq();
    USART_RX_STA = 0;
    for (i = 0; i < USART_REC_LEN; i++) USART_RX_BUF[i] = 0;

    param_load_all();             /* 加载 Flash 参数 */

    /* 打印启动信息 */
    {
        char ts[32];
        rtc_get_time_str(ts, sizeof(ts));
        printf("[%s] F1-Stepper on F407 ready\r\n", ts);
        printf("total_cycles: %d\r\n", total_cycles);
    }

    while (1)
    {
        /* 1. UART4 上位机接收 (9600) */
        if (USART_RX_STA_4 & 0x8000) {
            len = USART_RX_STA_4 & 0x3fff;
            USART_RX_STA_4 = 0;
            HAL_UART_Transmit(&UART4_Handler, (uint8_t*)USART_RX_BUF_4, len, 1000);
            while (__HAL_UART_GET_FLAG(&UART4_Handler, UART_FLAG_TC) != SET);
            printf("\r\n");
            cmd_dispatch(USART_RX_BUF_4, len);
        }

        /* 2. USART1 调试接收 (115200) */
        if (USART_RX_STA & 0x8000) {
            len = USART_RX_STA & 0x3fff;
            USART_RX_STA = 0;
            HAL_UART_Transmit(&UART4_Handler, (uint8_t*)USART_RX_BUF, len, 1000);
            while (__HAL_UART_GET_FLAG(&UART4_Handler, UART_FLAG_TC) != SET);
            printf("\r\n");
            cmd_dispatch(USART_RX_BUF, len);
        }

        /* 3. 硬件 IO 检测 (同 F1) */
        if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) == 1) {      /* PF6 → F1 PE2 正转 */
            printf("IO:RTF\r\n");
            cmd_rtf();
        }
        if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7) == 1) {      /* PF7 → F1 PE4 急停 */
            printf("IO:RTS\r\n");
            HAL_TIM_OC_Stop_IT(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY);
            HAL_TIM_Base_Stop_IT(&TIM5_Handler);
            time_out = 0;
            g_motion_sta = 0;
            step_count = 0;
            receive_flag = 0;
            g_zero_flag = 0;
            printf_flag = 0;
            reset_flag = 1;
        }
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == 1) {      /* PE2 → F1 PE3 复位 */
            time_out = 0;
            error = 0;
            g_zero_flag = 0;
        }

        /* 4. 运动完成后处理 */
        if (printf_flag == 1) {
            if (g_motion_sta == 0) {
                char ts[32];
                rtc_get_time_str(ts, sizeof(ts));
                printf("[%s] P%d\r\n", ts, station_x);

                HAL_GPIO_WritePin(ST1_DONE_GPIO_PORT, ST1_DONE_GPIO_PIN, GPIO_PIN_SET);
                delay_ms(100);
                HAL_GPIO_WritePin(ST1_DONE_GPIO_PORT, ST1_DONE_GPIO_PIN, GPIO_PIN_RESET);

                total_cycles++;
                if (total_cycles < 10000000) {
                    param_save_all();  /* 保存 cycles 到 Flash */
                }
                printf_flag = 0;
            }
        }

        delay_ms(50);
    }
}

/* MX_GPIO_Init: 保留 new 项目的实现 (FSMC, I2C, SPI 等) */
/* SystemClock_Config: 保留空壳 (实际由 Stm32_Clock_Init 完成) */
void SystemClock_Config(void) { }
