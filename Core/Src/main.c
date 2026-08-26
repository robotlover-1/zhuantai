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
#include "eth.h"
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
u8 g_no_compensate = 0;        /* 0=补偿, 1=一步到位不补偿 */
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

extern int time_out;

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
int PL_T = 60;   /* PL 疲劳测试大循环(约12小时) */
int PL_H = 30;   /* PL 疲劳测试小循环 */
int CS_T = 40;   /* CS 自动测试大循环(约12小时) */
int CS_H = 30;   /* CS 自动测试小循环 */

extern __IO int32_t  g_step_position;
extern __IO uint8_t  g_motion_sta;
extern __IO uint32_t g_add_pulse_count;

void SystemClock_Config(void);

/* ===== 临时诊断: 早期裸 UART5 打印, 定位 APP 启动卡点 (定位后删除) ===== */
static void diag_uart_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_12; g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_2; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &g);
    uint32_t bd = 16U * 115200U;
    uint32_t mant = 42000000U / bd;
    uint32_t frac = ((42000000U % bd) * 16U + bd / 2U) / bd;
    UART5->BRR = (mant << 4) | (frac & 0x0FU);
    UART5->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}
static void diag_puts(const char *s)
{
    while (*s) {
        while (!(UART5->SR & USART_SR_TXE)) {}
        UART5->DR = (uint8_t)*s++;
    }
}

int main(void)
{
    u8 len;
    int32_t i = 0;

    SCB->VTOR = 0x08010000U;     /* APP 已搬移到 bootloader 之后 */

    HAL_Init();
    Stm32_Clock_Init();            /* 168MHz */
    delay_init(168);
    diag_uart_init();
    diag_puts("\r\n[app] clk ok\r\n");
    rtc_init();                    /* RTC 从编译时间初始化 */
    diag_puts("[app] rtc ok\r\n");
    uart4_init(9600);              /* UART4 上位机 */
    diag_puts("[app] uart4 ok\r\n");
    USART_RX_STA_4 = 0;
    for (i = 0; i < USART_REC_LEN_4; i++) USART_RX_BUF_4[i] = 0;

    step_motor_init();             /* GPIO 初始化 */
    diag_puts("[app] motor init ok\r\n");
    atim_timx_pwm_chy_init(0xFFFF, 84 - 1);  /* TIM1_CH1 步进脉冲, 168M/84=2M 与F1一致 */
    TIM5_Init(5000-1, 8400-1);    /* 4Hz 超时 */
    TIM4_Init(5000-1, 16800-1);   /* 2Hz 用时计时 */
    diag_puts("[app] timers ok\r\n");

    uart_init(115200);            /* USART1 调试 */
    diag_puts("[app] uart ok\r\n");

    __enable_irq();
    USART_RX_STA = 0;
    for (i = 0; i < USART_REC_LEN; i++) USART_RX_BUF[i] = 0;

    param_load_all();             /* 加载 Flash 参数 */

    write_update_flag(0);         /* APP正常启动, 清除固件升级标志 (对齐F1, 防复位误进升级) */

    /* 以太网初始化 (DM9000 + LwIP + TCP Server)
     * 放在 param_load_all 之后: 使 g_eth_ip 等使用 Flash 中已加载的网络参数;
     * DM9000 初始化失败只打印日志, 不影响串口模式 */
    eth_init();

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
            USART_RX_BUF_4[len] = '\0';
            printf("%s\r\n", USART_RX_BUF_4);   /* 回显 */
            cmd_dispatch(USART_RX_BUF_4, len);
        }

        /* 2. UART5 调试接收 (115200) */
        if (USART_RX_STA & 0x8000) {
            len = USART_RX_STA & 0x3fff;
            USART_RX_STA = 0;
            USART_RX_BUF[len] = '\0';
            printf("%s\r\n", USART_RX_BUF);     /* 回显 */
            cmd_dispatch(USART_RX_BUF, len);
        }

        /* ---- 以太网处理 ---- */
        /* 1. LwIP 协议栈周期处理 (收包、定时器、TCP 发送) */
        MX_LWIP_Process();

        /* 2. TCP 接收处理 (镜像 UART5 的处理逻辑) */
        if (g_eth_rx_ready) {
            u16 eth_len = g_eth_rx_len;
            g_eth_rx_ready = 0;
            g_eth_rx_len = 0;
            g_cmd_src = CMD_SRC_TCP;
            printf("ETH_RX: %s\r\n", g_eth_rx_buf);     /* 回显 (同时发回 TCP) */
            cmd_dispatch(g_eth_rx_buf, eth_len);
            g_cmd_src = CMD_SRC_UART;
            {
                u16 i;
                for (i = 0; i < ETH_RX_BUF_SIZE; i++) g_eth_rx_buf[i] = 0;   /* 清理 */
            }
        }

        /* 3. 硬件 IO 检测 (光耦输入: 24V施加=LOW) */
        if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) == 0) {      /* PF6: 正转 (LOW有效) */
            printf("IO:RTF\r\n");
            cmd_rtf();
        }
        if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7) == 0) {      /* PF7: 急停 (LOW有效) */
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
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == 0) {      /* PE2: 复位 (LOW有效) */
            time_out = 0;
            error = 0;
            g_zero_flag = 0;
            reset_flag = 0;            /* 解除急停锁定 (恢复F1: 复位同时清reset_flag) */
            alarm_set(0);              /* 清除报警输出 */
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
                if (total_cycles % 1000 == 0) {
                    param_save_all();  /* 每1000次保存到 Flash, 减少擦写 */
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
