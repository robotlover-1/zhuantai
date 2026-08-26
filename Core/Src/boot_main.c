#include "stm32f4xx_hal.h"
#include "sys.h"
#include "delay.h"
#include "upgrade.h"
#include <stdio.h>

#define APP_BASE          0x08010000U
#define APP_END           0x08060000U   /* APP 区上界 = 参数区起点 */
#define UPDATE_FLAG_ADDR  0x0807C000U

/* ===== 系统时钟 168MHz (HSE=8MHz, 与 APP 的 Stm32_Clock_Init 一致) ===== */
static void boot_clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

/* ===== UART5 (PC12=TX, PD2=RX) 115200, 轮询收发, 不开RX中断 ===== */
static void boot_uart5_init(uint32_t baud)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();

    g.Pin = GPIO_PIN_12;                 /* PC12: TX, AF8 */
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &g);

    g.Pin = GPIO_PIN_2;                  /* PD2: RX, AF8 */
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &g);

    /* 波特率: UART5 挂 APB1 = 42MHz (注意不是84MHz, 只有定时器才×2, UART不翻倍)
     * BRR: 高12位 mantissa + 低4位 fraction, 计算同 HAL */
    uint32_t bauddiv = 16U * baud;
    uint32_t mantissa = 42000000U / bauddiv;
    uint32_t fraction = ((42000000U % bauddiv) * 16U + bauddiv / 2U) / bauddiv;
    UART5->BRR = (mantissa << 4) | (fraction & 0x0FU);
    UART5->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* ===== 底层 UART5 原始收发（供 ymodem.c 使用）===== */
void boot_uart_putc(uint8_t c)
{
    uint32_t t = 500000;
    while (!(UART5->SR & USART_SR_TC)) if (--t == 0) break;
    UART5->DR = c;
}

int boot_uart_getc(uint32_t timeout_ms)
{
    while (timeout_ms--) {
        if (UART5->SR & USART_SR_RXNE) return (int)(UART5->DR & 0xFF);
        if (UART5->SR & USART_SR_ORE)  { (void)UART5->DR; }  /* 清溢出错误 */
        delay_ms(1);
    }
    return -1;
}

void boot_uart_puts(const char *s)
{
    while (*s) boot_uart_putc((uint8_t)*s++);
}

/* printf 重定向到 UART5 (MicroLIB) */
int fputc(int ch, FILE *f)
{
    boot_uart_putc((uint8_t)ch);
    return ch;
}

/* ===== 全局 MSP：时钟使能 ===== */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* APP 有效性: 栈指针在 RAM 内, 复位向量在 APP 区 */
static int app_valid(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_BASE;
    uint32_t pc = *(volatile uint32_t *)(APP_BASE + 4);
    return (sp >= 0x20000000U && sp < 0x20020000U) &&
           (pc >= APP_BASE && pc < APP_END);
}

static void jump_to_app(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_BASE;
    uint32_t pc = *(volatile uint32_t *)(APP_BASE + 4);
    __disable_irq();
    SCB->VTOR = APP_BASE;
    __set_MSP(sp);
    ((void (*)(void))pc)();
}

static void clear_update_flag(void)
{
    HAL_FLASH_Unlock();
    if (*(volatile uint32_t *)UPDATE_FLAG_ADDR != 0)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UPDATE_FLAG_ADDR, 0);
    HAL_FLASH_Lock();
}

static void do_upgrade(void)
{
    upgrade_ctx_t ctx = {0};
    ctx.target_addr = APP_BASE;
    ctx.max_size    = APP_END - APP_BASE;
    int r = upgrade_receive(&ctx);
    if (r == 0)
        printf("BOOT: received %d bytes\r\n", (int)ctx.received);
    else
        printf("BOOT: receive error %d\r\n", r);
    if (r == 0 && app_valid()) {       /* 裸协议无校验, 写完后必须验证向量表 */
        clear_update_flag();
        boot_uart_puts("\r\nBOOT: upgrade OK, jumping...\r\n");
        delay_ms(200);
        jump_to_app();
    } else {
        boot_uart_puts("\r\nBOOT: upgrade failed, retrying...\r\n");
    }
}

int main(void)
{
    SCB->VTOR = 0x08000000U;      /* bootloader 向量表 */
    HAL_Init();
    boot_clock_init();            /* 168MHz */
    delay_init(168);
    boot_uart5_init(115200);      /* UART5 (PC12/PD2) 轮询收发 */

    printf("\r\nBOOT v1.0\r\n");
    printf("[boot] init ok\r\n");

    while (1) {
        uint32_t flag = *(volatile uint32_t *)UPDATE_FLAG_ADDR;
        printf("[boot] flag=%u\r\n", (unsigned)flag);
        if (flag == 1) {
            printf("[boot] -> upgrade\r\n");
            do_upgrade();                       /* 成功则跳转, 失败重试 */
        } else if (app_valid()) {
            printf("[boot] -> jump APP\r\n");
            delay_ms(100);
            jump_to_app();
        } else {
            printf("[boot] -> wait\r\n");
            do_upgrade();
        }
    }
}
