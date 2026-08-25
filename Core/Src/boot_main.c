#include "stm32f4xx_hal.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "ymodem.h"
#include <stdio.h>

#define APP_BASE          0x08010000U
#define APP_END           0x08060000U   /* APP 区上界 = 参数区起点 */
#define UPDATE_FLAG_ADDR  0x0807C000U

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

/* printf 重定向由 usart.c 的 fputc 提供（UART5） */

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
    ymodem_ctx_t ctx = {0};
    ctx.target_addr = APP_BASE;
    ctx.max_size    = APP_END - APP_BASE;
    int r = ymodem_receive(&ctx);
    if (r == 0) {
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
    Stm32_Clock_Init();           /* 168MHz */
    delay_init(168);
    uart_init(115200);            /* UART5 (PC12/PD2) */
    UART5->CR1 &= ~USART_CR1_RXNEIE;  /* 关闭RX中断, YModem用轮询收原始字节 */

    printf("\r\nBOOT v1.0\r\n");

    while (1) {
        if (*(volatile uint32_t *)UPDATE_FLAG_ADDR == 1) {
            printf("BOOT: update flag set, upgrade mode\r\n");
            do_upgrade();                       /* 成功则跳转, 失败重试 */
        } else if (app_valid()) {
            printf("BOOT: jump APP\r\n");
            delay_ms(100);
            jump_to_app();
        } else {
            printf("BOOT: no valid APP, waiting YModem...\r\n");
            do_upgrade();
        }
    }
}
