#include "stm32f4xx_hal.h"
#include "sys.h"
#include "delay.h"
#include "upgrade.h"

#define APP_BASE          0x08010000U
#define APP_END           0x08060000U   /* APP 区上界 = 参数区起点 */
#define UPDATE_FLAG_ADDR  0x0807C000U

/* ===== 系统时钟 168MHz (用内部 HSI 做 PLL 源, 无需外部晶振, 冷启动可靠)
 * HSI=16MHz: PLLM=16 → VCO输入1MHz → PLLN=336 → VCO 336MHz → /2 = 168MHz
 * APB1 = 168/4 = 42MHz (与 APP 用 HSE 时相同, UART 波特率不变) ===== */
static void boot_clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16;
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

/* ===== UART5 (PC12=TX, PD2=RX) 115200, 轮询收发, 不开RX中断 =====
 * 裸寄存器初始化 (不依赖 hal_uart/hal_dma, bootloader 最简) */
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

/* ===== 底层 UART5 原始收发 ===== */
void boot_uart_putc(uint8_t c)
{
    uint32_t timeout = 1000000U;
    while (!(UART5->SR & USART_SR_TXE)) {
        if (--timeout == 0U) return;
    }
    UART5->DR = c;
}

/* 等待发送完成(TC), 跳转/复位前调用, 避免最后一个字节丢 */
void boot_uart_wait_complete(void)
{
    uint32_t timeout = 1000000U;
    while (!(UART5->SR & USART_SR_TC)) {
        if (--timeout == 0U) break;
    }
}

int boot_uart_getc(uint32_t timeout_ms)
{
    while (timeout_ms--) {
        if (UART5->SR & USART_SR_RXNE) return (int)(UART5->DR & 0xFF);
        if (UART5->SR & USART_SR_ORE)  { (void)UART5->DR; }
        delay_ms(1);
    }
    return -1;
}

void boot_uart_puts(const char *s)
{
    while (*s) boot_uart_putc((uint8_t)*s++);
}

/* 十六进制打印 (printf 不可用时用) */
static void boot_uart_hex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    char buf[11];
    buf[0] = '0'; buf[1] = 'x';
    for (int i = 0; i < 8; i++) buf[2 + i] = hex[(v >> (28 - 4 * i)) & 0xF];
    buf[10] = '\0';
    boot_uart_puts(buf);
}

/* 打印复位原因 (RCC->CSR), 排查看门狗/连续复位 */
static void print_reset_cause(void)
{
    uint32_t csr = RCC->CSR;
    boot_uart_puts("[boot] RCC_CSR=0x");
    boot_uart_hex32(csr);
    boot_uart_puts("\r\n");
    if (csr & RCC_CSR_IWDGRSTF) boot_uart_puts("[boot] reset: IWDG\r\n");
    if (csr & RCC_CSR_WWDGRSTF) boot_uart_puts("[boot] reset: WWDG\r\n");
    if (csr & RCC_CSR_SFTRSTF)  boot_uart_puts("[boot] reset: software\r\n");
    if (csr & RCC_CSR_PINRSTF)  boot_uart_puts("[boot] reset: pin\r\n");
    if (csr & RCC_CSR_PORRSTF)  boot_uart_puts("[boot] reset: POR\r\n");
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

/* ===== 全局 MSP：时钟使能 ===== */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* SysTick 中断: HAL_InitTick 已使能, 必须有处理函数, 否则进 Default_Handler 死循环
 * delay_us/ms 是轮询 SysTick->VAL, 不依赖此中断 */
void SysTick_Handler(void)
{
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
    if (r == 0) {
        boot_uart_puts("BOOT: received=");
        boot_uart_hex32(ctx.received);
        boot_uart_puts(" bytes\r\n");
    } else {
        boot_uart_puts("BOOT: receive error ");
        boot_uart_hex32((uint32_t)r);
        boot_uart_puts("\r\n");
    }
    if (r == 0 && app_valid()) {       /* 裸协议无校验, 写完后必须验证向量表 */
        clear_update_flag();
        boot_uart_puts("\r\nBOOT: upgrade OK, jumping...\r\n");
        boot_uart_wait_complete();
        jump_to_app();
    } else {
        boot_uart_puts("\r\nBOOT: upgrade failed, retrying...\r\n");
    }
}

int main(void)
{
    SCB->VTOR = 0x08000000U;
    HAL_Init();
    boot_clock_init();
    delay_init(168);
    boot_uart5_init(115200);

    boot_uart_puts("\r\nBOOT v1.3\r\n");
    print_reset_cause();
    boot_uart_puts("[boot] init ok\r\n");

    while (1) {
        uint32_t flag = *(volatile uint32_t *)UPDATE_FLAG_ADDR;
        boot_uart_puts("[boot] flag=");
        boot_uart_hex32(flag);
        boot_uart_puts("\r\n");
        if (flag == 1) {
            boot_uart_puts("[boot] -> upgrade\r\n");
            do_upgrade();                       /* 成功则跳转, 失败重试 */
        } else if (app_valid()) {
            boot_uart_puts("[boot] -> jump APP\r\n");
            delay_ms(100);
            jump_to_app();
        } else {
            boot_uart_puts("[boot] -> wait\r\n");
            do_upgrade();
        }
    }
}
