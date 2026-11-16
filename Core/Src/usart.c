#include "sys.h"
#include "usart.h"
#include "eth.h"
#if SYSTEM_SUPPORT_OS
#include "includes.h"
#endif

/* ---- semihosting stubs ---- */
#if 1
#pragma import(__use_no_semihosting)
void _sys_exit(int x)       { x = x; }
void _ttywrch(int ch)       { (void)ch; }
int  $Sub$$_sys_open(const char *name, int mode, int len) { (void)name; (void)mode; (void)len; return -1; }
int  $Sub$$_sys_close(int fh)     { (void)fh; return -1; }
int  $Sub$$_sys_write(int fh, const unsigned char *buf, unsigned len, int mode)  { (void)fh; (void)buf; (void)mode; return (int)len; }
int  $Sub$$_sys_read(int fh, unsigned char *buf, unsigned len, int mode) { (void)fh; (void)buf; (void)mode; return 0; }
int  $Sub$$_sys_istty(int fh)     { (void)fh; return 1; }
int  $Sub$$_sys_seek(int fh, long pos) { (void)fh; (void)pos; return -1; }
long $Sub$$_sys_flen(int fh)      { (void)fh; return 0; }

int fputc(int ch, FILE *f)
{
    uint32_t timeout = 500000;
    while (!(UART5->SR & USART_SR_TC))
    {
        if (--timeout == 0) break;
    }
    UART5->DR = (uint8_t)ch;

    /* 如果当前命令来源是 TCP, 同步发送到 TCP 客户端 */
    if (g_cmd_src == CMD_SRC_TCP && g_tcp_client_connected)
    {
        eth_tcp_send_byte((uint8_t)ch);
    }
    return ch;
}
#endif

#if EN_UART5_RX
u8 USART_RX_BUF[USART_REC_LEN];
u8 USART_TX_BUF[2];
u16 USART_RX_STA = 0;
u8 aRxBuffer[RXBUFFERSIZE];
UART_HandleTypeDef UART5_Handler;

void uart_init(u32 bound)
{
    /* UART5: PC12(TX), PD2(RX), APB1=42MHz, 115200调试 */
    UART5_Handler.Instance = UART5;
    UART5_Handler.Init.BaudRate = bound;
    UART5_Handler.Init.WordLength = UART_WORDLENGTH_8B;
    UART5_Handler.Init.StopBits = UART_STOPBITS_1;
    UART5_Handler.Init.Parity = UART_PARITY_NONE;
    UART5_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART5_Handler.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&UART5_Handler);

    /* 等待 TE 就绪，发送测试字节稳定 TX */
    while (!(UART5->SR & USART_SR_TXE)) {}
    UART5->DR = 0x0A;
    while (!(UART5->SR & USART_SR_TC)) {}

    /* 清除上电噪声 */
    if (UART5->SR & USART_SR_RXNE) (void)UART5->DR;

    /* 使能 RX 中断 (手动管理, 不使用 HAL_UART_Receive_IT) */
    UART5->CR1 |= USART_CR1_RXNEIE;
}

u32 tpingg(u32 tmp)
{
    u32 num = 1;
    if (tmp == 0) return 1;
    while (tmp--) num *= 10;
    return num;
}

void send(uint16_t data)
{
    uint8_t send_buf[10];
    send_buf[0] = (data / 100) + '0';
    send_buf[1] = ((data % 100) / 10) + '0';
    send_buf[2] = (data % 10) + '0';
    (void)send_buf;
}

/* UART5 中断服务函数 (手动处理接收) */
void UART5_IRQHandler(void)
{
    u8 Res;
#if SYSTEM_SUPPORT_OS
    OSIntEnter();
#endif
    if (UART5->SR & USART_SR_RXNE)
    {
        Res = (u8)(UART5->DR & 0x00FF);

        if ((USART_RX_STA & 0x8000) == 0)
        {
            if (USART_RX_STA & 0x4000)
            {
                if (Res != 0x0a)
                    USART_RX_STA = 0;
                else
                    USART_RX_STA |= 0x8000;
            }
            else
            {
                if (Res == 0x0d)
                    USART_RX_STA |= 0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
                    USART_RX_STA++;
                    if (USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0;
                }
            }
        }
    }
    /* 清除错误标志 */
    if (UART5->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)UART5->DR;
    }
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}

/* ===== HAL_UART_MspInit (UART5 调试 + UART4 上位机) ===== */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance == UART5)
    {
        /* UART5_TX: PC12, AF8, push-pull */
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_UART5_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_12;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &gpio);

        /* UART5_RX: PD2, AF8, pull-up */
        __HAL_RCC_GPIOD_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_2;
        gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOD, &gpio);

        HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }
    else if (huart->Instance == UART4)
    {
        /* UART4_TX: PC10, UART4_RX: PC11, AF8 */
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_UART4_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_10;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOC, &gpio);

        gpio.Pin = GPIO_PIN_11;
        HAL_GPIO_Init(GPIOC, &gpio);

        HAL_NVIC_SetPriority(UART4_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }
}
#endif
