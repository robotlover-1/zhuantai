#include "sys.h"
#include "usart.h"	
#if SYSTEM_SUPPORT_OS
#include "includes.h"
#endif

#if 1
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};

FILE __stdout;
void _sys_exit(int x)
{
    x = x;
}

int fputc(int ch, FILE *f)
{
    uint32_t timeout = 500000;
    while (!(UART5->SR & USART_SR_TC))
    {
        if (--timeout == 0)
        {
            break;
        }
    }
    UART5->DR = (uint8_t)ch;
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
    u32 temp;
    u16 mantissa;
    u16 fraction;

    /* 直接寄存器配置，绕过HAL */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;     /* GPIOC时钟 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;     /* GPIOD时钟 */
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;    /* UART5时钟 */

    /* PC12(TX) AF_PP AF8 */
    GPIOC->MODER   &= ~(3 << 24);  GPIOC->MODER   |= (2 << 24);
    GPIOC->OTYPER  &= ~(1 << 12);
    GPIOC->OSPEEDR |=  (3 << 24);
    GPIOC->PUPDR   &= ~(3 << 24);
    GPIOC->AFR[1]  &= ~(0xF << 16); GPIOC->AFR[1] |= (8 << 16);

    /* PD2(RX) AF_PP + PULLUP */
    GPIOD->MODER   &= ~(3 << 4);   GPIOD->MODER   |= (2 << 4);
    GPIOD->OTYPER  &= ~(1 << 2);
    GPIOD->OSPEEDR |=  (3 << 4);
    GPIOD->PUPDR   &= ~(3 << 4);  GPIOD->PUPDR   |= (1 << 4);
    GPIOD->AFR[0]  &= ~(0xF << 8); GPIOD->AFR[0] |= (8 << 8);

    /* 波特率 = 42MHz / (16 * bound) */
    temp = 42000000 / (bound * 16);
    mantissa = (u16)(temp);
    fraction = (u16)(42000000 % (bound * 16) * 16 / (bound * 16) + 0.5);
    UART5->BRR = (mantissa << 4) | (fraction & 0xF);

    /* 先使能UART、TX、RX（暂不开RXNE中断，防止TX稳定前的噪声） */
    UART5->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    /* 发一个换行符稳定TX线（使TC标志可正常置位） */
    UART5->DR = 0x0A;
    while (!(UART5->SR & USART_SR_TC)) {}
    /* 清除可能因TX稳定前的噪声产生的RXNE标志 */
    if (UART5->SR & USART_SR_RXNE) (void)UART5->DR;
    /* 再开RXNE中断 */
    UART5->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);

    /* 保存句柄供ISR使用 */
    UART5_Handler.Instance = UART5;
}

/* uart_init()使用直接寄存器配置，不调用HAL，故此MspInit无需实现 */

u32 tpingg(u32 tmp)
{
    u32 num = 1;
    if (tmp == 0)
    {
        return 1;
    }
    else
    {
        while (tmp--)
        {
            num *= 10;
        }
        return num;
    }
}

void send(uint16_t data)
{
    uint8_t send[10];
    u32 send0 = 0;
    u32 send1 = 0;
    u32 send2 = 0;
    send0 = data / 100;
    send1 = (data % 100) / 10;
    send2 = data % 10;
    send[0] = send0 + '0';
    send[1] = send1 + '0';
    send[2] = send2 + '0';
    (void)send;
}

/* 调试计数器：每收到一个字节+1，主循环中打印 */
volatile u32 rx_debug_cnt = 0;

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
    /* 清除可能的错误标志（ORE/NE/FE），防止中断阻塞 */
    if (UART5->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)UART5->DR;
    }
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}
#endif
