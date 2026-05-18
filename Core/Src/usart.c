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
    while (!(USART1->SR & USART_SR_TC))
    {
        if (--timeout == 0)
        {
            break;
        }
    }
    USART1->DR = (uint8_t)ch;
    return ch;
}
#endif

#if EN_USART1_RX
u8 USART_RX_BUF[USART_REC_LEN];
u8 USART_TX_BUF[2];
u16 USART_RX_STA = 0;
u8 aRxBuffer[RXBUFFERSIZE];
USART_HandleTypeDef UART1_Handler;

void uart_init(u32 bound)
{
    u32 temp;
    u16 mantissa;
    u16 fraction;

    /* 直接寄存器配置，绕过HAL */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     /* GPIOA时钟 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;    /* USART1时钟 */

    /* PA9(TX) AF_PP */
    GPIOA->MODER   &= ~(3 << 18);  GPIOA->MODER   |= (2 << 18);
    GPIOA->OTYPER  &= ~(1 << 9);
    GPIOA->OSPEEDR |=  (3 << 18);
    GPIOA->PUPDR   &= ~(3 << 18);
    GPIOA->AFR[1]  &= ~(0xF << 4); GPIOA->AFR[1] |= (7 << 4);

    /* PA10(RX) AF_PP + PULLUP */
    GPIOA->MODER   &= ~(3 << 20);  GPIOA->MODER   |= (2 << 20);
    GPIOA->OTYPER  &= ~(1 << 10);
    GPIOA->OSPEEDR |=  (3 << 20);
    GPIOA->PUPDR   &= ~(3 << 20);  GPIOA->PUPDR   |= (1 << 20);
    GPIOA->AFR[1]  &= ~(0xF << 8); GPIOA->AFR[1] |= (7 << 8);

    /* 波特率 = 84MHz / (16 * bound) */
    temp = 84000000 / (bound * 16);
    mantissa = (u16)(temp);
    fraction = (u16)(84000000 % (bound * 16) * 16 / (bound * 16) + 0.5);
    USART1->BRR = (mantissa << 4) | (fraction & 0xF);

    /* 先使能USART、TX、RX（暂不开RXNE中断，防止TX稳定前的噪声） */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    /* 发一个换行符稳定TX线（使TC标志可正常置位） */
    USART1->DR = 0x0A;
    while (!(USART1->SR & USART_SR_TC)) {}
    /* 清除可能因TX稳定前的噪声产生的RXNE标志 */
    if (USART1->SR & USART_SR_RXNE) (void)USART1->DR;
    /* 再开RXNE中断 */
    USART1->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* 保存句柄供ISR使用 */
    UART1_Handler.Instance = USART1;
}

/* uart_init()使用直接寄存器配置，不调用HAL_USART_Init()，故此MspInit无需实现 */

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

void USART1_IRQHandler(void)
{
    u8 Res;
#if SYSTEM_SUPPORT_OS
    OSIntEnter();
#endif
    if (USART1->SR & USART_SR_RXNE)
    {
        Res = (u8)(USART1->DR & 0x00FF);
        
        
  
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
    if (USART1->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)USART1->DR;
    }
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}
#endif
