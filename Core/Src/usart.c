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
    while (!(USART1->SR & USART_SR_TC))
    {
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

    /* 使能USART、TX、RX、RXNE中断 */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* 保存句柄供ISR使用 */
    UART1_Handler.Instance = USART1;
}

void HAL_USART_MspInit(USART_HandleTypeDef *husart)
{
    GPIO_InitTypeDef GPIO_Initure = {0};

    if (husart->Instance == USART1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_Initure.Pin = GPIO_PIN_9;
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_Initure.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_Initure);

        GPIO_Initure.Pin = GPIO_PIN_10;
        HAL_GPIO_Init(GPIOA, &GPIO_Initure);

#if EN_USART1_RX
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif
    }
}

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
