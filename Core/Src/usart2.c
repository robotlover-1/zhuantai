#include "sys.h"
#include "usart2.h"	
#if SYSTEM_SUPPORT_OS
#include "includes.h"
#endif

#if EN_UART4_RX
u8 USART_RX_BUF_4[USART_REC_LEN_4];
u8 USART_TX_BUF_4[2];
u16 USART_RX_STA_4 = 0;
u8 aRxBuffer_4[RXBUFFERSIZE_4];
UART_HandleTypeDef UART4_Handler;

void uart4_init(u32 bound)
{
    UART4_Handler.Instance = UART4;
    UART4_Handler.Init.BaudRate = bound;
    UART4_Handler.Init.WordLength = UART_WORDLENGTH_8B;
    UART4_Handler.Init.StopBits = UART_STOPBITS_1;
    UART4_Handler.Init.Parity = UART_PARITY_NONE;
    UART4_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART4_Handler.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&UART4_Handler);
    HAL_UART_Receive_IT(&UART4_Handler, (u8 *)aRxBuffer_4, RXBUFFERSIZE_4);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_Initure = {0};

    if (huart->Instance == UART4)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_UART4_CLK_ENABLE();

        /* UART4_TX: PC10 */
        GPIO_Initure.Pin = GPIO_PIN_10;
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_Initure.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOC, &GPIO_Initure);

        /* UART4_RX: PC11 */
        GPIO_Initure.Pin = GPIO_PIN_11;
        HAL_GPIO_Init(GPIOC, &GPIO_Initure);

#if EN_UART4_RX
        HAL_NVIC_SetPriority(UART4_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
#endif
    }
}

void UART4_IRQHandler(void)
{
    u8 Res;
#if SYSTEM_SUPPORT_OS
    OSIntEnter();
#endif
    if ((__HAL_UART_GET_FLAG(&UART4_Handler, UART_FLAG_RXNE) != RESET))
    {
        Res = (u8)(UART4->DR & 0x00FF);
        if ((USART_RX_STA_4 & 0x8000) == 0)
        {
            if (USART_RX_STA_4 & 0x4000)
            {
                if (Res != 0x0a)
                    USART_RX_STA_4 = 0;
                else
                    USART_RX_STA_4 |= 0x8000;
            }
            else
            {
                if (Res == 0x0d)
                    USART_RX_STA_4 |= 0x4000;
                else
                {
                    USART_RX_BUF_4[USART_RX_STA_4 & 0X3FFF] = Res;
                    USART_RX_STA_4++;
                    if (USART_RX_STA_4 > (USART_REC_LEN_4 - 1))
                        USART_RX_STA_4 = 0;
                }
            }
        }
    }
    HAL_UART_IRQHandler(&UART4_Handler);
    HAL_UART_Receive_IT(&UART4_Handler, (u8 *)aRxBuffer_4, RXBUFFERSIZE_4);
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}
#endif
