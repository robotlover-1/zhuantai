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

/* HAL_UART_MspInit 已统一迁移至 usart.c (同时处理 USART1 + UART4) */

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
    /* 清除可能的错误标志（ORE/NE/FE），防止中断阻塞 */
    if (UART4->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)UART4->DR;
    }
    HAL_UART_IRQHandler(&UART4_Handler);
    HAL_UART_Receive_IT(&UART4_Handler, (u8 *)aRxBuffer_4, RXBUFFERSIZE_4);
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}
#endif
