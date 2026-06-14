#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"
#include "stm32f4xx_hal_uart.h" 

//////////////////////////////////////////////////////////////////////////////////	 
#define USART_REC_LEN  			200  		//定义最大接收字节数 200
#define EN_UART5_RX 			1			//使能（1）/禁止（0）UART5接收

extern u8  USART_RX_BUF[USART_REC_LEN]; 	//接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u8  USART_TX_BUF[2];
extern u16 USART_RX_STA;         			//接收状态标记	
extern UART_HandleTypeDef UART5_Handler; 	//UART5句柄
#define RXBUFFERSIZE   1 					//缓存大小

extern u8 aRxBuffer[RXBUFFERSIZE];			//HAL库UART5接收Buffer
//如果想串口中断接收，请不要注释以下宏定义

void uart_init(u32 bound);

u32 tpingg(u32 tmp);

void send(uint16_t data);
#endif
