#ifndef __USART2_H
#define __USART2_H

#include "stdio.h"	
#include "sys.h"
#include "stm32f4xx_hal_uart.h" 

//////////////////////////////////////////////////////////////////////////////////	 
#define USART_REC_LEN_4  			20  		//定义最大接收字节数 20
#define EN_UART4_RX 			1			//使能（1）/禁止（0）UART4接收

extern u8  USART_RX_BUF_4[USART_REC_LEN_4]; 	//接收缓冲,最大USART_REC_LEN_4个字节.末字节为换行符 
extern u8  USART_TX_BUF_4[2];
extern u16 USART_RX_STA_4;         			//接收状态标记	
extern UART_HandleTypeDef UART4_Handler; 	//UART4句柄
#define RXBUFFERSIZE_4   1 					//缓存大小

extern u8 aRxBuffer_4[RXBUFFERSIZE_4];		//HAL库UART4接收Buffer
//如果想串口中断接收，请不要注释以下宏定义

void uart4_init(u32 bound);
#endif
