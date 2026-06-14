#ifndef __SYS_H
#define __SYS_H	

#include "stm32f4xx.h"

//////////////////////////////////////////////////////////////////////////////////	 
// 本文件为ALIENTEK STM32F407开发板系统配置文件
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 创建日期:2019/9/17
// 版本：V1.7
// 版权所有，盗版必究。
// Copyright(C) 广州市星翼电子科技有限公司 2009-2019
// All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	 

/* 系统滴答时钟(ms)，在TIM6中断中递增 */
extern volatile uint32_t g_tick_ms; 
//0,不支持ucos
//1,支持ucos
#define SYSTEM_SUPPORT_OS		0		// 定义系统是否使用UCOS

	///////////////////////////////////////////////////////////////////////////////////
// 位带操作,实现51类似的GPIO控制功能
// 具体实现,参考<<CM3权威指南>>第5章(87页~92页).
// IO口操作宏定义
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;
typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  
typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;
typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 
typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;
typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8;  																    
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    
#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 

// IO口操作,只对单一IO口!
// 确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  // 输出
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  // 输入
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  // 输出
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  // 输入
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  // 输出
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  // 输入
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  // 输出
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  // 输入
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  // 输出
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  // 输入
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  // 输出
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  // 输入
#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  // 输出
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  // 输入

// 系统时钟配置

void Stm32_Clock_Init(void);					// 时钟系统配置
// 低功耗模式入口

void WFI_SET(void);				// 执行WFI指令

void INTX_DISABLE(void);		// 关闭所有中断

void INTX_ENABLE(void);			// 开启所有中断

void MSR_MSP(u32 addr);			// 设置主堆栈指针
#endif
