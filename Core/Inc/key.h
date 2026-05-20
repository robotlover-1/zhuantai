#ifndef _KEY_H
#define _KEY_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//KEY驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/11/27
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
#define KEY0        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)  //KEY0按键PE2
#define KEY1        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)  //KEY1按键PE3
#define KEY2        HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) 	//KEY2按键PB1
#define WK_UP       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP按键PA0
#define KEY0_PRES 	1  	//KEY0按下后返回值
#define KEY1_PRES	2	//KEY1按下后返回值
#define WKUP_PRES   4	//WKUP按下后返回值
#define KEY2_PRES   6	//KEY2按下后返回值

void KEY_Init(void);  //按键IO初始化函数

u8 KEY_Scan(u8 mode); //按键扫描函数
#endif
