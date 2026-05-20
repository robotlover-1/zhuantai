#include "exti.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "stdio.h"

//外部中断初始化
void EXTI_Init(void)
{
//    GPIO_InitTypeDef GPIO_Initure;
//    
//    __HAL_RCC_GPIOA_CLK_ENABLE();               //开启GPIOA时钟
//    
//    GPIO_Initure.Pin=GPIO_PIN_0;                //PA0
//    GPIO_Initure.Mode=GPIO_MODE_IT_RISING;      //上升沿触发
//    GPIO_Initure.Pull=GPIO_PULLDOWN;
//    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
//     
//    //中断线0-PA0
//    HAL_NVIC_SetPriority(EXTI0_IRQn,2,0);       //抢占优先级为2，子优先级为0
//    HAL_NVIC_EnableIRQ(EXTI0_IRQn);             //使能中断线0
//	  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
}

//外部中断初始化
extern int Ring;
extern int Ding;
void EXTI0_IRQHandler(void)
{
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);		//调用中断处理公用函数
}

//外部中断服务函数

//GPIO_Pin:中断引脚号
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 //printf("进入外部中断\r\n");
    delay_ms(10);      //消抖
    switch(GPIO_Pin)
    {
        case GPIO_PIN_0:
				 Ring=1;
				// HAL_NVIC_DisableIRQ(EXTI0_IRQn); 
	 //printf("进入外部中断\r\n");
            break;
				default: break;
    }
}
