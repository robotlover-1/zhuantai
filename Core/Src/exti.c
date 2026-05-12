#include "exti.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "stdio.h"

//�ⲿ�жϳ�ʼ��
void EXTI_Init(void)
{
//    GPIO_InitTypeDef GPIO_Initure;
//    
//    __HAL_RCC_GPIOA_CLK_ENABLE();               //����GPIOAʱ��
//    
//    GPIO_Initure.Pin=GPIO_PIN_0;                //PA0
//    GPIO_Initure.Mode=GPIO_MODE_IT_RISING;      //�����ش���
//    GPIO_Initure.Pull=GPIO_PULLDOWN;
//    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
//     
//    //�ж���0-PA0
//    HAL_NVIC_SetPriority(EXTI0_IRQn,2,0);       //��ռ���ȼ�Ϊ2�������ȼ�Ϊ0
//    HAL_NVIC_EnableIRQ(EXTI0_IRQn);             //ʹ���ж���0
//	  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
}


//�жϷ�����
extern int Ring;
extern int Ding;
void EXTI0_IRQHandler(void)
{
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);		//�����жϴ������ú���
}

//�жϷ����������Ҫ��������

//GPIO_Pin:�ж����ź�
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 //printf("�����ⲿ�ж�\r\n");
    delay_ms(10);      //����
    switch(GPIO_Pin)
    {
        case GPIO_PIN_0:
				 Ring=1;
				// HAL_NVIC_DisableIRQ(EXTI0_IRQn); 
				 printf("�����ؼ��\r\n");        				 
            break;
				default: break;
    }
}
