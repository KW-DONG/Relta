#include "led.h"


void LED_Init(led_t* LEDX)
{    	 
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(LEDX->RCC_AHB1Periph_GPIOX, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LEDX->GPIO_Pin_X;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(LEDX->GPIOX, &GPIO_InitStructure);
    GPIO_SetBits(LEDX->GPIOX,LEDX->GPIO_Pin_X);
}

void LED_Update(led_t* LEDX)
{
    if (LEDX->state==0)
    GPIO_SetBits(LEDX->GPIOX,LEDX->GPIO_Pin_X);
    else
    GPIO_ResetBits(LEDX->GPIOX, LEDX->GPIO_Pin_X);
}