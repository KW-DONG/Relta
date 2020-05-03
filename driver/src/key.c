#include "key.h"
#include "delay.h" 

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void KEY_Init(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
 
} 
/*
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//°´¼ü°´ËÉ¿ª±êÖ¾
	if(mode)key_up=1;  //Ö§³ÖÁ¬°´		  
	if(key_up&&(KEY0==1||WK_UP==1))
	{
		delay_ms(10);//È¥¶¶¶¯ 
		key_up=0;
		if(KEY0==1)return KEY0_PRES;
		else if(WK_UP==1)return WKUP_PRES;
	}else if(KEY0==0&&WK_UP==0)key_up=1; 	    
 	return 0;// ÎÞ°´¼ü°´ÏÂ
}
*/