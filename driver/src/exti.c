#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "key.h"



void EXTI0_IRQHandler(void)
{
	delay_ms(10);
	if(KEY_X == 0)	 
	{
		LED0 = 0;
		delay_ms(100);
	}		 
	EXTI_ClearITPendingBit(EXTI_Line0); 
	//EXTI_ClearFlag;
}	


void EXTI2_IRQHandler(void)
{
	delay_ms(10);
	while(KEY_Y==0)	 
	{				 
		//LED0=!LED0;	
		LED0 = 0;	
		delay_ms(1000);
		LED0 = 1;
		delay_ms(1000);
	}		 
	 EXTI_ClearITPendingBit(EXTI_Line2);
}

void EXTI3_IRQHandler(void)
{
	delay_ms(10);
	if(KEY_Z==0)	 
	{				 
		//LED0=!LED0;	
		LED0 = 0;	
		delay_ms(100);
	}		 
	 EXTI_ClearITPendingBit(EXTI_Line3);
}



void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	KEY_Init(); 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	/* ÅäÖÃEXTI_Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);//ÅäÖÃ

	/* ÅäÖÃEXTI_Line4 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//ÖÐ¶ÏÊÂ¼þ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //ÏÂ½µÑØ´¥·¢
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ÖÐ¶ÏÏßÊ¹ÄÜ
	EXTI_Init(&EXTI_InitStructure);//ÅäÖÃ

	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//ÖÐ¶ÏÊÂ¼þ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //ÏÂ½µÑØ´¥·¢
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ÖÐ¶ÏÏßÊ¹ÄÜ
	EXTI_Init(&EXTI_InitStructure);//ÅäÖÃ

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//Íâ²¿ÖÐ¶Ï0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//ÇÀÕ¼ÓÅÏÈ¼¶0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//×ÓÓÅÏÈ¼¶2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//Ê¹ÄÜÍâ²¿ÖÐ¶ÏÍ¨µÀ
	NVIC_Init(&NVIC_InitStructure);//ÅäÖÃ

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//Íâ²¿ÖÐ¶Ï4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//ÇÀÕ¼ÓÅÏÈ¼¶1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//×ÓÓÅÏÈ¼¶2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//Ê¹ÄÜÍâ²¿ÖÐ¶ÏÍ¨µÀ
	NVIC_Init(&NVIC_InitStructure);//ÅäÖÃ

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//Íâ²¿ÖÐ¶Ï4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//ÇÀÕ¼ÓÅÏÈ¼¶1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//×ÓÓÅÏÈ¼¶2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//Ê¹ÄÜÍâ²¿ÖÐ¶ÏÍ¨µÀ
	NVIC_Init(&NVIC_InitStructure);//ÅäÖÃ
  
}