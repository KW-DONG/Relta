#include "Driver_IRQHandler.h"
#include "Driver_Motor.h"

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

void USART1_IRQHandler(void)
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{
		Res =USART_ReceiveData(USART1);
		
		if((USART_RX_STA&0x8000)==0)
		{
			if(USART_RX_STA&0x4000)
			{
				if(Res!=0x0a)USART_RX_STA=0;
				else USART_RX_STA|=0x8000;
			}
			else
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					Decoder();
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;
				}		 
			}
        }
	}   		 
} 

void TIM3_IRQHandler(void)
{
	Motor_Set(&EXECUTION);
}