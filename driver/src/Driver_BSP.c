#include "Driver_BSP.h"
u8 USART_RX_BUF[USART_REC_LEN]; 

u16 USART_RX_STA=0;

// USART
void BSP_Uart_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
 

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
	
    USART_Cmd(USART1, ENABLE);
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
    #if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	//Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	

    #endif
	
}

void BSP_TIM3_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
    TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


//EXTI
void BSP_EXTIX_Init(void)
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


//motor
void BSP_Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //msx
    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);             
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //step and dir
    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);             
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);             
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}

void BSP_Motor_Set_Dir(u32 motorNum, u32 dir)
{
    switch (motorNum)
    {
        case 0:
        {
            if (dir == 0) GPIO_SetBits(GPIOC,GPIO_Pin_7);
            if (dir == 1) GPIO_ResetBits(GPIOC,GPIO_Pin_7);
        }
        case 1;
        {
            if (dir == 0) GPIO_SetBits(GPIOD,GPIO_Pin_7);
            if (dir == 1) GPIO_ResetBits(GPIOD,GPIO_Pin_7);
        }
        case 2;
        {
            if (dir == 0) GPIO_SetBits(GPIOC,GPIO_Pin_9);
            if (dir == 1) GPIO_ResetBits(GPIOC,GPIO_Pin_9);
        }
    }
}

void BSP_Motor_Set_Res(u32 res)
{
    switch(res)
    {
      //full step
      case 0: 
      {
        GPIO_ResetBits(GPIOB,GPIO_Pin_3);
        GPIO_ResetBits(GPIOB,GPIO_Pin_5);
        GPIO_ResetBits(GPIOB,GPIO_Pin_6);
      }

      //half step
      case 1:
      {
        GPIO_SetBits(GPIOB,GPIO_Pin_3);
        GPIO_ResetBits(GPIOB,GPIO_Pin_5);
        GPIO_ResetBits(GPIOB,GPIO_Pin_6);
      }

      //quarter step
      case 2:
      {
        GPIO_ResetBits(GPIOB,GPIO_Pin_3);
        GPIO_SetBits(GPIOB,GPIO_Pin_5);
        GPIO_ResetBits(GPIOB,GPIO_Pin_6);
      }
      case 3:
      {
        GPIO_SetBits(GPIOB,GPIO_Pin_3);
        GPIO_SetBits(GPIOB,GPIO_Pin_5);
        GPIO_ResetBits(GPIOB,GPIO_Pin_6);
      }
      case 4:
      {
        GPIO_SetBits(GPIOB,GPIO_Pin_3);
        GPIO_SetBits(GPIOB,GPIO_Pin_5);
        GPIO_SetBits(GPIOB,GPIO_Pin_6);
      }
    }
}

void BSP_Motor_Set_Step(u32 motorNum)
{
    switch (motorNum)
    {
        case 0: GPIO_SetBits(GPIOC,GPIO_Pin_6);
        case 1: GPIO_SetBits(GPIOD,GPIO_Pin_6);
        case 2: GPIO_SetBits(GPIOC,GPIO_Pin_8);
    }
}

void BSP_Motor_Reset_Step(u32 motorNum)
{
    switch (motorNum)
    {
        case 0: GPIO_ResetBits(GPIOC,GPIO_Pin_6);
        case 1: GPIO_ResetBits(GPIOD,GPIO_Pin_6);
        case 2: GPIO_ResetBits(GPIOC,GPIO_Pin_8);
    }
}


//LED
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);
}

void BSP_LED_Set(u32 LEDNum)
{
    switch (LEDNum)
    {
        case 0: GPIO_ResetBits(GPIOF,GPIO_Pin_9);
        case 1: GPIO_ResetBits(GPIOF,GPIO_Pin_10);
    }
}

void BSP_LED_Reset(u32 LEDNum)
{
    switch (LEDNum)
    {
        case 0: GPIO_GPIO_SetBits(GPIOF,GPIO_Pin_9);
        case 1: GPIO_GPIO_SetBits(GPIOF,GPIO_Pin_10);
    }
}


//Switch
void BSP_Switch_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);             
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;  
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

u32 BSP_Switch_Read(u32 switchNum)
{
    u32 i;
    switch (switchNum)
    {
        case 0: i = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0);
        case 1: i = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_1);
        case 2: i = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3);

    }
    return i;
}