#include "bsp.h"
#include "config.h"

void Bsp_Stepper_Init(stepper_t* stepperX)
{
    // NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	//Enables or disables the AHB1 peripheral clock.
	RCC_AHB1PeriphClockCmd(stepperX->RCC_AHB1Periph_GPIOX,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= stepperX->GPIO_Pin_X;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_Init(stepperX->GPIOX,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = stepperX->GPIO_Pin_X_Dir;
    GPIO_Init(stepperX->GPIOX_Dir,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = stepperX->GPIO_Pin_X_MS1;
    GPIO_Init(stepperX->GPIOX_MS1,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = stepperX->GPIO_Pin_X_MS2;
    GPIO_Init(stepperX->GPIOX_MS2,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = stepperX->GPIO_Pin_X_MS3;
    GPIO_Init(stepperX->GPIOX_MS3,&GPIO_InitStructure);

	//Enables the Low Speed APB (APB1) peripheral clock.
	RCC_APB1PeriphClockCmd(stepperX->RCC_APB1Periph_TIMX, ENABLE);
	
	//Changes the mapping of the specified pin.
	GPIO_PinAFConfig(stepperX->GPIOX,stepperX->GPIO_PinSourceX,stepperX->GPIO_AF_TIMX);

	TIM_TimeBaseInitStructure.TIM_Period        = stepperX->arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler     = stepperX->psc;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Down;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	//Initializes the TIMx Time Base Unit peripheral according to 
    //the specified parameters in the TIM_TimeBaseInitStruct.
	TIM_TimeBaseInit(stepperX->TIMX, &TIM_TimeBaseInitStructure);
    
	//Enables the specified TIM peripheral.
	TIM_Cmd(stepperX->TIMX, ENABLE);

	//Timer output compare
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;        // 选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;               // 输出极性:TIM输出比较极性低
    
	//Initializes the TIMx Channel1 according to the specified parameters in
	//the TIM_OCInitStruct.
	TIM_OCInitStructure.TIM_Pulse       = 0;//0 or arr/2, CCRx_Val

    if (stepperX->PWM_Ch==1)
    {
        TIM_OC1Init(stepperX->TIMX, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(stepperX->TIMX, TIM_OCPreload_Disable);
    }else if (stepperX->PWM_Ch==2)
    {
        TIM_OC2Init(stepperX->TIMX, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(stepperX->TIMX, TIM_OCPreload_Disable);
    }else if (stepperX->PWM_Ch==3)
    {
        TIM_OC3Init(stepperX->TIMX, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(stepperX->TIMX, TIM_OCPreload_Disable);
    }else
    {
        TIM_OC4Init(stepperX->TIMX, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(stepperX->TIMX, TIM_OCPreload_Disable);
    }
	//Enables the TIMx peripheral Preload register on CCR1.
	//Enables the specified TIM peripheral.
	TIM_Cmd(stepperX->TIMX, ENABLE);
    
}

void Bsp_Stepper_Update(stepper_t* stepperX)
{
    //update stepper state
    if (stepperX->PWM_Ch==1)
    {
        if (stepperX->state==stepper_ON)
        TIM_SetCompare1(stepperX->TIMX, stepperX->arr/2);
        else
        TIM_SetCompare1(stepperX->TIMX,0);
    }else if (stepperX->PWM_Ch==2)
    {
        if (stepperX->state==stepper_ON)
        TIM_SetCompare2(stepperX->TIMX, stepperX->arr/2);
        else
        TIM_SetCompare2(stepperX->TIMX,0);
    }else if (stepperX->PWM_Ch==3)
    {
        if (stepperX->state==stepper_ON)
        TIM_SetCompare3(stepperX->TIMX, stepperX->arr/2);
        else
        TIM_SetCompare3(stepperX->TIMX,0);
    }else
    {
        if (stepperX->state==stepper_ON)
        TIM_SetCompare4(stepperX->TIMX, stepperX->arr/2);
        else
        TIM_SetCompare4(stepperX->TIMX,0);
    }

    //update speed
    uint16_t psc = 84000000/(stepperX->freq*stepperX->arr);
    TIM_PrescalerConfig(stepperX->TIMX,psc,TIM_PSCReloadMode_Update);

    //update direction
    if (stepperX->dir==0)   GPIO_SetBits(stepperX->GPIOX_Dir, stepperX->GPIO_Pin_X_Dir);
    else                    GPIO_ResetBits(stepperX->GPIOX_Dir, stepperX->GPIO_Pin_X_Dir);

    //update IO state
    stepperX->pin_state_last = stepperX->pin_state;
    stepperX->pin_state = GPIO_ReadInputDataBit(stepperX->GPIOX, stepperX->GPIO_Pin_X);
}

void Bsp_Monitor_Init()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
    TIM_TimeBaseInitStructure.TIM_Period = TIM_ARR; 
	TIM_TimeBaseInitStructure.TIM_Prescaler=MONITOR_PSC;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM5,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Bsp_Switch_Init(switch_t* switchX)
{
    NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(switchX->EXTI_PortSourceGPIOX, switchX->EXTI_PinSourceX);

	EXTI_InitStructure.EXTI_Line = switchX->EXTI_LineX;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

    if (switchX->mode==NO)
    {
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    }else
    {
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    }
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = switchX->EXTIX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = switchX->NVIC_PP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = switchX->NVIC_SP;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Bsp_LED_Init(led_t* LEDX)
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

void Bsp_LED_Update(led_t* LEDX)
{
    if (LEDX->state==0)
    GPIO_SetBits(LEDX->GPIOX,LEDX->GPIO_Pin_X);
    else
    GPIO_ResetBits(LEDX->GPIOX, LEDX->GPIO_Pin_X);
}

void Bsp_UART_Init(uint32_t bound)
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
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
		
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
}

void Bsp_UART_Send(uint8_t* content, uint8_t len)
{
    uint8_t i;
    for (i=0;i<len;i++)
    {
        USART_SendData(USART1, content[i]);
        while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
    }
}
