#include "switch.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "type.h"
#include "buffer.h"
#include "stepper.h"


void Switch_Init(switch_t* switchX)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(switchX->RCC_AHB1Periph_GPIOX, ENABLE);
	GPIO_InitStructure.GPIO_Pin 	= switchX->GPIO_PinX;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	
	GPIO_Init(switchX->GPIOX,&GPIO_InitStructure);//PB6, PB11
}

void Switch_Read_State(switch_t* switchX)
{
    if (GPIO_ReadInputDataBit(switchX->GPIOX,switchX->GPIO_PinX) == 1)
    switchX->state = 0;
    else
    switchX->state = 1;
}


void Switch_EXTI_Init(void)
{
    NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void EXTI0_IRQHandler(void)
{
	delay_ms(10);
	{
		if (machine.state != ON) machine.state = ON;
        else machine.state = OFF;
		delay_ms(100);
	}		 
	 EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI4_IRQHandler(void)
{
	delay_ms(10);
	{
		machine.state = OFF;
        machine.xyz_v[0] = 0.0f;
        machine.xyz_v[1] = 0.0f;
        machine.xyz_v[2] = 0.0f;
        stepperA.state = STOP;
        stepperB.state = STOP;
        stepperC.state = STOP;

        Gcode_Buff_Clear(&gcode_list);
        Gcode_Buff_Init(&gcode_list);

        Ring_Buff_Clear(&block_list);
        Ring_Buff_Init(&block_list);
        
		delay_ms(100);
	}		 
	EXTI_ClearITPendingBit(EXTI_Line4);
}

