/**
 * @file Driver_BSP.h
 * @brief BSP initialization
 */
 
#ifndef __DRIVER_BSP_H
#define __DRIVER_BSP_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_conf.h"
#include "Driver_Motor.h"
#include "sys.h"

//USART
#define USART_REC_LEN  			200  	//maximum command length
#define EN_USART1_RX 			1		
	  	
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; 
extern uint16_t USART_RX_STA;   

void BSP_Uart_Init(uint32_t bound);

void BSP_TIM3_Init(uint16_t arr, uint16_t psc);

//motor
void BSP_Motor_Init(void);
void BSP_Motor_Set_Dir(uint32_t motorNum, uint32_t dir);
void BSP_Motor_Set_Res(uint32_t res);
void BSP_Motor_Set_Step(uint32_t motorNum);
void BSP_Motor_Reset_Step(uint32_t motorNum);

void BSP_LED_Init();
void BSP_LED_Set(uint32_t LEDNum);
void BSP_LED_Reset(uint32_t LEDNum);

void BSP_Switch_Init();
uint32_t BSP_Switch_Read(uint32_t switchNum);


#endif