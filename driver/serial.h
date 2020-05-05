
#ifndef __SERIAL_H
#define __SERIAL_H

#include "config.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "stm32f4xx.h"

#define USART_LEN  			200 //定义最大接收字节数 200
#define EN_USART1_RX 			1   //使能（1）/禁止（0）串口1接收

#define SUCCESS 0
#define FAIL    1

extern uint8_t  USART_RX_BUF[USART_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void Uart_Init(uint32_t bound);
void Send_Feedback(uint8_t status);
void Report_Coordinate();
void Buff_Init();

#endif