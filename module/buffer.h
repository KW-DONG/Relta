#ifndef __BUFFER_H
#define __BUFFER_H

#include "type.h"


void Block_Buff_Init(volatile block_buff_t* block);

uint8_t Block_Buff_Write(block_t block,volatile block_buff_t* ring_buff);

void Block_Buff_Clear(volatile block_buff_t* ring_buff);

void Uart_Buff_Init(uart_buff_t* uart_buff);

uint8_t Uart_Buff_Write(uart_buff_t* uart_buff, uint8_t content);

uint8_t Uart_Buff_Read(uart_buff_t* uart_buff);

void Uart_Buff_Clear(uart_buff_t* uart_buff);

float Uart_Buff_Read_Num(uart_buff_t* uart_buff);

#endif
