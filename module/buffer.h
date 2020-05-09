#ifndef __BUFFER_H
#define __BUFFER_H

#include "config.h"

void Gcode_Buff_Init(gcode_list_t* gcode_buff);

void Gcode_Buff_Write(gcode_list_t* gcode_buff, gcode_node_t* gcode_node);

void Gcode_Buff_Read(gcode_list_t* gcode_buff, gcode_node_t* temp_node);

void Gcode_Buff_Remove(gcode_list_t* gcode_buff);

void Gcode_Buff_Clear(gcode_list_t* gcode_buff);

void Block_Buff_Init(block_buff_t* block);

uint8_t Block_Buff_Write(stepper_exe_t block, block_buff_t* ring_buff);

uint8_t Block_Buff_Read(stepper_exe_t* block, block_buff_t* ring_buff);

void Block_Buff_Clear(block_buff_t* ring_buff);

void Uart_Buff_Init(uart_buff_t* uart_buff);

uint8_t Uart_Buff_Write(uart_buff_t* uart_buff, uint8_t content);

uint8_t Uart_Buff_Read(uart_buff_t* uart_buff);

void Uart_Buff_Clear(uart_buff_t* uart_buff);
#endif