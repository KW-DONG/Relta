#ifndef __BUFFER_H
#define __BUFFER_H

#include "config.h"

void Gcode_Buff_Init(linked_list_t* GCODE_BUFFER);

void Gcode_Buff_Write(linked_list_t* GCODE_BUFFER, gcode_node_t* gcode_node);

void Gcode_Buff_Read(linked_list_t* GCODE_BUFFER, gcode_node_t* temp_node);

void Gcode_Buff_Remove(linked_list_t* GCODE_BUFFER);

void Gcode_Buff_Clear(linked_list_t* GCODE_BUFFER);

void Ring_Buff_Init(ring_buff_t* block);

uint8_t Ring_Buff_Write(stepper_exe_t block, ring_buff_t* ring_buff);

uint8_t Ring_Buff_Read(stepper_exe_t* block, ring_buff_t* ring_buff);

void Ring_Buff_Clear(ring_buff_t* ring_buff);

#endif