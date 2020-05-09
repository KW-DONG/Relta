#ifndef __GCODE_H
#define __GCODE_H

#include "config.h"
#include "bsp.h"

#define FAIL    0

extern uint8_t USART_LEN;

extern uint8_t USART_RX_STA;

extern uint8_t* USART_RX_BUF;

enum GCODE
{
    G0 = 1,
    G1,
    G2,
    G3,
    G4,
    G28,
    G54,
    G55
};

enum motion_type
{
    linear_t = 0,
    arc_t,
    dwell_t,
    home_t
};

extern gcode_list_t GCODE_BUFF;

void Gcode_Interpret();

float Get_Key_Word(uint8_t head, uint8_t* buffer, uint8_t len);

float Char_to_Float(uint8_t temp[], uint8_t len);

float Ascii(uint8_t value);
#endif