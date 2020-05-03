#ifndef __GCODE_H
#define __GCODE_H

#include "config.h"

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

extern linked_list_t GCODE_BUFF;

void Gcode_Interpret();

float Get_Value(uint8_t len, uint8_t ch);

float Char_to_Float(uint8_t temp[], uint8_t len);

float Ascii(uint8_t value);
#endif