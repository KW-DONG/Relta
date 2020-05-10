#ifndef __GCODE_H
#define __GCODE_H

#include "config.h"
#include "bsp.h"

#define FAIL    0

enum GCODE
{
    G0 = 1,
    G1,
    G2,
    G3,
    G4,
    G28
};

enum Motion_Type
{
    home_t = 0,
    linear_t,
    arc_t,
    dwell_t
};


/**
 * @brief read uart buff and write gcode list
 * @param gcode_list
 * @param uart_buff
 */
void Gcode_Interpret(gcode_list_t* gcode_list, uart_buff_t* uart_buff);

/**
 * @brief char -> float
 * @param value
 * @return float
 */
float Ascii(uint8_t value);
#endif