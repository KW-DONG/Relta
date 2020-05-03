#ifndef __DRIVER_GCODE_
#define __DRIVER_GCODE_

#include "delay.h"
#include "led.h"
#include "Driver_BSP.h"
#include "sys.h"
#include "Driver_Motion.h"
#include "config.h"
#include "stm32f4xx_usart.h"

void Decoder();

void Gcode_G00();

void Gcode_G01();

void Gcode_G02();

void Gcode_G03();

void Send_Feedback();//send feedback to GUI

/**
 * @brief read figures from g code
 * @param t No. t+1 is the highest order of the figure
 * @note resolution supported is 0.1mm, output unit is 0.1mm
 */
uint32_t Read_Data(uint32_t t);


#endif