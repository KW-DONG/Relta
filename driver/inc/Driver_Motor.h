#ifndef __DRIVER_MOTOR_H
#define __DRIVER_MOTOR_H
#include "sys.h"
#include "Driver_BSP.h"
#include "Driver_Motion.h"
#include "delay.h"
#include "config.h"


#define fullStep 0
#define halfStep 1
#define quarterStep 2
#define eighthStep 3
#define sixteenthStep 4

#define clockwise 0
#define anticlockwise 1

#define motor_X 0
#define motor_Y 1
#define motor_Z 2

//motor features
#define freq 2000 //maximum frequency is 2000 steps per second
#define slider_unit 5 //1 mm = 5 steps

typedef struct 
{
    uint32_t Move_1;//for motor x
    uint32_t Move_2;//for motor y
    uint32_t Move_3;//for motor z
    uint32_t Move_Status;//1: ready to execute, 0: executed
}Motor_Execute;


/**
 * @brief update slider position
 * @param double_t total execution time (ms)
 * @param Position_Slider slider position information
 * @note process slider motion and update gloable
 */
uint32_t Motor_Control(double time, Position_Slider *SLIDER, Position_Slider *Sub_Slider, Motor_Execute *EXECUTION);

/**
 * @brief execute bsp function
 * @note use gloable variable move_X, move_Y and move_Z
 * @note this function is called by IRQhandler
 */
void Motor_Set(Motor_Execute *EXECUTION);

#endif