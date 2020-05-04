#ifndef __CONFIG_H
#define __CONFIG_H

#include "sys.h"
#include "type.h"



/**
 * 				Set 	Dir
 * Stepper_A: 	PB11	PB3
 * Stepper_B:	PC6		PC7
 * Stepper_C:	PB6		PB5
 * 
 * Switch_A:    PC6
 * Switch_B:    PC7
 * Switch_C:    PD6
 * 
 * 
 * 
 */


/********************SET_BY_USER*********************/
/**
 *             ^ y 
 *             |
 *             + A
 *          +  |
 *       +     |
 *  B +--------+---> x
 *       +     |0
 *          +  |
 *             + C
 * 
 */

//DELTA properties
//length in mm
#define DELTA_CHAIN_LEN 185.0f
#define DELTA_TOWER_RADIUS 145.0f
#define DELTA_EFFECTOR_RADIUS 37.15f
#define DELTA_EFFECTOR_HEIGHT 20.0f
#define DELTA_CARRIAGE_OFFSET 16.0f
#define n1 PI*INV(2.0f)
#define n2 PI
#define n3 PI*3.0f*INV(2.0f)
#define RESOLUTION 0.1f    //mm

#define CARRIAGE_LOW    250.0f
#define CARRIAGE_HIGH   300.0f

#define CARRIAGE_A_RESET    360.0f
#define CARRIAGE_B_RESET    360.0f
#define CARRIAGE_C_RESET    360.0f

#define SWITCH_HIGHT    360.0f

#define WORKSPACE_X 180.0f
#define WORKSPACE_Y 180.0f
#define WORKSPACE_Z 50.0f

//homing speed
#define HOMING_FEEDRATE_XYZ (50*60)

//steps per mm
//use (360/1.8*16)/(2*20)
#define STEPS_PER_UNIT 80

#define MAX_FEEDRATE        300
#define MAX_ACCELERATION    3000
#define MAX_FREQ            6000
#define JERK_FREQ           200
#define STEPPER_RES         EIGH

//the speed change that does not require acceleration
//the acceleration or deceleration can be accomplished with in one step
#define XYZJERK 200

#define ARR 8400

#define MONITOR_FREQ 200

#define T_CLK 84000000//84mhz

/*********************SELECT_MODE*********************/


//use gcode command -> the machine is controlled with gcode
//else please preload the path

#define EXECUTE_MACHINE YES

#if EXECUTE_MACHINE
#define USE_PRELOAD_PATH YES
#define USE_PLANNER NO
#endif


/****************GENERATE_AUTOMATICALLY****************/

#define R   (DELTA_TOWER_RADIUS - DELTA_CARRIAGE_OFFSET)
#define r   DELTA_EFFECTOR_RADIUS
#define L   DELTA_DIAGONAL_ROD
#define x1  (R-r)*cosf(n1)
#define y1  (R-r)*sinf(n1)
#define x2  (R-r)*cosf(n2)
#define y2  (R-r)*sinf(n2)
#define x3  (R-r)*cosf(n3)
#define y3  (R-r)*sinf(n3)

#define MONITOR_PSC MONITOR_FREQ*INV(ARR)


#endif