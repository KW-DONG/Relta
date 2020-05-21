#ifndef __CONFIG_H
#define __CONFIG_H
#include <math.h>

/**
 * IO Table
 * 				Set 	Dir
 * Stepper_A: 	PB11	PA15
 * Stepper_B:	PC6		PC7
 * Stepper_C:	PB6		PA6
 * 
 * MS1:         PF6
 * MS2:         PF7
 * MS3:         PF8
 * 
 * Switch_A:    PC2
 * Switch_B:    PC3
 * Switch_C:    PC4
 * Switch_R:    PC1
 * Switch_S:    PC0
 * 
 * LED_G:       PF9
 * LED_R:       PF10
 * 
 * UART_TX:     PA9
 * UART_RX:     PA10
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
#define n1 0.0f
#define n2 3.14f
#define n3 3.14f*3.0f*INV(2.0f)
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
#define STEPS_PER_UNIT      80

//carriage specification
#define MAX_ACCELERATION    1000    //mm per sec per sec
#define MAX_SPEED           1000    //mm per sec
#define JERK_SPEED          200     //mm per sec
#define STEPPER_RES         16      //1/x mm
#define TIM_ARR             8400    //1-65536
#define MONITOR_FREQ        200     //hz
#define T_CLK               84000000//hz

/*********************SELECT_MODE*********************/

#define USE_SPEED_CONTROLLER    1

#define USE_GCODE_COMMAND       1

#define USE_FORWARD_KINEMATICS  1

#define USE_SWITCH              0

#define USE_MONITOR             1

/****************GENERATE_AUTOMATICALLY****************/

#define R   (DELTA_TOWER_RADIUS - DELTA_CARRIAGE_OFFSET)
#define r   DELTA_EFFECTOR_RADIUS
#define L   DELTA_CHAIN_LEN
#define x1  (R-r)*cosf(n1)
#define y1  (R-r)*sinf(n1)
#define x2  (R-r)*cosf(n2)
#define y2  (R-r)*sinf(n2)
#define x3  (R-r)*cosf(n3)
#define y3  (R-r)*sinf(n3)

#define MONITOR_PSC MONITOR_FREQ/(TIM_ARR)*T_CLK

#define PSC_MIN     T_CLK/(MAX_SPEED*STEPS_PER_UNIT*TIM_ARR)

#define PSC_ACC     T_CLK/(MAX_ACCELERATION*STEPS_PER_UNIT*TIM_ARR)

#define PSC_JERK    T_CLK/(JERK_SPEED*STEPS_PER_UNIT*TIM_ARR)




#endif
