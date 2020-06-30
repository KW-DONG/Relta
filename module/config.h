#ifndef __CONFIG_H
#define __CONFIG_H
#include <math.h>

/**
 * IO Table
 * 				Set 	Dir     Detect  PWM
 * Stepper_A: 	PB11	PA15    PD2     TIM2_CH4
 * Stepper_B:	PC6		PC7     PC11    TIM3_CH1
 * Stepper_C:	PB6		PA6     PC9     TIM4_CH1
 * 
 * MS1:         PF6
 * MS2:         PF7
 * MS3:         PF8
 * 
 * Switch_A:    PC2
 * Switch_B:    PC3
 * Switch_C:    PC1
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
#define DELTA_CHAIN_LEN 264.0f
#define DELTA_TOWER_RADIUS 145.0f
#define DELTA_EFFECTOR_RADIUS 37.15f
#define DELTA_EFFECTOR_HEIGHT 20.0f
#define DELTA_CARRIAGE_OFFSET 16.0f
#define n1 0.0f
#define n2 3.14f
#define n3 3.14f*3.0f*INV(2.0f)
#define GRID_LEN 1.f
    //mm

#define CARRIAGE_LOW    250.0f
#define CARRIAGE_HIGH   300.0f

#define CARRIAGE_A_RESET    300.f
#define CARRIAGE_B_RESET    300.f
#define CARRIAGE_C_RESET    300.f

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
#define JERK_SPEED          20.f     //mm per sec
#define STEPPER_RES         16      //1/x mm
#define TIM_ARR             840    //1-65536
#define MONITOR_FREQ        1500      //hz
#define T_CLK               84000000//hz


/*********************SELECT_MODE*********************/

#define USE_TRAPEZOID_SPEED_PROFILE 0

#define USE_GCODE_COMMAND           0

#define USE_FORWARD_KINEMATICS      1

#define USE_SWITCH                  0

#define USE_MONITOR                 1

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

//shortcut command
#define STEPPER_A_ON                TIM_Cmd(TIM2,ENABLE)
#define STEPPER_A_OFF               TIM_Cmd(TIM2,DISABLE)
#define STEPPER_A_FREQ_UPDATE(f)    TIM_PrescalerConfig(TIM2,T_CLK/(f*TIM_ARR),TIM_PSCReloadMode_Update)
#define DIR_A_DOWN                  GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define DIR_A_UP                    GPIO_ResetBits(GPIOA, GPIO_Pin_15)
#define STEPPER_A_SCAN              (TIM2->CNT)>(TIM2->CCR4)
#define STEPPER_A_FREQ              T_CLK/(TIM2->PSC*TIM2->ARR)
#define STEPPER_A_CCR               TIM2->CCR4
#define STEPPER_A_DIR               GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15)

#define STEPPER_B_ON                TIM_Cmd(TIM3,ENABLE)
#define STEPPER_B_OFF               TIM_Cmd(TIM3,DISABLE)
#define STEPPER_B_FREQ_UPDATE(f)    TIM_PrescalerConfig(TIM3,T_CLK/(f*TIM_ARR),TIM_PSCReloadMode_Update)
#define DIR_B_DOWN                  GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define DIR_B_UP                    GPIO_ResetBits(GPIOC, GPIO_Pin_7)
#define STEPPER_B_SCAN              (TIM3->CNT)>(TIM3->CCR1)
#define STEPPER_B_FREQ              T_CLK/(TIM3->PSC*TIM3->ARR)
#define STEPPER_B_CCR               TIM3->CCR1
#define STEPPER_B_DIR               GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_7)

#define STEPPER_C_ON                TIM_Cmd(TIM4,ENABLE)
#define STEPPER_C_OFF               TIM_Cmd(TIM4,DISABLE)
#define STEPPER_C_FREQ_UPDATE(f)    TIM_PrescalerConfig(TIM4,T_CLK/(f*TIM_ARR),TIM_PSCReloadMode_Update)
#define DIR_C_DOWN                  GPIO_SetBits(GPIOA, GPIO_Pin_6)
#define DIR_C_UP                    GPIO_ResetBits(GPIOA, GPIO_Pin_6)
#define STEPPER_C_SCAN              (TIM4->CNT)>(TIM4->CCR1)
#define STEPPER_C_FREQ              T_CLK/(TIM4->PSC*TIM4->ARR)
#define STEPPER_C_CCR               TIM4->CCR1
#define STEPPER_C_DIR               GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_6)

#define FREQ2PSC(x)			        T_CLK/(x*TIM_ARR)

#define LED_RED_OFF                 GPIO_SetBits(GPIOF,GPIO_Pin_9)
#define LED_RED_ON                  GPIO_ResetBits(GPIOF,GPIO_Pin_9)
#define LED_GREEN_OFF               GPIO_SetBits(GPIOF,GPIO_Pin_10)
#define LED_GREEN_ON                GPIO_ResetBits(GPIOF,GPIO_Pin_10)

#define MS1_HIGH                    GPIO_SetBits(GPIOF,GPIO_Pin_6)
#define MS1_LOW                     GPIO_ResetBits(GPIOF,GPIO_Pin_6)
#define MS2_HIGH                    GPIO_SetBits(GPIOF,GPIO_Pin_7)
#define MS2_LOW                     GPIO_ResetBits(GPIOF,GPIO_Pin_7)
#define MS3_HIGH                    GPIO_SetBits(GPIOF,GPIO_Pin_8)
#define MS3_LOW                     GPIO_ResetBits(GPIOF,GPIO_Pin_8)

/*************************MANUAL_PATH**********************************/


#endif
