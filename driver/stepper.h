#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "type.h"
#include "config.h"

#define CLOCKWISE       0
#define ANTICLOCKWISE   1

//machine status
#define WORK            0
#define SLEEP           1
#define RESET           2

#define ARR     84-1

#define START    ARR/2
#define STOP   0

#define ACC     SQ(MAX_FREQ)/(STEPPER_RES*MONITOR_FREQ)

extern switch_t switchA;
extern switch_t switchB;
extern switch_t switchC;


typedef struct
{
    //A4988 set pin
    uint32_t        RCC_AHB1Periph_GPIOX;
    uint32_t        GPIO_Pin_X;
    uint32_t        RCC_APB1Periph_TIMX;
    uint16_t        GPIO_PinSourceX;
    uint8_t         GPIO_AF_TIMX;
    uint8_t         PWM_Ch;
    GPIO_TypeDef*   GPIOX;
    TIM_TypeDef*    TIMX;

    //A4988 dir pin
    uint32_t        RCC_AHB1Periph_GPIOX_Dir;
    uint32_t        GPIO_Pin_X_Dir;
    GPIO_TypeDef*   GPIOX_Dir;

    //A4988 MS1
    uint32_t        RCC_AHB1Periph_GPIOX_MS1;
    uint32_t        GPIO_Pin_X_MS1;
    GPIO_TypeDef*   GPIOX_MS1;
    
    //A4988 MS2
    uint32_t        RCC_AHB1Periph_GPIOX_MS2;
    uint32_t        GPIO_Pin_X_MS2;
    GPIO_TypeDef*   GPIOX_MS2;

    //A4988 MS3
    uint32_t        RCC_AHB1Periph_GPIOX_MS3;
    uint32_t        GPIO_Pin_X_MS3;
    GPIO_TypeDef*   GPIOX_MS3;

    uint8_t         state;//compare
    uint8_t         pin_state;
    uint8_t         pin_state_last;
    uint8_t         dir;
    int32_t         freq;//steps per second
    uint16_t        psc;

}stepper_t;


/**
 * 				Set 	Dir
 * Stepper 1: 	PB11	PB3
 * Stepper 2:	PC6		PC7
 * Stepper 3:	PB6		PB5
 * 
 */
extern stepper_t    stepperA;
extern stepper_t    stepperB;
extern stepper_t    stepperC;

extern stepper_exe_t block_c;

extern machine_t machine;

extern block_buff_t block_list;


//用占空比关电机
void Stepper_Step_Update(stepper_exe_t* blockX, stepper_t* stepperI,stepper_t* stepperJ, stepper_t* stepperK);

void Dwell_Step_Update(stepper_exe_t* stepper_abc);

void Block_To_Stepper(stepper_exe_t* blockX, stepper_t* stepperI,stepper_t* stepperJ, stepper_t* stepperK);

void Stepper_Direction();

uint8_t Block_Check(stepper_exe_t* stepper_abc, block_buff_t* list);

void Acc_Cnt(stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc_step, int32_t* dcc_step, stepper_exe_t* block);

void Acc_Planner(stepper_exe_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc1, int32_t* acc2);

void Stepper_Cnt(stepper_exe_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK);

//BSP
void Stepper_Init(stepper_t* stepperX);

void TIM5_Init(uint16_t arr, uint16_t psc);

void Stepper_Upgrade(stepper_t*stepperX);



#endif