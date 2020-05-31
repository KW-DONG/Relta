#ifndef __PLANNER_H
#define __PLANNER_H
#include "bsp.h"
#include "type.h"
#include "config.h"

extern stepper_t stepperA;
extern stepper_t stepperB;
extern stepper_t stepperC;
extern block_buff_t block_buff;
extern machine_t machine;

uint8_t Stepper_A_Pulse(void);

uint8_t Stepper_B_Pulse(void);

uint8_t Stepper_C_Pulse(void);

void Stepper_Init(stepper_t* stepperX);

void Stepper_A_Update(void);

void Stepper_B_Update(void);

void Stepper_C_Update(void);


#endif
