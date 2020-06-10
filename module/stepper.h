#ifndef __PLANNER_H
#define __PLANNER_H
#include "bsp.h"
#include "type.h"
#include "config.h"

extern volatile stepper_t stepperA;
extern volatile stepper_t stepperB;
extern volatile stepper_t stepperC;
extern machine_t machine;

uint8_t Stepper_A_Pulse(void);

uint8_t Stepper_B_Pulse(void);

uint8_t Stepper_C_Pulse(void);

void Stepper_Init( volatile stepper_t* stepperX);

void Stepper_A_Update(void);

void Stepper_B_Update(void);

void Stepper_C_Update(void);

void Micro_Step_Init(uint8_t ms);


#endif
