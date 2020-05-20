#ifndef __PLANNER_H
#define __PLANNER_H
#include "bsp.h"
#include "type.h"
#include "config.h"



void Motion_Check(machine_t* machine, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK);

/**
 * @brief count dwell steps
 * @param buffer
 * @note dwell before execute the stepper
 */
void Dwell_Step_Update(block_buff_t* buffer);


/**
 * @brief count the steps (distance) that each stepper has executed
 * @param exe
 * @param machine
 * @param stepperX
 * @note by detecting rising edge
 */
void Stepper_Count(block_buff_t* buffer, machine_t* machine,stepper_t* stepperX);

#endif
