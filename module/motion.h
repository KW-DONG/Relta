#ifndef __PLANNER_H
#define __PLANNER_H
#include "bsp.h"
#include "type.h"

#define CLOCKWISE       0
#define ANTICLOCKWISE   1

//machine status
#define WORK            0
#define SLEEP           1

#define ARR     84-1

#define START    ARR/2
#define STOP   0

#define ACC     SQ(MAX_FREQ)/(STEPPER_RES*MONITOR_FREQ)

void Motion_Check(machine_t* machine, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK);

/**
 * @brief count dwell steps
 * @param exe
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
