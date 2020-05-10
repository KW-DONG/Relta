#ifndef __PLANNER_H
#define __PLANNER_H

#include "type.h"
#include "config.h"
#include "bsp.h"

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

/**
 * @brief generate the acceleration and deceleration plan
 * @param block
 * @param stepperI
 * @param stepperJ
 * @param stepperK
 * @param acc1  steps need to accelerate from init speed to operation speed
 * @param acc2  steps need to decelerate from opeation speed to jerk speed
 * @note if the operation speed is lower than the jerk speed, acc2 will not be planned
 */
void Acc_Planner(stepper_exe_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc1, int32_t* acc2);

void Dwell_Step_Update(stepper_exe_t* stepper_abc);

uint8_t Block_Check(stepper_exe_t* stepper_abc, block_buff_t* list);

void Acc_Cnt(stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc_step, int32_t* dcc_step, stepper_exe_t* block);

void Stepper_Cnt(stepper_exe_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK);

#endif