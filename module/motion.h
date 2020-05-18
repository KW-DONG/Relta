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
 * @param stepperX
 * @note dwell before execute the stepper
 */
void Dwell_Step_Update(block_t* stepperX);

/**
 * @brief check whether the current block is executed
 * @param stepperX
 * @param list
 * @return 0 if the current block is executed
 *         1 if the current block is executing
 */
uint8_t Block_Check(block_t* stepperX, block_buff_t* list);

/**
 * @brief count the acceleration / deceleration steps
 * @param stepperI
 * @param stepperJ
 * @param stepperK
 * @param acc_step
 * @param dcc_step
 * @param block
 */
void Acc_Cnt(stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc_step, int32_t* dcc_step, block_t* block);

/**
 * @brief count the steps (distance) that each stepper has executed
 * @param block
 * @param stepperI
 * @param stepperJ
 * @param stepperK
 * @note by detecting rising edge
 */
void Stepper_Cnt(block_t* block, machine_t* machine,stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK);

#endif
