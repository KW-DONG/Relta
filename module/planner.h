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
 *      use this function when a new block is read
 */
void Acc_Planner(block_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc1, int32_t* acc2);

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
void Stepper_Cnt(block_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK);

#endif