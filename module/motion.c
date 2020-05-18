#include "planner.h"
#include "buffer.h"
#include "delta.h"
#include "motion.h"
#include "config.h"
#include "math.h"

//the functions will be called in the interrupt
//avoid using complex calulation and float calculation

void Dwell_Step_Update(block_t* stepperX)
{
    if (stepperX->step_dwell!=0)
    stepperX->step_dwell--;
}

uint8_t Block_Check(block_t* blockX, block_buff_t* list)
{
    uint8_t block_state;
    //whether current block is compeleted
    if (blockX->step[0]==0&&blockX->step[1]==0&&blockX->step[2]==0)
    block_state = Block_Buff_Read(blockX, list);
    if (block_state == TRUE) return 0;
    else return 1;
}

void Acceleration_Count(stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc_step, int32_t* dcc_step, block_t* block)
{
    //if acc
    if (acc_step[0]!=0)
    {
        stepperI->freq = stepperI->freq + ACC;
        acc_step[0]--;
        dcc_step[0]--;
    }else if (dcc_step[0]==0)
    {
        stepperI->freq = stepperI->freq - ACC;
    }else
    {
        stepperI->freq = block->norminal_freq[0];
        dcc_step[0]--;
    }
/*************************************************/
    if (acc_step[1]!=0)
    {
        stepperJ->freq = stepperJ->freq + ACC;
        acc_step[1]--;
        dcc_step[1]--;
    }else if (dcc_step[1]==0)
    {
        stepperJ->freq = stepperJ->freq - ACC;
    }else
    {
        stepperJ->freq = block->norminal_freq[1];
        dcc_step[1]--;
    }
/*************************************************/
    if (acc_step[2]!=0)
    {
        stepperK->freq = stepperK->freq + ACC;
        acc_step[2]--;
        dcc_step[2]--;
    }else if (dcc_step[2]==0)
    {
        stepperK->freq = stepperK->freq - ACC;
    }else
    {
        stepperK->freq = block->norminal_freq[2];
        dcc_step[2]--;
    }
}

void Stepper_Count(block_t* block, machine_t* machine,stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK)
{
    if (stepperI->pin_state_last==0&&stepperI->pin_state==1)
    {
        block->step[0]--;
        if (stepperI->dir == 0) machine->abc[0]-INV(STEPS_PER_UNIT);
        else                    machine->abc[0]+INV(STEPS_PER_UNIT);
    }
    if (stepperJ->pin_state_last==0&&stepperJ->pin_state==1)
    {
        block->step[1]--;
        if (stepperJ->dir == 0) machine->abc[1]-INV(STEPS_PER_UNIT);
        else                    machine->abc[1]+INV(STEPS_PER_UNIT);
    }
    if (stepperK->pin_state_last==0&&stepperK->pin_state==1)    
    {
        block->step[2]--;
        if (stepperK->dir == 0) machine->abc[2]-INV(STEPS_PER_UNIT);
        else                    machine->abc[2]+INV(STEPS_PER_UNIT);
    }
}

void Motion_Check(machine_t* machine, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK)
{
    //check carriage reset point and motor direction
    if (machine->abc[0]==CARRIAGE_A_RESET&&stepperI->dir==1)
    {
        stepperI->dir=stepper_DOWN;
        stepperI->state=stepper_OFF;
        Bsp_UART_Send("STEPPER_A_FAIL",15);
    }
    if (machine->abc[1]==CARRIAGE_B_RESET&&stepperJ->dir==1)
    {
        stepperJ->dir=stepper_DOWN;
        stepperJ->state=stepper_OFF;
        Bsp_UART_Send("STEPPER_B_FAIL",15);
    }
    if (machine->abc[0]==CARRIAGE_A_RESET&&stepperI->dir==1)
    {
        stepperK->dir=stepper_DOWN;
        stepperK->state=stepper_OFF;
        Bsp_UART_Send("STEPPER_C_FAIL",15);
    }

    //check boundary
    if (machine->xyz[0]>=90.0f||machine->xyz[0]<=-90.0f)
    {
        machine->state = machine_OFF;
        Bsp_UART_Send("END_X_FAIL",11);
    }else if (machine->xyz[1]>=90.0f||machine->xyz[1]<=-90.0f)
    {
        machine->state = machine_OFF;
        Bsp_UART_Send("END_Y_FAIL",11);
    }else if (machine->xyz[2]<=0.1f)
    {
        machine->state = machine_OFF;
        Bsp_UART_Send("END_Z_FAIL",11);
    }
}
