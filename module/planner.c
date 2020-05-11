#include "planner.h"
#include "buffer.h"
#include "delta.h"
#include "motion.h"
#include "config.h"

void Acc_Planner(block_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc1, int32_t* acc2)
{
    int32_t t_acc1[3];
    int32_t t_acc2[3];
    int32_t v_entr[3];
    int32_t v_out[3];

    if (block->dir[0]==stepperI->dir)   v_entr[0]==stepperI->freq;
    else                                v_entr[0]==0;

    if (block->dir[1]==stepperJ->dir)   v_entr[1]==stepperJ->freq;
    else                                v_entr[1]==0;

    if (block->dir[2]==stepperK->dir)   v_entr[2]==stepperK->freq;
    else                                v_entr[2]==0;

    if (block->freq[0]>JERK_FREQ)       v_out[0]==JERK_FREQ;
    else                                v_out[0]==block->freq[0];

    if (block->freq[1]>JERK_FREQ)       v_out[1]==JERK_FREQ;
    else                                v_out[1]==block->freq[1];

    if (block->freq[2]>JERK_FREQ)       v_out[2]==JERK_FREQ;
    else                                v_out[2]==block->freq[2];

    //calculate distance -> s
    int32_t s[3] = {block->step[0],block->step[1],block->step[2]};

    //positive -> accelerate
    //negative -> decelerate
    int32_t t_acc2[3] = {v_out-block->freq[0]*STEPPER_RES/SQ(MAX_FREQ),
                        v_out-block->freq[0]*STEPPER_RES/SQ(MAX_FREQ),
                        v_out-block->freq[0]*STEPPER_RES/SQ(MAX_FREQ)};

    //dcc distance -> s_acc2
    int32_t s_acc2[3] = {(block->freq[0]+v_out[0])*abs(t_acc2[0])/2,
                        (block->freq[1]+v_out[1])*abs(t_acc2[1])/2,
                        (block->freq[2]+v_out[2])*abs(t_acc2[2])/2};

    //positive -> accelerate
    //negative -> decelerate
    int32_t t_acc1[3] = {(block->freq[0]-v_entr[0])*STEPPER_RES/SQ(MAX_FREQ),
                        (block->freq[1]-v_entr[1])*STEPPER_RES/SQ(MAX_FREQ),
                        (block->freq[2]-v_entr[2])*STEPPER_RES/SQ(MAX_FREQ)};

    acc1[0] = t_acc1[0]*MONITOR_FREQ;
    acc1[1] = t_acc1[1]*MONITOR_FREQ;
    acc1[2] = t_acc1[2]*MONITOR_FREQ;

    //acc distance -> s_a
    int32_t s_acc1[3] = {(block->freq[0]+stepperI->freq)*abs(t_acc1[0])/2,
                        (block->freq[1]+stepperJ->freq)*abs(t_acc1[1])/2,
                        (block->freq[2]+stepperK->freq)*abs(t_acc1[2])/2};

    //n distance -> s_n
    int32_t s_n[3] = {s[0]-s_acc1[0]-s_acc2[0],s[1]-s_acc1[1]-s_acc2[1],s[2]-s_acc1[2]-s_acc2[2]};

    //n time -> t_n
    int32_t t_n[3] = {s_n[0]/block->freq[0],s_n[1]/block->freq[1],s_n[2]/block->freq[2]};

    //dcc at t_a + t_n
    int32_t t[3] = {abs(t_acc1[0])+t_n[0],abs(t_acc1[1])+t_n[1],abs(t_acc1[2])+t_n[2]};

    acc2[0] = t[0]*MONITOR_FREQ;
    acc2[1] = t[1]*MONITOR_FREQ;
    acc2[2] = t[2]*MONITOR_FREQ;
}

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

void Acc_Cnt(stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc_step, int32_t* dcc_step, block_t* block)
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
        stepperI->freq = block->freq[0];
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
        stepperJ->freq = block->freq[1];
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
        stepperK->freq = block->freq[2];
        dcc_step[2]--;
    }
}

void Stepper_Cnt(block_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK)
{
    if (stepperI->pin_state_last==0&&stepperI->pin_state==1)    block->step[0]--;
    if (stepperJ->pin_state_last==0&&stepperJ->pin_state==1)    block->step[1]--;
    if (stepperK->pin_state_last==0&&stepperK->pin_state==1)    block->step[2]--;
}


