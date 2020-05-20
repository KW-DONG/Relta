#include "buffer.h"
#include "delta.h"
#include "motion.h"
#include "config.h"
#include "math.h"

//the functions will be called in the interrupt
//avoid using complex calulation and float calculation

void Dwell_Step_Update(stepper_exe_t* exe)
{
    if (exe->step_dwell!=0)
    exe->step_dwell--;
}


void Stepper_Count(stepper_exe_t* exe, machine_t* machine,stepper_t* stepperX)
{
    if (stepperX->pin_state_last==0&&stepperX->pin_state==1)
    {
        exe->step[stepperX->id]--;
        if (stepperX->dir == 0) machine->carriage_move[stepperX->id]--;
        else                    machine->carriage_move[stepperX->id]++;

        if (exe->accelerate_until[stepperX->id]!=0)
        {
            exe->accelerate_until[stepperX->id]--;
            exe->decelerate_after[stepperX->id]--;
            stepperX->psc = stepperX->psc * (1+exe->accelerate_psc[stepperX->id]*ARR*INV(T_CLK));
        }else if (exe->decelerate_after[stepperX->id]==0)
        {
            stepperX->psc = stepperX->psc * (1-exe->decelerate_psc[stepperX->id]*ARR*INV(T_CLK));
        }
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
