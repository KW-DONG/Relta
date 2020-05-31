#include "machine.h"
#include "config.h"
#include "delta.h"

void Machine_Init()
{
    machine.abc[0] = CARRIAGE_A_RESET;
    machine.abc[1] = CARRIAGE_B_RESET;
    machine.abc[2] = CARRIAGE_C_RESET;

    Forward_Kinematics(machine.abc, machine.xyz);
    machine.fk_flag = 0;
    machine.interpret_flag = 0;
    machine.traj_flag = 0;
    machine.state = machine_OFF;
}

Machine_Update()
{
    for (uint8_t i=0;i<3;i++)
    machine.abc[i] = machine.abc[i] + (float)(abs(machine.carriage_move[i])/(machine.carriage_move[i]))*INV((float)GRID_LEN);
    Forward_Kinematics(machine.abc, machine.xyz);
}