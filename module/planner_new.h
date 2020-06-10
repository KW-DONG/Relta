#ifndef PLANNER_NEW_H
#define PLANNER_NEW_H
#include "type.h"

extern volatile block_buff_t block_buffer;
extern machine_t machine;

uint8_t Line_Z_Planner(float dz, float feedrate);

uint8_t Line_XYZ_Planner(float* xyz_c, float* xyz_t, float feedrate);

uint8_t Line_XYZ_Planner_1(float* xyz_i, float* xyz_c, float* xyz_t, float feedrate);

void Arc_Planner(float* xy_c, float* xy_t, float radius, float feedrate);

void Get_Pivot(float* xyz_t, float* xyz_c, float radius, float* xy_p);

void Block_Init(block_t* new_block, float* abc, float* abc_l, float* abc_v);

void Velocity_Decouple(float* xyz_c, float* xyz_t, float* xyz_v, float v_n);

#endif
