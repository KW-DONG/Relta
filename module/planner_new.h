#ifndef PLANNER_NEW_H
#define PLANNER_NEW_H
#include "type.h"

extern block_buff_t block_buffer;
extern machine_t machine;

void Line_Z_Planner(float dz);

void Line_XY_Planner(float* xy_c, float* xy_t, float feedrate);

void Arc_Planner(float* xy_c, float* xy_t, float radius, float feedrate);

void Get_Pivot(float* xyz_t, float* xyz_c, float radius, float* xy_p);

void Velocity_Decouple(float* xyz_c, float* xyz_t, float* xyz_v, float v_n);
#endif