#ifndef PLANNER_H
#define PLANNER_H
#include "config.h"
#include "type.h"
#include "delta.h"
#include "buffer.h"

extern ring_buff_t stepper_list;

extern float abc_v_l;//last carriage speed


void Linear_Planner(int32_t* path[3], uint16_t len,float v_n, float v_l, float dwell, float* abc_c);

void Arc_Planner(int32_t* path[2], uint16_t len ,float* xyz_c, float v_n, float v_l, float dwell);




#endif