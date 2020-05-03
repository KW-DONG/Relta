#ifndef __DELTA_H
#define __DELTA_H
#include "config.h"


extern float XYZ_C[3];//current end effector coordinate
extern float XYZ_N[3];//next end effector coordinate
extern float ABC_C[3];//current carriage coordinate
extern float ABC_C[3];//next carriage coordinate
extern float ABC_D[3];//carriage movement
extern delta_struct LDR;

void Delta_Init();

void Inverse_Kinematics(float* xyz, float* abc);

void Forward_Kinematics(float* abc, float* xyz);

void Jacobian_Matrix(float* xyz_v, float* xyz, float* abc, float* abc_v);


#endif