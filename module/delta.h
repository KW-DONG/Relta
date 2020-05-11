#ifndef __DELTA_H
#define __DELTA_H

/**
 * @brief Inverse Kinematics Convert
 * @param xyz   input cartesian coordinate 
 * @param abc   output carriage coordinate
 */
void Inverse_Kinematics(float* xyz, float* abc);

/**
 * @brief Forward Kinematics Convert
 * @param abc   input carriage coordinate
 * @param xyz   output cartesian coordinate 
 */
void Forward_Kinematics(float* abc, float* xyz);

/**
 * @brief end effector velocity -> carriage velocity
 * @param xyz_v end effector velocity vector
 * @param xyz   end effector cartesian coordinate
 * @param abc   carriage position
 * @param abc_v output carriage velocity
 */
void Jacobian_Matrix(float* xyz_v, float* xyz, float* abc, float* abc_v);

#endif