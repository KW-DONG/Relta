#ifndef PLANNER_H
#define PLANNER_H
#include "type.h"

extern block_buff_t block_buffer;
extern machine_t machine;

/**
 * @brief movement along the Z axis
 * @param dz z offset
 * @param feedrate
 * @return 0 success; 1 buffer full
 */
uint8_t Line_Z_Planner(float dz, float feedrate);

/**
 * @brief linear movement in XYZ space
 * @param xyz_i initial coordinate array
 * @param xyz_c tempt coordinate array
 * @param xyz_t terminal coordinate array
 * @param abc_l carriage position array
 * @return 0 success; 1 buffer full
 */
uint8_t Line_XYZ_Planner(float* xyz_i, float* xyz_c, float* xyz_t, float* abc_l, float feedrate);

/**
 * @brief arc movement in XY space
 * @param xyz_i initial coordinate array
 * @param xyz_c tempt coordinate array
 * @param xyz_t terminal coordinate array
 * @return 0 success; 1 buffer full
 */
uint8_t Arc_Planner(float* xyz_i, float* xyz_c, float* xyz_t, float radius, float feedrate);

/**
 * @brief calculate pivot
 * @param xyz_i initial coordinate array
 * @param xyz_t terminal coordinate array
 * @param radius 
 * @param xyz_p pivot coordinate array
 */
void Get_Pivot(float* xyz_i, float* xyz_t, float radius, float* xyz_p);

/**
 * @brief prepare and write a new block
 * @param new_block
 * @param abc current carriage position array
 * @param abc_l last carriage position array
 * @note check buffer length before call
 */
void Block_Init(block_t* new_block, float* abc, float* abc_l, float* abc_v);

/**
 * @brief v_n -> v_x + v_y + v_z
 * @param xyz_i initial coordinate array
 * @param xyz_t terminal coordinate array
 * @param xyz_v velocity array
 * @param v_n feedrate
 */
void Velocity_Decouple(float* xyz_i, float* xyz_t, float* xyz_v, float v_n);

#endif
