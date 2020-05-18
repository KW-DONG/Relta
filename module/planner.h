#ifndef __MOTION_H
#define __MOTION_H
#include "type.h"
#include "bsp.h"

#define PLUS    0
#define MINUS   1

#define CLOCKWISE       0
#define ANTICLOCKWISE   1

/**
 * The trajectory includes a path which is a set of points, and the velocity of each point.
 * In this program the trajectory is stored in a three-dimensional list.
 *          {[[x][x_v]],[[y][y_v]],[[z][z_v]]}
 *                          |
 *                  Kinematics convert
 *                          |
 *          {[[a][a_v]],[[b][b_v]],[[c][c_v]]}
 */

/**********************************Linear_Planner**********************************/

/**
 * @brief Main entrance of linear motion
 * @param xyz_t     target point
 * @param xyz_c     current point
 * @param velocity  operation velocity
 * @param dwell     dwell time before the motion
 */
void Linear_Planner(float* xyz_t, float* xyz_c, float velocity, float dwell, block_buff_t* buff);

/**
 * @brief param sorting
 * @param dx input
 * @param dy input
 * @param dz input
 * @param da output
 * @param db output
 * @param dc output
 * @note abs(da)>abs(db)>abs(dc), the preprocess of Bresenham's method
 */
void Min_Max(int32_t dx, int32_t dy, int32_t dz, int32_t* da, int32_t* db, int32_t* dc);

/**
 * @brief create a linear path
 * @param traj  trajectory
 * @param da
 * @param db
 * @param dc
 * @note abs(da)>abs(db)>abs(dc)
 *      path[i][0] = da
 *      path[i][1] = db
 *      path[i][2] = dc
 */
void Linear_Path(int32_t (*traj)[3][2], int32_t da, int32_t db, int32_t dc);

/**
 * @brief rearrange the path to x->y->z order
 * @param traj  trajectory
 * @param len   path length
 * @param path_case case 1: xyz
 *                  case 2: xzy
 *                  case 3: yxz
 *                  case 4: yzx
 *                  case 5: zxy
 *                  case 6: zyx
 */
void Linear_Path_Convert(int32_t (*traj)[3][2], uint16_t len,uint8_t case_path);

/************************************Arc_Planner***********************************/

/**
 * @brief main entrance of arc motion
 * @param xyz_t     target point
 * @param xyz_c     current point
 * @param radius    radius and direction
 * @param velocity  operation velocity
 * @param dwell     dwell time before motion
 * @note radius>0 -> clockwise, vice versa
 * 
 */
void Arc_Planner(float* xyz_t, float* xyz_c, float radius, float velocity, float dwell, block_buff_t* buff);

/**
 * @brief find the pivot point on a plane
 * @param xyz_t     target point
 * @param xyz_c     current point
 * @param radius
 * @param xy_p      output
 * @note radius includes direction
 */
void Get_Pivot(float* xyz_t, float* xyz_c, float radius, int32_t* xy_p);

/**
 * @brief return the amount of full sectors
 * @param s_t   sector number of target point
 * @param s_c   sector number of current point
 * @param s_c   direction
 */
uint8_t Count_Sector(uint8_t s_t, uint8_t s_c, uint8_t dir);

/**
 * @brief return the sector number of the given coordinate
 * @param xy_s  the coordinate
 * @note pivot point is (0,0)
 */
uint8_t Get_Sector(int32_t* xy_s);

/**
 * @brief find the coordinate in corresponding sector
 * @param xy_i  input coordinate
 * @param xy_o  output coordinate
 * @param s     sector number
 * @note the function can be used in both following cases:
 *      1. xy_i is in sector s and xy_o is in sector 1
 *      2. xy_i is in sector 1 and xy_o is in sector s
 */
void Sector_Convert(int32_t* xy_i, int32_t* xy_o, uint8_t s);

/**
 * @brief convert the path from sector 1 to sector s
 * @param traj  both input and output
 * @param len   size of the path
 * @param s     sector number
 */
void Path_Convert(int32_t (*traj)[3][2], uint16_t len, uint8_t s);

/**
 * @brief generate a path by cutting a full path
 * @param traj_full
 * @param traj_o    output
 * @param len_o     size of output path
 * @param x_i       start offset
 * @param x_o       finish offset
 */
void Arc_Path_Oper(int32_t (*traj_full)[3][2], int32_t (*traj_o)[3][2], uint16_t len_o, int32_t x_i, int32_t x_o);

/**
 * @brief generate full path in sector 1
 * @param path
 * @param len   path length
 * @param r     radius
 */
void Arc_Path_Full(int32_t (*traj)[3][2], uint16_t len, uint16_t radius);

/**
 * @brief generate a short path in sector 1
 * @param traj
 * @param x_i   start offset
 * @param len   path length
 * @param r     radius
 */
void Arc_Path_Part(int32_t (*traj)[3][2], uint16_t x_i, uint16_t len, uint16_t radius);

/**
 * @brief add offset to the path
 * @param path
 * @param len   path length
 * @param x0    x offset
 * @param y0    y offset
 * @param z0    z offset
 */
void Path_Add_Offset(int32_t (*traj)[3][2], uint16_t len ,int32_t x0, int32_t y0, int32_t z0);

/*******************************Kinematics_Planning******************************/

/**
 * @brief turning a path to a trajectory
 * @param traj  trajectory
 * @param traj_len
 * 
 */
void Kinematics_Planner(int32_t (*traj)[3][2], uint16_t traj_len,float v_n);

/**
 * @brief decouple the velocity: scalar -> vector
 * @param xyz_v output velocity
 * @param d_xyz velocity vector
 * @param v_n   velocity scalar
 */
void Velocity_Decouple(float* xyz_v, uint8_t* d_xyz, float v_n);

/**
 * @brief convert the carriage velocity to timer prescalar
 */
int32_t Velocity_To_Freq(float v);

/**
 * @brief generate the acceleration and deceleration plan
 * @param block
 * @param stepperI
 * @param stepperJ
 * @param stepperK
 * @param acc1  steps need to accelerate from init speed to operation speed
 * @param acc2  steps need to decelerate from opeation speed to jerk speed
 * @note if the operation speed is lower than the jerk speed, acc2 will not be planned
 *      use this function when a new block is read
 */
void Acceleration_Planner(block_t* block);

/**********************************Motion_Update*********************************/

/**
 * @brief trajectory -> ring buff
 * @param traj  trajectory in abc
 * @param len
 * @param ring_buff
 */
void Trej_Apply(int32_t (*traj)[3][2], uint32_t len, float dwell, block_buff_t* ring_buff);


#endif
