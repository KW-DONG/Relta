#include "type.h"
#include "config.h"
#include "planner_new.h"
#include "delta.h"
#include <math.h>
#include "buffer.h"

/*********************************WRITE_BUFFER*************************************/

uint8_t Line_Z_Planner(float dz, float feedrate)
{
    uint32_t step = dz * STEPS_PER_UNIT;
    
    block_t new_block;
    uint8_t dir;
    if (dz>0)   dir = carriage_UP;
    else        dir = carriage_DOWN;

    for (uint8_t i=0;i<3;i++)
    {
        new_block.step[i] = step;
        new_block.dir[i] = dir;
        new_block.maximum_velocity[i] = feedrate;
        new_block.maximum_freq[i] = (uint32_t)(feedrate*(float)STEPS_PER_UNIT);
    }
    if(Block_Buff_Write(new_block,&block_buffer))   return 1;
    return 0;
}

uint8_t Line_XYZ_Planner(float* xyz_c, float* xyz_t, float feedrate)
{
    float abc_l[3];
    float abc[3];
    float xyz_v[3];
    float abc_v[3];
    float d_x;
    uint8_t xy_single = 2;
    uint8_t pulse = 0;
    block_t new_block;

    Velocity_Decouple(xyz_c,xyz_t,xyz_v,feedrate);
    Inverse_Kinematics(xyz_c,abc_l);

    if ((xyz_c[0]-xyz_t[0])==0.f&&(xyz_c[1]-xyz_t[1])!=0.f)         xy_single = 1;
    else if ((xyz_c[0]-xyz_t[0])!=0.f&&(xyz_c[1]-xyz_t[1])==0.f)    xy_single = 0;
    else if ((xyz_c[0]-xyz_t[0])==0.f&&(xyz_c[1]-xyz_t[1])==0.f)    if (Line_Z_Planner(xyz_t[2]-xyz_c[2],feedrate)) return 1;
    else if (fabsf(xyz_c[1]-xyz_t[1])>=fabsf(xyz_c[0]-xyz_t[0])&&fabsf(xyz_c[1]-xyz_t[1])>=fabsf(xyz_c[2]-xyz_t[2]))
            d_x = 0.1f*fabsf(xyz_c[0]-xyz_t[0])*INV(fabsf(xyz_c[1]-xyz_t[1]));
    else if (fabsf(xyz_c[2]-xyz_t[2])>=fabsf(xyz_c[0]-xyz_t[0])&&fabsf(xyz_c[2]-xyz_t[2])>=fabsf(xyz_c[1]-xyz_t[1]))
            d_x = 0.1f*fabsf(xyz_c[0]-xyz_t[0])*INV(fabsf(xyz_c[2]-xyz_t[2]));
    
    if (xy_single!=2)
    {
        for (;xyz_t[xy_single]!=xyz_c[xy_single];xyz_c[xy_single] = xyz_c[xy_single] + 0.1f*fabsf(xyz_c[xy_single]-xyz_t[xy_single])*INV(xyz_c[xy_single]-xyz_t[xy_single]))
        {
            Inverse_Kinematics(xyz_c,abc);
            Jacobian_Matrix(xyz_v,xyz_c,abc,abc_v);
            Block_Init(&new_block, abc, abc_l, abc_v);
            if(Block_Buff_Write(new_block, &block_buffer))  return 1;
        }
    }else
    {
        float m_y = (xyz_t[1] - xyz_c[1])*INV(xyz_t[0] - xyz_c[0]);
        float m_z = (xyz_t[2] - xyz_c[2])*INV(xyz_t[0] - xyz_c[0]);
        float x_new = xyz_c[0];
        for (;fabsf(xyz_t[0]-xyz_c[0])>0.01f;)//error
        {
            Inverse_Kinematics(xyz_c,abc);
            Jacobian_Matrix(xyz_v,xyz_c,abc,abc_v);
            ///B's method
            x_new = x_new + d_x;
            if ((fabsf(m_y*x_new - xyz_c[1])>=(0.5f*GRID_LEN))&&(xyz_c[1]!=xyz_t[1]))
            {
                pulse = 1;
                xyz_c[1] += GRID_LEN*fabsf(xyz_t[1]-xyz_c[1])*INV(xyz_t[1]-xyz_c[1]);
            }
            if ((fabsf(m_z*x_new - xyz_c[2])>=(0.5f*GRID_LEN))&&(xyz_c[2]!=xyz_t[2]))
            {
                pulse = 1;
                xyz_c[2] += GRID_LEN*fabsf(xyz_t[2]-xyz_c[2])*INV(xyz_t[2]-xyz_c[2]);
            }
            if (fabsf(xyz_c[0] - x_new)>=GRID_LEN)
            {
                pulse = 1;
                xyz_c[0] += GRID_LEN*fabsf(xyz_t[0]-xyz_c[0])*INV(xyz_t[0]-xyz_c[0]);
            }
            if (pulse == 1)
            {
                Block_Init(&new_block, abc, abc_l, abc_v);
                if (new_block.step[0]!=0||new_block.step[1]!=0||new_block.step[2]!=0)
                if(Block_Buff_Write(new_block, &block_buffer))  return 1;
            }
            pulse = 0;
        }
    }
    return 0;
}


void Arc_Planner(float* xyz_c, float* xyz_t, float radius, float feedrate)
{
    static float xy_p[2];
    Get_Pivot(xyz_t,xyz_c,radius,xy_p);
    //arc angle


    //arc length

    //line 
}

void Get_Pivot(float* xyz_t, float* xyz_c, float radius, float* xy_p)
{
    //find pivot with radius and Pythagoras theorem
    float d_ct = sqrtf(SQ(xyz_c[0]-xyz_t[0])+SQ(xyz_c[1]-xyz_t[1]));
    
    //vector from current point to target point
    float ij_ct[2] = {xyz_t[0] - xyz_c[0], xyz_t[1] - xyz_c[1]};

    //unit vector of ij_ct
    float ij_ct_u[2] = {ij_ct[0]*INV(d_ct), ij_ct[1]*INV(d_ct)};

    //middle point between current point and target point
    float xy_m[2] = {(xyz_t[0]-xyz_c[0])*INV(2.0f), (xyz_t[0]-xyz_c[0])*INV(2.0f)};

    //unit vector from middle point to pivot
    float ij_mp_u[2];
    if (radius>0)
    {
        ij_mp_u[0] = ij_ct_u[1];
        ij_mp_u[1] = - ij_ct_u[0];
    }else
    {
        ij_mp_u[0] = - ij_ct_u[1];
        ij_mp_u[1] = ij_ct_u[0];
    }
    //distance between middle point and pivot
    float d_mp = sqrtf(SQ(d_ct*INV(2.0f))+SQ(radius));

    //vector from middle point to pivot
    float ij_mp[2] = {ij_mp_u[0]*d_mp, ij_mp_u[1]*d_mp};

    //pivot coordinate
    xy_p[0] = ij_ct[0]*INV(2.0f)+ij_mp[0];
    xy_p[1] = ij_ct[1]*INV(2.0f)+ij_mp[1];
}

void Velocity_Decouple(float* xyz_c, float* xyz_t, float* xyz_v, float v_n)
{
    float len = sqrtf(SQ(xyz_t[0]-xyz_c[0])+SQ(xyz_t[1]-xyz_c[1])+SQ(xyz_t[2]-xyz_c[2]));
    for (uint8_t i=0;i<3;i++)   xyz_v[i] = fabsf(xyz_c[i]-xyz_t[i])*INV(len)*v_n;
}

void Block_Init(block_t* new_block, float* abc, float* abc_l, float* abc_v)
{
    for (uint8_t i=0;i<3;i++)
    {
        new_block->step[i] = (uint32_t)(fabsf(abc[i] - abc_l[i])*(float)STEPS_PER_UNIT);
        new_block->maximum_velocity[i] = abc_v[i];
        new_block->flag = block_ready;
        if (abc[i]>abc_l[i])    new_block->dir[i] = carriage_UP;
        else                    new_block->dir[i] = carriage_DOWN;
        new_block->entry_velocity[i] = block_buffer.content[block_buffer.tail].leave_velocity[i];
        new_block->leave_velocity[i] = 0.f;
        new_block->maximum_freq[i] = (uint32_t)(new_block->maximum_velocity[i]*(float)STEPS_PER_UNIT);
        abc_l[i] = abc[i];
    }
}

/**********************************READ_BUFFER*************************************/

//update block_buffer
void Trapzoidal_Planner(block_t* block);




