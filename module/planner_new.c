#include "type.h"
#include "config.h"
#include "planner_new.h"
#include "delta.h"
#include <math.h>
#include "buffer.h"

/*********************************WRITE_BUFFER*************************************/

uint8_t Line_Z_Planner(float dz, float feedrate)
{
    uint32_t step = (uint32_t)fabsf(dz * STEPS_PER_UNIT);
    
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
    new_block.flag = block_ready;
    if(Block_Buff_Write(new_block,&block_buffer))   return 1;
    return 0;
}

uint8_t Line_XYZ_Planner_1(float* xyz_i, float* xyz_c, float* xyz_t, float feedrate)
{
    float abc_l[3];
    float abc[3];
    float xyz_v[3];
    float abc_v[3];
    float m_xy = 0.f;
    float m_xz = 0.f;
    float m_yx = 0.f;
    float m_yz = 0.f;
    float m_zx = 0.f;
    float m_zy = 0.f;
    uint64_t step = 0;

    Inverse_Kinematics(xyz_c,abc_l);
    Velocity_Decouple(xyz_i,xyz_t,xyz_v,feedrate);

    //dx max and dx > 0
    if (fabsf(xyz_c[0]-xyz_t[0])<GRID_LEN&&fabsf(xyz_c[1]-xyz_t[1])<GRID_LEN)
    {
        if (block_buffer.length<RINGBUFF_LEN)
        {
            Line_Z_Planner(xyz_t[2]-xyz_c[2],feedrate);
            for (uint8_t i=0;i<3;i++)   xyz_c[i] = xyz_t[i];
            return 0;
        }else   return 1;
    }else if (fabsf(xyz_t[0]-xyz_i[0])>=GRID_LEN&&fabsf(xyz_t[0]-xyz_i[0])>=fabsf(xyz_t[1]-xyz_i[1])&&fabsf(xyz_t[0]-xyz_i[0])>=fabsf(xyz_t[2]-xyz_i[2]))
    {
        m_xy = (xyz_t[1] - xyz_i[1])*INV(xyz_t[0] - xyz_i[0]);
        m_xz = (xyz_t[2] - xyz_i[2])*INV(xyz_t[0] - xyz_i[0]);
        step = (uint64_t)roundf(fabsf(xyz_t[0]-xyz_i[0])*(float)STEPS_PER_UNIT);
    
        //dy max and dy > 0
    }else if (fabsf(xyz_t[1]-xyz_i[1])>=GRID_LEN&&fabsf(xyz_t[1]-xyz_i[1])>=fabsf(xyz_t[0]-xyz_i[0])&&fabsf(xyz_t[1]-xyz_i[1])>=fabsf(xyz_t[2]-xyz_i[2]))
    {
        m_yx = (xyz_t[0] - xyz_i[0])*INV(xyz_t[1] - xyz_i[1]);
        m_yz = (xyz_t[2] - xyz_i[2])*INV(xyz_t[1] - xyz_i[1]);
        step = (uint64_t)roundf(fabsf(xyz_t[1]-xyz_i[1])*(float)STEPS_PER_UNIT);

        //dz max and dz > 0
    }else if (fabsf(xyz_t[2]-xyz_i[2])>=GRID_LEN&&fabsf(xyz_t[2]-xyz_i[2])>=fabsf(xyz_t[0]-xyz_i[0])&&fabsf(xyz_t[2]-xyz_i[2])>=fabsf(xyz_t[1]-xyz_i[1]))
    {
        m_zx = (xyz_t[0] - xyz_i[0])*INV(xyz_t[2] - xyz_i[2]);
        m_zy = (xyz_t[1] - xyz_i[1])*INV(xyz_t[2] - xyz_i[2]);
        step = (uint64_t)roundf(fabsf(xyz_t[2]-xyz_i[2])*(float)STEPS_PER_UNIT);
    }else   return 0;
    

    for (;step>0&&block_buffer.length<(RINGBUFF_LEN);step--)
    {
        if (fabsf(xyz_t[0]-xyz_c[0])>=GRID_LEN&&fabsf(xyz_t[0]-xyz_c[0])>=fabsf(xyz_t[1]-xyz_c[1])&&fabsf(xyz_t[0]-xyz_c[0])>=fabsf(xyz_t[2]-xyz_c[2])
            ||((fabsf(xyz_t[1]-xyz_c[1])>=GRID_LEN&&fabsf(xyz_t[1]-xyz_c[1])>=fabsf(xyz_t[0]-xyz_c[0])&&fabsf(xyz_t[1]-xyz_c[1])>=fabsf(xyz_t[2]-xyz_c[2]))
                &&(fabsf(m_yx*xyz_c[1] + xyz_i[0] - xyz_c[0])>=(0.5f*GRID_LEN)))
            ||((fabsf(xyz_t[2]-xyz_c[2])>=GRID_LEN&&fabsf(xyz_t[2]-xyz_c[2])>=fabsf(xyz_t[0]-xyz_c[0])&&fabsf(xyz_t[2]-xyz_c[2])>=fabsf(xyz_t[1]-xyz_c[1]))
                &&(fabsf(m_zx*xyz_c[2] + xyz_i[0] - xyz_c[0])>=(0.5f*GRID_LEN))))
        xyz_c[0] += GRID_LEN*fabsf(xyz_t[0]-xyz_c[0])*INV(xyz_t[0]-xyz_c[0]);

        if (fabsf(xyz_t[1]-xyz_c[1])>=GRID_LEN&&fabsf(xyz_t[1]-xyz_c[1])>fabsf(xyz_t[0]-xyz_c[0])&&fabsf(xyz_t[1]-xyz_c[1])>=fabsf(xyz_t[2]-xyz_c[2])
            ||((fabsf(xyz_t[0]-xyz_c[0])>=GRID_LEN&&fabsf(xyz_t[0]-xyz_c[0])>=fabsf(xyz_t[1]-xyz_c[1])&&fabsf(xyz_t[0]-xyz_c[0])>=fabsf(xyz_t[2]-xyz_c[2]))
                &&(fabsf(m_xy*(xyz_c[0]) + xyz_i[1] - xyz_c[1])>=(0.5f*GRID_LEN)))
            ||((fabsf(xyz_t[2]-xyz_c[2])>=GRID_LEN&&fabsf(xyz_t[2]-xyz_c[2])>fabsf(xyz_t[0]-xyz_c[0])&&fabsf(xyz_t[2]-xyz_c[2])>fabsf(xyz_t[1]-xyz_c[1]))
                &&(fabsf(m_zy*xyz_c[2] + xyz_i[1] - xyz_c[1])>=(0.5f*GRID_LEN))))
        xyz_c[1] += GRID_LEN*fabsf(xyz_t[1]-xyz_c[1])*INV(xyz_t[1]-xyz_c[1]);


        if (fabsf(xyz_t[2]-xyz_c[2])>=GRID_LEN&&fabsf(xyz_t[2]-xyz_c[2])>fabsf(xyz_t[0]-xyz_c[0])&&fabsf(xyz_t[2]-xyz_c[2])>fabsf(xyz_t[1]-xyz_c[1])
            ||((fabsf(xyz_t[0]-xyz_c[0])>=GRID_LEN&&fabsf(xyz_t[0]-xyz_c[0])>=fabsf(xyz_t[1]-xyz_c[1])&&fabsf(xyz_t[0]-xyz_c[0])>=fabsf(xyz_t[2]-xyz_c[2]))
                &&(fabsf(m_xz*xyz_c[0] + xyz_i[2] - xyz_c[2])>=(0.5f*GRID_LEN)))
            ||((fabsf(xyz_t[1]-xyz_c[1])>=GRID_LEN&&fabsf(xyz_t[1]-xyz_c[1])>fabsf(xyz_t[0]-xyz_c[0])&&fabsf(xyz_t[1]-xyz_c[1])>=fabsf(xyz_t[2]-xyz_c[2]))
                &&(fabsf(m_yz*xyz_c[1] + xyz_i[2] - xyz_c[2])>=(0.5f*GRID_LEN))))
        xyz_c[2] += GRID_LEN*fabsf(xyz_t[2]-xyz_c[2])*INV(xyz_t[2]-xyz_c[2]);

        Inverse_Kinematics(xyz_c,abc);
        Jacobian_Matrix(xyz_v,xyz_c,abc,abc_v);

        block_t new_block;
        Block_Init(&new_block, abc, abc_l, abc_v);
        if (new_block.step[0]!=0||new_block.step[1]!=0||new_block.step[2]!=0)
        Block_Buff_Write(new_block, &block_buffer);
    }
    if (step>0) return 1;
    else        return 0;
}


uint8_t Line_XYZ_Planner(float* xyz_c, float* xyz_t, float feedrate)
{
    static float abc_l[3];
    static float abc[3];
    float xyz_v[3];
    float abc_v[3];
    float d_x = 0.f;//always positive
    float d_y = 0.f;//always positive
    uint8_t pulse = 0;

    //decouple the feedrate
    Velocity_Decouple(xyz_c,xyz_t,xyz_v,feedrate);

    //apply inverse kinematics
    Inverse_Kinematics(xyz_c,abc_l);

    //step_x = 0, step_y != 0 and step_y >= step_z
    if (fabsf(xyz_c[0]-xyz_t[0])<GRID_LEN&&fabsf(xyz_c[1]-xyz_t[1])>GRID_LEN&&(fabsf(fabsf(xyz_c[1]-xyz_t[1])-fabsf(xyz_c[2]-xyz_t[2]))>=GRID_LEN))      d_y = GRID_LEN;
    //step_x = 0, step_y != 0 and step_y < step_z
    else if (fabsf(xyz_c[0]-xyz_t[0])<GRID_LEN&&fabsf(xyz_c[1]-xyz_t[1])>GRID_LEN&&(fabsf(fabsf(xyz_c[1]-xyz_t[1])-fabsf(xyz_c[2]-xyz_t[2]))<GRID_LEN))    d_y = GRID_LEN*fabsf(xyz_c[1]-xyz_t[1])*INV(fabsf(xyz_c[2]-xyz_t[2]));
    //step_x = 0, step_y = 0 and step_z != 0
    else if (fabsf(xyz_c[0]-xyz_t[0])<GRID_LEN&&fabsf(xyz_c[1]-xyz_t[1])<GRID_LEN)
    {
        if (block_buffer.length<RINGBUFF_LEN)
        {
            Line_Z_Planner(xyz_t[2]-xyz_c[2],feedrate);
            for (uint8_t i=0;i<3;i++)   xyz_c[i] = xyz_t[i];
            return 0;
        }else   return 1;
    }
    //step_x != 0 and step_y max
    else if (fabsf(xyz_c[1]-xyz_t[1])>=fabsf(xyz_c[0]-xyz_t[0])&&fabsf(xyz_c[1]-xyz_t[1])>=fabsf(xyz_c[2]-xyz_t[2]))    d_x = GRID_LEN*fabsf(xyz_c[0]-xyz_t[0])*INV(fabsf(xyz_c[1]-xyz_t[1]));
    //step_x != 0 and step_z max
    else if (fabsf(xyz_c[2]-xyz_t[2])>=fabsf(xyz_c[0]-xyz_t[0])&&fabsf(xyz_c[2]-xyz_t[2])>=fabsf(xyz_c[1]-xyz_t[1]))    d_x = GRID_LEN*fabsf(xyz_c[0]-xyz_t[0])*INV(fabsf(xyz_c[2]-xyz_t[2]));
    //step_x != 0 and step_x max
    else    d_x = GRID_LEN;
    
    if (d_y!=0.f)
    {
        uint64_t step = (uint64_t)(fabsf(xyz_t[1]-xyz_c[1])*INV(d_y));
        float y_new = xyz_c[1];
        uint8_t pulse;
        //calculate gradient
        float m_z = 0.f;
        if (fabsf(xyz_t[2]-xyz_c[2]>=GRID_LEN))  m_z = (xyz_t[2] - xyz_c[2])*INV(xyz_t[1] - xyz_c[1]);

        for (;step>0&&block_buffer.length<RINGBUFF_LEN;)
        {
            if ((xyz_t[1]-xyz_c[1])>GRID_LEN)          y_new = y_new + d_y;
            else if ((xyz_t[1]-xyz_c[1])<-GRID_LEN)     y_new = y_new - d_y;
            else                                return 0;
            if ((fabsf(m_z*y_new - xyz_c[2])>=(0.5f*GRID_LEN))&&(xyz_c[2]!=xyz_t[2]))
            {
                pulse = 1;
                xyz_c[2] += GRID_LEN*fabsf(xyz_t[2]-xyz_c[2])*INV(xyz_t[2]-xyz_c[2]);
            }
            if (fabsf(xyz_c[1] - y_new)>=GRID_LEN)
            {
                pulse = 1;
                xyz_c[1] += GRID_LEN*fabsf(xyz_t[1]-xyz_c[1])*INV(xyz_t[1]-xyz_c[1]);
                step--;
            }
            if (pulse !=0)
            {
                Inverse_Kinematics(xyz_c,abc);
                Jacobian_Matrix(xyz_v,xyz_c,abc,abc_v);

                block_t new_block;
                Block_Init(&new_block, abc, abc_l, abc_v);
                if (new_block.step[0]!=0||new_block.step[1]!=0||new_block.step[2]!=0)
                Block_Buff_Write(new_block, &block_buffer);
                pulse = 0;
            }
        }
        if (step>0) return 1;
        else        return 0;
    }else
    {
        float m_y = 0.f;
        float m_z = 0.f;
        
        if (fabsf(xyz_t[1]-xyz_c[1]>=GRID_LEN)) m_y = (xyz_t[1] - xyz_c[1])*INV(xyz_t[0] - xyz_c[0]);
        if (fabsf(xyz_t[2]-xyz_c[2]>=GRID_LEN)) m_z = (xyz_t[2] - xyz_c[2])*INV(xyz_t[0] - xyz_c[0]);

        float x_new = xyz_c[0];
        uint64_t step = (uint64_t)(fabsf(xyz_t[0]-xyz_c[0])*INV(d_x));


        for (;step>0&&block_buffer.length<RINGBUFF_LEN;)
        {
            ///B's method
            if ((xyz_t[0]-xyz_c[0])>GRID_LEN)          x_new = x_new + d_x;
            else if ((xyz_t[0]-xyz_c[0])<-GRID_LEN)     x_new = x_new - d_x;
            else                                return 0;
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
                step--;
            }
            if (pulse == 1)
            {
                block_t new_block;
                Inverse_Kinematics(xyz_c,abc);
                Jacobian_Matrix(xyz_v,xyz_c,abc,abc_v);

                Block_Init(&new_block, abc, abc_l, abc_v);
                if (new_block.step[0]!=0||new_block.step[1]!=0||new_block.step[2]!=0)
                Block_Buff_Write(new_block, &block_buffer);
                pulse = 0;
            }
        }
        if (step>0) return 1;
        else        return 0;
    }
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
    new_block->flag = block_ready;
    for (uint8_t i=0;i<3;i++)
    {
        
        new_block->step[i] = (uint32_t)roundf((fabsf(abc[i] - abc_l[i])*(float)STEPS_PER_UNIT));
        new_block->maximum_velocity[i] = abc_v[i];
        
        if (abc[i]>abc_l[i])    new_block->dir[i] = carriage_UP;
        else                    new_block->dir[i] = carriage_DOWN;
        new_block->entry_velocity[i] = block_buffer.content[block_buffer.tail].leave_velocity[i];
        new_block->leave_velocity[i] = 0.f;
        new_block->maximum_freq[i] = (uint32_t)roundf(fabs(abc_v[i])*(float)STEPS_PER_UNIT);
        abc_l[i] = abc[i];
    }
}

/**********************************READ_BUFFER*************************************/

//update block_buffer
void Trapzoidal_Planner(block_t* block);




