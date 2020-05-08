#include "planner.h"
#include "config.h"

/**
 *              v_n
 *         +----------------+
 *        /|                |\
 *       / |                | \
 *  v_i +  |                |  + v_o
 *   0  +--+----------------+--+
 * 
 *      Use v_n^2 - v_i^2 = 2*a*S
 */


void Linear_Planner(int32_t* path[3], uint16_t len,float v_n,
                    float v_l, float dwell, float* abc_c)
{
    uint16_t i;
    float xyz_n[3];//next
    float xyz_l[3];//last
    float xyz_v[3];//velocity
    float abc_n[3];
    float abc_l[3];
    float abc_v[3];

    uint8_t d_xyz[3];
    float d_abc[3];
    stepper_exe_t block;

    xyz_l[0] = path[0][0];
    xyz_l[1] = path[0][1];
    xyz_l[2] = path[0][2]; 

    abc_l[0] = abc_c[0];
    abc_l[1] = abc_c[1];
    abc_l[2] = abc_c[2];

    block.step_dwell = dwell * 1000;

    for (i=1;i=len;i++)
    {
        xyz_n[0] = (float)path[i][0]*INV(10.0);
        xyz_n[1] = (float)path[i][1]*INV(10.0);
        xyz_n[2] = (float)path[i][2]*INV(10.0);

        if (xyz_l[0]!=xyz_n[0]) d_xyz[0] = 1;
        else                    d_xyz[0] = 0;
        if (xyz_l[1]!=xyz_n[1]) d_xyz[1] = 1;
        else                    d_xyz[1] = 0;
        if (xyz_l[2]!=xyz_n[2]) d_xyz[2] = 1;
        else                    d_xyz[2] = 0;
        
        Velocity_Decouple(xyz_v,d_xyz,v_n);

        Inverse_Kinematics(xyz_n,abc_n);

        //find carriage movements
        d_abc[0] = abc_n[0] - abc_l[0];
        d_abc[1] = abc_n[1] - abc_l[1];
        d_abc[2] = abc_n[2] - abc_l[2];


        Jacobian_Matrix(xyz_v,xyz_n,abc_n,abc_v);

        Carriage_Oper(abc_l,abc_n,abc_v,dwell);

        abc_l[0] = abc_n[0];
        abc_l[1] = abc_n[1];
        abc_l[2] = abc_n[2];
        xyz_l[0] = xyz_n[0];
        xyz_l[1] = xyz_n[1];
        xyz_l[2] = xyz_n[2];
    }

}


//both velocity and acceleration
void Velocity_Decouple(float* xyz_v, uint8_t* d_xyz, float v_n)
{
    if (d_xyz[0]==1)
    {
        if (d_xyz[1]==1)
        {
            if (d_xyz[2]==1)
            {
                xyz_v[0] = v_n*INV(sqrtf(3.0));
                xyz_v[1] = xyz_v[0];
                xyz_v[2] = xyz_v[0];
            }else
            {
                xyz_v[0] = v_n*INV(sqrtf(2.0));
                xyz_v[1] = xyz_v[0];
                xyz_v[2] = 0.0;
            }
        }else
        {
            if (d_xyz[2]==1)
            {
                xyz_v[0] = v_n*INV(sqrtf(2.0));
                xyz_v[1] = 0.0;
                xyz_v[2] = xyz_v[0];
            }else
            {
                xyz_v[0] = v_n;
                xyz_v[1] = 0.0;
                xyz_v[2] = 0.0;
            }
        }
        
    }else
    {
        if (d_xyz[1]==1)
        {
            if (d_xyz[2]==1)
            {
                xyz_v[0] = 0.0;
                xyz_v[1] = v_n*INV(sqrtf(2.0));
                xyz_v[2] = xyz_v[1];
            }else
            {
                xyz_v[0] = 0.0;
                xyz_v[1] = v_n;
                xyz_v[2] = 0.0;
            }
        }else
        {
            xyz_v[0] = 0.0;
            xyz_v[1] = 0.0;
            xyz_v[2] = v_n;
        }
    }
}

/**
 * T_out = ((arr+1)*(psc+1))/T_clk
 * T_clk = 84,000,000
 * freq = INV(T_out)
 * 0<=psc<=65535
 * 0<=arr<=65535
 * T_out_max = 50s
 * velocity in mm/s
 * velocity>0
 * 
 */

uint16_t Velocity_To_Psc(float mm_per_s)
{
    float freq = mm_per_s * INV(STEPS_PER_UNIT);
    float t_out = INV(freq);
    float psc = t_out*84000000.0*INV(28000);//0.045hz -> 3000hz
    return (uint16_t)psc;
}

void Carriage_Oper(float* abc_c, float* abc_t, float* abc_v, float dwell)
{
    stepper_exe_t block;

    float d_abc[] = {abc_t[0]-abc_c[0],abc_t[1]-abc_c[1],abc_t[2]-abc_c[2]};

    if (d_abc[0]>0) block.dir[0] = 1;
    else            block.dir[0] = 0;
    if (d_abc[1]>0) block.dir[1] = 1;
    else            block.dir[1] = 0;
    if (d_abc[2]>0) block.dir[2] = 1;
    else            block.dir[2] = 0;

    block.step_dwell = dwell*MONITOR_FREQ;

    block.freq[0] = Velocity_To_Psc(abc_v[0]);
    block.freq[1] = Velocity_To_Psc(abc_v[1]);
    block.freq[2] = Velocity_To_Psc(abc_v[2]);

    block.step[0] = (uint16_t)abs((int32_t)(d_abc[0]*STEPS_PER_UNIT));
    block.step[1] = (uint16_t)abs((int32_t)(d_abc[1]*STEPS_PER_UNIT));
    block.step[2] = (uint16_t)abs((int32_t)(d_abc[2]*STEPS_PER_UNIT));

    Block_Buff_Write(block,&stepper_list);

}