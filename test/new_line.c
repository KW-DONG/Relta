#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define SQ(x)   ((x)*(x))
#define COS(x)  (cosf(x))
#define SIN(x)  (sinf(x))
#define INV(x)  (1.0f / (x))
#define E(x)    (powf(10.0f,x))
#define PI      3.14f

#define DELTA_CHAIN_LEN 185.0f
#define DELTA_TOWER_RADIUS 145.0f
#define DELTA_EFFECTOR_RADIUS 37.15f
#define DELTA_EFFECTOR_HEIGHT 20.0f
#define DELTA_CARRIAGE_OFFSET 16.0f
#define n1 0.0f
#define n2 3.14f
#define n3 3.14f*3.0f*INV(2.0f)
#define GRID_LEN 0.1f    //mm

#define CARRIAGE_LOW    250.0f
#define CARRIAGE_HIGH   300.0f

#define CARRIAGE_A_RESET    360.0f
#define CARRIAGE_B_RESET    360.0f
#define CARRIAGE_C_RESET    360.0f

#define SWITCH_HIGHT    360.0f

#define WORKSPACE_X 180.0f
#define WORKSPACE_Y 180.0f
#define WORKSPACE_Z 50.0f

//homing speed
#define HOMING_FEEDRATE_XYZ (50*60)

//steps per mm
//use (360/1.8*16)/(2*20)
#define STEPS_PER_UNIT      80

#define R   (DELTA_TOWER_RADIUS - DELTA_CARRIAGE_OFFSET)
#define r   DELTA_EFFECTOR_RADIUS
#define L   DELTA_CHAIN_LEN
#define x1  (R-r)*cosf(n1)
#define y1  (R-r)*sinf(n1)
#define x2  (R-r)*cosf(n2)
#define y2  (R-r)*sinf(n2)
#define x3  (R-r)*cosf(n3)
#define y3  (R-r)*sinf(n3)

typedef struct 
{
    //read by planner
    float       entry_velocity[3];
    float       maximum_velocity[3];
    float       leave_velocity[3];
    float       distance[3];
    float       acceleration[3];
    float       deceleration[3];

    //read by controller
    uint8_t     dir[3];
    uint32_t    accelerate_until[3];
    uint32_t    decelerate_after[3];
    uint32_t    accelerate_freq[3];
    uint32_t    decelerate_freq[3];

    //update in ISR
    uint32_t    step[3];
    uint32_t    step_dwell;
    uint8_t     flag;
}block_t;

enum {block_ready, block_busy};

typedef struct
{
    uint16_t head;
    uint16_t tail;
    uint16_t length;
    block_t* content[100];
}block_buff_t;

void Velocity_Decouple(float* xyz_c, float* xyz_t, float* xyz_v, float v_n)
{
    float len = sqrtf(SQ(xyz_t[0]-xyz_c[0])+SQ(xyz_t[1]-xyz_c[1])+SQ(xyz_t[2]-xyz_c[2]));
    for (uint8_t i=0;i<3;i++)   xyz_v[i] = fabsf(xyz_c[i]-xyz_t[i])*INV(len)*v_n;
}

void Inverse_Kinematics(float* xyz, float* abc)
{
    abc[0] = sqrtf(SQ(L)-SQ(xyz[0]-x1)-SQ(xyz[1]-y1))+xyz[2];
    abc[1] = sqrtf(SQ(L)-SQ(xyz[0]-x2)-SQ(xyz[1]-y2))+xyz[2];
    abc[2] = sqrtf(SQ(L)-SQ(xyz[0]-x3)-SQ(xyz[1]-y3))+xyz[2];
}

void Jacobian_Matrix(float* xyz_v, float* xyz, float* abc, float* abc_v)
{
    abc_v[0] = ((xyz[0]-x1)*INV(xyz[2]-abc[0])
             + (xyz[0]-x2)*INV(xyz[2]-abc[1])
             + (xyz[0]-x3)*INV(xyz[2]-abc[2]))*xyz_v[0];
    
    abc_v[1] = ((xyz[1]-y1)*INV(xyz[2]-abc[0])
             + (xyz[1]-y2)*INV(xyz[2]-abc[1])
             + (xyz[1]-y3)*INV(xyz[2]-abc[2]))*xyz_v[1];

    abc_v[2] = ((xyz[2]-y1)*INV(xyz[2]-abc[0])
             + (xyz[2]-y2)*INV(xyz[2]-abc[1])
             + (xyz[2]-y3)*INV(xyz[2]-abc[2]))*xyz_v[2];
}


void Line_XYZ_Planner(float* xyz_c, float* xyz_t, float feedrate)
{
    static float abc_l[3];
    static float abc[3];
    static float xyz_v[3];
    static float abc_v[3];
    float x_new;
    float d_x;
    uint8_t xy_single = 2;

    Velocity_Decouple(xyz_c,xyz_t,xyz_v,feedrate);
    Inverse_Kinematics(xyz_c,abc_l);

    if ((xyz_c[0]-xyz_t[0])==0.f&&(xyz_c[1]-xyz_t[1])!=0.f)         xy_single = 1;
    else if ((xyz_c[0]-xyz_t[0])!=0.f&&(xyz_c[1]-xyz_t[1])==0.f)    xy_single = 0;
    else if ((xyz_c[0]-xyz_t[0])==0.f&&(xyz_c[1]-xyz_t[1])==0.f)    return;
    else if (fabsf(xyz_c[1]-xyz_t[1])>fabsf(xyz_c[0]-xyz_t[0])&&fabsf(xyz_c[1]-xyz_t[1])>fabsf(xyz_c[2]-xyz_t[2]))
            d_x = 0.1f*fabsf(xyz_c[0]-xyz_t[0])*INV(fabsf(xyz_c[1]-xyz_t[1]));
    else if (fabsf(xyz_c[2]-xyz_t[2])>fabsf(xyz_c[0]-xyz_t[0])&&fabsf(xyz_c[2]-xyz_t[2])>fabsf(xyz_c[1]-xyz_t[1]))
            d_x = 0.1f*fabsf(xyz_c[0]-xyz_t[0])*INV(fabsf(xyz_c[2]-xyz_t[2]));
    
    if (xy_single!=2)
    {
        for (;xyz_t[xy_single]!=xyz_c[xy_single];xyz_c[xy_single] = xyz_c[xy_single] + 0.1f*fabsf(xyz_c[xy_single]-xyz_t[xy_single])*INV(xyz_c[xy_single]-xyz_t[xy_single]))
        {
            Inverse_Kinematics(xyz_c,abc);
            Jacobian_Matrix(xyz_v,xyz_c,abc,abc_v);
            block_t new_block;
            for (uint8_t i=0;i<3;i++)
            {
                new_block.step[i] = (uint32_t)(fabsf(abc[i] - abc_l[i])*(float)STEPS_PER_UNIT);
                new_block.maximum_velocity[i] = abc_v[i];
                if (abc[i]>abc_l[i])    new_block.dir[i] = 1;
                else                    new_block.dir[i] = 0;
                abc_l[i] = abc[i];
            }
            //Block_Buff_Write(&new_block, &block_buffer);
        }
    }else
    {
        float m_y = (xyz_t[1] - xyz_c[1])*INV(xyz_t[0] - xyz_c[0]);
        float m_z = (xyz_t[2] - xyz_c[2])*INV(xyz_t[0] - xyz_c[0]);
        for (;xyz_t[0]!=xyz_c[0]&&xyz_t[1]!=xyz_c[1];)
        {
            Inverse_Kinematics(xyz_c,abc);
            Jacobian_Matrix(xyz_v,xyz_c,abc,abc_v);
            block_t new_block;
            
            ///B's method
            x_new = x_new + d_x;
            if (fabsf(xyz_c[0] - x_new)>GRID_LEN)               xyz_c[0] += GRID_LEN*fabsf(xyz_t[0]-xyz_c[0])*INV(xyz_t[0]-xyz_c[0]);
            if (fabsf(m_y*x_new - xyz_c[1])>(0.5f*GRID_LEN))    xyz_c[1] += GRID_LEN*fabsf(xyz_t[1]-xyz_c[1])*INV(xyz_t[1]-xyz_c[1]);
            if (fabsf(m_y*x_new - xyz_c[1])>(0.5f*GRID_LEN))    xyz_c[2] += GRID_LEN*fabsf(xyz_t[2]-xyz_c[2])*INV(xyz_t[2]-xyz_c[2]);

            for (uint8_t i=0;i<3;i++)
            {
                new_block.step[i] = (uint32_t)(fabsf(abc[i] - abc_l[i])*(float)STEPS_PER_UNIT);
                new_block.maximum_velocity[i] = abc_v[i];
                if (abc[i]>abc_l[i])    new_block.dir[i] = 1;
                else                    new_block.dir[i] = 0;
                abc_l[i] = abc[i];
            }
            //Block_Buff_Write(&new_block, &block_buffer);
        }
    }
}


int main (void)
{
    float xyz_c[3] = {0.f,0.f,0.f};
    float xyz_t[3] = {5.f,8.f,9.f};
    Line_XYZ_Planner(xyz_c,xyz_t,20.f);
    return 0;
}