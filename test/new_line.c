#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#define SQ(x)   ((x)*(x))
#define COS(x)  (cosf(x))
#define SIN(x)  (sinf(x))
#define INV(x)  (1.0f / (x))
#define E(x)    (powf(10.0f,x))
#define PI      3.14f
#define RSQRT(x)    (1.0f / sqrtf(x))
#define HYPOT2(x,y) (SQ(x)+SQ(y))

#define DELTA_CHAIN_LEN 264.0f
#define DELTA_TOWER_RADIUS 145.0f
#define DELTA_EFFECTOR_RADIUS 37.15f
#define DELTA_EFFECTOR_HEIGHT 20.0f
#define DELTA_CARRIAGE_OFFSET 16.0f
#define n1 0.0f
#define n2 3.14f
#define n3 3.14f*3.0f*INV(2.0f)
#define GRID_LEN 0.1f    //mm

#define CARRIAGE_A_RESET    300.f
#define CARRIAGE_B_RESET    300.f
#define CARRIAGE_C_RESET    300.f

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
#define x1  (R-r)*cosf(n1)//91.85
#define y1  (R-r)*sinf(n1)//0
#define x2  (R-r)*cosf(n2)//-91.85
#define y2  (R-r)*sinf(n2)//0
#define x3  (R-r)*cosf(n3)//0
#define y3  (R-r)*sinf(n3)//-91.85

#define BUFF_LEN    10000

typedef struct
{
    //read by controller
    uint8_t     dir[3];
    uint32_t    maximum_freq[3];

    //update in ISR
    uint32_t    step[3];
    uint8_t     flag;
}block_t;

enum {block_ready, block_busy};

typedef struct
{
    uint16_t head;
    uint16_t tail;
    uint16_t length;
    block_t content[BUFF_LEN];
}block_buff_t;

block_buff_t block_buffer;

void Block_Init(block_t* new_block, float* abc, float* abc_l, float* abc_v)
{
    for (uint8_t i=0;i<3;i++)
    {
        new_block->step[i] = (uint32_t)(fabsf(abc[i] - abc_l[i])*(float)STEPS_PER_UNIT);
        new_block->flag = block_ready;
        if (abc[i]>abc_l[i])
        {
            new_block->dir[i] = 1;
            abc_l[i] += (float)new_block->step[i]*(float)INV(STEPS_PER_UNIT);
        }   
        else
        {
            new_block->dir[i] = 0;
            abc_l[i] -= (float)new_block->step[i]*(float)INV(STEPS_PER_UNIT);
        }
        new_block->maximum_freq[i] = (uint32_t)(fabsf(abc_v[i])*(float)STEPS_PER_UNIT);
        //abc_l[i] = abc[i];
    }
}

void Velocity_Decouple(float* xyz_c, float* xyz_t, float* xyz_v, float v_n)
{
    float len = sqrtf(SQ(xyz_t[0]-xyz_c[0])+SQ(xyz_t[1]-xyz_c[1])+SQ(xyz_t[2]-xyz_c[2]));
    for (uint8_t i=0;i<3;i++)   xyz_v[i] = (xyz_t[i]-xyz_c[i])*INV(len)*v_n;
}

void Inverse_Kinematics(float* xyz, float* abc)
{
    abc[0] = sqrtf(SQ(L)-SQ(xyz[0]-x1)-SQ(xyz[1]-y1))+xyz[2];
    abc[1] = sqrtf(SQ(L)-SQ(xyz[0]-x2)-SQ(xyz[1]-y2))+xyz[2];
    abc[2] = sqrtf(SQ(L)-SQ(xyz[0]-x3)-SQ(xyz[1]-y3))+xyz[2];
}

void Jacobian_Matrix(float* xyz_v, float* xyz, float* abc, float* abc_v)
{
    abc_v[0] = (xyz[0]-x1)*INV(xyz[2]-abc[0])*xyz_v[0]+(xyz[1]-y1)*INV(xyz[2]-abc[0])*xyz_v[1]+xyz_v[2];

    abc_v[1] = (xyz[0]-x2)*INV(xyz[2]-abc[1])*xyz_v[0]+(xyz[1]-y2)*INV(xyz[2]-abc[1])*xyz_v[1]+xyz_v[2];

    abc_v[2] = (xyz[0]-x3)*INV(xyz[2]-abc[2])*xyz_v[0]+(xyz[1]-y3)*INV(xyz[2]-abc[2])*xyz_v[1]+xyz_v[2];
}

void Forward_Kinematics(float* abc, float* xyz)
{
  // Create a vector in old coordinates along x axis of new coordinate
  const float p12[3] = { x2 - x1, y2 - y1, abc[1] - abc[0] },

  // Get the reciprocal of Magnitude of vector.
  d2 = SQ(p12[0]) + SQ(p12[1]) + SQ(p12[2]), inv_d = RSQRT(d2),

  // Create unit vector by multiplying by the inverse of the magnitude.
  ex[3] = { p12[0] * inv_d, p12[1] * inv_d, p12[2] * inv_d },

  // Get the vector from the origin of the new system to the third point.
  p13[3] = { x3 - x1, y3 - y1, abc[2] - abc[0] },

  // Use the dot product to find the component of this vector on the X axis.
  i = ex[0] * p13[0] + ex[1] * p13[1] + ex[2] * p13[2],

  // Create a vector along the x axis that represents the x component of p13.
  iex[3] = { ex[0] * i, ex[1] * i, ex[2] * i };

  // Subtract the X component from the original vector leaving only Y. We use the
  // variable that will be the unit vector after we scale it.
  float ey[3] = { p13[0] - iex[0], p13[1] - iex[1], p13[2] - iex[2] };

  // The magnitude and the inverse of the magnitude of Y component
  const float j2 = SQ(ey[0]) + SQ(ey[1]) + SQ(ey[2]), inv_j = RSQRT(j2);

  // Convert to a unit vector
  ey[0] *= inv_j; ey[1] *= inv_j; ey[2] *= inv_j;

  // The cross product of the unit x and y is the unit z
  // float[] ez = vectorCrossProd(ex, ey);
  const float ez[3] = {
    ex[1] * ey[2] - ex[2] * ey[1],
    ex[2] * ey[0] - ex[0] * ey[2],
    ex[0] * ey[1] - ex[1] * ey[0]
  },

  // We now have the d, i and j values defined in Wikipedia.
  // Plug them into the equations defined in Wikipedia for Xnew, Ynew and Znew
  Xnew = (d2) * inv_d * 0.5f,
  Ynew = ((SQ(i) + j2) * 0.5f - i * Xnew) * inv_j,
  Znew = sqrtf(SQ(L) - HYPOT2(Xnew, Ynew));//

  // Start from the origin of the old coordinates and add vectors in the
  // old coords that represent the Xnew, Ynew and Znew to find the point
  // in the old system.
  xyz[0] = x1+ex[0] * Xnew + ey[0] * Ynew - ez[0] * Znew;
  xyz[1] = y1+ex[1] * Xnew + ey[1] * Ynew - ez[1] * Znew;
  xyz[2] = abc[0] + ex[2] * Xnew + ey[2] * Ynew - ez[2] * Znew;
}

uint8_t Block_Buff_Write(block_t block, block_buff_t* ring_buff)
{
    if(ring_buff->length >= BUFF_LEN) return 1;

    ring_buff->content[ring_buff->tail] = block;

    for (uint8_t i=0;i<3;i++)
    {
        ring_buff->content[ring_buff->tail].maximum_freq[i] = block.maximum_freq[i];
        ring_buff->content[ring_buff->tail].step[i] = block.step[i];
        ring_buff->content[ring_buff->tail].dir[i] = block.dir[i];
    }
    ring_buff->tail = (ring_buff->tail+1)%BUFF_LEN;
    ring_buff->length ++;
    return 0;
}

void Block_Buff_Clear(volatile block_buff_t* ring_buff)
{
    if (ring_buff->length!=0)
    {
        ring_buff->head = (ring_buff->head+1)%BUFF_LEN;
        ring_buff->length--;
    }
}

uint8_t Line_Z_Planner(float dz, float feedrate)
{
    uint32_t step = (uint32_t)fabsf(dz * (float)STEPS_PER_UNIT);
    
    block_t new_block;
    uint8_t dir;
    if (dz>0)   dir = 1;
    else        dir = 0;

    for (uint8_t i=0;i<3;i++)
    {
        new_block.step[i] = step;
        new_block.dir[i] = dir;
        new_block.maximum_freq[i] = (uint32_t)(feedrate*(float)STEPS_PER_UNIT);
    }
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
        if (block_buffer.length<BUFF_LEN)
        {
            Line_Z_Planner(xyz_t[2]-xyz_i[2],feedrate);
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
    

    for (;step>0&&block_buffer.length<(BUFF_LEN);step--)
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

        //printf("x_c:%f, y_c:%f, z_c:%f\n",xyz_c[0],xyz_c[1],xyz_c[2]);
        Inverse_Kinematics(xyz_c,abc);
        Jacobian_Matrix(xyz_v,xyz_c,abc,abc_v);

        block_t new_block;
        Block_Init(&new_block, abc, abc_l, abc_v);
        if (new_block.step[0]!=0||new_block.step[1]!=0||new_block.step[2]!=0)
        Block_Buff_Write(new_block, &block_buffer);
        /* float xyz_temp[3];
        
        Forward_Kinematics(abc_l,xyz_temp);
        printf("x:%f, y:%f, z:%f\n",xyz_temp[0],xyz_temp[1],xyz_temp[2]);
        printf("a:%f, b:%f, c:%f\n",abc_l[0],abc_l[1],abc_l[2]); */

    }
    if (step>0) return 1;
    else        return 0;
}


uint8_t Line_XYZ_Planner(float* xyz_c, float* xyz_t, float feedrate)
{
    float abc_l[3];
    float abc[3];
    float xyz_v[3];
    float abc_v[3];
    float d_x = 0.f;
    float d_y = 0.f;
    uint8_t pulse = 0;

    Velocity_Decouple(xyz_c,xyz_t,xyz_v,feedrate);
    Inverse_Kinematics(xyz_c,abc_l);

    //dx = 0, dy != 0 and dy > dz
    if ((xyz_c[0]-xyz_t[0])==0.f&&(xyz_c[1]-xyz_t[1])!=0.f&&(fabsf(xyz_c[1]-xyz_t[1])>=fabsf(xyz_c[2]-xyz_t[2])))       d_y = GRID_LEN;
    //dx = 0, dy != 0 and dy < dz
    else if ((xyz_c[0]-xyz_t[0])==0.f&&(xyz_c[1]-xyz_t[1])!=0.f&&(fabsf(xyz_c[1]-xyz_t[1])<fabsf(xyz_c[2]-xyz_t[2])))   d_y = GRID_LEN*fabsf(xyz_c[1]-xyz_t[1])*INV(xyz_c[2]-xyz_t[2]);
    //dx = 0, dy = 0 and dz != 0
    else if ((xyz_c[0]-xyz_t[0])==0.f&&(xyz_c[1]-xyz_t[1])==0.f)
    {
        if (Line_Z_Planner(xyz_t[2]-xyz_c[2],feedrate)) return 1;
        else                                            return 0;
    }
    //dx != 0 and dy max
    else if (fabsf(xyz_c[1]-xyz_t[1])>=fabsf(xyz_c[0]-xyz_t[0])&&fabsf(xyz_c[1]-xyz_t[1])>=fabsf(xyz_c[2]-xyz_t[2]))    d_x = GRID_LEN*fabsf(xyz_c[0]-xyz_t[0])*INV(fabsf(xyz_c[1]-xyz_t[1]));
    //dx != 0 and dz max
    else if (fabsf(xyz_c[2]-xyz_t[2])>=fabsf(xyz_c[0]-xyz_t[0])&&fabsf(xyz_c[2]-xyz_t[2])>=fabsf(xyz_c[1]-xyz_t[1]))    d_x = GRID_LEN*fabsf(xyz_c[0]-xyz_t[0])*INV(fabsf(xyz_c[2]-xyz_t[2]));
    //dx != 0 and dx max
    else    d_x = GRID_LEN;
    
    if (d_y!=0.f)
    {
        uint64_t step = (uint64_t)(fabsf(xyz_t[1]-xyz_c[1])*INV(d_y));
        float y_new = xyz_c[1];
        uint8_t pulse;
        //calculate gradient
        float m_z = (xyz_t[2] - xyz_c[2])*INV(xyz_t[1] - xyz_c[1]);
        for (;step>0;)
        {
            if ((xyz_t[0]-xyz_c[0])>0)          y_new = y_new + d_y;
            else if ((xyz_t[0]-xyz_c[0])<0)     y_new = y_new - d_y;
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
                if (Block_Buff_Write(new_block, &block_buffer))  return 1;
                printf("");
                pulse = 0;
            }
        }
    }else
    {
        float m_y = (xyz_t[1] - xyz_c[1])*INV(xyz_t[0] - xyz_c[0]);
        float m_z = (xyz_t[2] - xyz_c[2])*INV(xyz_t[0] - xyz_c[0]);
        float x_new = xyz_c[0];
        uint64_t step = (uint64_t)(fabsf(xyz_t[0]-xyz_c[0])*INV(d_x));
        for (;step>0;)//error
        {

            ///B's method
            if ((xyz_t[0]-xyz_c[0])>0)          x_new = x_new + d_x;
            else if ((xyz_t[0]-xyz_c[0])<0)     x_new = x_new - d_x;
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
                if(Block_Buff_Write(new_block, &block_buffer))  return 1;
                pulse = 0;
            }
        }
    }
    return 0;
}



const float path_0[7][3] = {{0.f,0.f,0.f}
                           ,{80.f,80.f,0.f}
                           ,{-80.f,80.f,0.f}
                           ,{80.f,-80.f,0.f}
                           ,{-80.f,80.f,0.f}
                           ,{0.f,0.f,0.f}
                           ,{0.f,0.f,30.f}};

int main (void)
{
    uint8_t path_num = 1;
    float abc[3] = {CARRIAGE_A_RESET, CARRIAGE_B_RESET, CARRIAGE_C_RESET};
    float xyz_c[3];
    float xyz_i[3];
    float xyz[3];
    
    float xyz_t[3] = {-90.f,90.f,0.f};
    //Forward_Kinematics(abc,xyz);

    

    Forward_Kinematics(abc,xyz);
    printf("x:%f,y:%f,z:%f\n",xyz[0],xyz[1],xyz[2]);
    for (uint8_t i=0;i<3;i++)   xyz_c[i] = xyz[i];
    block_buffer.head = 0;
    block_buffer.length = 0;
    block_buffer.tail = 0;
    Line_XYZ_Planner_1(xyz,xyz_c,xyz_t,5.f);



    



    //Line_XYZ_Planner_1(xyz,xyz_c,xyz_t,5.f);
    /* while (path_num<7)
    {
        for (uint8_t i=0;i<3;i++)   xyz_c[i] = path_0[path_num-1][i];
        Line_XYZ_Planner_1(path_0[path_num-1], xyz_c,path_0[path_num],5.0f);
        printf("x_c:%f, y_c:%f, z_c:%f\n", xyz_c[0], xyz_c[1], xyz_c[2]);
        path_num ++;
    } */

    printf("number:%d\n", block_buffer.length);

    for (;block_buffer.length>0;)
    {
        //printf("maximum_velocity:%f %f %f\n", block_buffer.content[block_buffer.head].maximum_velocity[0]
        //                                    , block_buffer.content[block_buffer.head].maximum_velocity[1]
        //                                    , block_buffer.content[block_buffer.head].maximum_velocity[2]);

        if (block_buffer.content[block_buffer.head].dir[0]==1)  abc[0] += ((float)block_buffer.content[block_buffer.head].step[0])*INV((float)STEPS_PER_UNIT);
        else                                                    abc[0] -= ((float)block_buffer.content[block_buffer.head].step[0])*INV((float)STEPS_PER_UNIT);

        if (block_buffer.content[block_buffer.head].dir[1]==1)  abc[1] += ((float)block_buffer.content[block_buffer.head].step[1])*INV((float)STEPS_PER_UNIT);
        else                                                    abc[1] -= ((float)block_buffer.content[block_buffer.head].step[1])*INV((float)STEPS_PER_UNIT);

        if (block_buffer.content[block_buffer.head].dir[2]==1)  abc[2] += ((float)block_buffer.content[block_buffer.head].step[2])*INV((float)STEPS_PER_UNIT);
        else                                                    abc[2] -= ((float)block_buffer.content[block_buffer.head].step[2])*INV((float)STEPS_PER_UNIT);    

        Forward_Kinematics(abc,xyz);

        printf("x:%f,y:%f,z:%f\n",xyz[0],xyz[1],xyz[2]);


        /* printf("{{%d, %d, %d},"    , block_buffer.content[block_buffer.head].maximum_freq[0]
                                            , block_buffer.content[block_buffer.head].maximum_freq[1]
                                            , block_buffer.content[block_buffer.head].maximum_freq[2]);
        printf("{%d, %d, %d},"             , block_buffer.content[block_buffer.head].dir[0]
                                            , block_buffer.content[block_buffer.head].dir[1]
                                            , block_buffer.content[block_buffer.head].dir[2]);
        printf("{%d, %d, %d}},\n"            , block_buffer.content[block_buffer.head].step[0]
                                            , block_buffer.content[block_buffer.head].step[1]
                                            , block_buffer.content[block_buffer.head].step[2]); */
        //printf("number:%d\n", block_buffer.length);
        Block_Buff_Clear(&block_buffer);
    }

    for (uint8_t i=0;i<3;i++)
    {
        xyz_c[i] = xyz[i];
    }

    xyz_t[0] = 0.f;
    xyz_t[1] = 0.f;
    xyz_t[2] = 52.f;

    printf("\n");


    Line_XYZ_Planner_1(xyz,xyz_c,xyz_t,5.f);

    for (;block_buffer.length>0;)
    {
        //printf("maximum_velocity:%f %f %f\n", block_buffer.content[block_buffer.head].maximum_velocity[0]
        //                                    , block_buffer.content[block_buffer.head].maximum_velocity[1]
        //                                    , block_buffer.content[block_buffer.head].maximum_velocity[2]);

        if (block_buffer.content[block_buffer.head].dir[0]==1)  abc[0] += ((float)block_buffer.content[block_buffer.head].step[0])*INV((float)STEPS_PER_UNIT);
        else                                                    abc[0] -= ((float)block_buffer.content[block_buffer.head].step[0])*INV((float)STEPS_PER_UNIT);

        if (block_buffer.content[block_buffer.head].dir[1]==1)  abc[1] += ((float)block_buffer.content[block_buffer.head].step[1])*INV((float)STEPS_PER_UNIT);
        else                                                    abc[1] -= ((float)block_buffer.content[block_buffer.head].step[1])*INV((float)STEPS_PER_UNIT);

        if (block_buffer.content[block_buffer.head].dir[2]==1)  abc[2] += ((float)block_buffer.content[block_buffer.head].step[2])*INV((float)STEPS_PER_UNIT);
        else                                                    abc[2] -= ((float)block_buffer.content[block_buffer.head].step[2])*INV((float)STEPS_PER_UNIT);    

        Forward_Kinematics(abc,xyz);

        printf("x:%f,y:%f,z:%f\n",xyz[0],xyz[1],xyz[2]);

        Block_Buff_Clear(&block_buffer);
    }

    return 0;
}