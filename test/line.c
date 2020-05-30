#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define DELTA_CHAIN_LEN 185.0f
#define DELTA_TOWER_RADIUS 145.0f
#define DELTA_EFFECTOR_RADIUS 37.15f
#define DELTA_EFFECTOR_HEIGHT 20.0f
#define DELTA_CARRIAGE_OFFSET 16.0f
#define n1 0.0f
#define n2 3.14f
#define n3 3.14f*3.0f*INV(2.0f)
#define R   (DELTA_TOWER_RADIUS - DELTA_CARRIAGE_OFFSET)
#define r   DELTA_EFFECTOR_RADIUS
#define L   DELTA_CHAIN_LEN
#define x1  (R-r)*cosf(n1)
#define y1  (R-r)*sinf(n1)
#define x2  (R-r)*cosf(n2)
#define y2  (R-r)*sinf(n2)
#define x3  (R-r)*cosf(n3)
#define y3  (R-r)*sinf(n3)
#define SQ(x)   ((x)*(x))
#define COS(x)  (cosf(x))
#define SIN(x)  (sinf(x))
#define INV(x)  (1.0f / (x))
#define E(x)    (powf(10.0f,x))
#define PI      3.14f

void Inverse_Kinematics(float* xyz, float* abc)
{
    abc[0] = sqrtf(SQ(L)-SQ(xyz[0]-x1)-SQ(xyz[1]-y1))+xyz[2];
    abc[1] = sqrtf(SQ(L)-SQ(xyz[0]-x2)-SQ(xyz[1]-y2))+xyz[2];
    abc[2] = sqrtf(SQ(L)-SQ(xyz[0]-x3)-SQ(xyz[1]-y3))+xyz[2];
}

void Jacobian_Matrix(float* xyz_v, float* xyz, 
                    float* abc, float* abc_v)
{
    abc_v[0] = ((xyz[0]-x1)*INV(xyz[2]-abc[0])
             + (xyz[0]-x2)*INV(xyz[2]-abc[1])
             + (xyz[0]-x3)*INV(xyz[2]-abc[2]))*xyz_v[0];
    
    abc_v[1] = ((xyz[1]-y1)*INV(xyz[2]-abc[0])
             + (xyz[1]-y2)*INV(xyz[2]-abc[1])
             + (xyz[1]-y3)*INV(xyz[2]-abc[2]))*xyz_v[1];

    abc_v[2] = 3.0f*xyz_v[2];
}

void Linear_Path(int32_t (*traj)[3][2], uint32_t len_traj,int32_t d1, int32_t d2, int32_t d3)
{
    uint16_t i;

    for(i=1;i<len_traj;i++)
    {
        if(-5<(d2*i%d1)&&(d2*i%d1)<5)   traj[i][1][0] = traj[i-1][1][0];
        else if(d2>0)                   traj[i][1][0] = traj[i-1][1][0]+1;
        else                            traj[i][1][0] = traj[i-1][1][0]-1;

        if(-5<(d3*i%d1)&&(d3*i%d1)<5)   traj[i][2][0] = traj[i-1][2][0];
        else if(d3>0)                   traj[i][2][0] = traj[i-1][2][0]+1;
        else                            traj[i][2][0] = traj[i-1][2][0]-1;
    }
    if (d1>0)
    {
        for (i=1;i<len_traj;i++)     traj[i][0][0] = traj[i-1][0][0] + 1;
    }else
    {
        for (i=1;i<len_traj;i++)     traj[i][0][0] = traj[i-1][0][0] - 1;
    }
}

void Linear_Path_Convert(int32_t (*traj)[3][2], uint16_t len,uint8_t case_path)
{
    int32_t xyz[3];
    uint16_t i;
    if (case_path==1);
    else if (case_path==2)
    {
        for (i=0;i<len;i++)
        {
            xyz[1] = traj[i][2][0];
            xyz[2] = traj[i][1][0];
            traj[i][1][0] = xyz[1];
            traj[i][2][0] = xyz[2];
        }
    }else if (case_path==3)
    {
        for (i=0;i<len;i++)
        {
            xyz[0] = traj[i][1][0];
            xyz[1] = traj[i][0][0];
            traj[i][0][0] = xyz[0];
            traj[i][1][0] = xyz[1];
        }
    }else if (case_path==4)
    {
        for (i=0;i<len;i++)
        {
            xyz[0] = traj[i][2][0];
            xyz[1] = traj[i][0][0];
            xyz[2] = traj[i][1][0];
            traj[i][0][0] = xyz[0];
            traj[i][1][0] = xyz[1];
            traj[i][2][0] = xyz[2];
        }
    }else if (case_path==5)
    {
        for (i=0;i<len;i++)
        {
            xyz[0] = traj[i][2][0];
            xyz[1] = traj[i][0][0];
            xyz[2] = traj[i][1][0];
            traj[i][0][0] = xyz[0];
            traj[i][1][0] = xyz[1];
            traj[i][2][0] = xyz[2];
        }
    }else if (case_path==6)
    {
        for (i=0;i<len;i++)
        {
            xyz[0] = traj[i][2][0];
            xyz[2] = traj[i][0][0];
            traj[i][0][0] = xyz[0];
            traj[i][2][0] = xyz[2];
        }
    }
}

void Min_Max(int32_t dx, int32_t dy, int32_t dz, int32_t* d1, int32_t* d2, int32_t* d3)
{
    if (abs(dx)>=abs(dy))
    {
        if (abs(dz)>abs(dx))
        {
            d1[0] = dz;
            d2[0] = dx;
            d3[0] = dy;
        }
        else                    
        {
            d1[0] = dx;
            if (abs(dy)>=abs(dz))
            {d2[0] = dy;
             d3[0] = dz;} 
            else
            {d2[0] = dz;
             d3[0] = dy;} 
        }
    }
    else
    {
        if (abs(dz)>abs(dy))
        {
            d1[0] = dz;
            d2[0] = dy;
            d3[0] = dx;
        }
        else
        {
            d1[0] = dy;
            if (abs(dx)>=abs(dz))
            {d2[0] = dx;
             d3[0] = dz;} 
            else
            {d2[0] = dz;
             d3[0] = dx;} 
        }
    }
}

void Velocity_Decouple(float* xyz_v, uint8_t* d_xyz, float v_n)
{
    if (d_xyz[0]==1)
    {
        if (d_xyz[1]==1)
        {
            if (d_xyz[2]==1)
            {
                xyz_v[0] = v_n*INV(sqrtf(3.0f));
                xyz_v[1] = xyz_v[0];
                xyz_v[2] = xyz_v[0];
            }else
            {
                xyz_v[0] = v_n*INV(sqrtf(2.0f));
                xyz_v[1] = xyz_v[0];
                xyz_v[2] = 0.0f;
            }
        }else
        {
            if (d_xyz[2]==1)
            {
                xyz_v[0] = v_n*INV(sqrtf(2.0f));
                xyz_v[1] = 0.0f;
                xyz_v[2] = xyz_v[0];
            }else
            {
                xyz_v[0] = v_n;
                xyz_v[1] = 0.0f;
                xyz_v[2] = 0.0f;
            }
        }
        
    }else
    {
        if (d_xyz[1]==1)
        {
            if (d_xyz[2]==1)
            {
                xyz_v[0] = 0.0f;
                xyz_v[1] = v_n*INV(sqrtf(2.0f));
                xyz_v[2] = xyz_v[1];
            }else
            {
                xyz_v[0] = 0.0f;
                xyz_v[1] = v_n;
                xyz_v[2] = 0.0f;
            }
        }else
        {
            xyz_v[0] = 0.0f;
            xyz_v[1] = 0.0f;
            xyz_v[2] = v_n;
        }
    }
}

void Kinematics_Planner(int32_t (*traj)[3][2], uint16_t len,float v_n)
{
    uint16_t i;
    float xyz_n[3];//next
    float xyz_l[3];//last
    float xyz_v[3];//velocity
    float abc_n[3];
    float abc_v[3];

    uint8_t d_xyz[3];//velocity vector

    xyz_l[0] = traj[0][0][0];
    xyz_l[1] = traj[0][1][0];
    xyz_l[2] = traj[0][2][0];

    Inverse_Kinematics(xyz_l, abc_n);
    
    for (i=1;i<len;i++)
    {
        xyz_n[0] = (float)traj[i][0][0]*INV(10.0f);
        xyz_n[1] = (float)traj[i][1][0]*INV(10.0f);
        xyz_n[2] = (float)traj[i][2][0]*INV(10.0f);

        if (xyz_l[0]!=xyz_n[0]) d_xyz[0] = 1;
        else                    d_xyz[0] = 0;
        if (xyz_l[1]!=xyz_n[1]) d_xyz[1] = 1;
        else                    d_xyz[1] = 0;
        if (xyz_l[2]!=xyz_n[2]) d_xyz[2] = 1;
        else                    d_xyz[2] = 0;

        Velocity_Decouple(xyz_v,d_xyz,v_n);

        Inverse_Kinematics(xyz_n,abc_n);

        Jacobian_Matrix(xyz_v,xyz_n,abc_n,abc_v);

        traj[i][0][0] = (int32_t)(abc_n[0]*10.0f);
        traj[i][1][0] = (int32_t)(abc_n[1]*10.0f);
        traj[i][2][0] = (int32_t)(abc_n[2]*10.0f);

        //traj[i][0][1] = Velocity_To_Freq(abc_v[0]);
        //traj[i][1][1] = Velocity_To_Freq(abc_v[1]);
        //traj[i][2][1] = Velocity_To_Freq(abc_v[2]);

        xyz_l[0] = xyz_n[0];
        xyz_l[1] = xyz_n[1];
        xyz_l[2] = xyz_n[2];
    }
}

void Linear_Planner(float* xyz_t, float* xyz_c, float velocity, float dwell)
{
    int32_t dx = (int32_t)(xyz_t[0]*10) - (int32_t)(xyz_c[0]*10);
    int32_t dy = (int32_t)(xyz_t[1]*10) - (int32_t)(xyz_c[1]*10);
    int32_t dz = (int32_t)(xyz_t[2]*10) - (int32_t)(xyz_c[2]*10);

    int32_t d1,d2,d3;//abs(d1)>abs(d2)>abs(d3)

    Min_Max(dx,dy,dz,&d1,&d2,&d3);//sorting

    uint32_t len_traj = abs(d1);

    uint8_t case_path;

    int32_t traj[len_traj][3][2];

    if (d1==dx)
    {
        if (d2==dy) 
        {
            case_path = 1;//case 1: xyz
            traj[0][0][0] = xyz_c[0];
            traj[0][1][0] = xyz_c[1];
            traj[0][2][0] = xyz_c[2];
        }
        else
        {
            case_path = 2;//case 2: xzy
            traj[0][0][0] = xyz_c[0];
            traj[0][1][0] = xyz_c[2];
            traj[0][2][0] = xyz_c[1];
        }
    }else if (d1==dy)
    {
        if (d2==dx) 
        {
            case_path = 3;//case 3: yxz
            traj[0][0][0] = xyz_c[1];
            traj[0][1][0] = xyz_c[0];
            traj[0][2][0] = xyz_c[2];
        }
        else
        {
            case_path = 4;//case 4: yzx
            traj[0][0][0] = xyz_c[1];
            traj[0][1][0] = xyz_c[2];
            traj[0][2][0] = xyz_c[0];
        }
    }else
    {
        if (d2==dx) 
        {
            case_path = 5;//case 5: zxy
            traj[0][0][0] = xyz_c[2];
            traj[0][1][0] = xyz_c[0];
            traj[0][2][0] = xyz_c[1];
        }
        else
        {
            case_path = 6;//case 6: zyx
            traj[0][0][0] = xyz_c[2];
            traj[0][1][0] = xyz_c[1];
            traj[0][2][0] = xyz_c[0];
        }
    }
    
    Linear_Path(traj, len_traj, d1, d2, d3);

    for (int i=1;i<400;i++)
    {
        printf("%d,%d,%d\n", traj[i][0][0], traj[i][1][0], traj[i][2][0]);
    }

    Linear_Path_Convert(traj, len_traj,case_path);

    for (int i=1;i<400;i++)
    {
        printf("%d,%d,%d\n", traj[i][0][0], traj[i][1][0], traj[i][2][0]);
    }

    Kinematics_Planner(traj, len_traj, velocity);

    //Trej_Apply(traj, len_traj, dwell, buff);
    for (int i=1;i<400;i++)
    {
        printf("%d,%d,%d\n", traj[i][0][0], traj[i][1][0], traj[i][2][0]);
    }
}



int main(void)
{
    float xyz_c[3] = {0.0f,0.0f,0.0f};
    float xyz_t[3] = {90.0f,90.0f,0.0f};

    Linear_Planner(xyz_t,xyz_c,100,0);

    return 0;
}