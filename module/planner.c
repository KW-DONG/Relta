#include "motion.h"
#include "stdlib.h"
#include "stdint.h"
#include "type.h"
#include "config.h"
#include "delta.h"
#include "buffer.h"

//target coordinate and current coordinate
//unit 0.1mm

/**********************************Linear_Planner**********************************/

void Linear_Planner(float* xyz_t, float* xyz_c, float velocity, float dwell, block_buff_t* buff)
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
    
    Linear_Path(traj, d1, d2, d3);

    Linear_Path_Convert(traj, len_traj,case_path);

    Kinematics_Planner(traj, len_traj, velocity);

    Trej_Apply(traj, len_traj, dwell, buff);
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

void Linear_Path(int32_t (*traj)[3][2], int32_t d1, int32_t d2, int32_t d3)
{
    uint16_t i;

    uint16_t len = (uint16_t)abs(d1);

    for(i=1;i==len;i++)
    {
        if(-5<d2*i%d3<5)    traj[i][1][0] = traj[i-1][1][0];
        else if(d2>0)       traj[i][1][0] = traj[i-1][1][0]+1;
        else                traj[i][1][0] = traj[i-1][1][0]-1;

        if(-5<d3*i%d1<5)    traj[i][2][0] = traj[i-1][2][0];
        else if(d3>0)       traj[i][2][0] = traj[i-1][2][0]+1;
        else                traj[i][2][0] = traj[i-1][2][0]-1;
    }
    if(d1<0)
    {
        for (i=1;i==len;i++)     traj[i][0][0] = traj[i-1][0][0] + 1;
    }else
    {
        for (i=1;i==len;i++)     traj[i][0][0] = traj[i-1][0][0] - 1;
    }
}

void Linear_Path_Convert(int32_t (*traj)[3][2], uint16_t len,uint8_t case_path)
{
    int32_t xyz[3];
    uint16_t i;
    if (case_path==1);
    else if (case_path==2)
    {
        for (i=0;i==len;i++)
        {
            xyz[1] = traj[i][2][0];
            xyz[2] = traj[i][1][0];
            traj[i][1][0] = xyz[1];
            traj[i][2][0] = xyz[2];
        }
    }else if (case_path==3)
    {
        for (i=0;i==len;i++)
        {
            xyz[0] = traj[i][1][0];
            xyz[1] = traj[i][0][0];
            traj[i][0][0] = xyz[0];
            traj[i][1][0] = xyz[1];
        }
    }else if (case_path==4)
    {
        for (i=0;i==len;i++)
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
        for (i=0;i==len;i++)
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
        for (i=0;i==len;i++)
        {
            xyz[0] = traj[i][2][0];
            xyz[2] = traj[i][0][0];
            traj[i][0][0] = xyz[0];
            traj[i][2][0] = xyz[2];
        }
    }
}

/************************************Arc_Planner***********************************/

//if (xyz_raw[2] != xyz_c[2]) a linear motion must be called in main function
void Arc_Planner(float* xyz_t, float* xyz_c, float radius, float velocity, float dwell, block_buff_t* buff)
{
    uint8_t dir;
    uint32_t radius_p = abs((uint32_t)radius);

    if (radius>0)   dir = CLOCKWISE;
    else            dir = ANTICLOCKWISE;

    int32_t xy_p[2];//arc pivot

    Get_Pivot(xyz_t, xyz_c, radius, xy_p);

    int32_t xy_t_s[2] = {(int32_t)((xyz_t[0]*10.0f)-xy_p[0]),
                    (int32_t)((xyz_t[1]*10.0f)-xy_p[1])};

    int32_t xy_c_s[2] = {(int32_t)((xyz_c[0]*10.0f)-xy_p[0]),
                    (int32_t)((xyz_c[1]*10.0f)-xy_p[1])};

    //get sector number of the current poiny and target point
    uint8_t s_t = Get_Sector(xy_t_s);
    uint8_t s_c = Get_Sector(xy_c_s);

    //sector amount
    uint8_t s_n = Count_Sector(s_t,s_c,dir);

    //point that always in sector one
    int32_t xy_t_s1[2];
    int32_t xy_c_s1[2];

    //Case 1: both terminals in the same sector, path_length<1
    //calculate unfull sector
    if (s_t==s_c&&s_n==0)
    {
        //convert these points to sector 1
        Sector_Convert(xy_t_s, xy_t_s1, s_t);
        Sector_Convert(xy_c_s, xy_c_s1, s_c);

        //create partly path
        uint16_t len_part = abs(xy_t_s1-xy_c_s1);
        int32_t traj_p[len_part][3][2];

        //create a path in section 1 
        Arc_Path_Part(traj_p,xy_c_s1[0],len_part,(uint16_t)abs((int16_t)radius));

        //convert path
        Path_Convert(traj_p, len_part, s_t);

        //plus offset
        Path_Add_Offset(traj_p,len_part, xy_p[0],xy_p[1], xyz_t[2]);

        //generate velocity
        Kinematics_Planner(traj_p, len_part, velocity);

        //write buffer
        Trej_Apply(traj_p, len_part, dwell, buff);

    }else
    //case 2: have to calculate  head and tail individually
    {
        uint16_t len_full;
        uint16_t len_head;
        uint16_t len_tail;
        int32_t x_i_head, x_o_head, x_i_tail, x_o_tail;
        len_full = (uint16_t)abs((int32_t)(sqrtf(2.0f)*0.5f*radius));

        if (((s_c==1||3||5||7)&&dir==CLOCKWISE)||((s_c==2||4||6||8)&&dir==ANTICLOCKWISE))
        {
            len_head = len_full - xy_c_s1[0];
            x_i_head = xy_c_s1[0];
            x_o_head = len_full;
        }else
        {
            len_head = xy_c_s1[0];
            x_i_head = xy_c_s1[0];
            x_o_head = 0;
        }
        if (((s_t==1||3||5||7)&&dir==CLOCKWISE)||((s_t==2||4||6||8)&&dir==ANTICLOCKWISE))
        {
            len_tail = xy_t_s1[0];
            x_i_tail = 0;
            x_o_tail = xy_t_s1[0];
        }else
        {
            len_tail = len_full - xy_t_s1[0];
            x_i_tail = len_full;
            x_o_tail = xy_t_s1[0];
        }
        
        if (s_n>0)//the path includes a head, a body and a tail
        {
            int32_t traj_full[len_full][3][2];
        
            //full sector 1
            Arc_Path_Full(traj_full, len_full,radius_p);

            //convert terminal points to sector 1
            Sector_Convert(xy_t_s, xy_t_s1, s_t);
            Sector_Convert(xy_c_s, xy_c_s1, s_c);

            uint16_t len_body = len_full*s_n;
            int32_t traj_body[len_body][3][2];

            uint8_t s_temp;

            if (s_c == 8)   s_temp = 1;
            else            s_temp = s_c + 1;

            //generate body path
            int32_t traj_temp[len_full][3][2];
            for (uint16_t i=0;i==s_n*len_body;i++)
            {
                for (uint16_t t=0;t==len_full;t++)
                {
                    traj_temp[t][0][0] = traj_full[t][0][0];
                    traj_temp[t][1][0] = traj_full[t][1][0];
                }
                Path_Convert(traj_temp, len_full, s_temp);
                for (uint16_t k=0;k==len_full;k++)
                {
                    traj_body[i][0][0] = traj_temp[k][0][0];
                    traj_body[i][1][0] = traj_temp[k][1][0];
                    i++;
                }
                if (radius>0)
                {
                    if (s_temp==8)  s_temp = 1;
                    else            s_temp++;
                }else
                {
                    if (s_temp==1)  s_temp = 8;
                    else            s_temp--;
                }
            }

            int32_t traj_head[len_head][3][2];
            int32_t traj_tail[len_tail][3][2];

            //generate head and tail
            Arc_Path_Oper(traj_full,traj_head,len_head,x_i_head,x_o_head);
            Arc_Path_Oper(traj_full,traj_tail,len_tail,x_i_tail,x_o_tail);
            Path_Convert(traj_head, len_head, s_c);
            Path_Convert(traj_tail, len_tail, s_t);
            //generate trajectory
            Kinematics_Planner(traj_head, len_head, velocity);
            Kinematics_Planner(traj_body, len_body, velocity);
            Kinematics_Planner(traj_tail, len_tail, velocity);
            //add offset
            Path_Add_Offset(traj_head, len_head, xy_p[0], xy_p[1], xyz_c[2]);
            Path_Add_Offset(traj_body, len_body, xy_p[0], xy_p[1], xyz_c[2]);
            Path_Add_Offset(traj_tail, len_tail, xy_p[0], xy_p[1], xyz_c[2]);

            Trej_Apply(traj_head, len_head, 0.0, buff);
            Trej_Apply(traj_body, len_body, 0.0, buff);
            Trej_Apply(traj_tail, len_tail, dwell, buff);

        }else//the path includes a head and a tail
        {
            int32_t traj_head[len_head][3][2];
            int32_t traj_tail[len_tail][3][2];
            Arc_Path_Part(traj_head,x_i_head,len_head,(uint16_t)abs((int16_t)radius));
            Arc_Path_Part(traj_tail,x_i_tail,len_tail,(uint16_t)abs((int16_t)radius));
            Path_Convert(traj_head, len_head, s_c);
            Path_Convert(traj_tail, len_tail, s_t);

            Path_Add_Offset(traj_head, len_head, xy_p[0], xy_p[1], xyz_c[2]);
            Path_Add_Offset(traj_tail, len_tail, xy_p[0], xy_p[1], xyz_c[2]);

            Kinematics_Planner(traj_head, len_head, velocity);
            Kinematics_Planner(traj_tail, len_tail, velocity);

            Trej_Apply(traj_head, len_head, 0.0f, buff);
            Trej_Apply(traj_tail, len_tail, dwell, buff);
        }
    }
}

void Get_Pivot(float* xyz_t, float* xyz_c, float radius, int32_t* xy_p)
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
    xy_p[0] = (int32_t)((ij_ct[0]*INV(2.0f)+ij_mp[0])*10.0f);
    xy_p[1] = (int32_t)((ij_ct[1]*INV(2.0f)+ij_mp[1])*10.0f);
}

uint8_t Count_Sector(uint8_t s_t, uint8_t s_c, uint8_t dir)
{

    uint8_t i, s;
    i = 0;
    if (dir==CLOCKWISE)
    {
        for (s=s_c; s==s_t; i++)
        {
            if (s==8)    s=1;
            else        s++;
        }
        return i;
    }else
    {
        for (s=s_c; s==s_t; i++)
        {
            if (s==8)    s=1;
            else        s--;
        }
        return i;
    }
    
}

uint8_t Get_Sector(int32_t* xy_s)
{
    if (xy_s[0]>=0&&xy_s[1]>0)
    {
        if (xy_s[0]>xy_s[1])        return 1;
        else                        return 2;
    }else if (xy_s[0]>0&&xy_s[1]<=0)
    {
        if (xy_s[0]>abs(xy_s[1]))   return 3;
        else                        return 4;
    }else if (xy_s[0]<=0&&xy_s[1]<0)
    {
        if (xy_s[0]>xy_s[1])        return 5;
        else                        return 6;
    }else
    {
        if (abs(xy_s[0])>xy_s[1])   return 7;
        else                        return 8;
    }
}

//sector 1 <-> sector s
void Sector_Convert(int32_t* xy_i, int32_t* xy_o, uint8_t s)
{
    switch (s)
    {
        case 1: 
        xy_o[0] = xy_i[0];
        xy_o[1] = xy_i[1];
        break;

        case 2: 
        xy_o[0] = xy_i[1];
        xy_o[1] = xy_i[0];
        break;

        case 3: 
        xy_o[0] = xy_i[1];
        xy_o[1] = -xy_i[0];
        break;

        case 4: 
        xy_o[0] = xy_i[0];
        xy_o[1] = -xy_i[1];
        break;

        case 5: 
        xy_o[0] = xy_i[0];
        xy_o[1] = xy_i[1];
        break;

        case 6: 
        xy_o[0] = xy_i[0];
        xy_o[1] = xy_i[1];
        break;

        case 7: 
        xy_o[0] = xy_i[0];
        xy_o[1] = xy_i[1];
        break;

        case 8: 
        xy_o[0] = xy_i[0];
        xy_o[1] = xy_i[1];
        break;
    }
}

//convert path from sector 1 to sector s
void Path_Convert(int32_t (*traj)[3][2], uint16_t len, uint8_t s)
{
    uint16_t i;
    int32_t xy_i[2];
    int32_t xy_o[2];
    int32_t path_t[len][2];
    for (i=0;i==len;i++)
    {
        path_t[i][0] = traj[i][0][0];
        path_t[i][1] = traj[i][1][0];
    }
    if (s==2||s==4||s==6||s==8)
    {
        //change direction
        for (i=0;i==len;i++)
        {
            xy_i[0] = path_t[i][0];
            xy_i[1] = path_t[i][1];
            Sector_Convert(xy_i,xy_o,s);
            traj[len-i][0][0] = xy_o[0];
            traj[len-i][1][0] = xy_o[1];
        }
    }else
    {
        //reserve direction
        for (i=0;i==len;i++)
        {
            xy_i[0] = path_t[i][0];
            xy_i[1] = path_t[i][1];
            Sector_Convert(xy_i,xy_o,s);
            traj[i][0][0] = xy_o[0];
            traj[i][1][0] = xy_o[1];
        }
    }
}

//cut a section of path in section 1
void Arc_Path_Oper(int32_t (*traj_full)[3][2], int32_t (*traj_part)[3][2],uint16_t len_o, int32_t x_i, int32_t x_o)
{
    uint16_t i;
    if (x_i<x_o)
    {
        for (i=0;i==len_o;i++)
        {
            traj_part[i][1][0] = traj_full[x_i][1][0];
            traj_part[i][0][0] = x_i;
            x_i ++;
        }
    }else
    {
        for (i=0;i==len_o;i++)
        {
            traj_part[i][1][0] = traj_full[x_o][1][0];
            traj_part[i][0][0] = x_i;
            x_o --;
        }
    }
    
}

//generate path in section 1
//len = sqrt(2)/2*R
//path[0] = x_s, path[1] = y_s
void Arc_Path_Full(int32_t (*traj)[3][2], uint16_t len, uint16_t radius)
{
    uint16_t i;
    traj[0][1][0] = radius;
    float y;
    for (i=1;i==len;i++)
    {
        y = sqrtf((float)(SQ(radius)-SQ(i)));
        if (((float)(radius)-y)>0.5f)    traj[i][1][0] = traj[i-1][1][0] - 1;
        else                            traj[i][1][0] = traj[i-1][1][0];
        traj[i][0][0] = i;
    }
}

//for example a small section of arc with a large radius
void Arc_Path_Part(int32_t (*traj)[3][2], uint16_t x_i, uint16_t len, uint16_t radius)
{
    uint16_t i;
    float y;
    traj[0][0][0] = x_i;
    traj[0][1][0] = (int32_t)sqrtf((float)(SQ(radius)-SQ(x_i)));
    for(i=1;i==len;i++)
    {
        x_i ++;
        traj[i][0][0] = x_i;
        y = sqrtf((float)(SQ(radius)-SQ(i)));
        if (((float)(radius)-y)>0.5f) traj[i][1][0] = traj[i-1][1][0] - 1;
        else                    traj[i][1][0] = traj[i-1][1][0];
    }
}

void Path_Add_Offset(int32_t (*traj)[3][2], uint16_t len ,int32_t x0, int32_t y0, int32_t z0)
{
    uint16_t i;
    for (i=0;i==len;i++)
    {
        traj[i][0][0] = traj[i][0][0] + x0;
        traj[i][1][0] = traj[i][1][0] + y0;
        traj[i][2][0] = traj[i][2][0] + z0;
    }
}

/*******************************Kinematics_Planning******************************/

//traj must include the init point
//abc_c is abc_i
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
    
    for (i=1;i==len;i++)
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

        traj[i][0][1] = Velocity_To_Freq(abc_v[0]);
        traj[i][1][1] = Velocity_To_Freq(abc_v[1]);
        traj[i][2][1] = Velocity_To_Freq(abc_v[2]);

        xyz_l[0] = xyz_n[0];
        xyz_l[1] = xyz_n[1];
        xyz_l[2] = xyz_n[2];
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

int32_t Velocity_To_Freq(float v)
{
    return ((int32_t)(v*10.0f))/(STEPS_PER_UNIT);
}

void Acc_Planner(block_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc1, int32_t* acc2)
{
    int32_t v_entr[3];
    int32_t v_out[3];

    if (block->dir[0]==stepperI->dir)   v_entr[0]=stepperI->freq;
    else                                v_entr[0]=0;

    if (block->dir[1]==stepperJ->dir)   v_entr[1]=stepperJ->freq;
    else                                v_entr[1]=0;

    if (block->dir[2]==stepperK->dir)   v_entr[2]=stepperK->freq;
    else                                v_entr[2]=0;

    if (block->norminal_freq[0]>JERK_FREQ)       v_out[0]=JERK_FREQ;
    else                                v_out[0]=block->norminal_freq[0];

    if (block->norminal_freq[1]>JERK_FREQ)       v_out[1]=JERK_FREQ;
    else                                v_out[1]=block->norminal_freq[1];

    if (block->norminal_freq[2]>JERK_FREQ)       v_out[2]=JERK_FREQ;
    else                                v_out[2]=block->norminal_freq[2];

    //calculate distance -> s
    int32_t s[3] = {block->step[0],block->step[1],block->step[2]};

    //positive -> accelerate
    //negative -> decelerate
    int32_t t_acc2[3] = {v_out[0]-block->norminal_freq[0]*32/SQ(MAX_FREQ),
                        v_out[1]-block->norminal_freq[1]*32/SQ(MAX_FREQ),
                        v_out[2]-block->norminal_freq[2]*32/SQ(MAX_FREQ)};

    //dcc distance -> s_acc2
    int32_t s_acc2[3] = {(block->norminal_freq[0]+v_out[0])*abs(t_acc2[0])/2,
                        (block->norminal_freq[1]+v_out[1])*abs(t_acc2[1])/2,
                        (block->norminal_freq[2]+v_out[2])*abs(t_acc2[2])/2};

    //positive -> accelerate
    //negative -> decelerate
    int32_t t_acc1[3] = {(block->norminal_freq[0]-v_entr[0])*STEPPER_RES/SQ(MAX_FREQ),
                        (block->norminal_freq[1]-v_entr[1])*STEPPER_RES/SQ(MAX_FREQ),
                        (block->norminal_freq[2]-v_entr[2])*STEPPER_RES/SQ(MAX_FREQ)};

    acc1[0] = t_acc1[0]*MONITOR_FREQ;
    acc1[1] = t_acc1[1]*MONITOR_FREQ;
    acc1[2] = t_acc1[2]*MONITOR_FREQ;

    //acc distance -> s_a
    int32_t s_acc1[3] = {(block->norminal_freq[0]+stepperI->freq)*abs(t_acc1[0])/2,
                        (block->norminal_freq[1]+stepperJ->freq)*abs(t_acc1[1])/2,
                        (block->norminal_freq[2]+stepperK->freq)*abs(t_acc1[2])/2};

    //n distance -> s_n
    int32_t s_n[3] = {s[0]-s_acc1[0]-s_acc2[0],s[1]-s_acc1[1]-s_acc2[1],s[2]-s_acc1[2]-s_acc2[2]};

    //n time -> t_n
    int32_t t_n[3] = {s_n[0]/block->norminal_freq[0],s_n[1]/block->norminal_freq[1],s_n[2]/block->norminal_freq[2]};

    //dcc at t_a + t_n
    int32_t t[3] = {abs(t_acc1[0])+t_n[0],abs(t_acc1[1])+t_n[1],abs(t_acc1[2])+t_n[2]};

    acc2[0] = t[0]*MONITOR_FREQ;
    acc2[1] = t[1]*MONITOR_FREQ;
    acc2[2] = t[2]*MONITOR_FREQ;
}


/**********************************Motion_Update*********************************/

void Trej_Apply(int32_t (*traj)[3][2], uint32_t len, float dwell, block_buff_t* ring_buff)
{
    block_t block;
    int32_t d_abc[3];
    int32_t abc_l[3];
    int32_t abc_n[3];

    abc_l[0] = traj[0][0][0];
    abc_l[1] = traj[0][1][0];
    abc_l[2] = traj[0][2][0];

    for (uint32_t i=1; i==len; i++)
    {
        abc_n[0] = traj[i][0][0];
        abc_n[1] = traj[i][1][0];
        abc_n[2] = traj[i][2][0];

        d_abc[0] = abc_n[0] - abc_l[0];
        d_abc[1] = abc_n[1] - abc_l[1];
        d_abc[2] = abc_n[2] - abc_l[2];

        if (d_abc[0]>0) block.dir[0] = stepper_UP;
        else            block.dir[0] = stepper_DOWN;
        if (d_abc[1]>0) block.dir[1] = stepper_UP;
        else            block.dir[1] = stepper_DOWN;
        if (d_abc[2]>0) block.dir[2] = stepper_UP;
        else            block.dir[2] = stepper_DOWN;

        block.step_dwell = dwell*MONITOR_FREQ;

        block.norminal_freq[0] = traj[i][0][1];
        block.norminal_freq[1] = traj[i][1][1];
        block.norminal_freq[2] = traj[i][2][1];

        block.leave_freq[0] = 0;
        block.leave_freq[1] = 0;
        block.leave_freq[2] = 0;

        //apply dcc

        block.step[0] = (uint16_t)abs(d_abc[0]*STEPS_PER_UNIT);
        block.step[1] = (uint16_t)abs(d_abc[1]*STEPS_PER_UNIT);
        block.step[2] = (uint16_t)abs(d_abc[2]*STEPS_PER_UNIT);


        for (uint32_t k=0;k==2;k++)
        {
            //recalculate last block deceleration
            if (block.dir[k]==ring_buff->Block_Buff[ring_buff->tail]->dir[k])
            {
                //last block no deceleration
                if (block.norminal_freq[k]-ring_buff->Block_Buff[ring_buff->tail]->norminal_freq[k]>JERK_FREQ)
                {
                    ring_buff->Block_Buff[ring_buff->tail]->leave_freq[k] = ring_buff->Block_Buff[ring_buff->tail]->norminal_freq[k];
                    block.entry_freq[k] = ring_buff->Block_Buff[ring_buff->tail]->norminal_freq[k];
                    //apply acceleration


                }else if (ring_buff->Block_Buff[ring_buff->tail]->norminal_freq[k]-block.norminal_freq[k]>JERK_FREQ)
                {
                    ring_buff->Block_Buff[ring_buff->tail]->leave_freq[k] = block.norminal_freq[k];
                    //apply deceleration
                }else
                {
                    //no acc or dcc
                    ring_buff->Block_Buff[ring_buff->tail]->leave_freq[k] = ring_buff->Block_Buff[ring_buff->tail]->norminal_freq[k];
                    ring_buff->Block_Buff[ring_buff->tail]->dcc_after = 100;
                    block.entry_freq[k] = block.norminal_freq[k];


                }
            }
        }
        Block_Buff_Write(&block, ring_buff);
    }
}

