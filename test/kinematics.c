#include "stdio.h"
#include "config.h"

void Inverse_Kinematics(float xyz[], float abc[])
{
    abc[0] = sqrtf(SQ(R)-SQ(xyz[0]-x1)-SQ(xyz[1]-y1)+xyz[2]);
    abc[1] = sqrtf(SQ(R)-SQ(xyz[0]-x2)-SQ(xyz[1]-y2)+xyz[2]);
    abc[2] = sqrtf(SQ(R)-SQ(xyz[0]-x3)-SQ(xyz[1]-y3)+xyz[2]);
}

void Forward_Kinematics(float* abc, float* xyz)
{
    float vz32[3] = {x2-x3, y2-y3, abc[1]-abc[2]};

    float d = sqrtf(SQ(vz32[0])+SQ(vz32[1])+SQ(vz32[2]));

    float vX[3] = {vz32[0]*INV(d),vz32[1]*INV(d),vz32[2]*INV(d)};//x axis in new coordinate

    float vz31[3] = {x1 - x3,y1 - y3,abc[0] - abc[2]};

    float i = vz31[0]*vX[0] + vz31[1]*vX[1] + vz31[2]*vX[2];//dot product

    float vi[3] = {vX[0]*i,vX[1]*i,vX[2]*i};

    float vj[3] = {vz31[0]-vi[0],vz31[1]-vi[1],vz31[2]-vi[2]};

    float j = sqrtf(SQ(vj[0])+SQ(vj[1])+SQ(vj[2]));

    float vY[3] = {vj[0]*INV(j), vj[1]*INV(j), vj[2]*INV(j)};

    float vZ[3] = {vX[1]*vY[2]-vX[2]*vY[1],
                vX[2]*vY[0]-vX[0]*vY[2],
                vX[0]*vY[1]-vX[1]*vY[0]};//cross product
    
    float xe = d*0.5f;
    float ye = (SQ(xe))+SQ(xe-i)+SQ(j)*INV(2.0f*j);
    float ze = sqrtf(SQ(xe)+SQ(ye)+SQ(R));

    float vxe_N[3] = {xe*vX[0],xe*vX[1],xe*vX[2]};
    float vye_N[3] = {ye*vY[0],ye*vY[1],ye*vY[2]};
    float vze_N[3] = {ze*vZ[0],ze*vZ[1],ze*vZ[2]};

    float vze_O[3] = {vxe_N[0]+vye_N[0]+vze_N[0],
                    vxe_N[1]+vye_N[1]+vze_N[1],
                    vxe_N[2]+vye_N[2]+vze_N[2]};
    
    xyz[0] = x3+vze_O[0];
    xyz[1] = y3+vze_O[1];
    xyz[2] = abc[2]+vze_O[2];

}



int main ()
{
    float abc[3] = {360.0f,360.0f,360.0f};
    float xyz[3] = {0.0f, 0.0f, 0.0f};

    Forward_Kinematics(abc,xyz);

    for (int i=0;i<3;i++)
    {
        printf("%f",xyz[i]);
    }
    return 0;
}