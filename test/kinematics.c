#include "delta.h"
#include "stdio.h"

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