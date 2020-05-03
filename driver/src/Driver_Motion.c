#include "Driver_Motion.h"
#include "Driver_Gcode.h"
#include "math.h"
#include "config.h"

//extern Position_End END_EFFECTOR;

void Motion_Home(Position_End *END_EFFECTOR, Position_Slider *SLIDER)
{
    BSP_Motor_Set_Dir(0|1|2,1);
    BSP_Motor_Set_Res(0);
    while(BSP_Switch_Read(0)
        &&BSP_Switch_Read(1)
        &&BSP_Switch_Read(2))
    {
        if (BSP_Switch_Read(0))
        {
            BSP_Motor_Set_Step(1);
            delay_ms(100);
            BSP_Motor_Reset_Step(1);
        }
        if (BSP_Switch_Read(1))
        {
            BSP_Motor_Set_Step(1);
            delay_ms(100);
            BSP_Motor_Reset_Step(1);
        }
        if (BSP_Switch_Read(2))
        {
            BSP_Motor_Set_Step(2);
            delay_ms(100);
            BSP_Motor_Reset_Step(2);
        }
    }
    END_EFFECTOR->x = 0;
    END_EFFECTOR->y = 0;
    END_EFFECTOR->y = 0;
    SLIDER->z1 = 0;
    SLIDER->z2 = 0;
    SLIDER->z3 = 0;
    
}

void Motion_Init(LDR_Type *LDR, LDR_Pre *LDR_IK)
{
    LDR_IK->x1 = (double)((LDR->R-LDR->r)*cos(LDR->n1));
    LDR_IK->x2 = (double)((LDR->R-LDR->r)*cos(LDR->n2));
    LDR_IK->x3 = (double)((LDR->R-LDR->r)*cos(LDR->n3));
    LDR_IK->y1 = (double)((LDR->R-LDR->r)*sin(LDR->n1));
    LDR_IK->y2 = (double)((LDR->R-LDR->r)*sin(LDR->n2));
    LDR_IK->y3 = (double)((LDR->R-LDR->r)*sin(LDR->n3));
}

void Motion_Linear(Position_End *END_EFFECTOR, Position_Slider *SLIDER, Motion_End *Linear)
{
    double time;
    uint32_t feedback;

    //下一个坐标
    Position_End Sub_End;
    Position_Slider Sub_Slider;

    while (Sub_End.x != END_EFFECTOR->x 
        && Sub_End.y != END_EFFECTOR->y
        && Sub_End.z != END_EFFECTOR->z)
    {
        Motion_Sub_Linear();
        Motion_IK(&Sub_End, &Sub_Slider, &LDR, &LDR_IK);
        Motion_Velocity_Linear_Cartesian();

    }

    

}

void Motion_Circular(Position_End *END_EFFECTOR, Position_Slider *SLIDER, Motion_End *Circular, uint32_t dir)
{
    double time;
    uint32_t feedback;

    Position_End Sub_End;
    Position_Slider Sub_Slider;

    Motion_IK(&Sub_End, &Sub_Slider, &LDR, &LDR_IK);
}


void Motion_IK(Position_End *end, Position_Slider *slider, LDR_Type *LDR, LDR_Pre *LDR_IK)
{
    slider->z1 = (int32_t)sqrt((double)(LDR->L*LDR->L)
                    - ((double)(end->x)-LDR_IK->x1)*((double)(end->x)-LDR_IK->x1)
                    - ((double)(end->y)-LDR_IK->y1)*((double)(end->y)-LDR_IK->y1)
                    + (double)(end->z));

    slider->z2 = (int32_t)sqrt((double)(LDR->L*LDR->L)
                    - ((double)(end->x)-LDR_IK->x2)*((double)(end->x)-LDR_IK->x2)
                    - ((double)(end->y)-LDR_IK->y2)*((double)(end->y)-LDR_IK->y2)
                    + (double)(end->z));

    slider->z3 = (int32_t)sqrt((double)(LDR->L*LDR->L)
                    - ((double)(end->x)-LDR_IK->x3)*((double)(end->x)-LDR_IK->x3)
                    - ((double)(end->y)-LDR_IK->y3)*((double)(end->y)-LDR_IK->y3)
                    + (double)(end->z));
}

void Motion_Velocity_Linear_Cartesian(Motion_End *endMotion, Velocity_End *endVelocity, Position_End *END_EFFECTOR)
{
    endVelocity->x = (double)END_EFFECTOR->x;
}

void Motion_Velocity_Jacobian(Velocity_End *endVelocity
                            , Velocity_Slider *sliderVelocity
                            , LDR_Pre *LDR_IK
                            , Position_End *endPosition
                            , Position_Slider *sliderPosition)
{
    sliderVelocity->z1 = ((double)(endPosition->x) - LDR_IK->x1) * endVelocity->x
                        +((double)(endPosition->y) - LDR_IK->y1) * endVelocity->y
                        +((double)(endPosition->z) - (double)(sliderPosition->z1)) * endVelocity->z;

    sliderVelocity->z2 = ((double)(endPosition->x) - LDR_IK->x2) * endVelocity->x
                        +((double)(endPosition->y) - LDR_IK->y2) * endVelocity->y
                        +((double)(endPosition->z) - (double)(sliderPosition->z2)) * endVelocity->z;

    sliderVelocity->z3 = ((double)(endPosition->x) - LDR_IK->x3) * endVelocity->x
                        +((double)(endPosition->y) - LDR_IK->y3) * endVelocity->y
                        +((double)(endPosition->z) - (double)(sliderPosition->z3)) * endVelocity->z;
}

void Motion_Sub_Linear(Position_End *END_EFFECTOR
                        ,Position_End *endSubMove
                        ,uint32_t Res)
{

}