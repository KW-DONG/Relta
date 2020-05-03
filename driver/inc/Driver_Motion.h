#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "sys.h"
#include "Driver_Motor.h"
#include "stm32f4xx_gpio.h"
#include "config.h"

#define cartesian_res 1 //1 mm resolution
#define clockwise 0
#define anticlockwise 1


//motion
typedef struct
{
    //final coordinate
    int32_t x;
    int32_t y;
    int32_t z;

    //feedrate (velocity)
    int32_t f;

    //pivot coordinate
    int32_t i;
    uint32_t j;
} Motion_End;

//position
typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} Position_End;

typedef struct
{
    int32_t z1;
    int32_t z2;
    int32_t z3;
} Position_Slider;

//Robot parameter
typedef struct
{
    uint32_t L;
    uint32_t R;
    uint32_t r;
    double n1;
    double n2;
    double n3;
}LDR_Type;

typedef struct 
{
    double x1;
    double x2;
    double x3;
    double y1;
    double y2;
    double y3;
}LDR_Pre;


typedef struct 
{
    double x;
    double y;
    double z;
}Velocity_End;

typedef struct 
{
    double z1;
    double z2;
    double z3;
}Velocity_Slider;

void Motion_Init(LDR_Type *LDR, LDR_Pre *LDR_IK);

/**
 * @brief create sub-moves
 * @param Motion_End cartesian coordinate
 */
void Motion_Home(Position_End *END_EFFECTOR, Position_Slider *SLIDER);

void Motion_Linear(Position_End *END_EFFECTOR, Position_Slider *SLIDER, Motion_End *Linear);
void Motion_Circular(Position_End *END_EFFECTOR, Position_Slider *SLIDER, Motion_End *Circular, uint32_t dir);


/**
 * @brief transfer cartesian coordinate to slider position
 * @param Motion_End cartesian coordinate
 * @param Motion_Slider slider position
 */
void Motion_IK(Position_End *end, Position_Slider *slider, LDR_Type *ldr, LDR_Pre *LDR_IK);

//算出下一个中间坐标点
void Motion_Sub_Linear(Position_End *END_EFFECTOR
                        ,Position_End *endSubMove
                        ,uint32_t Res);

void Motion_Sub_Circular();

//把线速度换算成xyz坐标分量
void Motion_Velocity_Linear_Cartesian();
void Motion_Velocity_Circular_Cartesian();






/**
 * @brief map the velocity between end effector and sliders
 * @param endVelocity velocity of a sub move in cartesian
 * @param sliderVelocity output
 * @param LDR_IK x1 x2 x3 y1 y2 y3
 * @param endPosition a coordinate that the end effector will achieve
 * @param sliderPosition the position that the slider will achieve
 */
void Motion_Velocity_Jacobian(Velocity_End *endVelocity
                            , Velocity_Slider *sliderVelocity
                            , LDR_Pre *LDR_IK
                            , Position_End *endPosition
                            , Position_Slider *sliderPosition)

#endif