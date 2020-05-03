#include "Driver_Motor.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"

uint32_t Motor_Control(double time, Position_Slider *SLIDER, Position_Slider *Sub_Slider, Motor_Execute *EXECUTION)
{

    int32_t moves1;
    int32_t moves2;
    int32_t moves3;
    uint32_t n;

    //maximum steps devided by total steps 几格一跳, 二次切片
    if ((int)(Sub_Slider->z1 - SLIDER->z1) == 0) moves1 = 32767;
    else moves1 = (int)(time * freq) / ((int)(Sub_Slider->z1 - SLIDER->z1) * slider_unit);

    if ((int)(Sub_Slider->z2 - SLIDER->z2) == 0) moves2 = 32767;
    else moves2 = (int)(time * freq) / ((int)(Sub_Slider->z2 - SLIDER->z2) * slider_unit);
    
    if ((int)(Sub_Slider->z3 - SLIDER->z3) == 0) moves3 = 32767;
    else moves3 = (int)(time * freq) / ((int)(Sub_Slider->z3 - SLIDER->z3) * slider_unit);

    while (n<(int)(time * freq))
    {
        EXECUTION->Move_Status = 1;
        if (n%moves1 == 0)
        {
            if ((Sub_Slider->z1 - SLIDER->z1)>0)EXECUTION->Move_1 = 1;
            else EXECUTION->Move_1 = 2;
        }else{
            EXECUTION->Move_1 = 0;
        }
        if (n%moves2 == 0)
        {
            if ((Sub_Slider->z2 - SLIDER->z2)>0)EXECUTION->Move_2 = 1;
            else EXECUTION->Move_2 = 2;
        }else{
            EXECUTION->Move_2;
        }
        if (n%moves3 == 0)
        {
            if ((Sub_Slider->z3 - SLIDER->z3)>0)EXECUTION->Move_3 = 1;
            else EXECUTION->Move_3 = 2;
        }else{
            EXECUTION->Move_3 = 0;
        }
        while(EXECUTION->Move_Status == 0)
        n++;
    }
    return 0;// finished
}


void Motor_Set(Motor_Execute *EXECUTION)
{
    switch(EXECUTION->Move_1)
    {
        case 0: BSP_Motor_Reset_Step(motor_X);
        case 1: 
        {
            BSP_Motor_Set_Dir(motor_X,clockwise);
            BSP_Motor_Set_Step(motor_X);
            delay_ms(100);
            BSP_Motor_Reset_Step(motor_X);
        }
        case 2:
        {
            BSP_Motor_Set_Dir(motor_X,anticlockwise);
            BSP_Motor_Set_Step(motor_X);
            delay_ms(100);
            BSP_Motor_Reset_Step(motor_X);
        }
    }
        switch(EXECUTION->Move_2)
    {
        case 0: BSP_Motor_Reset_Step(motor_Y);
        case 1: 
        {
            BSP_Motor_Set_Dir(motor_Y,clockwise);
            BSP_Motor_Set_Step(motor_Y);
            delay_ms(100);
            BSP_Motor_Reset_Step(motor_Y);
        }
        case 2:
        {
            BSP_Motor_Set_Dir(motor_Y,anticlockwise);
            BSP_Motor_Set_Step(motor_Y);
            delay_ms(100);
            BSP_Motor_Reset_Step(motor_Y);
        }
    }
        switch(EXECUTION->Move_3)
    {
        case 0: BSP_Motor_Reset_Step(motor_Z);
        case 1: 
        {
            BSP_Motor_Set_Dir(motor_Z,clockwise);
            BSP_Motor_Set_Step(motor_Z);
            delay_ms(100);
            BSP_Motor_Reset_Step(motor_Z);
        }
        case 2:
        {
            BSP_Motor_Set_Dir(motor_Z,anticlockwise);
            BSP_Motor_Set_Step(motor_Z);
            delay_ms(100);
            BSP_Motor_Reset_Step(motor_Z);
        }
    }
    EXECUTION->Move_1 = 0;
    EXECUTION->Move_2 = 0;
    EXECUTION->Move_3 = 0;
    EXECUTION->Move_Status = 0;
}