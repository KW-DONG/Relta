#include "sys.h"
#include "delay.h"
#include "led.h"
#include "config.h"
#include "planner.h"
#include "switch.h"
#include "motion.h"
#include "type.h"
#include "buffer.h"
#include "gcode.h"
#include "stepper.h"

/******************************Hardware******************************/
stepper_t   stepperA;
stepper_t   stepperB;
stepper_t   stepperC;
switch_t    switchA;
switch_t    switchB;
switch_t    switchC;
switch_t    stop_start_key;
switch_t    reset_key;
machine_t   machine;

//buffer
linked_list_t gcode_list;
ring_buff_t block_list;

//block
stepper_exe_t block_current;

float XYZ_C[3];
float XYZ_T[3];
float XYZ_Arc[3];
float XYZ_Home[3];
float ABC_C[3];
float ABC_N[3];
float DWELL;

int main()
{
    delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delta_Init();

/****************************STEPPER_MOTOR*******************************/
    //stepper_A init
    stepperA.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOB;
    stepperA.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM2;
    stepperA.GPIO_Pin_X = GPIO_Pin_11;
    stepperA.GPIO_PinSourceX = GPIO_PinSource11;
    stepperA.GPIOX = GPIOA;
    stepperA.TIMX = TIM5;

    stepperA.RCC_AHB1Periph_GPIOX_Dir = RCC_AHB1Periph_GPIOB;
    stepperA.GPIO_Pin_X_Dir = GPIO_Pin_3;
    stepperA.GPIOX_Dir = GPIOB;
    Stepper_Init(&stepperA);

    //stepper_B init
    stepperB.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOC;
    stepperB.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM2;
    stepperB.GPIO_Pin_X = GPIO_Pin_11;
    stepperB.GPIO_PinSourceX = GPIO_PinSource11;
    stepperB.GPIOX = GPIOC;
    stepperB.TIMX = TIM6;

    stepperB.RCC_AHB1Periph_GPIOX_Dir = RCC_AHB1Periph_GPIOC;
    stepperB.GPIO_Pin_X_Dir = GPIO_Pin_7;
    stepperB.GPIOX_Dir = GPIOC;
    Stepper_Init(&stepperB);

    //stepper_C init
    stepperC.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOC;
    stepperC.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM2;
    stepperC.GPIO_Pin_X = GPIO_Pin_11;
    stepperC.GPIO_PinSourceX = GPIO_PinSource11;
    stepperC.GPIOX = GPIOC;
    stepperC.TIMX = TIM6;

    stepperC.RCC_AHB1Periph_GPIOX_Dir = RCC_AHB1Periph_GPIOC;
    stepperC.GPIO_Pin_X_Dir = GPIO_Pin_7;
    stepperC.GPIOX_Dir = GPIOC;
    Stepper_Init(&stepperC);

/*******************************SWITCH_KEY*******************************/
    //stop_start_key
    stop_start_key.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOA;
    stop_start_key.GPIOX = GPIOA;
    stop_start_key.GPIO_PinX = GPIO_Pin_0;
    Switch_Init(&stop_start_key);

    //reset key
    stop_start_key.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOE;
    stop_start_key.GPIOX = GPIOE;
    stop_start_key.GPIO_PinX = GPIO_Pin_4;
    Switch_Init(&stop_start_key);
    Switch_EXTI_Init();

    //switch A
    switchA.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOE;
    switchA.GPIOX = GPIOE;
    switchA.GPIO_PinX = GPIO_Pin_4;
    Switch_Init(&switchA);

    //switch B
    switchB.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOE;
    switchB.GPIOX = GPIOE;
    switchB.GPIO_PinX = GPIO_Pin_4;
    Switch_Init(&switchB);

    //switch C
    switchC.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOE;
    switchC.GPIOX = GPIOE;
    switchC.GPIO_PinX = GPIO_Pin_4;
    Switch_Init(&switchC);


    Gcode_Buff_Init(&gcode_list);
    Ring_Buff_Init(&stepper_list);

    //hardware init
    LED_Init();
    Uart_Init(115200);
    

    XYZ_Home[0] = 0.0f;
    XYZ_Home[1] = 0.0f;
    XYZ_Home[2] = 50.0f;

    gcode_node_t temp_node;
    uint32_t len;

    while (1)
    {
        #if EXECUTE_MACHINE
        #if USE_GCODE_COMMAND
        while(gcode_list.length != 0&&block_list.length<100)
        {
            Gcode_Buff_Read(&gcode_list, &temp_node);
            XYZ_T[0] = temp_node.x;
            XYZ_T[1] = temp_node.y;
            XYZ_T[2] = temp_node.z;

            if (temp_node.type == home_t)
            {
                Linear_Motion(XYZ_Home,XYZ_C,10.0,10.0);
                Auto_Home();
            }
            if (temp_node.type == linear_t)
            {
                Linear_Motion(XYZ_T,XYZ_C,temp_node.feedrate,DWELL);
            }
            if (temp_node.type == arc_t)
            {
                //arc
                if (XYZ_T[2]!=XYZ_C[2])
                {
                    XYZ_Arc[0] = XYZ_C[0];
                    XYZ_Arc[1] = XYZ_C[1];
                    XYZ_Arc[2] = XYZ_T[2];
                    Linear_Motion(XYZ_Arc,XYZ_C,temp_node.feedrate,DWELL);
                    Arc_Motion(XYZ_T,XYZ_Arc,temp_node.radius_dwell,temp_node.feedrate,0.0);
                }else
                {
                    Arc_Motion(XYZ_T,XYZ_C,temp_node.radius_dwell,temp_node.feedrate,DWELL);
                }
                
            }
            if (temp_node.type == dwell_t)
            {
                //dwell
                DWELL = temp_node.radius_dwell;
            }
            XYZ_C[0] = XYZ_T[0];
            XYZ_C[1] = XYZ_T[1];
            XYZ_C[2] = XYZ_T[2];
        }
        #endif
        #endif
    }
    
}


