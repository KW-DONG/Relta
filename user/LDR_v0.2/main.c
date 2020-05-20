#include "sys.h"
#include "delay.h"
#include "config.h"
#include "motion.h"
#include "type.h"
#include "buffer.h"
#include "gcode.h"
#include "planner.h"
#include "bsp.h"
#include "stm32f4xx.h"
#include "delta.h"

/******************************Hardware******************************/
stepper_t   stepperA;
stepper_t   stepperB;
stepper_t   stepperC;
switch_t    switchA;
switch_t    switchB;
switch_t    switchC;
switch_t    stop_start_key;
switch_t    reset_key;
led_t       led_red;
led_t       led_green;
machine_t   machine;
monitor_t   monitor;

//buffer
block_buff_t block_buff;
uart_buff_t  uart_buff;
command_t    command_c;

//block
block_t block_c;

//observation variable
volatile int32_t stepper_A_freq;
volatile int32_t stepper_B_freq;
volatile int32_t stepper_C_freq;


int main()
{
    float XYZ_C[3];
    float XYZ_Home[3] = {0.0f,0.0f,50.0f};
    float XYZ_R[3] = {0.0f,0.0f,300.0f};
    static float dwell;

    delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

/****************************STEPPER_MOTOR*******************************/
    //stepper_A init
    stepperA.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOB;
    stepperA.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM2;
    stepperA.GPIO_Pin_X = GPIO_Pin_11;
    stepperA.GPIO_PinSourceX = GPIO_PinSource11;
    stepperA.GPIOX = GPIOB;
    stepperA.TIMX = TIM2;
    stepperA.PWM_Ch = 4;
    stepperA.arr = TIM_ARR;
    stepperA.RCC_AHB1Periph_GPIOX_Dir = RCC_AHB1Periph_GPIOA;
    stepperA.GPIO_Pin_X_Dir = GPIO_Pin_15;
    stepperA.GPIOX_Dir = GPIOA;
    Bsp_Stepper_Init(&stepperA);

    //stepper_B init
    stepperB.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOC;
    stepperB.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM3;
    stepperB.GPIO_Pin_X = GPIO_Pin_6;
    stepperB.GPIO_PinSourceX = GPIO_PinSource6;
    stepperB.GPIOX = GPIOC;
    stepperB.TIMX = TIM3;
    stepperB.PWM_Ch = 1;
    stepperB.arr = TIM_ARR;
    stepperB.RCC_AHB1Periph_GPIOX_Dir = RCC_AHB1Periph_GPIOC;
    stepperB.GPIO_Pin_X_Dir = GPIO_Pin_7;
    stepperB.GPIOX_Dir = GPIOC;
    Bsp_Stepper_Init(&stepperB);

    //stepper_C init
    stepperC.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOB;
    stepperC.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM4;
    stepperC.GPIO_Pin_X = GPIO_Pin_6;
    stepperC.GPIO_PinSourceX = GPIO_PinSource6;
    stepperC.GPIOX = GPIOB;
    stepperC.TIMX = TIM4;
    stepperC.PWM_Ch = 1;
    stepperC.arr = TIM_ARR;
    stepperC.RCC_AHB1Periph_GPIOX_Dir = RCC_AHB1Periph_GPIOA;
    stepperC.GPIO_Pin_X_Dir = GPIO_Pin_6;
    stepperC.GPIOX_Dir = GPIOA;
    Bsp_Stepper_Init(&stepperC);

/*******************************SWITCH_KEY*******************************/
    //stop_start_key
    stop_start_key.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOC;
    stop_start_key.GPIOX = GPIOC;
    stop_start_key.GPIO_PinX = GPIO_Pin_0;
    stop_start_key.EXTI_LineX = EXTI_Line0;
    stop_start_key.EXTI_PinSourceX = EXTI_PinSource0;
    stop_start_key.EXTI_PortSourceGPIOX = EXTI_PortSourceGPIOC;
    stop_start_key.EXTIX_IRQn = EXTI0_IRQn;
    stop_start_key.mode = NC;
    stop_start_key.NVIC_PP = 1;
    stop_start_key.NVIC_SP = 1;
    Bsp_Switch_Init(&stop_start_key);

    //reset key
    reset_key.EXTI_LineX = EXTI_Line1;
    reset_key.EXTI_PinSourceX = EXTI_PinSource1;
    reset_key.EXTI_PortSourceGPIOX = EXTI_PortSourceGPIOC;
    reset_key.EXTIX_IRQn = EXTI1_IRQn;
    reset_key.GPIO_PinX = GPIO_Pin_1;
    reset_key.GPIOX = GPIOC;
    reset_key.mode = NC;
    reset_key.NVIC_PP = 1;
    reset_key.NVIC_SP = 1;
    reset_key.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOC;
    Bsp_Switch_Init(&reset_key);

    //switch A
    switchA.EXTI_LineX = EXTI_Line2;
    switchA.EXTI_PinSourceX = EXTI_PinSource2;
    switchA.EXTI_PortSourceGPIOX = EXTI_PortSourceGPIOC;
    switchA.EXTIX_IRQn = EXTI2_IRQn;
    switchA.GPIO_PinX = GPIO_Pin_2;
    switchA.GPIOX = GPIOC;
    switchA.mode = NC;
    switchA.NVIC_PP = 1;
    switchA.NVIC_SP = 1;
    switchA.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOC;
    Bsp_Switch_Init(&switchA);

    //switch B
    switchB.EXTI_LineX = EXTI_Line3;
    switchB.EXTI_PinSourceX = EXTI_PinSource3;
    switchB.EXTI_PortSourceGPIOX = EXTI_PortSourceGPIOC;
    switchB.EXTIX_IRQn = EXTI3_IRQn;
    switchB.GPIO_PinX = GPIO_Pin_3;
    switchB.GPIOX = GPIOC;
    switchB.mode = NC;
    switchB.NVIC_PP = 1;
    switchB.NVIC_SP = 1;
    switchB.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOC;
    Bsp_Switch_Init(&switchB);

    //switch C
    switchC.EXTI_LineX = EXTI_Line4;
    switchC.EXTI_PinSourceX = EXTI_PinSource4;
    switchC.EXTI_PortSourceGPIOX = EXTI_PortSourceGPIOC;
    switchC.EXTIX_IRQn = EXTI3_IRQn;
    switchC.GPIO_PinX = GPIO_Pin_3;
    switchC.GPIOX = GPIOC;
    switchC.mode = NC;
    switchC.NVIC_PP = 1;
    switchC.NVIC_SP = 1;
    switchC.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOC;
    Bsp_Switch_Init(&switchC);

/*********************************LED*********************************/
    led_red.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOF;
    led_red.GPIOX = GPIOF;
    led_red.GPIO_Pin_X = GPIO_Pin_9;
    Bsp_LED_Init(&led_red);

    led_green.RCC_AHB1Periph_GPIOX = RCC_AHB1Periph_GPIOF;
    led_green.GPIOX = GPIOF;
    led_green.GPIO_Pin_X = GPIO_Pin_10;
    Bsp_LED_Init(&led_green);

/*********************************MONITOR*****************************/
    monitor.arr = TIM_ARR;
    monitor.psc = MONITOR_PSC;
    monitor.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM5;
    monitor.TIMX = TIM5;
    monitor.TIMX_IRQn = TIM5_IRQn;
    Bsp_Monitor_Init(&monitor);
    
    Block_Buff_Init(&block_buff);

    //hardware init
    Bsp_UART_Init(115200);


    while (1)
    {
        //task 1: interprete g_code
        if (machine.interpret_flag == SET)
        {
            Gcode_Interpret(&command_c,&uart_buff);
            machine.interpret_flag = RESET;
            machine.traj_flag = SET;
        }

        //task 2: generate trajectory
        if (machine.traj_flag == SET)
        {
            if (command_c.type == home_t)
            {
                Linear_Planner(XYZ_Home,XYZ_C,10.0f,10.0f, &block_buff);
                Linear_Planner(XYZ_R,XYZ_Home,10.0f,10.0f, &block_buff);
            }else if (command_c.type == linear_t)
            {
                Linear_Planner(command_c.xyz, XYZ_C, command_c.feedrate, dwell, &block_buff);
                dwell = 0.0f;
            }else if (command_c.type == arc_t)
            {
                if (command_c.xyz[2]!=XYZ_C[2])
                {
                    float xyz_arc[3] = {XYZ_C[0],XYZ_C[1], command_c.xyz[2]};
                    Linear_Planner(xyz_arc, XYZ_C, 10.0f,dwell,&block_buff);
                    Arc_Planner(command_c.xyz,xyz_arc, command_c.radius_dwell,command_c.feedrate, 0.0f, &block_buff);
                }else
                {
                    Arc_Planner(command_c.xyz, XYZ_C, command_c.radius_dwell, command_c.feedrate, dwell, &block_buff);
                }
                dwell = 0.0f;
            }else if (command_c.type == dwell_t)
            {
                dwell = command_c.radius_dwell;
            }
            XYZ_C[0] = command_c.xyz[0];
            XYZ_C[1] = command_c.xyz[1];
            XYZ_C[2] = command_c.xyz[2];
            machine.traj_flag = RESET;
        }
        //task 3: apply FK and report feedback
        if (machine.carriage_move[0]!=0)    machine.abc[0] = machine.abc[0] + machine.carriage_move[0]*INV(STEPS_PER_UNIT);
        if (machine.carriage_move[1]!=0)    machine.abc[1] = machine.abc[1] + machine.carriage_move[1]*INV(STEPS_PER_UNIT);
        if (machine.carriage_move[2]!=0)    machine.abc[2] = machine.abc[2] + machine.carriage_move[2]*INV(STEPS_PER_UNIT);
        Forward_Kinematics(machine.abc, machine.xyz);
    }
}
void USART1_IRQHandler(void)
{
    uint8_t res;
    if (USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        res = USART_ReceiveData(USART1);
        Uart_Buff_Write(&uart_buff,res);
        if (res==13)    machine.interpret_flag = SET;
    }
}

//monitor planner
void TIM5_IRQHandler()
{
    if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)
	{
        static uint8_t block_state;
        
        Motion_Check(&machine, &stepperA, &stepperB, &stepperC);

        if (block_c.step[0]==0) stepperA.state=STOP;
        if (block_c.step[1]==0) stepperB.state=STOP;
        if (block_c.step[2]==0) stepperC.state=STOP;

        if (machine.state==machine_ON)
        {
            //check whether the current block is executing
            if (block_c.step[0]==0&&block_c.step[1]==0&&block_c.step[2]==0)
            {
                machine.fk_flag = SET;
                block_state = Block_Buff_Read(&block_c, &block_buff);
                if(block_state==TRUE)
                {
                    //always maximum acceleration
                    stepperA.freq = block_c.maximum_freq[0];
                    stepperB.freq = block_c.maximum_freq[1];
                    stepperC.freq = block_c.maximum_freq[2];

                    stepperA.state = START;
                    stepperB.state = START;
                    stepperC.state = START;
                }
            }
            else
            {
                if(block_c.step_dwell!=0)
                Dwell_Step_Update(&block_c);
                else
                {
                    Stepper_Count(&block_c, &machine, &stepperA, &stepperB, &stepperC);
                    Acceleration_Count(&stepperA, &stepperB, &stepperC, &block_c);
                }
            }
        }
        Bsp_Stepper_Update(&stepperA);
        Bsp_Stepper_Update(&stepperB);
        Bsp_Stepper_Update(&stepperC);
	}
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
}

//switch A
void EXTI2_IRQHandler(void)
{
	delay_ms(10);
    machine.abc[0] = CARRIAGE_A_RESET;
    if (stepperA.dir==1)
    {
        block_c.step[0] = 0;
        stepperA.dir = stepper_DOWN;
    }
    EXTI_ClearITPendingBit(EXTI_Line0);
}

//switch B
void EXTI3_IRQHandler(void)
{
    delay_ms(10);
    machine.abc[1] = CARRIAGE_B_RESET;
    if (stepperB.dir==1)
    {
        block_c.step[1] = 0;
        stepperB.dir = stepper_DOWN;
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
}

//switch C
void EXTI4_IRQHandler(void)
{
    delay_ms(10);
    machine.abc[2] = CARRIAGE_C_RESET;
    if (stepperC.dir==1)
    {
        block_c.step[2] = 0;
        stepperC.dir = stepper_DOWN;
    }
    EXTI_ClearITPendingBit(EXTI_Line2);
}

//switch R
void EXTI1_IRQHandler(void)
{
    delay_ms(10);
	machine.state = machine_OFF;
    machine.xyz_v[0] = 0.0f;
    machine.xyz_v[1] = 0.0f;
    machine.xyz_v[2] = 0.0f;
    stepperA.state = STOP;
    stepperB.state = STOP;
    stepperC.state = STOP;

    Block_Buff_Clear(&block_buff);
    Block_Buff_Init(&block_buff);
        
	delay_ms(100);
		 
	EXTI_ClearITPendingBit(EXTI_Line3);
}

//switch S
void EXTI0_IRQHandler(void)
{
    delay_ms(10);
	if (machine.state != machine_ON) machine.state = machine_ON;
    else machine.state = machine_OFF;
	delay_ms(100);
		 
	EXTI_ClearITPendingBit(EXTI_Line4);
}










