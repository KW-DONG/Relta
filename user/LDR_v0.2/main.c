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
block_t      block_c;

uint8_t pulse_A;
uint8_t pulse_B;
uint8_t pulse_C;


void Test_Path(void)
{
    float xyz_c[3] = {0,0,200};
    float xyz_t[3] = {0,0,0};
    Linear_Planner(xyz_t,xyz_c,10.0f,20.0f,&block_buff);
    delay_ms(100);
}

void Test_Block(void)
{
    block_t new_block;
    new_block.flag = block_ready;
    new_block.step_dwell = 0;

    for (uint8_t i=0; i<3;i++)
    {
        new_block.step[i] = 200;
        new_block.accelerate_freq[i] = 0;
        new_block.accelerate_until[i] = 0;
        new_block.decelerate_freq[i] = 0;
        new_block.decelerate_after[i] = new_block.step[i];
        new_block.dir[i] = carriage_DOWN;
    }
    Block_Buff_Write(&new_block, &block_buff);
}


int main()
{

/*******************************BUFFER********************************/
    Block_Buff_Init(&block_buff);
    Bsp_UART_Init(115200);
    delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	
	
/****************************STEPPER_MOTOR*******************************/
    //stepper_A init
    //set
    stepperA.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOB;
    stepperA.GPIO_Pin_X_Set = GPIO_Pin_11;
    stepperA.GPIOX_Set = GPIOB;
    //dir
    stepperA.RCC_AHB1Periph_GPIOX_Dir = RCC_AHB1Periph_GPIOA;
    stepperA.GPIO_Pin_X_Dir = GPIO_Pin_15;
    stepperA.GPIOX_Dir = GPIOA;
    //ms1
    stepperA.RCC_AHB1Periph_GPIOX_MS1 = RCC_AHB1Periph_GPIOF;
    stepperA.GPIO_Pin_X_MS1 = GPIO_Pin_6;
    stepperA.GPIOX_MS1 = GPIOF;
    //ms2
    stepperA.RCC_AHB1Periph_GPIOX_MS2 = RCC_AHB1Periph_GPIOF;
    stepperA.GPIO_Pin_X_MS2 = GPIO_Pin_7;
    stepperA.GPIOX_MS2 = GPIOF;
    //ms3
    stepperA.RCC_AHB1Periph_GPIOX_MS3 = RCC_AHB1Periph_GPIOF;
    stepperA.GPIO_Pin_X_MS3 = GPIO_Pin_8;
    stepperA.GPIOX_MS3 = GPIOF;
    //pwm_cnt
    stepperA.RCC_AHB1Periph_GPIOX_PWM = RCC_AHB1Periph_GPIOD;
    stepperA.GPIO_Pin_X_PWM = GPIO_Pin_2;
    stepperA.GPIOX_PWM = GPIOD;
    //TIM
    stepperA.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM2;
    stepperA.GPIO_PinSourceX = GPIO_PinSource11;
    stepperA.TIMX = TIM2;
    stepperA.PWM_Ch = 4;
    stepperA.arr = TIM_ARR;
    stepperA.GPIO_AF_TIMX = GPIO_AF_TIM2;
    //other
	stepperA.id = 0;
    stepperA.freq = 10;
    Bsp_Stepper_Init(&stepperA);

    //stepper_B init
    //set
    stepperB.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOC;
    stepperB.GPIO_Pin_X_Set = GPIO_Pin_6;
    stepperB.GPIOX_Set = GPIOC;
    //dir
    stepperB.RCC_AHB1Periph_GPIOX_Dir = RCC_AHB1Periph_GPIOC;
    stepperB.GPIO_Pin_X_Dir = GPIO_Pin_7;
    stepperB.GPIOX_Dir = GPIOC;
    //ms1
    stepperB.RCC_AHB1Periph_GPIOX_MS1 = RCC_AHB1Periph_GPIOF;
    stepperB.GPIO_Pin_X_MS1 = GPIO_Pin_6;
    stepperB.GPIOX_MS1 = GPIOF;
    //ms2
    stepperB.RCC_AHB1Periph_GPIOX_MS2 = RCC_AHB1Periph_GPIOF;
    stepperB.GPIO_Pin_X_MS2 = GPIO_Pin_7;
    stepperB.GPIOX_MS2 = GPIOF;
    //ms3
    stepperB.RCC_AHB1Periph_GPIOX_MS3 = RCC_AHB1Periph_GPIOF;
    stepperB.GPIO_Pin_X_MS3 = GPIO_Pin_8;
    stepperB.GPIOX_MS3 = GPIOF;
    //pwm_cnt
    stepperB.RCC_AHB1Periph_GPIOX_PWM = RCC_AHB1Periph_GPIOC;
    stepperB.GPIO_Pin_X_PWM = GPIO_Pin_11;
    stepperB.GPIOX_PWM = GPIOC;
    //TIM
    stepperB.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM3;
    stepperB.GPIO_PinSourceX = GPIO_PinSource6;
    stepperB.TIMX = TIM3;
    stepperB.PWM_Ch = 1;
    stepperB.arr = TIM_ARR;
    stepperB.GPIO_AF_TIMX = GPIO_AF_TIM3;
    //other
	stepperB.id = 1;
    stepperB.freq = 10;
    Bsp_Stepper_Init(&stepperB);

    //stepper_C init
    //set
    stepperC.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOB;
    stepperC.GPIO_Pin_X_Set = GPIO_Pin_6;
    stepperC.GPIOX_Set = GPIOB;
    //dir
    stepperC.RCC_AHB1Periph_GPIOX_Dir = RCC_AHB1Periph_GPIOA;
    stepperC.GPIO_Pin_X_Dir = GPIO_Pin_6;
    stepperC.GPIOX_Dir = GPIOA;
    //ms1
    stepperC.RCC_AHB1Periph_GPIOX_MS1 = RCC_AHB1Periph_GPIOF;
    stepperC.GPIO_Pin_X_MS1 = GPIO_Pin_6;
    stepperC.GPIOX_MS1 = GPIOF;
    //ms2
    stepperC.RCC_AHB1Periph_GPIOX_MS2 = RCC_AHB1Periph_GPIOF;
    stepperC.GPIO_Pin_X_MS2 = GPIO_Pin_7;
    stepperC.GPIOX_MS2 = GPIOF;
    //ms3
    stepperC.RCC_AHB1Periph_GPIOX_MS3 = RCC_AHB1Periph_GPIOF;
    stepperC.GPIO_Pin_X_MS3 = GPIO_Pin_8;
    stepperC.GPIOX_MS3 = GPIOF;
    //pwm_cnt
    stepperC.RCC_AHB1Periph_GPIOX_PWM = RCC_AHB1Periph_GPIOC;
    stepperC.GPIO_Pin_X_PWM = GPIO_Pin_10;
    stepperC.GPIOX_PWM = GPIOC;
    //TIM
    stepperC.RCC_APB1Periph_TIMX = RCC_APB1Periph_TIM4;
    stepperC.GPIO_PinSourceX = GPIO_PinSource6;
    stepperC.TIMX = TIM4;
    stepperC.PWM_Ch = 1;
    stepperC.arr = TIM_ARR;
    stepperC.GPIO_AF_TIMX = GPIO_AF_TIM4;
    //other
	stepperC.id = 2;
    stepperC.freq = 10;
    Bsp_Stepper_Init(&stepperC);

#if USE_SWITCH
/*******************************SWITCH_KEY*******************************/
    //stop_start_key
    stop_start_key.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOC;
    stop_start_key.GPIOX_Set = GPIOC;
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
    reset_key.GPIOX_Set = GPIOC;
    reset_key.mode = NC;
    reset_key.NVIC_PP = 1;
    reset_key.NVIC_SP = 1;
    reset_key.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOC;
    Bsp_Switch_Init(&reset_key);

    //switch A
    switchA.EXTI_LineX = EXTI_Line2;
    switchA.EXTI_PinSourceX = EXTI_PinSource2;
    switchA.EXTI_PortSourceGPIOX = EXTI_PortSourceGPIOC;
    switchA.EXTIX_IRQn = EXTI2_IRQn;
    switchA.GPIO_PinX = GPIO_Pin_2;
    switchA.GPIOX_Set = GPIOC;
    switchA.mode = NC;
    switchA.NVIC_PP = 2;
    switchA.NVIC_SP = 1;
    switchA.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOC;
    Bsp_Switch_Init(&switchA);

    //switch B
    switchB.EXTI_LineX = EXTI_Line3;
    switchB.EXTI_PinSourceX = EXTI_PinSource3;
    switchB.EXTI_PortSourceGPIOX = EXTI_PortSourceGPIOC;
    switchB.EXTIX_IRQn = EXTI3_IRQn;
    switchB.GPIO_PinX = GPIO_Pin_3;
    switchB.GPIOX_Set = GPIOC;
    switchB.mode = NC;
    switchB.NVIC_PP = 2;
    switchB.NVIC_SP = 1;
    switchB.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOC;
    Bsp_Switch_Init(&switchB);

    //switch C
    switchC.EXTI_LineX = EXTI_Line4;
    switchC.EXTI_PinSourceX = EXTI_PinSource4;
    switchC.EXTI_PortSourceGPIOX = EXTI_PortSourceGPIOC;
    switchC.EXTIX_IRQn = EXTI3_IRQn;
    switchC.GPIO_PinX = GPIO_Pin_3;
    switchC.GPIOX_Set = GPIOC;
    switchC.mode = NC;
    switchC.NVIC_PP = 2;
    switchC.NVIC_SP = 1;
    switchC.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOC;
    Bsp_Switch_Init(&switchC);
#endif
/*********************************LED*********************************/
    led_red.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOF;
    led_red.GPIOX_Set = GPIOF;
    led_red.GPIO_Pin_X_Set = GPIO_Pin_9;
    Bsp_LED_Init(&led_red);

    led_green.RCC_AHB1Periph_GPIOX_Set = RCC_AHB1Periph_GPIOF;
    led_green.GPIOX_Set = GPIOF;
    led_green.GPIO_Pin_X_Set = GPIO_Pin_10;
    Bsp_LED_Init(&led_green);

    Bsp_Monitor_Init();

    float XYZ_C[3];
    float XYZ_Home[3] = {0.0f,0.0f,50.0f};
    float XYZ_R[3] = {0.0f,0.0f,300.0f};
    static float dwell;

    machine.state = machine_ON;
    machine.interpret_flag = RESET;
    machine.fk_flag = RESET;
    machine.traj_flag = RESET;

    //create coordinate system
    machine.abc[0] = CARRIAGE_A_RESET;
    machine.abc[1] = CARRIAGE_B_RESET;
    machine.abc[2] = CARRIAGE_C_RESET;

    Forward_Kinematics(machine.abc, machine.xyz);

    stepperA.state = stepper_ON;
    stepperB.state = stepper_ON;
    stepperC.state = stepper_ON;

    for (uint8_t i=0; i<3;i++)
    {
        block_c.step[i] = 500;
        block_c.accelerate_freq[i] = 2;
        block_c.accelerate_until[i] = 200;
        block_c.decelerate_freq[i] = 2;
        block_c.decelerate_after[i] = block_c.step[i];
        block_c.dir[i] = carriage_DOWN;
    }

    TIM_SetCompare4(TIM2,TIM_ARR/2);
	TIM_SetCompare1(TIM3,TIM_ARR/2);
	TIM_SetCompare1(TIM4,TIM_ARR/2);

    while (1)
    {
        //Test_Path();
        Test_Block();
        #if USE_GCODE_COMMAND
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
        #endif

        #if USE_FORWARD_KINEMATICS
        //task 3: apply FK and report feedback
        if (machine.carriage_move[0]!=0)    machine.abc[0] = machine.abc[0] + machine.carriage_move[0]*INV(STEPS_PER_UNIT);
        if (machine.carriage_move[1]!=0)    machine.abc[1] = machine.abc[1] + machine.carriage_move[1]*INV(STEPS_PER_UNIT);
        if (machine.carriage_move[2]!=0)    machine.abc[2] = machine.abc[2] + machine.carriage_move[2]*INV(STEPS_PER_UNIT);
        //Forward_Kinematics(machine.abc, machine.xyz);
        #endif

        // led_red.state = 0;
        // Bsp_LED_Update(&led_red);
        // delay_ms(50);
        // led_red.state = 1;
        // Bsp_LED_Update(&led_red);
        // delay_ms(50);

        stepperA.freq = STEPPER_A_FREQ;
        stepperB.freq = STEPPER_B_FREQ;
        stepperC.freq = STEPPER_C_FREQ;

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
        if (machine.state==machine_OFF)
        {
            STEPPER_A_OFF;
            STEPPER_B_OFF;
            STEPPER_C_OFF;
        }else
        {
            //check current clock
            if (block_c.step[0]==0)    STEPPER_A_OFF;
            if (block_c.step[1]==0)    STEPPER_B_OFF;
            if (block_c.step[2]==0)    STEPPER_C_OFF;

            //scan pwm
            stepperA.pin_state = STEPPER_A_SCAN;
            if (stepperA.pin_state==1&&stepperA.pin_state_last==0)
            pulse_A = 1;
            else
            pulse_A = 0;
            stepperA.pin_state_last = stepperA.pin_state;

            stepperB.pin_state = STEPPER_B_SCAN;
            if (stepperB.pin_state==1&&stepperB.pin_state_last==0)
            pulse_B = 1;
            else
            pulse_B = 0;
            stepperB.pin_state_last = stepperB.pin_state;

            stepperC.pin_state = STEPPER_C_SCAN;
            if (stepperC.pin_state==1&&stepperC.pin_state_last==0)
            pulse_C = 1;
            else
            pulse_C = 0;
            stepperC.pin_state_last = stepperC.pin_state;

            if (block_c.step[0]==0&&block_c.step[1]==0&&block_c.step[2]==0)
            {
                uint8_t temp;
                temp = Block_Buff_Read(&block_c,&block_buff);
                if (temp==0)
                {
                    if (block_c.dir[0]==carriage_UP)    {DIR_A_UP;}
                    else                                {DIR_A_DOWN;}
                    if (block_c.dir[0]==carriage_UP)    {DIR_B_UP;}
                    else                                {DIR_B_DOWN;}
                    if (block_c.dir[0]==carriage_UP)    {DIR_C_UP;}
                    else                                {DIR_C_DOWN;}

                    STEPPER_A_ON;
                    STEPPER_B_ON;
                    STEPPER_C_ON;
                }
            }else if (block_c.step_dwell!=0)
            {
                block_c.step_dwell--;
            }else
            {
                //update current block
                if (pulse_A==1)
                {
                    block_c.step[0]--;
                    if (block_c.accelerate_until[0]!=0)
                    {
                         block_c.accelerate_until[0]--;
                         block_c.decelerate_after[0]--;
                         STEPPER_A_FREQ_UPDATE(STEPPER_A_FREQ*(1-block_c.accelerate_freq[0]));
                     }else if (block_c.decelerate_after[0]==0)
                     {
                         STEPPER_A_FREQ_UPDATE(STEPPER_A_FREQ*(1+block_c.accelerate_freq[0]));
                     }
                }
                
                if (pulse_B==1)
                {
                    block_c.step[1]--;
                    // if (block_c.accelerate_until[1]!=0)
                    // {
                    //     block_c.accelerate_until[1]--;
                    //     block_c.decelerate_after[1]--;
                    //     STEPPER_B_FREQ_UPDATE(STEPPER_B_FREQ*(1-block_c.accelerate_freq[1]));
                    // }else if (block_c.decelerate_after[1]==0)
                    // {
                    //     STEPPER_B_FREQ_UPDATE(STEPPER_B_FREQ*(1+block_c.accelerate_freq[1]));
                    // }
                }

                if (pulse_C==1)
                {
                    block_c.step[2]--;
                    // if (block_c.accelerate_until[2]!=0)
                    // {
                    //     block_c.accelerate_until[2]--;
                    //     block_c.decelerate_after[2]--;
                    //     STEPPER_C_FREQ_UPDATE(STEPPER_C_FREQ*(1-block_c.accelerate_freq[2]));
                    // }else if (block_c.decelerate_after[2]==0)
                    // {
                    //     STEPPER_C_FREQ_UPDATE(STEPPER_C_FREQ*(1+block_c.accelerate_freq[2]));
                    // }
                }
            }
        }
        //     else//current block is still executing
        //     {
        //         // if(block_c.step_dwell!=0)
        //         // {
        //         //      Dwell_Step_Update(&block_buff);
        //         //      monitor.state = exe_dwell;
        //         // }
        //         //else
        //         {
        //             Stepper_Count(&block_c, &machine, &stepperA);
        //             Stepper_Count(&block_c, &machine, &stepperB);
        //             Stepper_Count(&block_c, &machine, &stepperC);
        //             monitor.state = exe_block;
        //         }
        //     }
        // }
        // Bsp_Stepper_Update(&stepperA);
        // Bsp_Stepper_Update(&stepperB);
        // Bsp_Stepper_Update(&stepperC);
        // monitor.state = update;
	}
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
}

#if USE_SWITCH
//switch A
void EXTI2_IRQHandler(void)
{
	delay_ms(10);
    machine.abc[0] = CARRIAGE_A_RESET;
    if (stepperA.dir==1)
    {
        block_buff.content[block_buff.head]->step[0] = 0;
        stepperA.dir = carriage_DOWN;
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
        block_buff.content[block_buff.head]->step[1] = 0;
        stepperB.dir = carriage_DOWN;
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
        block_buff.content[block_buff.head]->step[2] = 0;
        stepperC.dir = carriage_DOWN;
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
    stepperA.state = stepper_OFF;
    stepperB.state = stepper_OFF;
    stepperC.state = stepper_OFF;

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

#endif






