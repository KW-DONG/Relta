#include "sys.h"
#include "delay.h"
#include "config.h"
#include "stepper.h"
#include "type.h"
#include "buffer.h"
#include "planner.h"
#include "bsp.h"
#include "delta.h"
#include "machine.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "test.h"
#include "gcode.h"

/******************************Hardware******************************/
stepper_t   stepperA;
stepper_t   stepperB;
stepper_t   stepperC;
machine_t   machine;
uint8_t     planner_result;

//buffer
block_buff_t block_buffer;
uart_buff_t uart_buffer;
command_t command_c;

int main()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
    delay_init(168);

	//Bsp_UART_Init(115200);
    Bsp_EXTI0_Init();
    Bsp_EXTI1_Init();
    Bsp_EXTI2_Init();
    Bsp_EXTI3_Init();
    Bsp_EXTI4_Init();
    Bsp_LED_Init();
    Bsp_Stepper_Init();
    Bsp_Switch_Init();
    Bsp_KEY_Init();
    Bsp_TIM2_PWM_Init(FREQ2PSC(500));//Stepper_A
    Bsp_TIM3_PWM_Init(FREQ2PSC(500));//Stepper_B
    Bsp_TIM4_PWM_Init(FREQ2PSC(500));//Stepper_C
    STEPPER_A_OFF;
    STEPPER_B_OFF;
    STEPPER_C_OFF;
    Bsp_TIM5_Init(FREQ2PSC(5000));

    Block_Buff_Init(&block_buffer);
    Machine_Init();
    Stepper_Init(&stepperA);
    Stepper_Init(&stepperB);
    Stepper_Init(&stepperC);

    Micro_Step_Init(16);

/*************************CUSTOM_PATH******************************/

    uint32_t path_num = 0;
    uint32_t block_num = 0;
    for (uint8_t i=0;i<3;i++)
    {
        machine.xyz_t[i] = path_0[path_num][i];
    }
    
    machine.feedrate = path_0[path_num][3];
    planner_result = Line_XYZ_Planner(machine.xyz_i,machine.xyz_c,machine.xyz_t, machine.abc_l ,machine.feedrate);

    
    block_t new_block;

    while (1)
    {
        
        if (planner_result&&block_buffer.length<(RINGBUFF_LEN))
        {
            planner_result = Line_XYZ_Planner_1(machine.xyz_i,machine.xyz_c,machine.xyz_t,machine.abc_l, machine.feedrate);
            //if (planner_result==0)  machine.state = machine_OFF;
        }
        else if ((!planner_result)&&block_buffer.length==0&&path_num<10&&machine.state==machine_ON)
        {
            path_num++;
            //while (block_buffer.length>0);
            Machine_Update();
            for (uint8_t i=0;i<3;i++)
            {
                machine.xyz_i[i] = machine.xyz[i];
                machine.xyz_c[i] = machine.xyz[i];
                machine.xyz_t[i] = path_0[path_num][i];
                machine.abc_l[i] = machine.abc[i];
            }
            machine.feedrate = path_0[path_num][3];
            planner_result = Line_XYZ_Planner_1(machine.xyz_i,machine.xyz_c,machine.xyz_t,machine.abc_l,machine.feedrate);
        }
      
        //BLOCK_UPDATE
        if ((machine.state==machine_ON)&&
            block_buffer.content[block_buffer.head].step[0]==0&&block_buffer.content[block_buffer.head].step[1]==0&&block_buffer.content[block_buffer.head].step[1]==0&&
            block_buffer.content[block_buffer.head].flag==block_exe)
        {
            block_buffer.content[block_buffer.head].flag = block_free;
            Block_Buff_Clear(&block_buffer);
        }
        //BLOCK_INIT
        if ((machine.state==machine_ON)&&
            block_buffer.content[block_buffer.head].flag==block_ready)
        {
            STEPPER_A_FREQ_UPDATE(block_buffer.content[block_buffer.head].maximum_freq[0]);
            STEPPER_B_FREQ_UPDATE(block_buffer.content[block_buffer.head].maximum_freq[1]);
            STEPPER_C_FREQ_UPDATE(block_buffer.content[block_buffer.head].maximum_freq[2]);

            if (block_buffer.content[block_buffer.head].dir[0]==carriage_UP)    DIR_A_UP;
            else                                                                DIR_A_DOWN;
            if (block_buffer.content[block_buffer.head].dir[1]==carriage_UP)    DIR_B_UP;
            else                                                                DIR_B_DOWN;
            if (block_buffer.content[block_buffer.head].dir[2]==carriage_UP)    DIR_C_UP;
            else                                                                DIR_C_DOWN;

            block_buffer.content[block_buffer.head].flag = block_exe;
        }    
        Machine_Update();	
    }
}

void USART1_IRQHandler(void)
{
    uint8_t res;
    if (USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        res = USART_ReceiveData(USART1);
        Uart_Buff_Write(&uart_buffer,res);
        if (res==13)    machine.interpret_flag = SET;
    }
}

//the machine will only execute the first block
void TIM5_IRQHandler()
{
    if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)
	{
        //LED
        if ((machine.state==machine_ON)&&
            (block_buffer.content[block_buffer.head].step[0]>0||block_buffer.content[block_buffer.head].step[1]>0||block_buffer.content[block_buffer.head].step[2]>0)&&
            (block_buffer.content[block_buffer.head].flag==block_exe))
        {
            LED_GREEN_ON;
            LED_RED_OFF;         
        }
        else
        {
            LED_GREEN_OFF;
            LED_RED_ON;
        }

        //STEPPER_A
        if ((machine.state==machine_ON)&&
            (block_buffer.content[block_buffer.head].step[0]>0&&
            (block_buffer.content[block_buffer.head].flag==block_exe)))
        {
            STEPPER_A_ON;
            stepperA.state = stepper_ON;
            if (Stepper_A_Pulse())
            {
                block_buffer.content[block_buffer.head].step[0]--;
                if (STEPPER_A_DIR)  machine.carriage_move[0] --;
                else                machine.carriage_move[0] ++;
            }
        }
        else
        {
            STEPPER_A_OFF;
            stepperA.state = stepper_OFF;
        }

        //STEPPER_B
        if ((machine.state==machine_ON)&&
            (block_buffer.content[block_buffer.head].step[1]>0&&
            (block_buffer.content[block_buffer.head].flag==block_exe)))
        {
            STEPPER_B_ON;
            stepperB.state = stepper_ON;
            if (Stepper_B_Pulse())
            {
                block_buffer.content[block_buffer.head].step[1]--;
                if (STEPPER_B_DIR)  machine.carriage_move[1] --;
                else                machine.carriage_move[1] ++;
            }
        }
        else
        {
            STEPPER_B_OFF;
            stepperB.state = stepper_OFF;
        }

        //STEPPER_C
        if ((machine.state==machine_ON)&&
            (block_buffer.content[block_buffer.head].step[2]>0&&
            (block_buffer.content[block_buffer.head].flag==block_exe)))
        {
            STEPPER_C_ON;
            stepperC.state = stepper_ON;
            if (Stepper_C_Pulse())
            {
                block_buffer.content[block_buffer.head].step[2]--;
                if (STEPPER_C_DIR)  machine.carriage_move[2] --;
                else                machine.carriage_move[2] ++;
            }
        }
        else
        {
            STEPPER_C_OFF;
            stepperC.state = stepper_OFF;
        }

    }
    Stepper_A_Update();
    Stepper_B_Update();
    Stepper_C_Update();
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
}

//switch A
void EXTI2_IRQHandler(void)
{
	delay_ms(10);

    if (EXTI2_SCAN == 0)
    {
        STEPPER_A_OFF;
        machine.abc[0] = CARRIAGE_A_RESET;
        block_buffer.content[block_buffer.head].step[0] = 0;
    }
    EXTI_ClearITPendingBit(EXTI_Line2);
}

//switch B
void EXTI3_IRQHandler(void)
{
    delay_ms(10);

    if (EXTI3_SCAN == 0)
    {
        STEPPER_B_OFF;
        machine.abc[1] = CARRIAGE_B_RESET;
        block_buffer.content[block_buffer.head].step[1] = 0;
    }
    EXTI_ClearITPendingBit(EXTI_Line3);
}

//switch C
void EXTI1_IRQHandler(void)
{
    delay_ms(10);
    if (EXTI1_SCAN == 0)
    {
        STEPPER_C_OFF;
        machine.abc[0] = CARRIAGE_C_RESET;
        block_buffer.content[block_buffer.head].step[2] = 0;
    }
	EXTI_ClearITPendingBit(EXTI_Line1);
}

//switch S
void EXTI0_IRQHandler(void)
{
    delay_ms(10);
    if (EXTI0_SCAN == 0)    machine.state = machine_OFF;	 
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI4_IRQHandler(void)
{
    delay_ms(10);
    if (EXTI4_SCAN == 0)    machine.state = machine_ON;
    EXTI_ClearITPendingBit(EXTI_Line4);
}


