#include "sys.h"
#include "delay.h"
#include "config.h"
#include "stepper.h"
#include "type.h"
#include "buffer.h"
#include "gcode.h"
#include "planner_new.h"
#include "bsp.h"
#include "stm32f4xx.h"
#include "delta.h"
#include "machine.h"

/******************************Hardware******************************/
stepper_t   stepperA;
stepper_t   stepperB;
stepper_t   stepperC;
machine_t   machine;

//buffer
block_buff_t block_buff;
uart_buff_t  uart_buff;
command_t    command_c;

int main()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
    delay_init(168);

	Bsp_UART_Init(115200);
    Bsp_EXTI0_Init();
    Bsp_EXTI1_Init();
    Bsp_EXTI2_Init();
    Bsp_EXTI3_Init();
    Bsp_EXTI4_Init();
    Bsp_LED_Init();
    Bsp_Stepper_Init();
    Bsp_Switch_Init();
    Bsp_KEY_Init();
    Bsp_TIM2_PWM_Init(FREQ2PSC(1));//Stepper_A
    Bsp_TIM3_PWM_Init(FREQ2PSC(1));//Stepper_B
    Bsp_TIM4_PWM_Init(FREQ2PSC(1));//Stepper_C
    Bsp_TIM5_Init(FREQ2PSC(100));

    Block_Buff_Init(&block_buff);
    Machine_Init();
    Stepper_Init(&stepperA);
    Stepper_Init(&stepperB);
    Stepper_Init(&stepperC);

/*************************CUSTOM_PATH******************************/
    float xyz_t[3] = {0.f,0.f,0.f};
    Line_XYZ_Planner(machine.xyz,xyz_t,10.f);

    while (1)
    {
        //Test_Path();
        //Test_Block();
        
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
        Uart_Buff_Write(&uart_buff,res);
        if (res==13)    machine.interpret_flag = SET;
    }
}

//monitor planner
void TIM5_IRQHandler()
{
    if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)
	{
        if (machine.state==machine_ON)
        {
            LED_RED_OFF;
            LED_GREEN_ON;
            if (block_buff.content[block_buff.head].step[0]==0) STEPPER_A_OFF;
            else                                                STEPPER_A_ON;
            if (block_buff.content[block_buff.head].step[1]==0) STEPPER_B_OFF;
            else                                                STEPPER_B_ON;
            if (block_buff.content[block_buff.head].step[2]==0) STEPPER_C_OFF;
            else                                                STEPPER_C_ON;

            if (block_buff.content[block_buff.head].step[0]==0
                &&block_buff.content[block_buff.head].step[1]==0
                &&block_buff.content[block_buff.head].step[2]==0
                &&block_buff.length>0)
            {
                Block_Buff_Clear(&block_buff);
                STEPPER_A_FREQ_UPDATE(block_buff.content[block_buff.head].maximum_freq[0]);
                STEPPER_B_FREQ_UPDATE(block_buff.content[block_buff.head].maximum_freq[1]);
                STEPPER_C_FREQ_UPDATE(block_buff.content[block_buff.head].maximum_freq[2]);
                if (block_buff.content[block_buff.head].dir[0]==carriage_UP)    DIR_A_UP;
                else                                                            DIR_A_DOWN;
                if (block_buff.content[block_buff.head].dir[1]==carriage_UP)    DIR_B_UP;
                else                                                            DIR_B_DOWN;
                if (block_buff.content[block_buff.head].dir[2]==carriage_UP)    DIR_C_UP;
                else                                                            DIR_C_DOWN;
            }
            else
            {
                if (Stepper_A_Pulse())
                {
                    block_buff.content[block_buff.head].step[0]--;
                    if (STEPPER_A_DIR)  machine.carriage_move[0] ++;
                    else                machine.carriage_move[0] --;
                }
                
                if (Stepper_B_Pulse())
                {
                    block_buff.content[block_buff.head].step[1]--;
                    if (STEPPER_B_DIR)  machine.carriage_move[1] ++;
                    else                machine.carriage_move[1] --;
                }
                if (Stepper_C_Pulse())
                {
                    block_buff.content[block_buff.head].step[2]--;
                    if (STEPPER_C_DIR)  machine.carriage_move[2] ++;
                    else                machine.carriage_move[2] --;
                }
            }
            Stepper_A_Update();
            Stepper_B_Update();
            Stepper_C_Update();
        }else
        {
            LED_RED_ON;
            LED_GREEN_OFF;
        }
	}
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
}

//switch A
void EXTI2_IRQHandler(void)
{
	delay_ms(10);
    STEPPER_A_OFF;
    machine.abc[0] = CARRIAGE_A_RESET;
    block_buff.content[block_buff.head].step[0] = 0;
    EXTI_ClearITPendingBit(EXTI_Line0);
}

//switch B
void EXTI3_IRQHandler(void)
{
    delay_ms(10);
    STEPPER_B_OFF;
    machine.abc[1] = CARRIAGE_B_RESET;
    block_buff.content[block_buff.head].step[1] = 0;
    EXTI_ClearITPendingBit(EXTI_Line1);
}

//switch C
void EXTI1_IRQHandler(void)
{
    delay_ms(10);
    STEPPER_C_OFF;
    machine.abc[0] = CARRIAGE_C_RESET;
    block_buff.content[block_buff.head].step[2] = 0;
	EXTI_ClearITPendingBit(EXTI_Line3);
}

//switch S
void EXTI0_IRQHandler(void)
{
    delay_ms(10);
    machine.state = machine_OFF;	 
	EXTI_ClearITPendingBit(EXTI_Line4);
}

void EXTI4_IRQHandler(void)
{
    delay_ms(10);
    machine.state = machine_ON;
    EXTI_ClearITPendingBit(EXTI_Line2);
}