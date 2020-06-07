#include "sys.h"
#include "delay.h"
#include "config.h"
#include "stepper.h"
#include "type.h"
#include "buffer.h"
#include "planner_new.h"
#include "bsp.h"
#include "delta.h"
#include "machine.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "test.h"

/******************************Hardware******************************/
stepper_t   stepperA;
stepper_t   stepperB;
stepper_t   stepperC;
machine_t   machine;
uint8_t     planner_result;
float xyz_c[3] = {90.f,90.f,112.f};
float xyz_t[3] = {90.f,90.f,0.f};

//buffer
block_buff_t block_buffer;

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
    Bsp_TIM2_PWM_Init(FREQ2PSC(2));//Stepper_A
    Bsp_TIM3_PWM_Init(FREQ2PSC(2));//Stepper_B
    Bsp_TIM4_PWM_Init(FREQ2PSC(2));//Stepper_C
    STEPPER_A_OFF;
    STEPPER_B_OFF;
    STEPPER_C_OFF;
    Bsp_TIM5_Init(FREQ2PSC(5000));

    Block_Buff_Init(&block_buffer);
    Machine_Init();
    Stepper_Init(&stepperA);
    Stepper_Init(&stepperB);
    Stepper_Init(&stepperC);
    Micro_Step_Init(4);
    Bsp_Microstep_force();

/*************************CUSTOM_PATH******************************/
    //uint8_t path_num = 0;

    Test_Block_0();
    Test_Block_1();
    //Test_Block_2();
    //Test_Block_3();

    //machine.xyz_t[0] = 90.f;
    //machine.xyz_t[1] = 90.f;
    //machine.xyz_t[2] = 0.f;
	
    //planner_result = Line_XYZ_Planner(xyz_c,xyz_t,50.f);

    uint32_t block_num = 0;
    block_t new_block;

    while (1)
    {
        
        /* if (block_buffer.length<RINGBUFF_LEN&&block_num<564)
        {
            new_block.maximum_freq[0] = block_path[block_num][0][0];
            new_block.maximum_freq[1] = block_path[block_num][0][1];
            new_block.maximum_freq[2] = block_path[block_num][0][2];
            new_block.dir[0] = block_path[block_num][1][0];
            new_block.dir[1] = block_path[block_num][1][1];
            new_block.dir[2] = block_path[block_num][1][2];
            new_block.step[0] = block_path[block_num][2][0];
            new_block.step[1] = block_path[block_num][2][1];
            new_block.step[2] = block_path[block_num][2][2];
            new_block.flag = block_ready;
            Block_Buff_Write(new_block,&block_buffer);
			block_num++;
        } */




        //if (planner_result&&block_buffer.length<RINGBUFF_LEN)
        //{
        //    planner_result = Line_XYZ_Planner(xyz_c, xyz_t,50.f);
        //}
        //Test_Block_2();
        //Test_Block_3();
        Machine_Update();
        delay_ms(100);
        //block_buffer.content[block_buffer.head].flag = block_free;
        //Block_Buff_Clear(&block_buffer);
        	
    }
}
//void USART1_IRQHandler(void)
//{
//    uint8_t res;
//    if (USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)
//    {
//        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//        res = USART_ReceiveData(USART1);
//        Uart_Buff_Write(&uart_buff,res);
//        if (res==13)    machine.interpret_flag = SET;
//    }
//}

//the machine will only execute the first block
void TIM5_IRQHandler()
{
    if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)
	{
        if (machine.state==machine_ON)
        {
            LED_RED_OFF;
            LED_GREEN_ON;

            if (block_buffer.content[block_buffer.head].step[0]==0)    STEPPER_A_OFF;
            if (block_buffer.content[block_buffer.head].step[1]==0)    STEPPER_B_OFF;
            if (block_buffer.content[block_buffer.head].step[2]==0)    STEPPER_C_OFF;

            if (block_buffer.content[block_buffer.head].flag==block_exe)
            {
                if (block_buffer.content[block_buffer.head].step[0]==0)    STEPPER_A_OFF;
                else                                                       STEPPER_A_ON;
                if (block_buffer.content[block_buffer.head].step[1]==0)    STEPPER_B_OFF;
                else                                                       STEPPER_B_ON;
                if (block_buffer.content[block_buffer.head].step[2]==0)    STEPPER_C_OFF;
                else                                                       STEPPER_C_ON;

                if (Stepper_A_Pulse()&&block_buffer.content[block_buffer.head].step[0]>0)
                {
                    block_buffer.content[block_buffer.head].step[0]--;
                    if (STEPPER_A_DIR)  machine.carriage_move[0] ++;
                    else                machine.carriage_move[0] --;
                }
                if (Stepper_B_Pulse()&&block_buffer.content[block_buffer.head].step[1]>0)
                {
                    block_buffer.content[block_buffer.head].step[1]--;
                    if (STEPPER_B_DIR)  machine.carriage_move[1] ++;
                    else                machine.carriage_move[1] --;
                }
                if (Stepper_C_Pulse()&&block_buffer.content[block_buffer.head].step[2]>0)
                {
                    block_buffer.content[block_buffer.head].step[2]--;
                    if (STEPPER_C_DIR)  machine.carriage_move[2] ++;
                    else                machine.carriage_move[2] --;
                }

                STEPPER_A_FREQ_UPDATE(block_buffer.content[block_buffer.head].maximum_freq[0]);
                STEPPER_B_FREQ_UPDATE(block_buffer.content[block_buffer.head].maximum_freq[1]);
                STEPPER_C_FREQ_UPDATE(block_buffer.content[block_buffer.head].maximum_freq[2]);

                if (block_buffer.content[block_buffer.head].step[0]==0&&block_buffer.content[block_buffer.head].step[1]==0
                &&block_buffer.content[block_buffer.head].step[2]==0&&block_buffer.length>0&&block_buffer.content[block_buffer.head].flag==block_exe)
                {
                    block_buffer.content[block_buffer.head].flag = block_free;
                    Block_Buff_Clear(&block_buffer);
                }
            }
            else if (block_buffer.content[block_buffer.head].flag==block_ready)
            {
                block_buffer.content[block_buffer.head].flag=block_exe;
                STEPPER_A_FREQ_UPDATE(block_buffer.content[block_buffer.head].maximum_freq[0]);
                STEPPER_B_FREQ_UPDATE(block_buffer.content[block_buffer.head].maximum_freq[1]);
                STEPPER_C_FREQ_UPDATE(block_buffer.content[block_buffer.head].maximum_freq[2]);

                if (block_buffer.content[block_buffer.head].dir[0]==carriage_UP)    DIR_A_UP;
                else                                                                DIR_A_DOWN;
                if (block_buffer.content[block_buffer.head].dir[1]==carriage_UP)    DIR_B_UP;
                else                                                                DIR_B_DOWN;
                if (block_buffer.content[block_buffer.head].dir[2]==carriage_UP)    DIR_C_UP;
                else                                                                DIR_C_DOWN;

                if (block_buffer.content[block_buffer.head].step[0]==0)    STEPPER_A_OFF;
                else                                                       STEPPER_A_ON;
                if (block_buffer.content[block_buffer.head].step[1]==0)    STEPPER_B_OFF;
                else                                                       STEPPER_B_ON;
                if (block_buffer.content[block_buffer.head].step[2]==0)    STEPPER_C_OFF;
                else                                                       STEPPER_C_ON;
            }else
            {
                STEPPER_A_OFF;
                STEPPER_B_OFF;
                STEPPER_C_OFF;
            }
        }else
        {
            LED_RED_ON;
            LED_GREEN_OFF;
            STEPPER_A_OFF;
            STEPPER_B_OFF;
            STEPPER_C_OFF;
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
    STEPPER_A_OFF;
    machine.abc[0] = CARRIAGE_A_RESET;
    block_buffer.content[block_buffer.head].step[0] = 0;
    EXTI_ClearITPendingBit(EXTI_Line2);
}

//switch B
void EXTI3_IRQHandler(void)
{
    delay_ms(10);
    STEPPER_B_OFF;
    machine.abc[1] = CARRIAGE_B_RESET;
    block_buffer.content[block_buffer.head].step[1] = 0;
    EXTI_ClearITPendingBit(EXTI_Line3);
}

//switch C
void EXTI1_IRQHandler(void)
{
    delay_ms(10);
    STEPPER_C_OFF;
    machine.abc[0] = CARRIAGE_C_RESET;
    block_buffer.content[block_buffer.head].step[2] = 0;
	EXTI_ClearITPendingBit(EXTI_Line1);
}

//switch S
void EXTI0_IRQHandler(void)
{
    delay_ms(10);
    machine.state = machine_OFF;	 
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI4_IRQHandler(void)
{
    delay_ms(10);
    machine.state = machine_ON;
    EXTI_ClearITPendingBit(EXTI_Line4);
}


