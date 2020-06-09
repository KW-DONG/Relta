#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pwm.h"

#define STEPS_PER_UNIT      80

//carriage specification
#define MAX_ACCELERATION    1000    //mm per sec per sec
#define MAX_SPEED           1000    //mm per sec
#define JERK_SPEED          200     //mm per sec
#define STEPPER_RES         16      //1/x mm
#define TIM_ARR             8400    //1-65536
#define MONITOR_FREQ        20      //hz
#define T_CLK               84000000//hz

#define STEPPER_A_ON        TIM_SetCompare4(TIM2,TIM_ARR/2)
#define STEPPER_A_OFF       TIM_SetCompare4(TIM2,0)
#define STEPPER_A_FREQ_UPDATE(f)   TIM_PrescalerConfig(TIM2,T_CLK/(f*TIM_ARR),TIM_PSCReloadMode_Immediate)
#define DIR_A_UP            GPIO_SetBits(GPIOA, GPIO_Pin_15);
#define DIR_A_DOWN          GPIO_ResetBits(GPIOA, GPIO_Pin_15);
#define STEPPER_A_SCAN      GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)
#define STEPPER_A_FREQ      T_CLK/((TIM2->PSC)*(TIM2->ARR))

#define STEPPER_B_ON        TIM_SetCompare1(TIM3,TIM_ARR/2)
#define STEPPER_B_OFF       TIM_SetCompare1(TIM3,0)
#define STEPPER_B_FREQ_UPDATE(f)   TIM_PrescalerConfig(TIM3,T_CLK/(f*TIM_ARR),TIM_PSCReloadMode_Immediate)
#define DIR_B_UP            GPIO_SetBits(GPIOC, GPIO_Pin_7);
#define DIR_B_DOWN          GPIO_ResetBits(GPIOC, GPIO_Pin_7);
#define STEPPER_B_SCAN      GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11)
#define STEPPER_B_FREQ      T_CLK/((TIM3->PSC)*(TIM3->ARR))

#define STEPPER_C_ON        TIM_SetCompare1(TIM4,TIM_ARR/2)
#define STEPPER_C_OFF       TIM_SetCompare1(TIM4,0)
#define STEPPER_C_FREQ_UPDATE(f)   TIM_PrescalerConfig(TIM4,T_CLK/(f*TIM_ARR),TIM_PSCReloadMode_Immediate)
#define DIR_C_UP            GPIO_SetBits(GPIOA, GPIO_Pin_6);
#define DIR_C_DOWN          GPIO_ResetBits(GPIOA, GPIO_Pin_6);
#define STEPPER_C_SCAN      GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)
#define STEPPER_C_FREQ      T_CLK/((TIM4->PSC)*(TIM4->ARR))

#define FREQ2PSC(x)			T_CLK/(x*TIM_ARR)

volatile u8 stepper1_state;
volatile u8 stepper1_state_last;
volatile u8 stepper2_state;
volatile u8 stepper2_state_last;
volatile u8 stepper3_state;
volatile u8 stepper3_state_last;
volatile u32 step_1;
volatile u32 step_2;
volatile u32 step_3;
volatile u32 stepper1_freq;
volatile u32 stepper2_freq;
volatile u32 stepper3_freq;
volatile u32 stepper1_ccr;
volatile u32 stepper2_ccr;
volatile u32 stepper3_ccr;


int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
 	TIM2_PWM_Init(TIM_ARR-1,FREQ2PSC(10)-1);	//84M/84=1Mhz的计数频率,重装载值500，所以PWM频率为 1M/500=2Khz.
	TIM3_PWM_Init(TIM_ARR-1,FREQ2PSC(10)-1);
	TIM4_PWM_Init(TIM_ARR-1,FREQ2PSC(10)-1);
	TIM5_Init(TIM_ARR-1, FREQ2PSC(5000)-1);
	TIM2_PWM_Detect();
	TIM3_PWM_Detect();
	TIM4_PWM_Detect();
	LED_Init();
	
	stepper1_state = 0;
	stepper1_state_last = 0;
	stepper2_state = 0;
	stepper2_state_last = 0;
	stepper3_state = 0;
	stepper3_state_last = 0;
	TIM_SetCompare4(TIM2,2000);
	TIM_SetCompare1(TIM3,2000);
	TIM_SetCompare1(TIM4,2000);
	step_1 = 0;
	step_2 = 0;
	step_3 = 0;
	

   while(1) //实现比较值从0-300递增，到300后从300-0递减，循环
	{
		GPIO_ResetBits(GPIOF,GPIO_Pin_9);  //LED0对应引脚GPIOF.9拉低，亮  等同LED0=0;
		GPIO_SetBits(GPIOF,GPIO_Pin_10);   //LED1对应引脚GPIOF.10拉高，灭 等同LED1=1;
		delay_ms(100);  		   //延时300ms
		GPIO_SetBits(GPIOF,GPIO_Pin_9);	   //LED0对应引脚GPIOF.0拉高，灭  等同LED0=1;
		GPIO_ResetBits(GPIOF,GPIO_Pin_10); //LED1对应引脚GPIOF.10拉低，亮 等同LED1=0;
		delay_ms(100);
		
 	}
}

void TIM5_IRQHandler()
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)
	{
		stepper1_freq = STEPPER_A_FREQ;
		stepper2_freq = STEPPER_B_FREQ;
		stepper3_freq = STEPPER_C_FREQ;
		
		if (step_1>10000)	step_1 = 0;
		if (step_2>1000)	step_2 = 0;
		if (step_3>100)		step_3 = 0;
		
		if (stepper1_freq>1000)		stepper1_freq = 10;
		if (stepper2_freq>1000)		stepper2_freq = 10;
		if (stepper3_freq>1000)		stepper3_freq = 10;
		
		
		stepper1_state = STEPPER_A_SCAN;
		if (stepper1_state==1&&stepper1_state_last==0)
		{
			step_1++;
			//stepper1_freq = stepper1_freq + 1;		
		}
		stepper1_state_last = stepper1_state;
		
		stepper2_state = STEPPER_B_SCAN;
		if (stepper2_state==1&&stepper2_state_last==0)
		{
			step_2++;
			//stepper2_freq = stepper2_freq + 2;	
		}
		stepper2_state_last = stepper2_state;
		
		stepper3_state = STEPPER_C_SCAN;
		if (stepper3_state==1&&stepper3_state_last==0)
		{
			step_3++;
			//stepper3_freq = stepper3_freq + 3;	
		}
		stepper3_state_last = stepper3_state;
		
		//TIM_PrescalerConfig(TIM2,T_CLK/(stepper1_freq*TIM_ARR),TIM_PSCReloadMode_Update);
		//TIM_PrescalerConfig(TIM3,T_CLK/(stepper2_freq*TIM_ARR),TIM_PSCReloadMode_Update);
		//TIM_PrescalerConfig(TIM4,T_CLK/(stepper3_freq*TIM_ARR),TIM_PSCReloadMode_Update);
		
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位
	
	
}














