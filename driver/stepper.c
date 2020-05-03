#include "stepper.h"
#include "buffer.h"
#include "switch.h"
#include "delta.h"
#include "motion.h"
/**
 * 				Set 	Dir
 * Stepper 1: 	PB11	PB3
 * Stepper 2:	PC6		PC7
 * Stepper 3:	PB6		PB5
 * 
 */

//main function of the stepper monitor
//dwell before work
void TIM5_IRQHandler()
{
    if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)
	{
        static uint8_t block_state;
        static int32_t acc_step[3];
        static int32_t dcc_step[3];
        static int32_t acc1_step[3];//head
        static int32_t acc2_step[3];//tail

        //if the switch is touched the motor stop
        Switch_Read_State(&switchA);
        Switch_Read_State(&switchB);
        Switch_Read_State(&switchC);
        if (switchA.state == 0)
        {
            block_c.step[0] = 0;
            machine.abc[0] = 0.0f;//tower A reset point
        }
        if (switchB.state == 0)
        {
            block_c.step[1] = 0;
            machine.abc[1] = 0.0f;
        }
        if (switchC.state == 0)
        {
            block_c.step[2] = 0;
            machine.abc[2] = 0.0f;
        }

        if (block_c.step[0]==0) stepperA.state==STOP;
        if (block_c.step[1]==0) stepperB.state==STOP;
        if (block_c.step[2]==0) stepperC.state==STOP;

        if (machine.state==ON)
        {
            //check whether the current block is executing
            if (block_c.step[0]==0&&block_c.step[1]==0&&block_c.step[2]==0&&block_c.step_dwell==0)
            {
                block_state = Ring_Buff_Read(&block_c, &block_list);
                if(block_state==TRUE)
                {
                    //always maximum acceleration
                    Acc_Planner(&block_c, &stepperA, &stepperB, &stepperC, acc_step, dcc_step);

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
                    Stepper_Cnt(&block_c, &stepperA, &stepperB, &stepperC);
                    //Psc_Update(&block_c, &stepperA, &stepperB, &stepperC);
                    Forward_Kinematics(machine.abc,machine.xyz);
                }
            }
        }
        Stepper_Upgrade(&stepperA);
        Stepper_Upgrade(&stepperB);
        Stepper_Upgrade(&stepperC);
	}
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
}

void Dwell_Step_Update(stepper_exe_t* stepper_abc)
{
    if (stepper_abc->step_dwell!=0)
    stepper_abc->step_dwell--;
}

//when the carriage reached the reset point its direction must be restricted 

void Block_To_Stepper(stepper_exe_t* blockX, stepper_t* stepperI,
                        stepper_t* stepperJ, stepper_t* stepperK)
{
    //update direction
    stepperI->dir = blockX->dir[0];
    stepperJ->dir = blockX->dir[1];
    stepperK->dir = blockX->dir[2];

    //update psc
    stepperI->freq = blockX->freq[0];
    stepperJ->freq = blockX->freq[1];
    stepperK->freq = blockX->freq[2];
}

//if returns 0, the input empty block will be replaced by a new one 
uint8_t Block_Check(stepper_exe_t* blockX, ring_buff_t* list)
{
    uint8_t block_state;
    //whether current block is compeleted
    if (blockX->step[0]==0&&blockX->step[1]==0&&blockX->step[2]==0)
    block_state = Ring_Buff_Read(blockX, list);
    if (block_state == TRUE) return 0;
    else return 1;
}

//use when a new block is read
//find the frequency difference
//workout the amount of freq need to be added in each term 
void Acc_Planner(stepper_exe_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc1[3], int32_t* acc2[3])
{
    int32_t f_d_a[3];//acc freq difference

    if (block->dir==stepperI->dir)  f_d_a[0] = abs(stepperI->freq-block->freq[0]);
    else                            f_d_a[0] = abs(stepperI->freq+block->freq[0]);

    if (block->dir==stepperI->dir)  f_d_a[1] = abs(stepperJ->freq-block->freq[1]);
    else                            f_d_a[1] = abs(stepperJ->freq+block->freq[1]);

    if (block->dir==stepperI->dir)  f_d_a[0] = abs(stepperK->freq-block->freq[2]);
    else                            f_d_a[0] = abs(stepperK->freq+block->freq[2]);
    
    //calculate distance -> s
    int32_t s[3] = {block->step[0],block->step[1],block->step[2]};

    //dcc time -> t_d
    int32_t t_d[3] = {abs(JERK_FREQ-block->freq[0])*STEPPER_RES/SQ(MAX_FREQ),
                    abs(JERK_FREQ-block->freq[0])*STEPPER_RES/SQ(MAX_FREQ),
                    abs(JERK_FREQ-block->freq[0])*STEPPER_RES/SQ(MAX_FREQ)};

    //dcc distance -> s_d
    int32_t s_d[3] = {(block->freq[0]+20)*t_d[0]/2,(block->freq[1]+20)*t_d[1]/2,(block->freq[2]+20)*t_d[2]/2};

    //accc time -> t_a
    int32_t t_a[3] = {f_d_a[0]*STEPPER_RES/SQ(MAX_FREQ),f_d_a[1]*STEPPER_RES/SQ(MAX_FREQ),f_d_a[2]*STEPPER_RES/SQ(MAX_FREQ)};

    acc1[0] = t_a[0]*MONITOR_FREQ;
    acc1[1] = t_a[1]*MONITOR_FREQ;
    acc1[2] = t_a[2]*MONITOR_FREQ;

    //acc distance -> s_a
    int32_t s_a[3] = {(block->freq[0]+stepperI->freq)*t_d[0]/2,(block->freq[1]+stepperJ->freq)*t_d[1]/2,(block->freq[2]+stepperK->freq)*t_d[2]/2};

    //n distance -> s_n
    int32_t s_n[3] = {s[0]-s_a[0]-s_d[0],s[1]-s_a[1]-s_d[1],s[2]-s_a[2]-s_d[2]};

    //n time -> t_n
    int32_t t_n[3] = {s_n[0]/block->freq[0],s_n[1]/block->freq[1],s_n[2]/block->freq[2]};

    //dcc at t_a + t_n
    int32_t t[3] = {t_a[0]+t_n[0],t_a[1]+t_n[1],t_a[2]+t_n[2]};

    dcc[0] = t[0]*MONITOR_FREQ;
    dcc[1] = t[1]*MONITOR_FREQ;
    dcc[2] = t[2]*MONITOR_FREQ;
}

void ADcc_Cnt(stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK, int32_t* acc_step, int32_t* dcc_step, stepper_exe_t* block)
{
    //if acc
    if (acc_step[0]!=0)
    {
        stepperI->freq = stepperI->freq + ACC;
        acc_step[0]--;
        dcc_step[0]--;
    }else if (dcc_step[0]==0)
    {
        stepperI->freq = stepperI->freq - ACC;
    }else
    {
        stepperI->freq = block->freq[0];
        dcc_step[0]--;
    }
/*************************************************/
    if (acc_step[1]!=0)
    {
        stepperJ->freq = stepperJ->freq + ACC;
        acc_step[1]--;
        dcc_step[1]--;
    }else if (dcc_step[1]==0)
    {
        stepperJ->freq = stepperJ->freq - ACC;
    }else
    {
        stepperJ->freq = block->freq[1];
        dcc_step[1]--;
    }
/*************************************************/
    if (acc_step[2]!=0)
    {
        stepperK->freq = stepperK->freq + ACC;
        acc_step[2]--;
        dcc_step[2]--;
    }else if (dcc_step[2]==0)
    {
        stepperK->freq = stepperK->freq - ACC;
    }else
    {
        stepperK->freq = block->freq[2];
        dcc_step[2]--;
    }
}


void Stepper_Cnt(stepper_exe_t* block, stepper_t* stepperI, stepper_t* stepperJ, stepper_t* stepperK)
{
    if (stepperI->pin_state_last==0&&stepperI->pin_state==1)    block->step[0]--;
    if (stepperJ->pin_state_last==0&&stepperJ->pin_state==1)    block->step[1]--;
    if (stepperK->pin_state_last==0&&stepperK->pin_state==1)    block->step[2]--;
}

/****************************************Board_Support_Package**************************************/

void Stepper_Init(stepper_t* stepperX)
{
	// NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	//Enables or disables the AHB1 peripheral clock.
	RCC_AHB1PeriphClockCmd(stepperX->RCC_AHB1Periph_GPIOX,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= stepperX->GPIO_Pin_X;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	
	GPIO_Init(stepperX->GPIOX,&GPIO_InitStructure);

	//Enables the Low Speed APB (APB1) peripheral clock.
	RCC_APB1PeriphClockCmd(stepperX->RCC_APB1Periph_TIMX, ENABLE);
	
	//Changes the mapping of the specified pin.
	GPIO_PinAFConfig(stepperX->GPIOX,stepperX->GPIO_PinSourceX,stepperX->GPIO_AF_TIMX);

	TIM_TimeBaseInitStructure.TIM_Period        = ARR;
    TIM_TimeBaseInitStructure.TIM_Prescaler     = stepperX->psc;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Down;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	//Initializes the TIMx Time Base Unit peripheral according to 
    //the specified parameters in the TIM_TimeBaseInitStruct.
	TIM_TimeBaseInit(stepperX->TIMX, &TIM_TimeBaseInitStructure);
    
	//Enables the specified TIM peripheral.
	TIM_Cmd(stepperX->TIMX, ENABLE);

	//Timer output compare
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;        // 选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;               // 输出极性:TIM输出比较极性低
    
	//Initializes the TIMx Channel1 according to the specified parameters in
	//the TIM_OCInitStruct.
	TIM_OCInitStructure.TIM_Pulse       = STOP;//0 or arr/2, CCRx_Val

    if (stepperX->PWM_Ch==1)
    {
        TIM_OC1Init(stepperX->TIMX, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(stepperX->TIMX, TIM_OCPreload_Disable);
    }else if (stepperX->PWM_Ch==2)
    {
        TIM_OC2Init(stepperX->TIMX, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(stepperX->TIMX, TIM_OCPreload_Disable);
    }else if (stepperX->PWM_Ch==3)
    {
        TIM_OC3Init(stepperX->TIMX, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(stepperX->TIMX, TIM_OCPreload_Disable);
    }else
    {
        TIM_OC4Init(stepperX->TIMX, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(stepperX->TIMX, TIM_OCPreload_Disable);
    }
	//Enables the TIMx peripheral Preload register on CCR1.
	//Enables the specified TIM peripheral.
	TIM_Cmd(stepperX->TIMX, ENABLE);
}

//TIM5 monitor
void TIM5_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///使能TIM3时钟
	
    TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM5,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//stepper struct to hardware
void Stepper_Upgrade(stepper_t* stepperX)
{
    //update stepper state
    if (stepperX->PWM_Ch==1)
    {
        if (stepperX->state==START)
        TIM_SetCompare1(stepperX->TIMX, ARR/2);
        else
        TIM_SetCompare1(stepperX->TIMX,0);
    }else if (stepperX->PWM_Ch==2)
    {
        if (stepperX->state==START)
        TIM_SetCompare2(stepperX->TIMX, ARR/2);
        else
        TIM_SetCompare2(stepperX->TIMX,0);
    }else if (stepperX->PWM_Ch==3)
    {
        if (stepperX->state==START)
        TIM_SetCompare3(stepperX->TIMX, ARR/2);
        else
        TIM_SetCompare3(stepperX->TIMX,0);
    }else
    {
        if (stepperX->state==START)
        TIM_SetCompare4(stepperX->TIMX, ARR/2);
        else
        TIM_SetCompare4(stepperX->TIMX,0);
    }

    //update speed
    uint16_t psc = T_CLK/(stepperX->freq*ARR);
    TIM_PrescalerConfig(stepperX->TIMX,psc,TIM_PSCReloadMode_Update);

    //update direction
    if (stepperX->dir==0)   GPIO_SetBits(stepperX->GPIOX_Dir, stepperX->GPIO_Pin_X_Dir);
    else                    GPIO_ResetBits(stepperX->GPIOX_Dir, stepperX->GPIO_Pin_X_Dir);

    //update IO state
    stepperX->pin_state_last = stepperX->pin_state;
    stepperX->pin_state = GPIO_ReadInputDataBit(stepperX->GPIOX, stepperX->GPIO_Pin_X);
}


