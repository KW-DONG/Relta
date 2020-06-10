#ifndef __TYPE_H
#define __TYPE_H
#include <math.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

#define SQ(x)   ((x)*(x))
#define COS(x)  (cosf(x))
#define SIN(x)  (sinf(x))
#define INV(x)  (1.0f / (x))
#define E(x)    (powf(10.0f,x))
#define PI      3.14f
#define RSQRT(x)    (1.0f / sqrtf(x))
#define HYPOT2(x,y) (SQ(x)+SQ(y))

#define RINGBUFF_LEN 5

#define TRUE 1
#define FALSE 0


/****************************BUFF_TYPE************************/

/**
 * radius>0 G2
 * radius<0 G3
 * xyz!=0 && radius=0 && feedrate!=0 G1
 * xyz!=0 && radius=0 && feedrate==0 G0
 * 
 * radius_dwell depend on the type
 */

typedef struct _command
{
    float xyz[3];
    float feedrate;
    float radius_dwell;
    uint8_t type;
}command_t;


typedef struct 
{
    uint16_t    head;
    uint16_t    tail;
    uint16_t    length;
    uint8_t     content[100];
}uart_buff_t;

typedef struct 
{
    //read by planner
    float       entry_velocity[3];
    float       maximum_velocity[3];
    float       leave_velocity[3];
    float       distance[3];
    float       acceleration[3];
    float       deceleration[3];

    //read by controller
    uint8_t     dir[3];
    uint32_t    accelerate_until[3];
    uint32_t    decelerate_after[3];
    uint32_t    accelerate_freq[3];
    uint32_t    decelerate_freq[3];
    uint32_t    maximum_freq[3];

    //update in ISR
    uint32_t    step[3];
    uint32_t    step_dwell;
    uint8_t     flag;
}block_t;

enum {block_ready,  //ready to be executed
    block_exe,      //is executing
    block_free,     //has been executed and is ready to be planned
    block_busy,     //planning
    };

typedef struct
{
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t length;
    volatile block_t content[RINGBUFF_LEN];
}block_buff_t;

typedef struct 
{
    //feedback information
    float   xyz[3];
    float   abc[3];
    uint8_t state;

    //commanding tempt
    float   xyz_c[3];//current
    float   xyz_t[3];//target
    float   feedrate;
    uint8_t command_flag;

    //flags
    uint8_t interpret_flag;
    uint8_t fk_flag;
    uint8_t traj_flag;

    //update by interrupt
    int32_t carriage_move[3];

}machine_t;

enum machine_state
{
    machine_OFF,
    machine_ON,
    machine_RESET,
    machine_ERROR
};

/**************************BSP_BUFF*************************/

enum{stepper_OFF, stepper_ON};

enum{carriage_DOWN, carriage_UP};

typedef struct _stepper
{
    //update in timer interrupt routine
    volatile uint8_t    pin_state;
    volatile uint8_t    pin_state_last;

    //only used as feedback or initialization
    volatile uint8_t    state;//compare
    volatile uint8_t    dir;
    volatile int32_t    freq;//steps per second

}stepper_t;

enum {read_block, exe_block, exe_dwell, update};



#endif
