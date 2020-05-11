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

#undef  SQ(x)
#define SQ(x)   ((x)*(x))
#define COS(x)  (cosf(x))
#define SIN(x)  (sinf(x))
#define INV(x)  (1.0f / (x))
#define E(x)    (powf(10.0f,x))
#define PI      3.14f

#define RINGBUFF_LEN 200
#define LINKED_LIST_LEN 200

#define TRUE 1
#define FALSE 0

#define FULL    2
#define HALF    4
#define QURT    8
#define EIGH    16

/****************************BUFF_TYPE************************/

/**
 * radius>0 G2
 * radius<0 G3
 * xyz!=0 && radius=0 && feedrate!=0 G1
 * xyz!=0 && radius=0 && feedrate==0 G0
 * 
 * radius_dwell depend on the type
 */

typedef struct _node
{
    float   x,y,z,
            feedrate,
            radius_dwell;
    uint8_t coordinate_sys;
    uint8_t type;
    struct _node* next;
}gcode_node_t;

typedef struct 
{
    gcode_node_t*       head;
    gcode_node_t*       tail;
    uint32_t            length;
}gcode_list_t;

typedef struct 
{
    uint16_t    head;
    uint16_t    tail;
    uint16_t    length;
    uint8_t     content[100];
}uart_buff_t;

typedef struct 
{
    uint8_t     dir[3];
    int32_t    freq[3];//pps
    uint16_t    step[3];
    uint32_t    step_dwell;//calculated with monitor frequency
}block_t;

typedef struct
{
    uint16_t head;
    uint16_t tail;
    uint16_t length;
    block_t* Block_Buff[RINGBUFF_LEN];
}block_buff_t;

typedef struct 
{
    float   xyz[3];
    float   abc[3];
    float   xyz_v[3];
    float   feedrate;
    uint8_t state;
}machine_t;

enum machine_state
{
    machine_ON,
    machine_OFF,
    machine_RESET,
    machine_ERROR
};

/**************************BSP_BUFF*************************/
enum stepper_state
{
    stepper_ON,
    stepper_OFF
};

typedef struct _stepper
{
    //A4988 set pin
    uint32_t        RCC_AHB1Periph_GPIOX;
    uint32_t        GPIO_Pin_X;
    GPIO_TypeDef*   GPIOX;

    //pwm
    uint32_t        arr;
    uint32_t        RCC_APB1Periph_TIMX;
    uint16_t        GPIO_PinSourceX;
    uint8_t         GPIO_AF_TIMX;
    uint8_t         PWM_Ch;

    TIM_TypeDef*    TIMX;

    //A4988 dir pin
    uint32_t        RCC_AHB1Periph_GPIOX_Dir;
    uint32_t        GPIO_Pin_X_Dir;
    GPIO_TypeDef*   GPIOX_Dir;

    //A4988 MS1
    uint32_t        RCC_AHB1Periph_GPIOX_MS1;
    uint32_t        GPIO_Pin_X_MS1;
    GPIO_TypeDef*   GPIOX_MS1;
    
    //A4988 MS2
    uint32_t        RCC_AHB1Periph_GPIOX_MS2;
    uint32_t        GPIO_Pin_X_MS2;
    GPIO_TypeDef*   GPIOX_MS2;

    //A4988 MS3
    uint32_t        RCC_AHB1Periph_GPIOX_MS3;
    uint32_t        GPIO_Pin_X_MS3;
    GPIO_TypeDef*   GPIOX_MS3;

    uint8_t         state;//compare
    uint8_t         pin_state;
    uint8_t         pin_state_last;
    uint8_t         dir;
    int32_t         freq;//steps per second
    uint16_t        psc;

}stepper_t;

typedef struct _monitor
{
    TIM_TypeDef*    TIMX;
    uint32_t        RCC_APB1Periph_TIMX;
    uint8_t         TIMX_IRQn;
    uint32_t        arr;
    uint16_t        psc;

}monitor_t;

typedef struct _switch
{
    //gpio
    uint32_t        RCC_AHB1Periph_GPIOX;
    uint32_t        GPIO_PinX;
    GPIO_TypeDef*   GPIOX;

    //exti
    uint32_t        EXTI_LineX;
    uint8_t         EXTI_PortSourceGPIOX;
    uint8_t         EXTI_PinSourceX;
    uint8_t         EXTIX_IRQn;

    //nvic
    uint8_t         NVIC_PP;
    uint8_t         NVIC_SP;

    //NO or NC
    uint8_t         mode;

    uint8_t state;//close or open
}switch_t;

typedef struct _led
{
    uint32_t        RCC_AHB1Periph_GPIOX;
    uint32_t        GPIO_Pin_X;
    GPIO_TypeDef*   GPIOX;

    uint8_t         state;
}led_t;


#endif