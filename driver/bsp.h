#ifndef __BSP_H
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "type.h"
#include "sys.h"

/******************************STEPPER***********************************/

#define START   1
#define STOP    0

typedef struct
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

/**
 * @brief Init the stepper motor
 * @param stepperX
 * @param arr
 * @note allocate IO, clock and corresponding pwm channel
 */
void Bsp_Stepper_Init(stepper_t* stepperX);

/**
 * @brief Update the motor
 * @param stepperX
 * @note set the motor state and speed as well as detecting the IO state
 */
void Bsp_Stepper_Update(stepper_t* stepperX);

/***************************************MONITOR*********************************/

typedef struct 
{
    TIM_TypeDef*    TIMX;
    uint32_t        RCC_APB1Periph_TIMX;
    uint8_t         TIMX_IRQn;
    uint32_t        arr;
    uint16_t        psc;

}monitor_t;

void Bsp_Monitor_Init(monitor_t* monitor);

/**************************************SWITCH*********************************/

#define NO  0
#define NC  1

typedef struct 
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

void Bsp_Switch_Init(switch_t* switchX);

/**************************************LED***********************************/

typedef struct
{
    uint32_t        RCC_AHB1Periph_GPIOX;
    uint32_t        GPIO_Pin_X;
    GPIO_TypeDef*   GPIOX;

    uint8_t         state;
}led_t;

void Bsp_LED_Init(led_t* LEDX);

void Bsp_LED_Update(led_t* LEDX);

/**************************************PROTOCAL***********************************/

void Bsp_UART_Init(uint32_t bound);

void Bsp_UART_Send(uint8_t* content, uint8_t len);

#endif