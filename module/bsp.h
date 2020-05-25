#ifndef __BSP_H

#include "type.h"

extern uint8_t pulse_A;

extern uint8_t pulse_B;

extern uint8_t pulse_C;

/******************************STEPPER***********************************/

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
uint8_t Bsp_Stepper_Update(stepper_t* stepperX);

/***************************************MONITOR*********************************/

void Bsp_Monitor_Init(void);

/**************************************SWITCH*********************************/

#define NO  0
#define NC  1

void Bsp_Switch_Init(switch_t* switchX);

/**************************************LED***********************************/

void Bsp_LED_Init(led_t* LEDX);

void Bsp_LED_Update(led_t* LEDX);

/**************************************PROTOCAL***********************************/

void Bsp_UART_Init(uint32_t bound);

void Bsp_UART_Send(uint8_t* content, uint8_t len);

#endif
