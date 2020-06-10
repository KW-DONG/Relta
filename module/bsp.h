#ifndef __BSP_H

#include "type.h"

void Bsp_Microstep_force(void);

void Bsp_Stepper_Init(void);

void Bsp_Switch_Init(void);

void Bsp_EXTI0_Init(void);

void Bsp_EXTI1_Init(void);

void Bsp_EXTI2_Init(void);

void Bsp_EXTI3_Init(void);

void Bsp_EXTI4_Init(void);

void Bsp_TIM2_PWM_Init(uint32_t psc);

void Bsp_TIM3_PWM_Init(uint32_t psc);

void Bsp_TIM4_PWM_Init(uint32_t psc);

void TIM2_PWM_Detect(void);

void TIM3_PWM_Detect(void);

void TIM4_PWM_Detect(void);

void Bsp_TIM5_Init(uint32_t psc);

void Bsp_LED_Init(void);

void Bsp_KEY_Init(void);

/**************************************PROTOCAL***********************************/

void Bsp_UART_Init(uint32_t bound);

void Bsp_UART_Send(uint8_t* content, uint8_t len);

#endif
