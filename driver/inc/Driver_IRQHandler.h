#ifndef __DRIVER_INTERRUPT_
#define __DRIVER_INTERRUPT_

#include "sys.h"
#include "Driver_BSP.h"
#include "Driver_Gcode.h"

void EXTI0_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void USART1_IRQHandler(void);
void TIM3_IRQHandler(void); //update motor angles



#endif