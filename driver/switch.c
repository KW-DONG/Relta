#include "switch.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "type.h"
#include "buffer.h"
#include "stepper.h"


void Switch_Read_State(switch_t* switchX)
{
    if (GPIO_ReadInputDataBit(switchX->GPIOX,switchX->GPIO_PinX) == 1)
    switchX->state = 0;
    else
    switchX->state = 1;
}


