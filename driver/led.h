#ifndef __LED_H
#define __LED_H
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "stm32f4xx.h"

typedef struct
{
    uint32_t        RCC_AHB1Periph_GPIOX;
    uint32_t        GPIO_Pin_X;
    GPIO_TypeDef*   GPIOX;

    uint8_t         state;
}led_t;

void LED_Init(led_t* LEDX);

void LED_Update(led_t* LEDX);

#endif