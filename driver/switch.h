#ifndef __SWITCH_H
#define __SWITCH_H

#include "config.h"
#include "type.h"

extern machine_t machine;

extern gcode_list_t gcode_list;

extern block_buff_t block_list;

extern block_buff_t;

typedef struct 
{
    uint32_t        RCC_AHB1Periph_GPIOX;
    uint32_t        GPIO_PinX;
    GPIO_TypeDef*   GPIOX;
    uint8_t type;//normal open or normal closed

    uint8_t state;//close or open
}switch_t;

void Switch_Init(switch_t* switchX);

void Switch_Read_State(switch_t* switchX);

void Switch_EXTI_Init(void);

#endif