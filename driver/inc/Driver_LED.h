#ifndef __DRIVER_LED_H
#define __DRIVER_LED_H
#include "sys.h"
#include "Driver_BSP.h"

#define LED0 0
#define LED1 1

void LED_Set(uint32_t LEDNum);

void LED_Reset(uint32_t LEDNum);

void LED_Test(void);

#endif