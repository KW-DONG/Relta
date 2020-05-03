#ifndef __LED_H
#define __LED_H
#include "sys.h"

#define LED0 PFout(9)	// DS0
#define LED1 PFout(10)	// DS1	 

void LED_Init(void);//Initialization
void LED_G00(int delay);
void LED_G01(int delay);
void LED_G02(int delay);
void LED_G03(int delay);

#endif