#include "Driver_LED.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void LED_Set(uint32_t LEDNum)
{
    BSP_LED_Set(LEDNum);
}

void LED_Reset(uint32_t LEDNum)
{
    BSP_LED_Reset(LEDNum);
}

//模拟单电机运行
void LED_Test(void)
{
    switch(move_X)
    {
        case 0: BSP_LED_Reset(0);
        case 1:
        {
            BSP_LED_Set(1);
            BSP_LED_Set(0);
            delay_ms(100);
            BSP_LED_Reset(1);
            BSP_LED_Reset(0);
        }
        case 2:
        {
            BSP_LED_Reset(1);
            BSP_LED_Set(0);
            delay_ms(100);
            BSP_LED_Reset(0);
        }
    }
}