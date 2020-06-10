#include "buffer.h"
#include "delta.h"
#include "stepper.h"
#include "config.h"
#include "math.h"

//the functions will be called in the interrupt
//avoid using complex calulation and float calculation

void Micro_Step_Init(uint8_t ms)
{
    if (ms==2||ms==8|ms==16)    MS1_HIGH;
    else                        MS1_LOW;
    if (ms==4||ms==8|ms==16)    MS2_HIGH;
    else                        MS2_LOW;
    if (ms==16)                 MS3_HIGH;
    else                        MS3_LOW;
}

void Stepper_Init(volatile stepper_t* stepperX)
{
    stepperX->state = stepper_OFF;
    stepperX->dir = carriage_DOWN;
    stepperX->freq = 10;
    stepperX->pin_state = 0;
    stepperX->pin_state_last = 0;
}

void Stepper_A_Update(void)
{
    stepperA.freq    = STEPPER_A_FREQ;
    stepperA.dir     = STEPPER_A_DIR;
}

void Stepper_B_Update(void)
{
    stepperB.freq    = STEPPER_B_FREQ;
    stepperB.dir     = STEPPER_B_DIR;
}

void Stepper_C_Update(void)
{
    stepperC.freq    = STEPPER_C_FREQ;
    stepperC.dir     = STEPPER_C_DIR;
}

uint8_t Stepper_A_Pulse(void)
{
    uint8_t pulse;
    stepperA.pin_state = STEPPER_A_SCAN;
    if (stepperA.pin_state==1&&stepperA.pin_state_last==0)
    pulse = 1;
    else
    pulse = 0;
    stepperA.pin_state_last = stepperA.pin_state;
    return pulse;
}

uint8_t Stepper_B_Pulse(void)
{
    uint8_t pulse;
    stepperB.pin_state = STEPPER_B_SCAN;
    if (stepperB.pin_state==1&&stepperB.pin_state_last==0)
    pulse = 1;
    else
    pulse = 0;
    stepperB.pin_state_last = stepperB.pin_state;
    return pulse;
}

uint8_t Stepper_C_Pulse(void)
{
    uint8_t pulse;
    stepperC.pin_state = STEPPER_C_SCAN;
    if (stepperC.pin_state==1&&stepperC.pin_state_last==0)
    pulse = 1;
    else
    pulse = 0;
    stepperC.pin_state_last = stepperC.pin_state;
    return pulse;
}
