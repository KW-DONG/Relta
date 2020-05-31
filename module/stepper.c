#include "buffer.h"
#include "delta.h"
#include "stepper.h"
#include "config.h"
#include "math.h"

//the functions will be called in the interrupt
//avoid using complex calulation and float calculation


void Stepper_Init(stepper_t* stepperX)
{
    stepperX->state = stepper_OFF;
    stepperX->dir = carriage_DOWN;
    stepperX->freq = 1;
    stepperX->pin_state = 0;
    stepperX->pin_state_last = 0;
}

void Stepper_A_Update(void)
{
    if (STEPPER_A_CCR!=0)   stepperA.state = stepper_ON;
    else                    stepperA.state = stepper_OFF;
    stepperA.freq    = STEPPER_A_FREQ;
    stepperA.dir     = STEPPER_A_DIR;
}

void Stepper_B_Update(void)
{
    if (STEPPER_B_CCR!=0)   stepperB.state = stepper_ON;
    else                    stepperB.state = stepper_OFF;
    stepperB.freq    = STEPPER_B_FREQ;
    stepperB.dir     = STEPPER_B_DIR;
}

void Stepper_C_Update(void)
{
    if (STEPPER_C_CCR!=0)   stepperC.state = stepper_ON;
    else                    stepperC.state = stepper_OFF;
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
