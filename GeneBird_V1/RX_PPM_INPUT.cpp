#include "RX_PPM_INPUT.h"
#include "config.h"
#include <Arduino.h>

uint8_t Pin = PPM_RX_INPUT
void  RX_PWM::RX_PWM_Initialize {   
            pinMode(Pin, INPUT_PULLUP)
            attachInterrupt(digitalPinToInterrupt(Pin), PPM_Value, RISING);
}

uint8_t RX_PWM::PPM_Value
