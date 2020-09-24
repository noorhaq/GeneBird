#ifndef RX_PWM_INPUT_H
#define RX_PWM_INPUT_H
#include <Arduino.h>
#
class RX_PWM{
    byte curr_isr;
    public:
    RX_PWM(uint8_t pin);
    int16_t RX_PWM_Initialize(void);
    uint16_t Value (void);
    ~RX_PWM(void);
};
#endif