#include <Arduino.h>

class RX_PWM{
    public:
    RX_PWM(uint8_t pin);
    int16_t RX_PWM_Initialize(void);
    uint16_t Value (void);
}