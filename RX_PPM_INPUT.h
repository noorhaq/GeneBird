#ifndef RX_PPM_INPUT_H
#define RX_PPM_INPUT_H
#include <Arduino.h>

class RX_PPM{
    public:
    RX_PPM(uint8_t pin);
    void RX_PPM_Initialize(void);
    uint16_t PPM_Value(void);
    void PPM_test(void);
    ~RX_PPM(void);
}
#endif