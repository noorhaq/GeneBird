#include "RX_INPUT.h"
#include "config.h"
#include <Arduino.h>
uint8_t Channels = TOTAL_RX_CHANNEL_NUMBER_PWM;


const byte MAX_ISR_COUNT = 10; 

byte isr_count = 0;
byte ISR_PIN[MAX_ISR_COUNT];
unsigned int isr_value[MAX_ISR_COUNT];
bool isr_last_state[MAX_ISR_COUNT];
bool isr_trigger_state[MAX_ISR_COUNT];
unsigned long isr_timer[MAX_ISR_COUNT];
unsigned long isr_age[MAX_ISR_COUNT];

void ISR(byte isr) {
    unsigned long now = micros();
    bool state_now = digitalRead(ISR_PIN[isr]);
    if (state_now != isr_last_state[isr]) {
        if (state_now == isr_trigger_state[isr]) {
            isr_timer[isr] = now;
        } else {
            isr_value[isr] = (unsigned int)(now - isr_timer[isr]);
            isr_age[isr] = now;
        }
        isr_last_state[isr] = state_now;
    }
}

void ISR_0() {
    ISR(0);
}

void ISR_1() {
    ISR(1);
}

void ISR_2() {
    ISR(2);
}

void ISR_3() {
    ISR(3);
}

void ISR_4() {
    ISR(4);
}

void ISR_5() {
    ISR(5);
}

void ISR_6() {
    ISR(6);
}

void ISR_7() {
    ISR(7);
}

void ISR_8() {
    ISR(8);
}

void ISR_9() {
    ISR(9);
}


RX_PWM::RX_PWM(byte pin) {
    curr_isr = isr_count;
    isr_count++;
    
    ISR_PIN[curr_isr] = pin;
    pinMode(ISR_PIN[curr_isr], INPUT);
}

int16_t  RX_PWM::RX_PWM_Initialize {
    isr_last_state[curr_isr] = digitalRead(ISR_PIN[curr_isr]);
    isr_trigger_state[curr_isr] = 1; //i.e 1: For Normal Invert Measurement set it to false
    
    switch (curr_isr) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_3, CHANGE);
            break;
        case 4:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_4, CHANGE);
            break;
        case 5:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_5, CHANGE);
            break;
        case 6:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_6, CHANGE);
            break;
        case 7:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_7, CHANGE);
            break;
        case 8:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_8, CHANGE);
            break;
        case 9:
            attachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]), ISR_9, CHANGE);
            break;
        default:
            return -1; // Error.
    }
    return 0; // Success.
}

unsigned int RX_PWM::getValue() {
    return isr_value[curr_isr];
}

void RX_PWM::~RX_PWM() {
    detachInterrupt(digitalPinToInterrupt(ISR_PIN[curr_isr]));
}
