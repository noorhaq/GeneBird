/*
 * buzzer.c
 *
 *  Created on: 13 April 2020
 */

#ifndef BUZZER_C_
#define BUZZER_C_

void initBuzzer(void);
void BuzzerShort(void);
void BuzzerLong(void);
void GreenLed(bool enable);
void RedLed(bool enable);
void BlueLed(bool enable);
void MixLed(int R, int G, int B);

#endif /* BUZZER_C_ */
