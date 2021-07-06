/*
 * pwm.h
 *  Created on: 13 April 2020
 */

#ifndef QUADCOPTER_PWM_H_
#define QUADCOPTER_PWM_H_

void initPWM(void);

void writeMotorL(float value);
void writeMotorR(float value);
void writeMotorB(float value);
void writeMotorF(float value);

#endif /* QUADCOPTER_PWM_H_ */
