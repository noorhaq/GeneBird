/*
 * pid.h
 *
 *  Created on: 13 April 2020
 */

#ifndef PID_H_
#define PID_H_

float ratePID(float setpoint, float dt, int axle);
float stabilizePID(float setpoint, float dt, int axle);



#endif /* PID_H_ */
