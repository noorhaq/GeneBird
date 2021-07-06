/*
 * time.h
 *
 *  Created on: 13 April 2020
 */

#ifndef QUADCOPTER_TIME_H_
#define QUADCOPTER_TIME_H_


void Delay(uint32_t delay);

void initTime(void);

uint32_t millis(void);

uint32_t micros(void);

#endif /* QUADCOPTER_TIME_H_ */
