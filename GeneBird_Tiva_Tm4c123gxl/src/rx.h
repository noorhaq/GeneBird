/*
 * rx.h
 *
 *  Created on: 13 April 2020
 */
#ifndef QUADCOPTER_RX_H_
#define QUADCOPTER_RX_H_

typedef enum {
    RX_ROLL = 0,   // 0
    RX_PITCH,			 // 1
    RX_THROTTLE,	 // 2
    RX_YAW,					// 3
    RX_AUX1,			// 4
    RX_AUX2,			// 5
    RX_NUM_CHANNELS,
} rxChannel_e;

void initRX(void );

float getRXchannel(rxChannel_e channel);

int Armed(void);

#endif /* QUADCOPTER_RX_H_ */
