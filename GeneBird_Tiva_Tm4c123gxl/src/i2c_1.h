/*
 * i2c.h
 * 
 *  Created on: 13 April 2020
 */

#ifndef QUADCOPTER_I2C_H_
#define QUADCOPTER_I2C_H_

void initI2C(void);
void readI2C(uint8_t slave_addr, uint8_t reg, int *data);
void writeI2C(uint8_t slave_addr, uint8_t reg, uint8_t data);


#endif /* QUADCOPTER_I2C_H_ */
