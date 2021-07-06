/*
 * mpu.h
 *
  *  Created on: 13 April 2020
 */

#ifndef QUADCOPTER_MPU6050_H_
#define QUADCOPTER_MPU6050_H_

void MPUtestConnection(void);
void initMPU6050(void);
void readMPU(void);
float getAccZ(void);
float getMPUangleY(void);
float getMPUangleX(void);
float getGyroX(void);
float getGyroY(void);
float getGyroZ(void);
int dataReadyMPU(void);
void calibrateMPU(void);

#endif /* QUADCOPTER_MPU6050_H_ */
