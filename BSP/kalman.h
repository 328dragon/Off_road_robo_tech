/*
 * kalman.h
 *
 *  Created on: Mar 21, 2022
 *      Author: LX
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "main.h"
#include <stdint.h>
#include "i2c.h"
#include "mpu6050.h"

#ifdef __cplusplus
extern "C"
{
#endif
    // 开始

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;



void MPU6050_Read_All(IMU_Parameter *IMU_Data);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

	#ifdef __cplusplus
}
#endif


#endif /* KALMAN_H_ */

