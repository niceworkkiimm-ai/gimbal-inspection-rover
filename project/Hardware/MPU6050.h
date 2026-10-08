#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h"
#include <stdint.h>

typedef struct
{
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
    double Ax, Ay, Az;
    int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
    double Gx, Gy, Gz;
    float Temperature;
    double KalmanAngleX, KalmanAngleY;
} MPU6050_t;

typedef struct
{
    double Q_angle, Q_bias, R_measure;
    double angle, bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(void);
void MPU6050_Read_Accel(MPU6050_t *d);
void MPU6050_Read_Gyro(MPU6050_t *d);
void MPU6050_Read_Temp(MPU6050_t *d);
void MPU6050_Read_All(MPU6050_t *d);
double Kalman_getAngle(Kalman_t *K, double newAngle, double newRate, double dt);

#endif
