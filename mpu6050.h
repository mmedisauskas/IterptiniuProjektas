#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;

void mpu6050_init(void);
void MPU6050_Read_Accel(float *Ax, float *Ay, float *Az);
void MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz);

#endif /* MPU6050_H */