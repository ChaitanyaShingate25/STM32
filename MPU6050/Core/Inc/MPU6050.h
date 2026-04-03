/*
 * MPU6050.h
 *
 *  Created on: Jan 30, 2026
 *      Author: chaitanya
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include <main.h>
// Addresses
#define MPU6050 0x68 << 1
#define PWR_MGMT_1_REG   0x6B
#define WHO_AM_I_REG     0x75
#define Gyro_reg 0x1B
#define Accel_reg 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG  0x43

// Function Prototype
void MPU6050_init();
void MPU6050_ReadAccel(int16_t *accel_data);
void MPU6050_ReadGyro(int16_t *gyro_data);
void Callibrate_Accel(int16_t *offset);
void Callibrate_Gyro(int16_t *offset);
void Update_Gyro(float *Gx, float *Gy, float *Gz);
void MPU6050_GetAngles(int16_t *Accel_raw, int16_t *Gyro_raw, float *pitch, float *roll, float *yaw, float dt);
#endif /* INC_MPU6050_H_ */
