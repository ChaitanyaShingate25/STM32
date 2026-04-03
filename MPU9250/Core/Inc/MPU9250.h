/*
 * MPU9250.h
 *
 *  Created on: Mar 6, 2026
 *      Author: chaitanya
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include <stdint.h>
#include "main.h"

// Device Addresses
#define MPU9250_ADDR         (0x68 << 1)
#define AK8963_ADDR          (0x0C << 1) // Magnetometer address

// Register Map
#define PWR_MGMT_1           0x6B
#define INT_PIN_CFG          0x37
#define ACCEL_CONFIG         0x1C
#define GYRO_CONFIG          0x1B
#define ACCEL_XOUT_H         0x3B
#define GYRO_XOUT_H          0x43

// AK8963 Registers
#define AK8963_CNTL          0x0A
#define AK8963_HXL           0x03 // Mag data starts here
#define AK8963_ASAX          0x10 // Sensitivity adjustment

// Function Prototypes
void MPU9250_Init();
void MPU9250_ReadAccel(int16_t *accel_data);
void MPU9250_ReadGyro(int16_t *gyro_data);
void MPU9250_ReadMag(int16_t *mag_data);

void Calibrate_Accel(int16_t *offset);
void Calibrate_Gyro(int16_t *offset);
void Calibrate_Mag(float *dest1, float *dest2);

void MPU9250_GetAngles(int16_t *Accel_raw, int16_t *Gyro_raw, int16_t *Mag_raw, float *pitch, float *roll, float *yaw, float dt);

#endif
