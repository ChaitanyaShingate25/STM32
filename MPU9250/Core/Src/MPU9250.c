/*
 * MPU9250.c
 *
 *  Created on: Mar 6, 2026
 *      Author: chaitanya
 */


#include "MPU9250.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

void MPU9250_Init() {
    uint8_t data;

    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1, 1, &data, 1, 100);

    data = 0x02;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, INT_PIN_CFG, 1, &data, 1, 100);

    data = 0x08; // +/- 4g
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, ACCEL_CONFIG, 1, &data, 1, 100);
    data = 0x08; // 500 dps
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, GYRO_CONFIG, 1, &data, 1, 100);

    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, AK8963_CNTL, 1, &data, 1, 100);
    HAL_Delay(10);
    data = 0x16; // 16-bit output, 100Hz continuous mode
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, AK8963_CNTL, 1, &data, 1, 100);
}

void MPU9250_ReadAccel(int16_t *accel_data)
{
    uint8_t rec_data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x3B, 1, rec_data, 6, HAL_MAX_DELAY);

    accel_data[0] = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    accel_data[1] = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    accel_data[2] = (int16_t)(rec_data[4] << 8 | rec_data[5]);
}

void MPU9250_ReadGyro(int16_t *gyro_data)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x43, 1, Rec_Data, 6, 1000);

    gyro_data[0] = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    gyro_data[1] = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    gyro_data[2] = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

void Calibrate_Accel(int16_t *offset)
{
    char msg[35];
    sprintf(msg, "Calibrating Accel... Keep Flat\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    int32_t sum[3] = {0};
    int16_t data[3];

    // Increase samples for better precision (optional)
    for (int i = 0; i < 2000; i++)
    {
        MPU9250_ReadAccel(data);
        sum[0] += data[0];
        sum[1] += data[1];
        // For Z-axis, we subtract 1g (8192 for +/- 4g range)
        // so the offset represents the error, not gravity itself.
        sum[2] += (data[2] - 8192);
        HAL_Delay(2);
    }

    offset[0] = (int16_t)(sum[0] / 2000);
    offset[1] = (int16_t)(sum[1] / 2000);
    offset[2] = (int16_t)(sum[2] / 2000);

    sprintf(msg, "Accel Calibrated!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void Calibrate_Gyro(int16_t *offset)
{
    char msg[35];
    sprintf(msg, "Calibrating Gyro... Do Not Move\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    int32_t sum[3] = {0};
    int16_t data[3];

    for (int i = 0; i < 2000; i++)
    {
        MPU9250_ReadGyro(data);
        sum[0] += data[0];
        sum[1] += data[1];
        sum[2] += data[2];
        HAL_Delay(2);
    }

    offset[0] = (int16_t)(sum[0] / 2000);
    offset[1] = (int16_t)(sum[1] / 2000);
    offset[2] = (int16_t)(sum[2] / 2000);

    sprintf(msg, "Gyro Calibrated!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void Update_Gyro(float *Gx, float *Gy, float *Gz)
{

    float gyro_deadband = 0.15f;

    if(fabs(*Gx) < gyro_deadband) *Gx = 0;
    if(fabs(*Gy) < gyro_deadband) *Gy = 0;
    if(fabs(*Gz) < gyro_deadband) *Gz = 0;
}

void MPU9250_ReadMag(int16_t *mag_data) {
    uint8_t rec_data[7]; // 6 data bytes + ST2 status byte

    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, AK8963_HXL, 1, rec_data, 7, 100);

    mag_data[0] = (int16_t)(rec_data[1] << 8 | rec_data[0]);
    mag_data[1] = (int16_t)(rec_data[3] << 8 | rec_data[2]);
    mag_data[2] = (int16_t)(rec_data[5] << 8 | rec_data[4]);
}

void Calibrate_Mag(float *mag_bias, float *mag_scale) {

    char msg[] = "Rotate sensor in figure-8 for 15s...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

    int16_t mag_temp[3];
    int32_t mag_max[3] = {-32767, -32767, -32767};
    int32_t mag_min[3] = {32767, 32767, 32767};

    for (int i = 0; i < 1500; i++) {
        MPU9250_ReadMag(mag_temp);
        for (int j = 0; j < 3; j++) {
            if (mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
            if (mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
        }
        HAL_Delay(10);
    }

    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2.0f;
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2.0f;
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2.0f;

    float avg_rad = ((mag_max[0] - mag_min[0]) + (mag_max[1] - mag_min[1]) + (mag_max[2] - mag_min[2])) / 6.0f;
    mag_scale[0] = avg_rad / ((mag_max[0] - mag_min[0]) / 2.0f);
    mag_scale[1] = avg_rad / ((mag_max[1] - mag_min[1]) / 2.0f);
    mag_scale[2] = avg_rad / ((mag_max[2] - mag_min[2]) / 2.0f);
}

void MPU9250_GetAngles(int16_t *Accel_raw, int16_t *Gyro_raw, int16_t *Mag_raw, float *pitch, float *roll, float *yaw, float dt)
{
    // 1. Convert Raw to Physical Values
    float Ax = Accel_raw[0] / 8192.0f;
    float Ay = Accel_raw[1] / 8192.0f;
    float Az = Accel_raw[2] / 8192.0f;

    float Gx = Gyro_raw[0] / 65.5f;
    float Gy = Gyro_raw[1] / 65.5f;
    float Gz = Gyro_raw[2] / 65.5f;

    // 2. Apply the Deadband Filter to stop drift
    Update_Gyro(&Gx, &Gy, &Gz);

    // 3. Pitch and Roll (Accel based)
    float accelRoll  = atan2f(Ay, Az) * 57.2958f;
    float accelPitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 57.2958f;

    // 4. Complementary Filter for Pitch and Roll
    float alpha = 0.98f;
    *roll  = alpha * (*roll + Gx * dt) + (1.0f - alpha) * accelRoll;
    *pitch = alpha * (*pitch + Gy * dt) + (1.0f - alpha) * accelPitch;

    // 5. Magnetometer Yaw Calculation (Tilt Compensated)
    float r_rad = *roll * 0.0174533f;
    float p_rad = *pitch * 0.0174533f;

    // Simplified Tilt Compensation
    float MX = Mag_raw[0] * cosf(p_rad) + Mag_raw[2] * sinf(p_rad);
    float MY = Mag_raw[0] * sinf(r_rad) * sinf(p_rad) + Mag_raw[1] * cosf(r_rad) - Mag_raw[2] * sinf(r_rad) * cosf(p_rad);

    float magYaw = atan2f(-MY, MX) * 57.2958f;

    *yaw = 0.95f * (*yaw + Gz * dt) + 0.05f * magYaw;
}
