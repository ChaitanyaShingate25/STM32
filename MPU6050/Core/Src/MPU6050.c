#include "MPU6050.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

void MPU6050_init()
{
	uint8_t data;

	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050, 0x6B, 1, &data, 1, HAL_MAX_DELAY);

	data = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050, Accel_reg, 1, &data, 1, HAL_MAX_DELAY);

	data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050, Gyro_reg, 1, &data, 1, HAL_MAX_DELAY);
}

void MPU6050_ReadAccel(int16_t *accel_data)
{
	uint8_t rec_data[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050, 0x3B, 1, rec_data, 6, HAL_MAX_DELAY);

    accel_data[0] = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    accel_data[1] = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    accel_data[2] = (int16_t)(rec_data[4] << 8 | rec_data[5]);
}

void MPU6050_ReadGyro(int16_t *gyro_data)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050, 0x43, 1, Rec_Data, 6, 1000); // GYRO_XOUT_H

    gyro_data[0] = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    gyro_data[1] = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    gyro_data[2] = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

void Callibrate_Accel(int16_t *offset)
{
	char msg[25];
	sprintf(msg, "Callibrating Accel\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	int32_t sum[3] = {0};
	int16_t data[3];
	int16_t i = 0;
	for (i = 0; i < 2000; i++)
	{
		MPU6050_ReadAccel(data);
		sum[0] += data[0];
		sum[1] += data[1];
		sum[2] += data[2];
		HAL_Delay(2);
	}

	offset[0] = sum[0] / 2000;
	offset[1] = sum[1] / 2000;
	offset[2] = sum[2] / 2000;
}

void Callibrate_Gyro(int16_t *offset)
{
	char msg[20];
	sprintf(msg, "Callibrating Gyro\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	int32_t sum[3] = {0};
	int16_t data[3];
	int16_t i = 0;
	for (i = 0; i < 2000; i++)
	{
		MPU6050_ReadGyro(data);
		sum[0] += data[0];
		sum[1] += data[1];
		sum[2] += data[2];
		HAL_Delay(2);
	}

	offset[0] = sum[0] / 2000;
	offset[1] = sum[1] / 2000;
	offset[2] = sum[2] / 2000;
}

void Update_Gyro(float *Gx, float *Gy, float *Gz)
{
	float gyro_deadband = 0.8;

	if(fabs(*Gx) < gyro_deadband) *Gx = 0;
	if(fabs(*Gy) < gyro_deadband) *Gy = 0;
	if(fabs(*Gz) < gyro_deadband) *Gz = 0;
}

void MPU6050_GetAngles(int16_t *Accel_raw, int16_t *Gyro_raw, float *pitch, float *roll, float *yaw, float dt)
{

    float Ax = Accel_raw[0] / 8192.0f;
    float Ay = Accel_raw[1] / 8192.0f;
    float Az = Accel_raw[2] / 8192.0f;

    float Gx = Gyro_raw[0] / 65.5f;
    float Gy = Gyro_raw[1] / 65.5f;
    float Gz = Gyro_raw[2] / 65.5f;

    float accelRoll  = atan2f(Ay, Az) * 57.2958f;
    float accelPitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 57.2958f;

    float alpha = 0.98f;

    *roll  = alpha * (*roll + Gx * dt) + (1.0f - alpha) * accelRoll;
    *pitch = alpha * (*pitch + Gy * dt) + (1.0f - alpha) * accelPitch;
    *yaw += Gz * dt;
}
