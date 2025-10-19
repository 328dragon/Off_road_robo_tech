/*
 * mpu6050.c
 *
 *  Created on: Mar 5, 2022
 *      Author: LX
 */

#include "mpu6050.h"

/**********默认0xD0地址**********/
// #define MPU6050_ADDR 0xD0
// extern I2C_HandleTypeDef hi2c2;
MPU6050_Handle_t mpu6050_dragon;

void MPU6050_Handle_Init(I2C_HandleTypeDef *i2c_handle, // 该实例使用的I2C句柄
						 uint8_t dev_addr,				// 该实例的设备地址（0xD0或0xD2，取决于AD0引脚）
						 MPU6050_Handle_t *mpu6050_handle)
{
	mpu6050_handle->i2c_handle = i2c_handle;
	mpu6050_handle->dev_addr = dev_addr;
	uint8_t check, data;
	HAL_I2C_Mem_Read(i2c_handle, dev_addr, MPU_DEVICE_ID_REG, 1, &check, 1, 0xff); // 读设备

	if (check == 104) // 如果是0x68
	{
		data = 0x00;
		HAL_I2C_Mem_Write(i2c_handle, dev_addr, MPU_PWR_MGMT1_REG, 1, &data, 1, 0xff);
		data = 0x07;
		HAL_I2C_Mem_Write(i2c_handle, dev_addr, MPU_SAMPLE_RATE_REG, 1, &data, 1, 0xff);
		data = 0x00;
		HAL_I2C_Mem_Write(i2c_handle, dev_addr, MPU_CFG_REG, 1, &data, 1, 0xff);
		data = 0x00;
		HAL_I2C_Mem_Write(i2c_handle, dev_addr, MPU_GYRO_CFG_REG, 1, &data, 1, 0xff);
	}
}

void MPU6050_Handle_GET_Data(MPU6050_Handle_t *mpu6050_handle)
{
	uint8_t buf[14] = {0};

	HAL_I2C_Mem_Read(mpu6050_handle->i2c_handle, mpu6050_handle->dev_addr, MPU_ACCEL_XOUTH_REG, 1, buf, 14, 1000);

	float accelx = (float)(((int16_t)(buf[0] << 8) + buf[1]) / 16384.0f);
	float accely = (float)(((int16_t)(buf[2] << 8) + buf[3]) / 16384.0f);
	float accelz = (float)(((int16_t)(buf[4] << 8) + buf[5]) / 16384.0f);
	float temp = (float)(((int16_t)(buf[6] << 8) + buf[7]) / 340 + 36.53f);
	float gyrox = (float)(((int16_t)(buf[8] << 8) + buf[9]) / 131.0f);
	float gyroy = (float)(((int16_t)(buf[10] << 8) + buf[11]) / 131.0f);
	float gyroz = (float)(((int16_t)(buf[12] << 8) + buf[13]) / 131.0f);

	mpu6050_handle->_imu_data.Accel_X = accelx - mpu6050_handle->_imu_data.offset_Accel_X;
	mpu6050_handle->_imu_data.Accel_Y = accely - mpu6050_handle->_imu_data.offset_Accel_Y;
	mpu6050_handle->_imu_data.Accel_Z = accelz - mpu6050_handle->_imu_data.offset_Accel_Z;
	mpu6050_handle->_imu_data.Temp = temp;
	mpu6050_handle->_imu_data.Gyro_X = gyrox - mpu6050_handle->_imu_data.offset_Gyro_X;
	mpu6050_handle->_imu_data.Gyro_Y = gyroy - mpu6050_handle->_imu_data.offset_Gyro_Y;
	mpu6050_handle->_imu_data.Gyro_Z = gyroz - mpu6050_handle->_imu_data.offset_Gyro_Z;
}
