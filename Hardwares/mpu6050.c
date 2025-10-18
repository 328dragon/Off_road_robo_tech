/*
 * mpu6050.c
 *
 *  Created on: Mar 5, 2022
 *      Author: LX
 */

#include "mpu6050.h"

extern I2C_HandleTypeDef hi2c2;
IMU_Parameter IMU_Data;

#define MPU6050_ADDR 0xD0

float gyrox, gyroy, gyroz, accelx, accely, accelz, temp;

void MPU6050_Init(void)
{
	uint8_t check,data;
	HAL_I2C_Mem_Read(&hi2c2 ,MPU6050_ADDR,MPU_DEVICE_ID_REG,1,&check ,1,0xff); //读设备

	if(check == 104)//如果是0x68
	{
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDR,MPU_PWR_MGMT1_REG,1,&data,1,0xff);
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDR,MPU_SAMPLE_RATE_REG,1,&data,1,0xff);
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDR,MPU_CFG_REG,1,&data,1,0xff);
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDR,MPU_GYRO_CFG_REG,1,&data,1,0xff);
	}
}


void MPU6050_GET_Data(void)
{
	uint8_t buf[14]={0};

	HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,1,buf,14,1000);

	accelx = (float) (((int16_t) (buf[0] << 8) + buf[1])/16384.0f);
	accely = (float) (((int16_t) (buf[2] << 8) + buf[3])/16384.0f);
	accelz = (float) (((int16_t) (buf[4] << 8) + buf[5])/16384.0f);
	temp = (float) (((int16_t) (buf[6] << 8) + buf[7])/340 + 36.53f);
	gyrox = (float) (((int16_t) (buf[8] << 8) + buf[9])/131.0f);
	gyroy = (float) (((int16_t) (buf[10] << 8) + buf[11])/131.0f);
	gyroz = (float) (((int16_t) (buf[12] << 8) + buf[13])/131.0f);

	IMU_Data.Accel_X = accelx - IMU_Data.offset_Accel_X;
	IMU_Data.Accel_Y = accely - IMU_Data.offset_Accel_Y;
	IMU_Data.Accel_Z = accelz - IMU_Data.offset_Accel_Z;
	IMU_Data.Temp = temp;
	IMU_Data.Gyro_X = gyrox - IMU_Data.offset_Gyro_X;
	IMU_Data.Gyro_Y = gyroy - IMU_Data.offset_Gyro_Y;
	IMU_Data.Gyro_Z = gyroz - IMU_Data.offset_Gyro_Z;
}


//#include "mpu6050.h"

//void MPU6050_Init(I2C_HandleTypeDef *hi2c)
//{
//    uint8_t data;

//    // 唤醒MPU6050
//    data = 0;
//    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

//    // 设置采样率
//    data = 0x07;
//    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

//    // 配置陀螺仪和加速度计
//    data = 0x00;
//    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

//    // 配置陀螺仪量程
//    data = 0x00;
//    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

//    // 配置加速度计量程
//    data = 0x00;
//    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

//    // 使能中断
//    data = 0x01;
//    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
//}
//void MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c, float *gx, float *gy, float *gz)
//{
//    uint8_t buffer[6];

//    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);

////    *gx = (int16_t)((buffer[0] << 8) | buffer[1]);
////    *gy = (int16_t)((buffer[2] << 8) | buffer[3]);
////    *gz = (int16_t)((buffer[4] << 8) | buffer[5]);
//	
//    *gx = (float)((buffer[0] << 8) | buffer[1]);
//    *gy = (float)((buffer[2] << 8) | buffer[3]);
//    *gz = (float)((buffer[4] << 8) | buffer[5]);
//}

//void MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az)
//{
//    uint8_t buffer[6];

//    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);

////    *ax = (int16_t)((buffer[0] << 8) | buffer[1]);
////    *ay = (int16_t)((buffer[2] << 8) | buffer[3]);
////    *az = (int16_t)((buffer[4] << 8) | buffer[5]);
//	
//    *ax = (float)((buffer[0] << 8) | buffer[1]);
//    *ay = (float)((buffer[2] << 8) | buffer[3]);
//    *az = (float)((buffer[4] << 8) | buffer[5]);
//}