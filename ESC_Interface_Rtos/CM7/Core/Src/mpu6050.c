/*
 * mpu6050.c
 *
 *  Created on: Nov 24, 2024
 *      Author: javim_1oc1nxl
 */



 /* MPUXX50.c
 *
 *  Created on: Feb 27, 2022
 *      Author: MarkSherstan
 */

#include "mpu6050.h"
#include <stdio.h>
#include "myprintf.h"

// Structures
struct RawData
{
    int16_t ax, ay, az, gx, gy, gz;
} rawData;


typedef union {
	int32_t raw_int;
	float raw_float;
} raw_conv_type;

struct SensorData
{
    float ax, ay, az, gx, gy, gz;
} sensorData;

struct GyroCal
{
    float x, y, z;
} gyroCal;

struct Attitude
{
    float r, p, y;
} attitude;

// Variables
static uint8_t mpu6050_addr;
static float dt_val, tau_val;
static float aScaleFactor, gScaleFactor;

/// @brief Set the IMU address, check for connection, reset IMU, and set full range scale.
/// @param I2Cx Pointer to I2C structure config.
/// @param addr Hex address based on AD0 pin - 0x68 low or 0x69 high.
/// @param aScale Set accelerometer full scale range: 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
/// @param gScale Set gyroscope full scale range: 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
/// @param tau Set tau value for the complementary filter (typically 0.98).
/// @param dt Set sampling rate in seconds determined by the timer interrupt.

void mpu6050_init(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt)
{
    uint8_t select =0;
	// Save values
	mpu6050_addr = addr << 1;
    tau_val = tau;
    dt_val = dt;


    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(I2Cx, (mpu6050_addr + 0), 1, I2C_TIMOUT_MS);
    if (ret == HAL_OK){
  	  printf("The device is OK\n\r");
    }
    else {
  	  printf("The device is not ready \n\r");
    }

    // Quit sleep mode and enable temperature sensor
    select = 0x00;
    ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr + 0), PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);
    if (ret == HAL_OK){
  	  printf("Out of sleep mode and temp sensor on is OK\n\r");
    }
    else {
  	  printf("sleep mode and temp sensor error \n\r");
    }

    // Set the full scale ranges
    ret = MPU_writeAccFullScaleRange(I2Cx, aScale);
    if (ret == HAL_OK){
  	  printf("Acc scale is OK\n\r");
    }
    else {
  	  printf("Acc scale not ready \n\r");
    }


    ret = MPU_writeGyroFullScaleRange(I2Cx, gScale);
    if (ret == HAL_OK){
  	  printf("Gyro scale is OK\n\r");
    }
    else {
  	  printf("Gyro scale not ready \n\r");
    }

}

/// @brief Set the accelerometer full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.

HAL_StatusTypeDef MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale)
{
    //Variable init
    uint8_t select;
    HAL_StatusTypeDef ret = HAL_OK;

    // Set the value
    switch (aScale) {
    case AFSR_2G:
        aScaleFactor = 16384.0;
        select = 0x00;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_4G:
        aScaleFactor = 8192.0;
        select = 0x08;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_8G:
        aScaleFactor = 4096.0;
        select = 0x10;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_16G:
        aScaleFactor = 2048.0;
        select = 0x18;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        aScaleFactor = 8192.0;
        select = 0x08;
        ret =  HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
    return ret;
}

/// @brief Set the gyroscope full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.


HAL_StatusTypeDef MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale)
{
    // Variable init
    uint8_t select;
    HAL_StatusTypeDef ret = HAL_OK;

    // Set the value
    switch (gScale)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.0;
        select = 0x00;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.5;
        select = 0x08;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.8;
        select = 0x10;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.4;
        select = 0x18;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gScaleFactor = 65.5;
        select = 0x08;
        ret = HAL_I2C_Mem_Write(I2Cx, (mpu6050_addr+0), GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }

  return ret;
}

/// @brief Read raw data from IMU.
/// @param I2Cx Pointer to I2C structure config.

void MPU_readRawData(I2C_HandleTypeDef *I2Cx)
{
    // Init buffer
    uint8_t buf[14];



    // Subroutine for reading the raw data
    HAL_I2C_Mem_Read(I2Cx, (mpu6050_addr+1), ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    // Bit shift the data
    rawData.ax = buf[0] << 8 | buf[1];
    rawData.ay = buf[2] << 8 | buf[3];
    rawData.az = buf[4] << 8 | buf[5];
    // temperature = buf[6] << 8 | buf[7];
    rawData.gx = buf[8] << 8 | buf[9];
    rawData.gy = buf[10] << 8 | buf[11];
    rawData.gz = buf[12] << 8 | buf[13];

    //printf("X-axis accelerometer is ax,ay,az = [%f, %f, %f]\n\r", (float)rawData.ax, (float)rawData.ay, (float)rawData.az);
    //printf("X-axis gyroscope is     gx,gy,gz = [%f, %f, %f]\n\r", (float)rawData.gx,(float)rawData.gy,(float)rawData.gz);
}

/// @brief Find offsets for each axis of gyroscope.
/// @param I2Cx Pointer to I2C structure config.
/// @param numCalPoints Number of data points to average.

//void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
//{
//    // Init
//    int32_t x = 0;
//    int32_t y = 0;
//    int32_t z = 0;
//
//    // Zero guard
//    if (numCalPoints == 0)
//    {
//        numCalPoints = 1;
//    }
//
//    // Save specified number of points
//    for (uint16_t ii = 0; ii < numCalPoints; ii++)
//    {
//        MPU_readRawData(I2Cx);
//        x += rawData.gx;
//        y += rawData.gy;
//        z += rawData.gz;
//        HAL_Delay(3);
//    }
//
//    // Average the saved data points to find the gyroscope offset
//    gyroCal.x = (float)x / (float)numCalPoints;
//    gyroCal.y = (float)y / (float)numCalPoints;
//    gyroCal.z = (float)z / (float)numCalPoints;
//}

/// @brief Calculate the real world sensor values.
/// @param I2Cx Pointer to I2C structure config.

void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx)
{
    // Get raw values from the IMU
    MPU_readRawData(I2Cx);

    gyroCal.x = 0;
    gyroCal.y = 0;
    gyroCal.z = 0;

    // Convert accelerometer values to g's
    sensorData.ax = rawData.ax / aScaleFactor;
    sensorData.ay = rawData.ay / aScaleFactor;
    sensorData.az = rawData.az / aScaleFactor;

    // Compensate for gyro offset
    sensorData.gx = rawData.gx - gyroCal.x;
    sensorData.gy = rawData.gy - gyroCal.y;
    sensorData.gz = rawData.gz - gyroCal.z;

    // Convert gyro values to deg/s
    sensorData.gx /= gScaleFactor;
    sensorData.gy /= gScaleFactor;
    sensorData.gz /= gScaleFactor;

    printf("\n\rX-axis accelerometer is ax,ay,az = [%f, %f, %f]", sensorData.ax, sensorData.ay, sensorData.az);
    printf("\n\rX-axis gyroscope is gx,gy,gz = [%f, %f, %f]", sensorData.gx, sensorData.gy, sensorData.gz);
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter.
/// @param I2Cx Pointer to I2C structure config.

//void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx)
//{
//    // Read processed data
//    MPU_readProcessedData(I2Cx);
//
//    // Complementary filter
//    float accelPitch = atan2(sensorData.ay, sensorData.az) * RAD2DEG;
//    float accelRoll = atan2(sensorData.ax, sensorData.az) * RAD2DEG;
//
//    attitude.r = tau_val * (attitude.r - sensorData.gy * dt_val) + (1 - tau_val) * accelRoll;
//    attitude.p = tau_val * (attitude.p + sensorData.gx * dt_val) + (1 - tau_val) * accelPitch;
//    attitude.y += sensorData.gz * dt_val;
//}
