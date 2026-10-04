/*
 * mpu6050.h
 *
 *  Created on: 2026年8月31日
 *      Author: wongsenyui
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"

/* 7-bit Device Address (AD0 connected to GND -> 0x68, shifted for HAL) */
#define MPU6050_I2C_ADDR         (0x68 << 1)

/* Register Map Definitions */
#define MPU6050_SMPLRT_DIV       0x19
#define MPU6050_CONFIG           0x1A
#define MPU6050_GYRO_CONFIG      0x1B
#define MPU6050_ACCEL_CONFIG     0x1C
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_TEMP_OUT_H       0x41
#define MPU6050_GYRO_XOUT_H      0x43
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_WHO_AM_I         0x75

/* Sensitivity Scale Factors */
#define MPU6050_ACCEL_SENS_2G    16384.0f   // LSB/g
#define MPU6050_ACCEL_SENS_4G    8192.0f    // LSB/g
#define MPU6050_ACCEL_SENS_8G    4096.0f    // LSB/g
#define MPU6050_ACCEL_SENS_16G   2048.0f    // LSB/g

#define MPU6050_GYRO_SENS_250    131.0f     // LSB/(deg/s)
#define MPU6050_GYRO_SENS_500    65.5f      // LSB/(deg/s)
#define MPU6050_GYRO_SENS_1000   32.8f      // LSB/(deg/s)
#define MPU6050_GYRO_SENS_2000   16.4f      // LSB/(deg/s)

typedef struct {
    /* Raw 16-bit 2's complement registers */
    int16_t accel_raw[3];
    int16_t temp_raw;
    int16_t gyro_raw[3];

    /* Scaled engineering units */
    float accel_g[3];      // Accelerations in g
    float temp_c;          // Temperature in Celsius
    float gyro_dps[3];     // Angular rates in deg/sec
} MPU6050_t;

/* API Functions */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data);

#endif /* MPU6050_H_ */