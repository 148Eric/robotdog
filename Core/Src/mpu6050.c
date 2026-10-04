/*
 * mpu6050.c
 *
 *  Created on: 2026年8月31日
 *      Author: wongsenyui
 */

#include "mpu6050.h"

/**
 * @brief Initialize MPU6050: wakes device, sets clock source, filters, and full-scale ranges.
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data;
    HAL_StatusTypeDef status;

    // 1. Verify Device ID (WHO_AM_I should return 0x68)
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, 100);
    if (status != HAL_OK || check != 0x68) {
        return HAL_ERROR;
    }

    // 2. Wake device up and select PLL with X-axis gyro reference (CLKSEL = 1)
    data = 0x01;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, 100);
    if (status != HAL_OK) return status;

    // 3. Set Sample Rate Divider: Sample Rate = 1kHz / (1 + 7) = 125 Hz
    data = 0x07;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, 100);
    if (status != HAL_OK) return status;

    // 4. Configure DLPF: DLPF_CFG = 3 (Accel Bandwidth = 44Hz, Gyro Bandwidth = 42Hz)
    data = 0x03;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_CONFIG, 1, &data, 1, 100);
    if (status != HAL_OK) return status;

    // 5. Accelerometer Configuration: AFS_SEL = 0 (+/- 2g)
    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, 100);
    if (status != HAL_OK) return status;

    // 6. Gyroscope Configuration: FS_SEL = 0 (+/- 250 dps)
    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, 100);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
 * @brief Reads all 14 sensor registers (Accel, Temp, Gyro) in a single burst transaction.
 */
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data) {
    uint8_t raw_buf[14];
    HAL_StatusTypeDef status;

    // Read 14 bytes starting from ACCEL_XOUT_H (0x3B)
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, MPU6050_ACCEL_XOUT_H, 1, raw_buf, 14, 100);
    if (status != HAL_OK) {
        return status;
    }

    // Parse Accelerometer (16-bit MSB first)
    data->accel_raw[0] = (int16_t)((raw_buf[0] << 8) | raw_buf[1]);
    data->accel_raw[1] = (int16_t)((raw_buf[2] << 8) | raw_buf[3]);
    data->accel_raw[2] = (int16_t)((raw_buf[4] << 8) | raw_buf[5]);

    // Parse Temperature
    data->temp_raw     = (int16_t)((raw_buf[6] << 8) | raw_buf[7]);

    // Parse Gyroscope
    data->gyro_raw[0]  = (int16_t)((raw_buf[8] << 8) | raw_buf[9]);
    data->gyro_raw[1]  = (int16_t)((raw_buf[10] << 8) | raw_buf[11]);
    data->gyro_raw[2]  = (int16_t)((raw_buf[12] << 8) | raw_buf[13]);

    // Convert to floating-point values (+/-2g and +/-250 deg/s scaling)
    data->accel_g[0] = (float)data->accel_raw[0] / MPU6050_ACCEL_SENS_2G;
    data->accel_g[1] = (float)data->accel_raw[1] / MPU6050_ACCEL_SENS_2G;
    data->accel_g[2] = (float)data->accel_raw[2] / MPU6050_ACCEL_SENS_2G;

    // Standard temperature formula: Temp_degC = (TEMP_OUT / 340) + 36.53
    data->temp_c     = ((float)data->temp_raw / 340.0f) + 36.53f;

    data->gyro_dps[0] = (float)data->gyro_raw[0] / MPU6050_GYRO_SENS_250;
    data->gyro_dps[1] = (float)data->gyro_raw[1] / MPU6050_GYRO_SENS_250;
    data->gyro_dps[2] = (float)data->gyro_raw[2] / MPU6050_GYRO_SENS_250;

    return HAL_OK;
}