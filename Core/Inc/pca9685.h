/*
 * pca9685.h
 *
 *  Created on: 2026年8月2日
 *      Author: wongsenyui
 */

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_

#include "i2c.h"

// PCA9685 Register Definitions
#define PCA9685_ADDR      (0x40 << 1) // Default 7-bit 0x40 shifted left for HAL (0x80)
#define PCA9685_MODE1     0x00
#define PCA9685_PRESCALE  0xFE
#define LED0_ON_L         0x06

// Function Prototypes
void PCA9685_Init(I2C_HandleTypeDef *hi2c);
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on_tick, uint16_t off_tick);
void PCA9685_SetServoAngle(I2C_HandleTypeDef *hi2c, uint8_t channel, float angle);
void PCA9685_SetServoAngleSmooth(I2C_HandleTypeDef *hi2c, uint8_t channel, uint8_t start_angle, uint8_t target_angle, uint8_t speed_delay_ms);

#endif /* INC_PCA9685_H_ */
