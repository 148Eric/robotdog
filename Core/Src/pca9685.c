/*
 * pca9685.c
 *
 *  Created on: 2026年8月2日
 *      Author: wongsenyui
 */

#include "pca9685.h"

// Helper function to write 1 byte to a PCA9685 register
static void PCA9685_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

// Initialize PCA9685 for 50Hz Servo control
void PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
    // Put PCA9685 to sleep to allow writing to the PRE_SCALE register
    PCA9685_WriteReg(hi2c, PCA9685_MODE1, 0x10);

    // Set prescaler to 121 (0x79) for 50 Hz PWM refresh rate
    PCA9685_WriteReg(hi2c, PCA9685_PRESCALE, 0x79);

    // Wake up chip and enable Register Auto-Increment mode
    PCA9685_WriteReg(hi2c, PCA9685_MODE1, 0x20);
    HAL_Delay(5);
}

// Set raw ON and OFF pulse ticks (0 to 4095) for a specific channel
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on_tick, uint16_t off_tick)
{
    uint8_t base_reg = LED0_ON_L + (4 * channel);
    uint8_t data[4];

    data[0] = on_tick & 0xFF;         // ON Low Byte
    data[1] = (on_tick >> 8) & 0xFF;  // ON High Byte
    data[2] = off_tick & 0xFF;        // OFF Low Byte
    data[3] = (off_tick >> 8) & 0xFF; // OFF High Byte

    HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, base_reg, I2C_MEMADD_SIZE_8BIT, data, 4, HAL_MAX_DELAY);
}

// Convert 0-180 degrees directly to PCA9685 timing ticks
void PCA9685_SetServoAngle(I2C_HandleTypeDef *hi2c, uint8_t channel, float angle)
{
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    const float min_tick = 102.0f; // 0.5 ms for 0 degrees
    const float max_tick = 512.0f; // 2.5 ms for 180 degrees

    // Float math, rounded to the nearest tick
    uint16_t off_tick = (uint16_t)(min_tick + angle * (max_tick - min_tick) / 180.0f + 0.5f);

    PCA9685_SetPWM(hi2c, channel, 0, off_tick);
}

// void PCA9685_SetServoAngleSmooth(I2C_HandleTypeDef *hi2c, uint8_t channel, uint8_t start_angle, uint8_t target_angle, uint8_t speed_delay_ms)
// {
//     if (start_angle < target_angle)
//     {
//         for (int angle = start_angle; angle <= target_angle; angle++)
//         {
//             PCA9685_SetServoAngle(hi2c, channel, (uint8_t)angle);
//             HAL_Delay(speed_delay_ms); // Small delay controls movement speed (e.g., 10ms)
//         }
//     }
//     else
//     {
//         for (int angle = start_angle; angle >= target_angle; angle--)
//         {
//             PCA9685_SetServoAngle(hi2c, channel, (uint8_t)angle);
//             HAL_Delay(speed_delay_ms);
//         }
//     }
// }
/*
servo 0 right front knee 
servo 1 right front hip
servo 2 right front ankle
servo 3 left front ankle
servo 4 left front hip
servo 5 left front knee
servo 6 right back knee
servo 7 right back hip
servo 8 right back ankle
servo 9 left back ankle
servo 10 left back hip
servo 11 left back knee
*/