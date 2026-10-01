#ifndef MPU9250_H
#define MPU9250_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Seven-bit sensor address, NOT CubeMX's OwnAddress1. */
#define MPU9250_ADDRESS_AD0_LOW  0x68U
#define MPU9250_ADDRESS_AD0_HIGH 0x69U

typedef enum {
    MPU9250_OK = 0,
    MPU9250_NOT_READY,
    MPU9250_INVALID_ARGUMENT,
    MPU9250_I2C_ERROR,
    MPU9250_WRONG_ID,
    MPU9250_NOT_INITIALIZED
} MPU9250_Result_t;

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint16_t address_hal;
    uint8_t who_am_i;
    bool initialized;
    HAL_StatusTypeDef last_hal_status;
} MPU9250_Handle_t;

typedef struct {
    float acceleration_m_s2[3]; /* Sensor XYZ, includes gravity. */
    float angular_velocity_rad_s[3]; /* Sensor XYZ, no bias subtraction. */
    float temperature_c;
    uint32_t tick_ms; /* MCU read completion time, not ROS epoch. */
} MPU9250_Sample_t;

/* Supports MPU9250 (ID 0x71) and MPU6500 (ID 0x70) accel/gyro/temp.
 * The observed ID remains available in who_am_i. No magnetometer detection.
 * Call once after MX_I2C1_Init, before motor control starts. Blocks about
 * 200 ms for reset/startup. Configures +/-4 g, +/-500 deg/s, 100 Hz output,
 * ~41 Hz gyro and ~44.8 Hz accelerometer low-pass filters. No INT wire needed.
 * All calls must come from one task (or be protected by an external mutex).
 * This driver reads accel/gyro/temp only; no magnetometer or orientation. */
MPU9250_Result_t MPU9250_Init(MPU9250_Handle_t *handle,
                            I2C_HandleTypeDef *i2c, uint8_t address_7bit);

/* On any result other than OK, sample remains unchanged. Only use/publish
 * samples returned with OK. Each HAL transfer has a finite 5 ms timeout;
 * HAL's own bus-busy handling can add latency. */
MPU9250_Result_t MPU9250_Read(MPU9250_Handle_t *handle,
                            MPU9250_Sample_t *sample);

#endif
