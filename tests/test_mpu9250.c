#include "mpu9250.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t registers[128];
static uint16_t expected_address;
static HAL_StatusTypeDef io_status;
static unsigned int writes;
static unsigned int fail_write;
static uint32_t tick;

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c, uint16_t address,
    uint16_t reg, uint16_t address_size, uint8_t *data, uint16_t len, uint32_t timeout)
{
    assert(i2c && address == expected_address);
    assert(address_size == I2C_MEMADD_SIZE_8BIT && timeout > 0 && timeout <= 5);
    assert(reg + len <= sizeof(registers));
    if (io_status != HAL_OK) return io_status;
    memcpy(data, registers + reg, len);
    if (reg == 0x3A) registers[reg] = 0; /* data-ready read clears the bit */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c, uint16_t address,
    uint16_t reg, uint16_t address_size, uint8_t *data, uint16_t len, uint32_t timeout)
{
    assert(i2c && address == expected_address);
    assert(address_size == I2C_MEMADD_SIZE_8BIT && timeout > 0 && timeout <= 5);
    assert(reg + len <= sizeof(registers));
    ++writes;
    if (writes == fail_write) return HAL_TIMEOUT;
    if (io_status != HAL_OK) return io_status;
    memcpy(registers + reg, data, len);
    return HAL_OK;
}
void HAL_Delay(uint32_t ms) { tick += ms; }
uint32_t HAL_GetTick(void) { return tick; }

static void Put16(unsigned int reg, int value)
{
    uint16_t bits = (uint16_t)value;
    registers[reg] = (uint8_t)(bits >> 8);
    registers[reg + 1] = (uint8_t)bits;
}

int main(void)
{
    I2C_HandleTypeDef bus = {0};
    MPU9250_Handle_t sensor = {0};
    MPU9250_Sample_t sample = {0}, before;
    assert(MPU9250_Read(&sensor, &sample) == MPU9250_NOT_INITIALIZED);
    assert(MPU9250_Init(NULL, &bus, 0x68) == MPU9250_INVALID_ARGUMENT);
    assert(MPU9250_Init(&sensor, NULL, 0x68) == MPU9250_INVALID_ARGUMENT);
    assert(MPU9250_Init(&sensor, &bus, 0xD0) == MPU9250_INVALID_ARGUMENT);

    expected_address = 0xD0;
    registers[0x75] = 0x68; /* Unsupported MPU6050 must still be rejected. */
    assert(MPU9250_Init(&sensor, &bus, 0x68) == MPU9250_WRONG_ID);
    assert(sensor.who_am_i == 0x68 && !sensor.initialized && writes == 0);
    registers[0x75] = 0x70;
    assert(MPU9250_Init(&sensor, &bus, 0x68) == MPU9250_OK);
    assert(sensor.initialized && sensor.who_am_i == 0x70);
    registers[0x3A] = 1;
    Put16(0x3F, 8192);
    assert(MPU9250_Read(&sensor, &sample) == MPU9250_OK);
    assert(fabsf(sample.acceleration_m_s2[2] - 9.80665f) < 0.0001f);
    registers[0x75] = 0x71;
    for (unsigned int fail = 1; fail <= 5; ++fail)
    {
        writes = 0; fail_write = fail;
        assert(MPU9250_Init(&sensor, &bus, 0x68) == MPU9250_I2C_ERROR);
        assert(!sensor.initialized && sensor.last_hal_status == HAL_TIMEOUT);
    }
    fail_write = 0;
    expected_address = 0xD2;
    assert(MPU9250_Init(&sensor, &bus, 0x69) == MPU9250_OK);
    assert(sensor.initialized && registers[0x19] == 9 && registers[0x1B] == 8);
    assert(registers[0x1C] == 8 && registers[0x38] == 1);

    registers[0x3A] = 1;
    Put16(0x3B, 8192); Put16(0x3D, -8192); Put16(0x3F, -32768);
    Put16(0x41, 0);
    Put16(0x43, 6550); Put16(0x45, -6550); Put16(0x47, 32767);
    assert(MPU9250_Read(&sensor, &sample) == MPU9250_OK);
    assert(fabsf(sample.acceleration_m_s2[0] - 9.80665f) < 0.0001f);
    assert(fabsf(sample.acceleration_m_s2[1] + 9.80665f) < 0.0001f);
    assert(fabsf(sample.acceleration_m_s2[2] + 39.2266f) < 0.0001f);
    assert(fabsf(sample.angular_velocity_rad_s[0] - 1.74532925f) < 0.0001f);
    assert(fabsf(sample.angular_velocity_rad_s[1] + 1.74532925f) < 0.0001f);
    assert(sample.angular_velocity_rad_s[2] > 8.7f);
    assert(sample.temperature_c == 21.0f && sample.tick_ms == tick);
    before = sample;
    assert(MPU9250_Read(&sensor, &sample) == MPU9250_NOT_READY);
    assert(memcmp(&before, &sample, sizeof(sample)) == 0);
    io_status = HAL_TIMEOUT;
    assert(MPU9250_Read(&sensor, &sample) == MPU9250_I2C_ERROR);
    assert(memcmp(&before, &sample, sizeof(sample)) == 0);
    assert(sensor.last_hal_status == HAL_TIMEOUT);
    assert(MPU9250_Init(&sensor, &bus, 0x69) == MPU9250_I2C_ERROR);
    assert(!sensor.initialized);
    puts("MPU9250: address, ID, initialization failures, SI conversion, stale-data and timeout tests passed");
}
