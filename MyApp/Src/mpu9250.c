#include "mpu9250.h"
#include <stddef.h>

#define REG_SMPLRT_DIV   0x19U
#define REG_INT_ENABLE   0x38U
#define REG_INT_STATUS   0x3AU
#define REG_PWR_MGMT_1   0x6BU
#define REG_PWR_MGMT_2   0x6CU
#define REG_WHO_AM_I     0x75U
#define MPU9250_ID      0x71U
#define MPU6500_ID      0x70U
#define IO_TIMEOUT_MS   5U

static MPU9250_Result_t ReadRegisters(MPU9250_Handle_t *h, uint8_t reg,
                                     uint8_t *data, uint16_t size)
{
    h->last_hal_status = HAL_I2C_Mem_Read(h->i2c, h->address_hal, reg,
                            I2C_MEMADD_SIZE_8BIT, data, size, IO_TIMEOUT_MS);
    return h->last_hal_status == HAL_OK ? MPU9250_OK : MPU9250_I2C_ERROR;
}

static MPU9250_Result_t WriteRegisters(MPU9250_Handle_t *h, uint8_t reg,
                                      uint8_t *data, uint16_t size)
{
    h->last_hal_status = HAL_I2C_Mem_Write(h->i2c, h->address_hal, reg,
                            I2C_MEMADD_SIZE_8BIT, data, size, IO_TIMEOUT_MS);
    return h->last_hal_status == HAL_OK ? MPU9250_OK : MPU9250_I2C_ERROR;
}

static int32_t SignedBigEndian(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 8) | data[1];
    return value >= 32768U ? (int32_t)value - 65536 : (int32_t)value;
}

MPU9250_Result_t MPU9250_Init(MPU9250_Handle_t *h,
                            I2C_HandleTypeDef *i2c, uint8_t address_7bit)
{
    if (h == NULL)
        return MPU9250_INVALID_ARGUMENT;
    *h = (MPU9250_Handle_t){0};
    if (i2c == NULL || (address_7bit != MPU9250_ADDRESS_AD0_LOW &&
                        address_7bit != MPU9250_ADDRESS_AD0_HIGH))
        return MPU9250_INVALID_ARGUMENT;
    h->i2c = i2c;
    h->address_hal = (uint16_t)address_7bit << 1;

    /* Also allows a sensor powered at the same time as the MCU to start. */
    HAL_Delay(100U);
    if (ReadRegisters(h, REG_WHO_AM_I, &h->who_am_i, 1U) != MPU9250_OK)
        return MPU9250_I2C_ERROR;
    /* MPU6500 shares the accel/gyro/temp registers used below.
     * Preserve the observed ID; accepting it does not imply a magnetometer. */
    if (h->who_am_i != MPU9250_ID && h->who_am_i != MPU6500_ID)
        return MPU9250_WRONG_ID;

    uint8_t value = 0x80U; /* DEVICE_RESET */
    if (WriteRegisters(h, REG_PWR_MGMT_1, &value, 1U) != MPU9250_OK)
        return MPU9250_I2C_ERROR;
    HAL_Delay(100U);
    value = 0x01U; /* Wake, auto-select best available clock. */
    if (WriteRegisters(h, REG_PWR_MGMT_1, &value, 1U) != MPU9250_OK)
        return MPU9250_I2C_ERROR;
    value = 0x00U; /* Enable all accelerometer and gyroscope axes. */
    if (WriteRegisters(h, REG_PWR_MGMT_2, &value, 1U) != MPU9250_OK)
        return MPU9250_I2C_ERROR;

    /* Registers 0x19..0x1D: divider, gyro DLPF, gyro range,
     * accel range, accel DLPF. 1 kHz / (9 + 1) = 100 Hz. */
    uint8_t config[] = {9U, 3U, 0x08U, 0x08U, 3U};
    if (WriteRegisters(h, REG_SMPLRT_DIV, config, sizeof(config)) != MPU9250_OK)
        return MPU9250_I2C_ERROR;
    value = 0x01U; /* Raw-data-ready status; INT wire may remain unconnected. */
    if (WriteRegisters(h, REG_INT_ENABLE, &value, 1U) != MPU9250_OK)
        return MPU9250_I2C_ERROR;
    HAL_Delay(40U);
    h->initialized = true;
    return MPU9250_OK;
}

MPU9250_Result_t MPU9250_Read(MPU9250_Handle_t *h, MPU9250_Sample_t *sample)
{
    if (h == NULL || sample == NULL)
        return MPU9250_INVALID_ARGUMENT;
    if (!h->initialized)
        return MPU9250_NOT_INITIALIZED;

    /* One coherent burst: INT_STATUS followed by accel, temperature, gyro.
     * Reading INT_STATUS clears data-ready; no duplicate stale publication. */
    uint8_t data[15];
    MPU9250_Result_t result = ReadRegisters(h, REG_INT_STATUS, data, sizeof(data));
    if (result != MPU9250_OK)
        return result;
    if ((data[0] & 1U) == 0U)
        return MPU9250_NOT_READY;

    MPU9250_Sample_t next = {0};
    for (unsigned int axis = 0; axis < 3U; ++axis)
    {
        next.acceleration_m_s2[axis] =
            (float)SignedBigEndian(&data[1U + axis * 2U]) * (9.80665f / 8192.0f);
        next.angular_velocity_rad_s[axis] =
            (float)SignedBigEndian(&data[9U + axis * 2U]) *
            (0.017453292519943295f / 65.5f);
    }
    next.temperature_c = (float)SignedBigEndian(&data[7]) / 333.87f + 21.0f;
    next.tick_ms = HAL_GetTick();
    *sample = next;
    return MPU9250_OK;
}
