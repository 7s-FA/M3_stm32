#include "imu_bus.h"
#include "i2c.h"
#include "cmsis_os.h"
#include <stdbool.h>

void IMUBus_Delay(uint32_t milliseconds)
{
    /* Yield to the ROS task during sensor reset and bus recovery. */
    osDelay((milliseconds * osKernelGetTickFreq() + 999U) / 1000U);
}

HAL_StatusTypeDef IMUBus_Recover(void)
{
    /* PB8/PB9 are this board's I2C1 SCL/SDA. Open drain only; a held-low
     * clock/data line is reported as a fault, never driven high forcibly. */
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    GPIO_InitTypeDef pins = {0};
    pins.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    pins.Mode = GPIO_MODE_OUTPUT_OD;
    pins.Pull = GPIO_NOPULL;
    pins.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &pins);
    IMUBus_Delay(1);
    for (unsigned i = 0; i < 9 &&
            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET; ++i) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET) break;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        IMUBus_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        IMUBus_Delay(1);
    }
    /* STOP: SDA low while SCL low, release SCL, then release SDA. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    IMUBus_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    IMUBus_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    IMUBus_Delay(1);
    bool released = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET &&
                    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET;
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    HAL_StatusTypeDef result = HAL_I2C_Init(&hi2c1);
    return result != HAL_OK ? result : (released ? HAL_OK : HAL_BUSY);
}
