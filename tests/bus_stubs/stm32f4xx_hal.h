#include "../imu_stubs/stm32f4xx_hal.h"
#include <stdbool.h>
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { unsigned Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
#define GPIO_PIN_8 256
#define GPIO_PIN_9 512
#define GPIOB ((void*)1)
#define GPIO_MODE_OUTPUT_OD 17
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C1_FORCE_RESET() ((void)0)
#define __HAL_RCC_I2C1_RELEASE_RESET() ((void)0)
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *);
void HAL_GPIO_Init(void *, GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(void *, unsigned, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(void *, unsigned);
