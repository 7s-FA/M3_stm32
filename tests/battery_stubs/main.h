#include "stm32f4xx_hal.h"
#define BUZZER_GPIO_Port ((void *)0)
#define BUZZER_Pin 16U
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
void HAL_GPIO_WritePin(void *, uint16_t, GPIO_PinState);
