#ifndef IMU_BUS_H
#define IMU_BUS_H
#include "stm32f4xx_hal.h"
/* IMU task only. Bounded bus clear; never accesses the drive UART. */
HAL_StatusTypeDef IMUBus_Recover(void);
void IMUBus_Delay(uint32_t milliseconds);
#endif
