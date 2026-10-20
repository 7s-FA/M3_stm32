#ifndef XL430_TEST_HAL_H
#define XL430_TEST_HAL_H
#include <stdint.h>
typedef struct { uint32_t ErrorCode; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define HAL_UART_ERROR_NONE 0U
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
#endif
