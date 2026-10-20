#include <stdint.h>
#include <stddef.h>
typedef struct { unsigned remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; int gState; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 0
#define __HAL_DMA_GET_COUNTER(d) ((d)->remaining)
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define portTICK_RATE_MS 1
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *,uint8_t *,size_t);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *,size_t);
