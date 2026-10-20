#include <assert.h>
#include <stdio.h>
#include "../MyApp/Src/microros_transport.c"
static HAL_StatusTypeDef open_result=HAL_OK;
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u,uint8_t *b,size_t n)
{ (void)b;u->hdmarx->remaining=n;return open_result; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *b,size_t n)
{ (void)u;(void)b;(void)n;return HAL_OK; }
int osDelay(uint32_t n) { (void)n;return 0; }
int main(void) {
 DMA_HandleTypeDef dma={0};UART_HandleTypeDef uart={.hdmarx=&dma};
 struct uxrCustomTransport t={.args=&uart};uint8_t out[8],err=0;
 assert(cubemx_transport_open(&t));
 dma_buffer[0]=11;dma_buffer[1]=22;dma.remaining=UART_DMA_BUFFER_SIZE-2;
 assert(cubemx_transport_read(&t,out,8,1,&err)==2 && out[0]==11 && out[1]==22);
 assert(cubemx_transport_close(&t) && cubemx_transport_open(&t));
 dma_buffer[0]=33;dma.remaining=UART_DMA_BUFFER_SIZE-1;
 assert(cubemx_transport_read(&t,out,8,1,&err)==1 && out[0]==33);
 dma_head=0;dma.remaining=0;
 assert(cubemx_transport_read(&t,out,8,1,&err)==0); /* DMA wrap is index zero. */
 open_result=HAL_ERROR;assert(!cubemx_transport_open(&t));
 puts("PASS UART DMA: reopen discards old cursor, wrap, open failure");
}
