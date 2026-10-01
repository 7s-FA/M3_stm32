#include "microros_platform.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include <errno.h>
#include <unistd.h>
#include <rcutils/allocator.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

void MicroROSPlatform_Init(void)
{
    rmw_uros_set_custom_transport(
        true,
        (void *)&huart2,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    rcutils_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;
    if (!rcutils_set_default_allocator(&freeRTOS_allocator))
    {
        Error_Handler();
    }

}

/* rclc expects POSIX sleep; suspend the calling RTOS task in tick units. */
int usleep(useconds_t microseconds)
{
    if (microseconds == 0U)
        return 0;

    /* Round up, then allow one extra tick for the current partial tick. */
    uint32_t ticks = (uint32_t)(((uint64_t)microseconds *
                                osKernelGetTickFreq() + 999999ULL) / 1000000ULL);
    if (osDelay(ticks + 1U) != osOK)
    {
        errno = EINVAL;
        return -1;
    }
    return 0;
}
