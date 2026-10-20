#ifndef XL430_H
#define XL430_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XL430_DEFAULT_TIMEOUT_MS  50U
#define XL430_MAX_VELOCITY_VALUE  265

typedef enum
{
    XL430_OK = 0,
    XL430_ERROR_INVALID_ARGUMENT,
    XL430_ERROR_UART,
    XL430_ERROR_TIMEOUT,
    XL430_ERROR_PACKET,
    XL430_ERROR_CRC,
    XL430_ERROR_ID,
    XL430_ERROR_DEVICE
} XL430_Result_t;

typedef struct
{
    UART_HandleTypeDef *uart;
    uint8_t id;
    uint8_t last_device_error;
    uint32_t timeout_ms;
} XL430_Handle_t;

typedef struct
{
    int32_t velocity;
    int32_t position;
} XL430_WheelState_t;

/**
 * @brief Attach an XL430 instance to an already initialized half-duplex UART.
 *
 * CubeMX must initialize the UART in Single-Wire Half-Duplex mode. For a
 * factory-default XL430 use ID 1 and 57,600 baud.
 */
XL430_Result_t XL430_Init(XL430_Handle_t *handle,
                          UART_HandleTypeDef *uart,
                          uint8_t id);

/** Ping the actuator and optionally return its model and firmware version. */
XL430_Result_t XL430_Ping(XL430_Handle_t *handle,
                          uint16_t *model_number,
                          uint8_t *firmware_version);

/** Enable or disable output torque by writing Torque Enable(64). */
XL430_Result_t XL430_SetTorque(XL430_Handle_t *handle, bool enable);

/** Read back Torque Enable(64). */
XL430_Result_t XL430_ReadTorque(XL430_Handle_t *handle, bool *enabled);

/**
 * @brief Select Velocity Control Mode, command zero velocity, and enable torque.
 *
 * @param bus_watchdog_ticks Bus Watchdog value in 20 ms units (0 disables it,
 *                           valid enabled range is 1..127).
 */
XL430_Result_t XL430_ConfigureVelocityControl(XL430_Handle_t *handle,
                                              uint8_t bus_watchdog_ticks);

/* Leaves torque OFF. Selects mode 4 and velocity-based profiles, preserving
 * the other Drive Mode bits. The caller must seed Goal Position before enable. */
XL430_Result_t XL430_ConfigurePositionControl(XL430_Handle_t *handle,
                                             uint32_t velocity, uint32_t acceleration,
                                             uint8_t bus_watchdog_ticks);
XL430_Result_t XL430_SetGoalPosition(XL430_Handle_t *handle, int32_t position);
XL430_Result_t XL430_SyncWriteGoalPosition(XL430_Handle_t *left, int32_t left_position,
                                          XL430_Handle_t *right, int32_t right_position);

/** Read Present Input Voltage(144), raw units of 0.1 V. */
XL430_Result_t XL430_ReadVoltage(XL430_Handle_t *handle, uint16_t *voltage_raw);

/** Set Goal Velocity(104), in XL430 raw units (1 unit is approximately 0.229 rpm). */
XL430_Result_t XL430_SetGoalVelocity(XL430_Handle_t *handle,
                                     int32_t velocity);

/**
 * @brief Set two wheel Goal Velocity(104) values with one broadcast Sync Write.
 *
 * The two handles must share the same half-duplex UART. Broadcast writes do not
 * generate Status Packets, so this function returns after transmission.
 */
XL430_Result_t XL430_SyncWriteGoalVelocity(XL430_Handle_t *left,
                                           int32_t left_velocity,
                                           XL430_Handle_t *right,
                                           int32_t right_velocity);

/**
 * @brief Read Present Velocity(128) and Present Position(132) from two wheels.
 *
 * One broadcast Sync Read request is followed by one Status Packet from each
 * actuator, in the same ID order used in the request.
 */
XL430_Result_t XL430_SyncReadWheelState(XL430_Handle_t *left,
                                        XL430_WheelState_t *left_state,
                                        XL430_Handle_t *right,
                                        XL430_WheelState_t *right_state);

#ifdef __cplusplus
}
#endif

#endif /* XL430_H */
