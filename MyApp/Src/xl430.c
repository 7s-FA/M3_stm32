#include "xl430.h"

#include <stddef.h>
#include <string.h>

#define XL430_PROTOCOL_RESERVED        0x00U
#define XL430_INSTRUCTION_PING         0x01U
#define XL430_INSTRUCTION_READ         0x02U
#define XL430_INSTRUCTION_WRITE        0x03U
#define XL430_INSTRUCTION_SYNC_READ    0x82U
#define XL430_INSTRUCTION_SYNC_WRITE   0x83U
#define XL430_STATUS_INSTRUCTION       0x55U
#define XL430_BROADCAST_ID             0xFEU

#define XL430_ADDRESS_DRIVE_MODE       10U
#define XL430_ADDRESS_OPERATING_MODE   11U
#define XL430_ADDRESS_TORQUE_ENABLE    64U
#define XL430_ADDRESS_BUS_WATCHDOG     98U
#define XL430_ADDRESS_GOAL_VELOCITY    104U
#define XL430_ADDRESS_PROFILE_ACCEL    108U
#define XL430_ADDRESS_PROFILE_VELOCITY 112U
#define XL430_ADDRESS_GOAL_POSITION    116U
#define XL430_ADDRESS_PRESENT_VELOCITY 128U
#define XL430_ADDRESS_PRESENT_POSITION 132U
#define XL430_VELOCITY_CONTROL_MODE    1U
#define XL430_WHEEL_STATE_DATA_LENGTH  8U

#define XL430_HEADER_SIZE              7U
#define XL430_MIN_STATUS_LENGTH        4U
#define XL430_MAX_PARAMETERS           16U
#define XL430_MAX_TX_PACKET_SIZE       (10U + 2U * XL430_MAX_PARAMETERS)
#define XL430_MAX_RX_PACKET_SIZE       (11U + 2U * XL430_MAX_PARAMETERS)

static uint16_t XL430_UpdateCRC(uint16_t crc, const uint8_t *data, size_t length);
static XL430_Result_t XL430_TransmitInstruction(XL430_Handle_t *handle,
                                                uint8_t packet_id,
                                                uint8_t instruction,
                                                const uint8_t *parameters,
                                                uint16_t parameter_length);
static XL430_Result_t XL430_Transfer(XL430_Handle_t *handle,
                                    uint8_t instruction,
                                    const uint8_t *parameters,
                                    uint16_t parameter_length,
                                    uint8_t *response_parameters,
                                    uint16_t response_capacity,
                                    uint16_t *response_length);
static XL430_Result_t XL430_ReceiveStatus(XL430_Handle_t *handle,
                                         uint8_t *parameters,
                                         uint16_t parameter_capacity,
                                         uint16_t *parameter_length);
static XL430_Result_t XL430_ReceiveByte(UART_HandleTypeDef *uart,
                                       uint8_t *value,
                                       uint32_t started_at,
                                       uint32_t timeout_ms);
static XL430_Result_t XL430_WriteByte(XL430_Handle_t *handle,
                                     uint16_t address,
                                     uint8_t value);
static XL430_Result_t XL430_ReadByte(XL430_Handle_t *handle,
                                    uint16_t address,
                                    uint8_t *value);
static XL430_Result_t XL430_WriteData(XL430_Handle_t *handle,
                                     uint16_t address,
                                     const uint8_t *data,
                                     uint16_t data_length);
static XL430_Result_t XL430_ReadData(XL430_Handle_t *handle,
                                    uint16_t address,
                                    uint8_t *data,
                                    uint16_t data_length);

XL430_Result_t XL430_ReadVoltage(XL430_Handle_t *handle, uint16_t *voltage_raw)
{
    uint8_t data[2];
    if (voltage_raw == NULL) return XL430_ERROR_INVALID_ARGUMENT;
    XL430_Result_t result = XL430_ReadData(handle, 144U, data, sizeof(data));
    if (result == XL430_OK)
        *voltage_raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return result;
}

XL430_Result_t XL430_Init(XL430_Handle_t *handle,
                          UART_HandleTypeDef *uart,
                          uint8_t id)
{
    if ((handle == NULL) || (uart == NULL) || (id > 252U))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    handle->uart = uart;
    handle->id = id;
    handle->last_device_error = 0U;
    handle->timeout_ms = XL430_DEFAULT_TIMEOUT_MS;

    if (HAL_HalfDuplex_EnableReceiver(handle->uart) != HAL_OK)
    {
        return XL430_ERROR_UART;
    }

    return XL430_OK;
}

XL430_Result_t XL430_Ping(XL430_Handle_t *handle,
                          uint16_t *model_number,
                          uint8_t *firmware_version)
{
    uint8_t parameters[3];
    uint16_t parameter_length = 0U;
    XL430_Result_t result;

    result = XL430_Transfer(handle,
                            XL430_INSTRUCTION_PING,
                            NULL,
                            0U,
                            parameters,
                            sizeof(parameters),
                            &parameter_length);
    if (result != XL430_OK)
    {
        return result;
    }

    if (parameter_length != sizeof(parameters))
    {
        return XL430_ERROR_PACKET;
    }

    if (model_number != NULL)
    {
        *model_number = (uint16_t)parameters[0]
                      | ((uint16_t)parameters[1] << 8U);
    }

    if (firmware_version != NULL)
    {
        *firmware_version = parameters[2];
    }

    return XL430_OK;
}

XL430_Result_t XL430_SetTorque(XL430_Handle_t *handle, bool enable)
{
    return XL430_WriteByte(handle,
                           XL430_ADDRESS_TORQUE_ENABLE,
                           enable ? 1U : 0U);
}

XL430_Result_t XL430_ReadTorque(XL430_Handle_t *handle, bool *enabled)
{
    uint8_t value = 0U;
    XL430_Result_t result;

    if (enabled == NULL)
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    result = XL430_ReadByte(handle, XL430_ADDRESS_TORQUE_ENABLE, &value);
    if (result != XL430_OK)
    {
        return result;
    }

    *enabled = (value != 0U);
    return XL430_OK;
}

XL430_Result_t XL430_ConfigureVelocityControl(XL430_Handle_t *handle,
                                              uint8_t bus_watchdog_ticks)
{
    XL430_Result_t result;

    if ((handle == NULL) || (bus_watchdog_ticks > 127U))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    /* Operating Mode can only be changed while torque is disabled. */
    result = XL430_SetTorque(handle, false);
    if (result != XL430_OK)
    {
        return result;
    }

    /* A watchdog error is cleared by writing zero before reconfiguration. */
    result = XL430_WriteByte(handle, XL430_ADDRESS_BUS_WATCHDOG, 0U);
    if (result == XL430_OK)
    {
        result = XL430_WriteByte(handle,
                                 XL430_ADDRESS_OPERATING_MODE,
                                 XL430_VELOCITY_CONTROL_MODE);
    }
    if (result == XL430_OK)
    {
        uint8_t mode;
        result = XL430_ReadByte(handle, XL430_ADDRESS_OPERATING_MODE, &mode);
        if (result == XL430_OK && mode != XL430_VELOCITY_CONTROL_MODE)
            result = XL430_ERROR_DEVICE;
    }
    if (result == XL430_OK)
    {
        result = XL430_SetGoalVelocity(handle, 0);
    }
    if (result == XL430_OK)
    {
        result = XL430_SetTorque(handle, true);
    }
    if ((result == XL430_OK) && (bus_watchdog_ticks != 0U))
    {
        result = XL430_WriteByte(handle,
                                 XL430_ADDRESS_BUS_WATCHDOG,
                                 bus_watchdog_ticks);
    }

    if (result != XL430_OK)
    {
        (void)XL430_SetTorque(handle, false);
    }

    return result;
}

static XL430_Result_t XL430_WriteU32(XL430_Handle_t *handle, uint16_t address,
                                      uint32_t value)
{
    uint8_t data[4] = {(uint8_t)value, (uint8_t)(value >> 8),
                       (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
    return XL430_WriteData(handle, address, data, sizeof(data));
}

XL430_Result_t XL430_ConfigurePositionControl(XL430_Handle_t *handle,
                                             uint32_t velocity, uint32_t acceleration,
                                             uint8_t bus_watchdog_ticks)
{
    uint8_t drive_mode, mode;
    if (!handle || velocity == 0 || velocity > XL430_MAX_VELOCITY_VALUE ||
        acceleration == 0 || acceleration > 32767U || bus_watchdog_ticks > 127U)
        return XL430_ERROR_INVALID_ARGUMENT;
    XL430_Result_t result = XL430_SetTorque(handle, false);
    if (result == XL430_OK) result = XL430_WriteByte(handle, XL430_ADDRESS_BUS_WATCHDOG, 0U);
    if (result == XL430_OK) result = XL430_ReadByte(handle, XL430_ADDRESS_DRIVE_MODE, &drive_mode);
    /* Disable time profile and torque-on-by-goal-update, retain direction. */
    if (result == XL430_OK) result = XL430_WriteByte(handle, XL430_ADDRESS_DRIVE_MODE, drive_mode & ~0x0CU);
    if (result == XL430_OK) result = XL430_WriteByte(handle, XL430_ADDRESS_OPERATING_MODE, 4U);
    if (result == XL430_OK) result = XL430_ReadByte(handle, XL430_ADDRESS_OPERATING_MODE, &mode);
    if (result == XL430_OK && mode != 4U) result = XL430_ERROR_DEVICE;
    if (result == XL430_OK) result = XL430_WriteU32(handle, XL430_ADDRESS_PROFILE_ACCEL, acceleration);
    if (result == XL430_OK) result = XL430_WriteU32(handle, XL430_ADDRESS_PROFILE_VELOCITY, velocity);
    if (result == XL430_OK) result = XL430_WriteByte(handle, XL430_ADDRESS_BUS_WATCHDOG, bus_watchdog_ticks);
    return result;
}

XL430_Result_t XL430_SetGoalPosition(XL430_Handle_t *handle, int32_t position)
{
    if (position < -1048575 || position > 1048575)
        return XL430_ERROR_INVALID_ARGUMENT;
    return XL430_WriteU32(handle, XL430_ADDRESS_GOAL_POSITION, (uint32_t)position);
}

XL430_Result_t XL430_SyncWriteGoalPosition(XL430_Handle_t *left, int32_t left_position,
                                          XL430_Handle_t *right, int32_t right_position)
{
    if (!left || !right || !left->uart || left->uart != right->uart ||
        left->id == right->id || left_position < -1048575 || left_position > 1048575 ||
        right_position < -1048575 || right_position > 1048575)
        return XL430_ERROR_INVALID_ARGUMENT;
    uint8_t data[14] = {XL430_ADDRESS_GOAL_POSITION, 0U, 4U, 0U, left->id};
    uint32_t l = (uint32_t)left_position, r = (uint32_t)right_position;
    for (unsigned i = 0; i < 4; ++i) data[5+i] = (uint8_t)(l >> (8*i));
    data[9] = right->id;
    for (unsigned i = 0; i < 4; ++i) data[10+i] = (uint8_t)(r >> (8*i));
    return XL430_TransmitInstruction(left, XL430_BROADCAST_ID,
                                     XL430_INSTRUCTION_SYNC_WRITE, data, sizeof(data));
}

XL430_Result_t XL430_SetGoalVelocity(XL430_Handle_t *handle,
                                     int32_t velocity)
{
    uint8_t data[4];
    uint32_t raw_value;

    if ((handle == NULL) ||
        (velocity < -XL430_MAX_VELOCITY_VALUE) ||
        (velocity > XL430_MAX_VELOCITY_VALUE))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    raw_value = (uint32_t)velocity;
    data[0] = (uint8_t)(raw_value & 0xFFU);
    data[1] = (uint8_t)((raw_value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((raw_value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((raw_value >> 24U) & 0xFFU);

    return XL430_WriteData(handle,
                           XL430_ADDRESS_GOAL_VELOCITY,
                           data,
                           sizeof(data));
}

XL430_Result_t XL430_SyncWriteGoalVelocity(XL430_Handle_t *left,
                                           int32_t left_velocity,
                                           XL430_Handle_t *right,
                                           int32_t right_velocity)
{
    uint8_t parameters[14];
    uint32_t left_raw;
    uint32_t right_raw;

    if ((left == NULL) || (right == NULL) ||
        (left->uart == NULL) || (right->uart == NULL) ||
        (left->uart != right->uart) || (left->id == right->id) ||
        (left_velocity < -XL430_MAX_VELOCITY_VALUE) ||
        (left_velocity > XL430_MAX_VELOCITY_VALUE) ||
        (right_velocity < -XL430_MAX_VELOCITY_VALUE) ||
        (right_velocity > XL430_MAX_VELOCITY_VALUE))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    left->last_device_error = 0U;
    right->last_device_error = 0U;
    left_raw = (uint32_t)left_velocity;
    right_raw = (uint32_t)right_velocity;

    /* Sync Write header: start address and bytes written per actuator. */
    parameters[0] = (uint8_t)(XL430_ADDRESS_GOAL_VELOCITY & 0xFFU);
    parameters[1] = (uint8_t)((XL430_ADDRESS_GOAL_VELOCITY >> 8U) & 0xFFU);
    parameters[2] = 4U;
    parameters[3] = 0U;

    parameters[4] = left->id;
    parameters[5] = (uint8_t)(left_raw & 0xFFU);
    parameters[6] = (uint8_t)((left_raw >> 8U) & 0xFFU);
    parameters[7] = (uint8_t)((left_raw >> 16U) & 0xFFU);
    parameters[8] = (uint8_t)((left_raw >> 24U) & 0xFFU);

    parameters[9] = right->id;
    parameters[10] = (uint8_t)(right_raw & 0xFFU);
    parameters[11] = (uint8_t)((right_raw >> 8U) & 0xFFU);
    parameters[12] = (uint8_t)((right_raw >> 16U) & 0xFFU);
    parameters[13] = (uint8_t)((right_raw >> 24U) & 0xFFU);

    return XL430_TransmitInstruction(left,
                                     XL430_BROADCAST_ID,
                                     XL430_INSTRUCTION_SYNC_WRITE,
                                     parameters,
                                     sizeof(parameters));
}

XL430_Result_t XL430_SyncReadWheelState(XL430_Handle_t *left,
                                        XL430_WheelState_t *left_state,
                                        XL430_Handle_t *right,
                                        XL430_WheelState_t *right_state)
{
    uint8_t request[6];
    uint8_t left_data[XL430_WHEEL_STATE_DATA_LENGTH];
    uint8_t right_data[XL430_WHEEL_STATE_DATA_LENGTH];
    uint16_t response_length = 0U;
    uint32_t raw_value;
    XL430_Result_t result;

    if ((left == NULL) || (left_state == NULL) ||
        (right == NULL) || (right_state == NULL) ||
        (left->uart == NULL) || (right->uart == NULL) ||
        (left->uart != right->uart) || (left->id == right->id))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    left->last_device_error = 0U;
    right->last_device_error = 0U;

    request[0] = (uint8_t)(XL430_ADDRESS_PRESENT_VELOCITY & 0xFFU);
    request[1] = (uint8_t)((XL430_ADDRESS_PRESENT_VELOCITY >> 8U) & 0xFFU);
    request[2] = XL430_WHEEL_STATE_DATA_LENGTH;
    request[3] = 0U;
    request[4] = left->id;
    request[5] = right->id;

    result = XL430_TransmitInstruction(left,
                                       XL430_BROADCAST_ID,
                                       XL430_INSTRUCTION_SYNC_READ,
                                       request,
                                       sizeof(request));
    if (result != XL430_OK)
    {
        return result;
    }

    result = XL430_ReceiveStatus(left,
                                 left_data,
                                 sizeof(left_data),
                                 &response_length);
    if ((result != XL430_OK) ||
        (response_length != XL430_WHEEL_STATE_DATA_LENGTH))
    {
        return (result != XL430_OK) ? result : XL430_ERROR_PACKET;
    }

    response_length = 0U;
    result = XL430_ReceiveStatus(right,
                                 right_data,
                                 sizeof(right_data),
                                 &response_length);
    if ((result != XL430_OK) ||
        (response_length != XL430_WHEEL_STATE_DATA_LENGTH))
    {
        return (result != XL430_OK) ? result : XL430_ERROR_PACKET;
    }

    raw_value = (uint32_t)left_data[0]
              | ((uint32_t)left_data[1] << 8U)
              | ((uint32_t)left_data[2] << 16U)
              | ((uint32_t)left_data[3] << 24U);
    memcpy(&left_state->velocity, &raw_value, sizeof(raw_value));

    raw_value = (uint32_t)left_data[4]
              | ((uint32_t)left_data[5] << 8U)
              | ((uint32_t)left_data[6] << 16U)
              | ((uint32_t)left_data[7] << 24U);
    memcpy(&left_state->position, &raw_value, sizeof(raw_value));

    raw_value = (uint32_t)right_data[0]
              | ((uint32_t)right_data[1] << 8U)
              | ((uint32_t)right_data[2] << 16U)
              | ((uint32_t)right_data[3] << 24U);
    memcpy(&right_state->velocity, &raw_value, sizeof(raw_value));

    raw_value = (uint32_t)right_data[4]
              | ((uint32_t)right_data[5] << 8U)
              | ((uint32_t)right_data[6] << 16U)
              | ((uint32_t)right_data[7] << 24U);
    memcpy(&right_state->position, &raw_value, sizeof(raw_value));

    return XL430_OK;
}

static XL430_Result_t XL430_WriteByte(XL430_Handle_t *handle,
                                     uint16_t address,
                                     uint8_t value)
{
    return XL430_WriteData(handle, address, &value, sizeof(value));
}

static XL430_Result_t XL430_ReadByte(XL430_Handle_t *handle,
                                    uint16_t address,
                                    uint8_t *value)
{
    if (value == NULL)
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    return XL430_ReadData(handle, address, value, sizeof(*value));
}

static XL430_Result_t XL430_WriteData(XL430_Handle_t *handle,
                                     uint16_t address,
                                     const uint8_t *data,
                                     uint16_t data_length)
{
    uint8_t parameters[XL430_MAX_PARAMETERS];
    uint16_t response_length = 0U;

    if ((handle == NULL) || (data == NULL) || (data_length == 0U) ||
        (data_length > (XL430_MAX_PARAMETERS - 2U)))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    parameters[0] = (uint8_t)(address & 0xFFU);
    parameters[1] = (uint8_t)((address >> 8U) & 0xFFU);
    memcpy(&parameters[2], data, data_length);

    return XL430_Transfer(handle,
                          XL430_INSTRUCTION_WRITE,
                          parameters,
                          (uint16_t)(data_length + 2U),
                          NULL,
                          0U,
                          &response_length);
}

static XL430_Result_t XL430_ReadData(XL430_Handle_t *handle,
                                    uint16_t address,
                                    uint8_t *data,
                                    uint16_t data_length)
{
    uint8_t request[4];
    uint16_t response_length = 0U;
    XL430_Result_t result;

    if ((handle == NULL) || (data == NULL) || (data_length == 0U) ||
        (data_length > XL430_MAX_PARAMETERS))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    request[0] = (uint8_t)(address & 0xFFU);
    request[1] = (uint8_t)((address >> 8U) & 0xFFU);
    request[2] = (uint8_t)(data_length & 0xFFU);
    request[3] = (uint8_t)((data_length >> 8U) & 0xFFU);

    result = XL430_Transfer(handle,
                            XL430_INSTRUCTION_READ,
                            request,
                            sizeof(request),
                            data,
                            data_length,
                            &response_length);
    if (result != XL430_OK)
    {
        return result;
    }

    return (response_length == data_length) ? XL430_OK : XL430_ERROR_PACKET;
}

static XL430_Result_t XL430_Transfer(XL430_Handle_t *handle,
                                    uint8_t instruction,
                                    const uint8_t *parameters,
                                    uint16_t parameter_length,
                                    uint8_t *response_parameters,
                                    uint16_t response_capacity,
                                    uint16_t *response_length)
{
    XL430_Result_t result;

    if ((handle == NULL) || (handle->uart == NULL) ||
        (response_length == NULL))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    handle->last_device_error = 0U;
    *response_length = 0U;

    result = XL430_TransmitInstruction(handle,
                                       handle->id,
                                       instruction,
                                       parameters,
                                       parameter_length);
    if (result != XL430_OK)
    {
        return result;
    }

    return XL430_ReceiveStatus(handle,
                               response_parameters,
                               response_capacity,
                               response_length);
}

static XL430_Result_t XL430_TransmitInstruction(XL430_Handle_t *handle,
                                                uint8_t packet_id,
                                                uint8_t instruction,
                                                const uint8_t *parameters,
                                                uint16_t parameter_length)
{
    uint8_t packet[XL430_MAX_TX_PACKET_SIZE];
    uint16_t packet_length;
    uint16_t protocol_length;
    uint16_t crc;
    HAL_StatusTypeDef hal_result;

    if ((handle == NULL) || (handle->uart == NULL) ||
        (parameter_length > XL430_MAX_PARAMETERS) ||
        ((parameter_length > 0U) && (parameters == NULL)))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    packet[0] = 0xFFU;
    packet[1] = 0xFFU;
    packet[2] = 0xFDU;
    packet[3] = XL430_PROTOCOL_RESERVED;
    packet[4] = packet_id;
    packet[7] = instruction;
    uint16_t end = 8U;
    for (uint16_t i = 0; i < parameter_length; ++i) {
        packet[end++] = parameters[i];
        if (end >= 10U && packet[end-3] == 0xFFU &&
            packet[end-2] == 0xFFU && packet[end-1] == 0xFDU)
            packet[end++] = 0xFDU;
    }
    packet_length = end + 2U;
    protocol_length = packet_length - XL430_HEADER_SIZE;
    packet[5] = (uint8_t)protocol_length;
    packet[6] = (uint8_t)(protocol_length >> 8);

    crc = XL430_UpdateCRC(0U, packet, packet_length - 2U);
    packet[packet_length - 2U] = (uint8_t)(crc & 0xFFU);
    packet[packet_length - 1U] = (uint8_t)((crc >> 8U) & 0xFFU);

    /* Clear stale UART error state before starting a new request. */
    __HAL_UART_CLEAR_OREFLAG(handle->uart);
    handle->uart->ErrorCode = HAL_UART_ERROR_NONE;

    if (HAL_HalfDuplex_EnableTransmitter(handle->uart) != HAL_OK)
    {
        return XL430_ERROR_UART;
    }

    hal_result = HAL_UART_Transmit(handle->uart,
                                   packet,
                                   packet_length,
                                   handle->timeout_ms);
    if (hal_result != HAL_OK)
    {
        (void)HAL_HalfDuplex_EnableReceiver(handle->uart);
        return (hal_result == HAL_TIMEOUT) ? XL430_ERROR_TIMEOUT
                                           : XL430_ERROR_UART;
    }

    /* HAL_UART_Transmit waits for TC, so the complete stop bit is sent before
     * the shared wire is released and the receiver is enabled. */
    if (HAL_HalfDuplex_EnableReceiver(handle->uart) != HAL_OK)
    {
        return XL430_ERROR_UART;
    }

    return XL430_OK;
}

static XL430_Result_t XL430_ReceiveStatus(XL430_Handle_t *handle,
                                         uint8_t *parameters,
                                         uint16_t parameter_capacity,
                                         uint16_t *parameter_length)
{
    static const uint8_t header_pattern[4] = {0xFFU, 0xFFU, 0xFDU, 0x00U};
    uint8_t packet[XL430_MAX_RX_PACKET_SIZE];
    uint8_t value;
    uint8_t header_index = 0U;
    uint16_t protocol_length;
    uint16_t total_length;
    uint16_t payload_length;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint32_t started_at;
    XL430_Result_t result;

    started_at = HAL_GetTick();

    while (header_index < sizeof(header_pattern))
    {
        result = XL430_ReceiveByte(handle->uart,
                                   &value,
                                   started_at,
                                   handle->timeout_ms);
        if (result != XL430_OK)
        {
            return result;
        }

        if (value == header_pattern[header_index])
        {
            packet[header_index++] = value;
        }
        else
        {
            header_index = (value == 0xFFU) ? 1U : 0U;
            if (header_index == 1U)
            {
                packet[0] = value;
            }
        }
    }

    for (uint16_t index = 4U; index < XL430_HEADER_SIZE; ++index)
    {
        result = XL430_ReceiveByte(handle->uart,
                                   &packet[index],
                                   started_at,
                                   handle->timeout_ms);
        if (result != XL430_OK)
        {
            return result;
        }
    }

    protocol_length = (uint16_t)packet[5]
                    | ((uint16_t)packet[6] << 8U);

    if ((protocol_length < XL430_MIN_STATUS_LENGTH) ||
        (protocol_length > (XL430_MAX_RX_PACKET_SIZE - XL430_HEADER_SIZE)))
    {
        return XL430_ERROR_PACKET;
    }

    total_length = (uint16_t)(XL430_HEADER_SIZE + protocol_length);

    for (uint16_t index = XL430_HEADER_SIZE; index < total_length; ++index)
    {
        result = XL430_ReceiveByte(handle->uart,
                                   &packet[index],
                                   started_at,
                                   handle->timeout_ms);
        if (result != XL430_OK)
        {
            return result;
        }
    }

    received_crc = (uint16_t)packet[total_length - 2U]
                 | ((uint16_t)packet[total_length - 1U] << 8U);
    calculated_crc = XL430_UpdateCRC(0U, packet, total_length - 2U);

    if (received_crc != calculated_crc)
    {
        return XL430_ERROR_CRC;
    }

    /* CRC covers the stuffed wire packet; unstuff only after checking it. */
    uint16_t decoded_end = XL430_HEADER_SIZE;
    for (uint16_t i = XL430_HEADER_SIZE; i < total_length - 2U; ++i) {
        packet[decoded_end++] = packet[i];
        if (decoded_end >= 10U && packet[decoded_end-3] == 0xFFU &&
            packet[decoded_end-2] == 0xFFU && packet[decoded_end-1] == 0xFDU) {
            if (++i >= total_length - 2U || packet[i] != 0xFDU)
                return XL430_ERROR_PACKET;
        }
    }
    protocol_length = decoded_end - XL430_HEADER_SIZE + 2U;

    if (packet[4] != handle->id)
    {
        return XL430_ERROR_ID;
    }

    if (packet[7] != XL430_STATUS_INSTRUCTION)
    {
        return XL430_ERROR_PACKET;
    }

    handle->last_device_error = packet[8];
    if (handle->last_device_error != 0U)
    {
        return XL430_ERROR_DEVICE;
    }

    payload_length = (uint16_t)(protocol_length - XL430_MIN_STATUS_LENGTH);
    if (payload_length > parameter_capacity)
    {
        return XL430_ERROR_PACKET;
    }

    if ((payload_length > 0U) && (parameters == NULL))
    {
        return XL430_ERROR_INVALID_ARGUMENT;
    }

    if (payload_length > 0U)
    {
        memcpy(parameters, &packet[9], payload_length);
    }

    *parameter_length = payload_length;
    return XL430_OK;
}

static XL430_Result_t XL430_ReceiveByte(UART_HandleTypeDef *uart,
                                       uint8_t *value,
                                       uint32_t started_at,
                                       uint32_t timeout_ms)
{
    uint32_t elapsed;
    uint32_t remaining;
    HAL_StatusTypeDef hal_result;

    elapsed = HAL_GetTick() - started_at;
    if (elapsed >= timeout_ms)
    {
        return XL430_ERROR_TIMEOUT;
    }

    remaining = timeout_ms - elapsed;
    hal_result = HAL_UART_Receive(uart, value, 1U, remaining);

    if (hal_result == HAL_OK)
    {
        return XL430_OK;
    }

    return (hal_result == HAL_TIMEOUT) ? XL430_ERROR_TIMEOUT
                                       : XL430_ERROR_UART;
}

static uint16_t XL430_UpdateCRC(uint16_t crc, const uint8_t *data, size_t length)
{
    for (size_t byte_index = 0U; byte_index < length; ++byte_index)
    {
        crc ^= (uint16_t)data[byte_index] << 8U;

        for (uint8_t bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x8005U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}
