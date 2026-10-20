#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <sensor_msgs/msg/joint_state.h>

typedef struct
{
    int32_t left_state, right_state, combined_state;
    int32_t left_velocity, right_velocity;
    int32_t left_position, right_position;
    bool sample_valid;
    uint32_t sample_tick_ms, sample_age_ms;
    int32_t read_result;
} DriveState_t;

/* Call from the application task only, including the executor callbacks. */
void DriveControl_Init(void);
void DriveControl_Disconnect(void);
void DriveControl_SetTorque(bool enabled);
void DriveControl_SetVelocity(float linear_mps, float angular_radps);
void DriveControl_SetRawVelocity(bool left, int32_t velocity);
/* move <left pulses> <right pulses> <profile velocity> [positive request ID].
 * Copies/validates the bounded string; motor I/O runs in Update. */
void DriveControl_MoveCommand(const char *command, size_t length);
size_t DriveControl_FormatMoveStatus(char *buffer, size_t capacity);
void DriveControl_Update(void);
bool DriveControl_IsEnabled(void);
void DriveControl_GetState(DriveState_t *state);
/* The caller owns preallocated JointState name/position/velocity storage. */
bool DriveControl_UpdateJointState(sensor_msgs__msg__JointState *message);

#endif
