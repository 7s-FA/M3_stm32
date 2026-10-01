#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <sensor_msgs/msg/joint_state.h>

typedef struct
{
    int32_t left_state, right_state, combined_state;
    int32_t left_velocity, right_velocity;
    int32_t left_position, right_position;
} DriveState_t;

/* Call from the application task only, including the executor callbacks. */
void DriveControl_Init(void);
void DriveControl_SetTorque(bool enabled);
void DriveControl_SetVelocity(float linear_mps, float angular_radps);
void DriveControl_SetRawVelocity(bool left, int32_t velocity);
void DriveControl_Update(void);
bool DriveControl_IsEnabled(void);
void DriveControl_GetState(DriveState_t *state);
/* The caller owns preallocated JointState name/position/velocity storage. */
bool DriveControl_UpdateJointState(sensor_msgs__msg__JointState *message);

#endif
