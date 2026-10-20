#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct
{
    int32_t left_state, right_state, combined_state;
    int32_t left_velocity, right_velocity;
    int32_t left_position, right_position;
    bool sample_valid;
    uint32_t sample_tick_ms, sample_age_ms;
    int32_t read_result;
} DriveState_t;

/* Drive task only. ROS callbacks use app_tasks.h instead. */
void DriveControl_Init(void);
void DriveControl_Disconnect(void);
void DriveControl_SetTorque(bool enabled);
void DriveControl_SetVelocity(float linear_mps, float angular_radps);
void DriveControl_SetVelocityAt(float linear_mps, float angular_radps, uint32_t received_tick);
void DriveControl_SetRawVelocity(bool left, int32_t velocity);
/* move <left pulses> <right pulses> <profile velocity> [positive request ID].
 * Copies/validates the bounded string; motor I/O runs in Update. */
void DriveControl_MoveCommand(const char *command, size_t length);
size_t DriveControl_FormatMoveStatus(char *buffer, size_t capacity);
void DriveControl_Update(void);
bool DriveControl_IsEnabled(void);
void DriveControl_GetState(DriveState_t *state);
/* Copies calibrated joint values without calling ROS or accessing hardware. */
bool DriveControl_GetJointValues(double position[2], double velocity[2]);

#endif
