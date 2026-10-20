#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "drive_control.h"
#include "imu.h"

typedef struct {
    DriveState_t state;
    bool enabled;
    bool joint_valid;
    uint32_t captured_tick;
    double position[2], velocity[2];
    char move_status[192];
    bool battery_valid;
    float battery_voltage;
    uint32_t battery_tick;
} AppDriveSnapshot;

/* Initialize before starting the scheduler. All client calls are ROS-task-only.
 * Commands are copied to the drive task and acknowledged before returning.
 * Snapshots never hold a mutex across peripheral or ROS I/O. */
void AppTasks_Init(void);
void AppTasks_WaitReady(void);
void AppDriveTask(void *argument);
void AppImuTask(void *argument);
void AppDrive_SetTorque(bool enabled);
void AppDrive_SetVelocity(float linear, float angular);
void AppDrive_SetRawVelocity(bool left, int32_t velocity);
void AppDrive_MoveCommand(const char *command, size_t length);
void AppDrive_Disconnect(void);
void AppDrive_GetSnapshot(AppDriveSnapshot *snapshot);
void AppImu_GetSnapshot(IMU_Reading_t *reading);

#endif
