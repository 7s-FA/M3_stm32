#include "app_tasks.h"
#include "app_config.h"
#include "battery_monitor.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "main.h"
#include <string.h>

typedef enum { DRIVE_TORQUE, DRIVE_VELOCITY, DRIVE_RAW, DRIVE_MOVE,
               DRIVE_DISCONNECT } DriveCommandType;
typedef struct {
    DriveCommandType type;
    uint32_t received_tick;
    bool flag;
    float linear, angular;
    int32_t velocity;
    size_t length;
    char text[XL430_CMD_BUFFER_SIZE];
} DriveCommand;

static osMessageQueueId_t commands;
static osSemaphoreId_t command_done, workers_ready;
static osMutexId_t drive_mutex, imu_mutex;
static StaticQueue_t command_cb;
static uint32_t command_storage[(sizeof(DriveCommand) + 3U) / 4U];
static StaticSemaphore_t done_cb, ready_cb, drive_mutex_cb, imu_mutex_cb;
static AppDriveSnapshot drive_snapshot;
static IMU_Reading_t imu_snapshot;

static void Check(osStatus_t status)
{
    if (status != osOK) Error_Handler();
}

static uint32_t PeriodTicks(uint32_t milliseconds)
{
    return (milliseconds * osKernelGetTickFreq() + 999U) / 1000U;
}

void AppTasks_Init(void)
{
    const osMessageQueueAttr_t queue_attr = {
        .name = "driveCommands", .cb_mem = &command_cb, .cb_size = sizeof(command_cb),
        .mq_mem = command_storage, .mq_size = sizeof(command_storage)
    };
    const osSemaphoreAttr_t done_attr = {
        .name = "driveDone", .cb_mem = &done_cb, .cb_size = sizeof(done_cb)
    };
    const osSemaphoreAttr_t ready_attr = {
        .name = "workersReady", .cb_mem = &ready_cb, .cb_size = sizeof(ready_cb)
    };
    const osMutexAttr_t drive_attr = {
        .name = "driveSnapshot", .attr_bits = osMutexPrioInherit,
        .cb_mem = &drive_mutex_cb, .cb_size = sizeof(drive_mutex_cb)
    };
    const osMutexAttr_t imu_attr = {
        .name = "imuSnapshot", .attr_bits = osMutexPrioInherit,
        .cb_mem = &imu_mutex_cb, .cb_size = sizeof(imu_mutex_cb)
    };
    commands = osMessageQueueNew(1, sizeof(DriveCommand), &queue_attr);
    command_done = osSemaphoreNew(1, 0, &done_attr);
    workers_ready = osSemaphoreNew(2, 0, &ready_attr);
    drive_mutex = osMutexNew(&drive_attr);
    imu_mutex = osMutexNew(&imu_attr);
    if (!commands || !command_done || !workers_ready || !drive_mutex || !imu_mutex)
        Error_Handler();
}

void AppTasks_WaitReady(void)
{
    Check(osSemaphoreAcquire(workers_ready, osWaitForever));
    Check(osSemaphoreAcquire(workers_ready, osWaitForever));
}

static void CaptureDrive(void)
{
    AppDriveSnapshot snapshot = {0};
    DriveControl_GetState(&snapshot.state);
    snapshot.captured_tick = HAL_GetTick();
    snapshot.enabled = DriveControl_IsEnabled();
    snapshot.joint_valid = DriveControl_GetJointValues(snapshot.position, snapshot.velocity);
    (void)DriveControl_FormatMoveStatus(snapshot.move_status, sizeof(snapshot.move_status));
    snapshot.battery_valid = BatteryMonitor_GetVoltage(
        &snapshot.battery_voltage, &snapshot.battery_tick);
    Check(osMutexAcquire(drive_mutex, osWaitForever));
    drive_snapshot = snapshot;
    Check(osMutexRelease(drive_mutex));
}

static void ApplyCommand(const DriveCommand *command)
{
    switch (command->type) {
    case DRIVE_TORQUE: DriveControl_SetTorque(command->flag); break;
    case DRIVE_VELOCITY:
        DriveControl_SetVelocityAt(command->linear, command->angular, command->received_tick);
        break;
    case DRIVE_RAW:
        DriveControl_SetRawVelocity(command->flag, command->velocity);
        break;
    case DRIVE_MOVE: DriveControl_MoveCommand(command->text, command->length); break;
    case DRIVE_DISCONNECT: DriveControl_Disconnect(); break;
    }
}

void AppDriveTask(void *argument)
{
    (void)argument;
    DriveControl_Init();
    CaptureDrive();
    Check(osSemaphoreRelease(workers_ready));
    const uint32_t period = PeriodTicks(DRIVE_CONTROL_PERIOD_MS);
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - next) >= 0) {
            DriveControl_Update();
            CaptureDrive();
            next += period;
            now = osKernelGetTickCount();
            /* Skip missed periods after a peripheral timeout; never busy catch up. */
            if ((int32_t)(now - next) >= 0) next = now + period;
        }
        DriveCommand command;
        osStatus_t result = osMessageQueueGet(commands, &command, NULL, next - now);
        if (result == osOK) {
            ApplyCommand(&command);
            CaptureDrive();
            Check(osSemaphoreRelease(command_done));
        } else if (result != osErrorTimeout && result != osErrorResource) {
            Error_Handler();
        }
    }
}

void AppImuTask(void *argument)
{
    (void)argument;
    IMU_InitSensor();
    IMU_Reading_t reading;
    IMU_GetReading(&reading);
    Check(osMutexAcquire(imu_mutex, osWaitForever));
    imu_snapshot = reading;
    Check(osMutexRelease(imu_mutex));
    Check(osSemaphoreRelease(workers_ready));
    const uint32_t period = PeriodTicks(IMU_READ_PERIOD_MS);
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        IMU_Update();
        IMU_GetReading(&reading);
        Check(osMutexAcquire(imu_mutex, osWaitForever));
        imu_snapshot = reading;
        Check(osMutexRelease(imu_mutex));
        next += period;
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - next) >= 0) next = now + period;
        osStatus_t result = osDelayUntil(next);
        /* A higher-priority drive transaction can pass the deadline between
         * the tick check and this call. The next iteration skips the overrun. */
        if (result != osOK && result != osErrorParameter) Error_Handler();
    }
}

static void Submit(const DriveCommand *command)
{
    /* One producer and one outstanding command: no overflow, reordering, or
     * unacknowledged commands to replay after ROS teardown. Disconnect is also
     * acknowledged only after both motor shutdown attempts complete. */
    Check(osMessageQueuePut(commands, command, 0, osWaitForever));
    Check(osSemaphoreAcquire(command_done, osWaitForever));
}

void AppDrive_SetTorque(bool enabled)
{
    const DriveCommand command = {.type = DRIVE_TORQUE, .flag = enabled};
    Submit(&command);
}

void AppDrive_SetVelocity(float linear, float angular)
{
    const DriveCommand command = {.type = DRIVE_VELOCITY, .linear = linear,
        .angular = angular, .received_tick = HAL_GetTick()};
    Submit(&command);
}

void AppDrive_SetRawVelocity(bool left, int32_t velocity)
{
    const DriveCommand command = {.type = DRIVE_RAW, .flag = left,
        .velocity = velocity, .received_tick = HAL_GetTick()};
    Submit(&command);
}

void AppDrive_MoveCommand(const char *text, size_t length)
{
    DriveCommand command = {.type = DRIVE_MOVE, .length = length};
    /* Preserve the drive parser's rejection of null/oversized input without
     * retaining the ROS receive-buffer pointer across tasks. */
    if (text && length < sizeof(command.text))
        memcpy(command.text, text, length);
    else
        command.length = sizeof(command.text);
    Submit(&command);
}

void AppDrive_Disconnect(void)
{
    const DriveCommand command = {.type = DRIVE_DISCONNECT};
    Submit(&command);
}

void AppDrive_GetSnapshot(AppDriveSnapshot *snapshot)
{
    Check(osMutexAcquire(drive_mutex, osWaitForever));
    *snapshot = drive_snapshot;
    Check(osMutexRelease(drive_mutex));
    if (snapshot->state.sample_age_ms != UINT32_MAX)
        snapshot->state.sample_age_ms += HAL_GetTick() - snapshot->captured_tick;
}

void AppImu_GetSnapshot(IMU_Reading_t *reading)
{
    Check(osMutexAcquire(imu_mutex, osWaitForever));
    *reading = imu_snapshot;
    Check(osMutexRelease(imu_mutex));
}
