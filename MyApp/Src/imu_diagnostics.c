#include "imu_diagnostics.h"
#include "imu.h"
#include "app_config.h"
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>

static rcl_publisher_t publisher;
static std_msgs__msg__String message;
static char text[448];
static bool ready;
static uint32_t last_tick, loops, spin_errors, status_publish_errors;
static rcl_ret_t last_spin_error;

static const char *ResultName(MPU9250_Result_t result)
{
    switch (result) {
    case MPU9250_OK: return "OK";
    case MPU9250_NOT_READY: return "NO_NEW_DATA";
    case MPU9250_INVALID_ARGUMENT: return "INVALID_ARGUMENT";
    case MPU9250_I2C_ERROR: return "I2C_ERROR";
    case MPU9250_WRONG_ID: return "WRONG_SENSOR_ID";
    case MPU9250_NOT_INITIALIZED: return "NOT_INITIALIZED";
    default: return "UNKNOWN";
    }
}

rcl_ret_t IMUDiagnostics_Init(rcl_node_t *node)
{
    message.data.data = text;
    message.data.size = 0;
    message.data.capacity = sizeof(text);
    publisher = rcl_get_zero_initialized_publisher();
    rcl_ret_t result = rclc_publisher_init_best_effort(&publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "imu/status");
    ready = result == RCL_RET_OK;
    last_tick = HAL_GetTick();
    return result;
}

void IMUDiagnostics_Update(rcl_ret_t spin_result)
{
    ++loops;
    if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT) {
        ++spin_errors;
        last_spin_error = spin_result;
    }
    uint32_t now = HAL_GetTick();
    if (!ready || (uint32_t)(now - last_tick) < 1000U)
        return;
    last_tick = now;
    uint32_t age = imu_status.samples ?
        (uint32_t)(now - imu_status.sample.tick_ms) : UINT32_MAX;
    int length = snprintf(text, sizeof(text),
        "fw=imu-ahrs-v6.7-battery cal=%s n=%lu/%lu bias_mrad_s=[%ld,%ld,%ld] rejected=%lu "
        "attitude=%u initialized=%u who_am_i=0x%02X model=%s result=%s "
        "hal=%u samples=%lu read_errors=%lu pub_err=%lu "
        "time_synced=%u age_ms=%lu loops=%lu "
        "spin_errors=%lu spin_rc=%ld status_err=%lu",
        imu_status.calibration.state == GYRO_CAL_READY ? "READY" :
            (imu_status.calibration.state == GYRO_CAL_WARMUP ? "WARMUP" : "COLLECTING"),
        (unsigned long)imu_status.calibration.count,
        (unsigned long)IMU_CAL_SAMPLES,
        (long)(imu_status.calibration.bias[0] * 1000.0f),
        (long)(imu_status.calibration.bias[1] * 1000.0f),
        (long)(imu_status.calibration.bias[2] * 1000.0f),
        (unsigned long)imu_status.calibration.rejected,
        (unsigned)imu_status.attitude_valid, (unsigned)imu_status.initialized, (unsigned)imu_status.who_am_i,
        imu_status.who_am_i == 0x70U ? "MPU6500" :
            (imu_status.who_am_i == 0x71U ? "MPU9250" : "UNKNOWN"),
        ResultName(imu_status.last_result), (unsigned)imu_status.last_hal_status,
        (unsigned long)imu_status.samples, (unsigned long)imu_status.read_errors,
        (unsigned long)imu_status.publish_errors,
        (unsigned)rmw_uros_epoch_synchronized(), (unsigned long)age,
        (unsigned long)loops, (unsigned long)spin_errors, (long)last_spin_error,
        (unsigned long)status_publish_errors);
    if (length < 0) return;
    message.data.size = (size_t)length < sizeof(text) ? (size_t)length : sizeof(text) - 1U;
    /* Fits a 512-byte best-effort MTU; no time-sync or sensor-success gate. */
    if (rcl_publish(&publisher, &message, NULL) != RCL_RET_OK) {
        ++status_publish_errors;
        rcl_reset_error();
    }
}

void IMUDiagnostics_Fini(rcl_node_t *node)
{
    ready = false;
    if (publisher.impl && rcl_publisher_fini(&publisher, node) != RCL_RET_OK)
        rcl_reset_error();
    publisher = rcl_get_zero_initialized_publisher();
}
