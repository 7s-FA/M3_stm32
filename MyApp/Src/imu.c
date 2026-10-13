#include "imu.h"
#include "imu_attitude.h"
#include "app_config.h"
#include "i2c.h"
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <limits.h>
#include <string.h>

IMU_Status_t imu_status;
static MPU9250_Handle_t sensor;
static IMUAttitude attitude;
static rcl_publisher_t publisher;
static sensor_msgs__msg__Imu message;
static bool publisher_ready;
static uint32_t last_attempt;
static uint32_t interval_ms;

void IMU_InitSensor(void)
{
    memset(&imu_status, 0, sizeof(imu_status));
    IMUAttitude_Init(&attitude);
    imu_status.last_result = MPU9250_Init(&sensor, &hi2c1, IMU_ADDRESS_7BIT);
    imu_status.who_am_i = sensor.who_am_i;
    imu_status.last_hal_status = sensor.last_hal_status;
    imu_status.initialized = imu_status.last_result == MPU9250_OK;
    GyroCalibration_Init(&imu_status.calibration, HAL_GetTick());
    last_attempt = HAL_GetTick();
    interval_ms = IMU_READ_PERIOD_MS;
}

rcl_ret_t IMU_InitPublisher(rcl_node_t *node)
{
    static char frame_id[] = IMU_FRAME_ID;
    memset(&message, 0, sizeof(message));
    message.header.frame_id.data = frame_id;
    message.header.frame_id.size = sizeof(frame_id) - 1U;
    message.header.frame_id.capacity = sizeof(frame_id);
    /* Covariance zero means unknown. Orientation is valid only after calibration. */
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;
    publisher = rcl_get_zero_initialized_publisher();
    rcl_ret_t result = rclc_publisher_init_best_effort(&publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");
    publisher_ready = result == RCL_RET_OK;
    /* frame_id has static storage: do not call generated message fini(). */
    return result;
}

void IMU_Update(void)
{
    if (!imu_status.initialized)
        return;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_attempt) < interval_ms)
        return;
    last_attempt = now;
    imu_status.sample_valid = false;
    imu_status.attitude_valid = false;
    imu_status.last_result = MPU9250_Read(&sensor, &imu_status.sample);
    imu_status.last_hal_status = sensor.last_hal_status;
    if (imu_status.last_result == MPU9250_NOT_READY)
        return;
    if (imu_status.last_result != MPU9250_OK)
    {
        ++imu_status.read_errors;
        GyroCalibration_Reject(&imu_status.calibration);
        /* Do not repeatedly stall the drive loop on a disconnected sensor. */
        interval_ms = IMU_ERROR_RETRY_MS;
        return;
    }
    interval_ms = IMU_READ_PERIOD_MS;
    imu_status.sample_valid = true;
    ++imu_status.samples;
    if (!GyroCalibration_Update(&imu_status.calibration,
            imu_status.sample.tick_ms, imu_status.sample.acceleration_m_s2,
            imu_status.sample.angular_velocity_rad_s))
        return;
    float gyro[3];
    for (unsigned i=0;i<3;++i)
        gyro[i]=imu_status.sample.angular_velocity_rad_s[i]-imu_status.calibration.bias[i];
    imu_status.attitude_valid = IMUAttitude_Update(&attitude, imu_status.sample.tick_ms,
        imu_status.sample.acceleration_m_s2, gyro);
    if (!imu_status.attitude_valid) return;
    message.orientation.w=attitude.q[0];
    message.orientation.x=attitude.q[1];
    message.orientation.y=attitude.q[2];
    message.orientation.z=attitude.q[3];
    message.orientation_covariance[0]=0.0;
    if (!publisher_ready || !rmw_uros_epoch_synchronized())
        return;

    /* Timestamp at read completion; no claim of exact hardware sample time. */
    int64_t stamp_ns = rmw_uros_epoch_nanos();
    int64_t seconds = stamp_ns / 1000000000LL;
    if (stamp_ns <= 0 || seconds > INT32_MAX)
        return;
    message.header.stamp.sec = (int32_t)seconds;
    message.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);
    message.linear_acceleration.x = imu_status.sample.acceleration_m_s2[0];
    message.linear_acceleration.y = imu_status.sample.acceleration_m_s2[1];
    message.linear_acceleration.z = imu_status.sample.acceleration_m_s2[2];
    message.angular_velocity.x = imu_status.sample.angular_velocity_rad_s[0]
        - imu_status.calibration.bias[0];
    message.angular_velocity.y = imu_status.sample.angular_velocity_rad_s[1]
        - imu_status.calibration.bias[1];
    message.angular_velocity.z = imu_status.sample.angular_velocity_rad_s[2]
        - imu_status.calibration.bias[2];
    if (rcl_publish(&publisher, &message, NULL) != RCL_RET_OK)
    {
        ++imu_status.publish_errors;
        rcl_reset_error();
    }
}
