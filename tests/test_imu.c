#include "imu.h"
#include "imu_bus.h"
#include "app_config.h"
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
static uint32_t tick;
static unsigned int reads, publications, error_resets, finalizations;
static bool synchronized;
static unsigned recover_calls;
static HAL_StatusTypeDef recovery_result = HAL_OK;
HAL_StatusTypeDef IMUBus_Recover(void) { ++recover_calls; return recovery_result; }
static void update(void) { IMU_Update(); IMU_Reading_t reading; IMU_GetReading(&reading); IMU_Publish(&reading); }
static MPU9250_Result_t init_result = MPU9250_OK, read_result = MPU9250_OK;
static rcl_ret_t publish_result = RCL_RET_OK;
static sensor_msgs__msg__Imu published;

uint32_t HAL_GetTick(void) { return tick; }
MPU9250_Result_t MPU9250_Init(MPU9250_Handle_t *h, I2C_HandleTypeDef *bus, uint8_t addr)
{
    assert(bus == &hi2c1 && addr == 0x68);
    h->who_am_i = 0x71;
    h->last_hal_status = HAL_OK;
    return init_result;
}
MPU9250_Result_t MPU9250_Read(MPU9250_Handle_t *h, MPU9250_Sample_t *out)
{
    ++reads;
    h->last_hal_status = read_result == MPU9250_I2C_ERROR ? HAL_TIMEOUT : HAL_OK;
    if (read_result == MPU9250_OK)
        *out = (MPU9250_Sample_t){.acceleration_m_s2 = {1, 2, 3},
            .angular_velocity_rad_s = {4, 5, 6}, .tick_ms = tick};
    return read_result;
}
bool rmw_uros_epoch_synchronized(void) { return synchronized; }
int64_t rmw_uros_epoch_nanos(void) { return 1700000000123456789LL; }
rcl_publisher_t rcl_get_zero_initialized_publisher(void)
{
    return (rcl_publisher_t){0};
}
const rosidl_message_type_support_t *rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Imu(void)
{
    static const rosidl_message_type_support_t support = {0};
    return &support;
}
rcl_ret_t rclc_publisher_init_best_effort(rcl_publisher_t *p, const rcl_node_t *n,
    const rosidl_message_type_support_t *s, const char *topic)
{
    assert(p && n && s && strcmp(topic, "imu/data") == 0);
    p->impl = (void *)&published;
    return RCL_RET_OK;
}
rcl_ret_t rcl_publish(const rcl_publisher_t *p, const void *msg,
                     rmw_publisher_allocation_t *allocation)
{
    assert(p && allocation == NULL);
    published = *(const sensor_msgs__msg__Imu *)msg;
    ++publications;
    return publish_result;
}
rcl_ret_t rcl_publisher_fini(rcl_publisher_t *p, rcl_node_t *n)
{ assert(p && p->impl && n); ++finalizations; return RCL_RET_OK; }
void rcutils_reset_error(void) { ++error_resets; }

int main(void)
{
    rcl_node_t node = {0};
    IMU_InitSensor();
    assert(IMU_InitPublisher(&node) == RCL_RET_OK);
    IMU_Update();
    assert(reads == 0);
    tick = 20; IMU_Update();
    assert(reads == 1 && publications == 0 && imu_status.sample_valid);
    synchronized = true;
    tick = 40; IMU_Update();
    assert(publications == 0 && reads == 2); /* Successful fresh read is gated. */
    imu_status.calibration.state = GYRO_CAL_READY;
    imu_status.calibration.bias[0] = 0.125f;
    tick = 60; update();
    assert(publications == 1 && imu_status.samples == 3);
    assert(published.header.stamp.sec == 1700000000);
    assert(published.header.stamp.nanosec == 123456789);
    assert(strcmp(published.header.frame_id.data, "imu_link") == 0);
    assert(published.orientation_covariance[0] == 0.0);
    assert(published.linear_acceleration.z == 3 && published.angular_velocity.x == 3.875);
    read_result = MPU9250_NOT_READY;
    tick = 80; update();
    assert(publications == 1 && !imu_status.sample_valid && imu_status.read_errors == 0);
    read_result = MPU9250_I2C_ERROR;
    tick = 100; update();
    assert(publications == 1 && imu_status.read_errors == 1);
    unsigned int before = reads;
    tick = 119; IMU_Update();
    assert(reads == before && recover_calls == 0);
    recovery_result = HAL_BUSY;
    tick = 120; IMU_Update();
    assert(reads == before && recover_calls == 1);
    recovery_result = HAL_OK;
    read_result = MPU9250_OK;
    tick = 1120; update();
    assert(recover_calls == 2 && imu_status.recoveries == 1);
    assert(publications == 1 && imu_status.sample_valid); /* Gap is not integrated. */
    publish_result = RCL_RET_ERROR;
    tick = 1140; update();
    assert(IMU_PublishErrors() == 1 && error_resets == 1);
    IMU_FiniPublisher(&node);
    assert(finalizations == 1);
    before = publications; tick += 20; update();
    assert(publications == before && imu_status.calibration.state == GYRO_CAL_READY);
    assert(imu_status.calibration.bias[0] == 0.125f);
    IMU_FiniPublisher(&node); assert(finalizations == 1); /* Idempotent teardown. */
    assert(IMU_InitPublisher(&node) == RCL_RET_OK);
    publish_result = RCL_RET_OK; tick += 20; update();
    assert(publications == before + 1);
    tick = UINT32_MAX - 10U; IMU_InitSensor();
    before = reads;
    tick = 15; IMU_Update();
    assert(reads == before + 1); /* Tick wrap must not block acquisition. */
    init_result = MPU9250_WRONG_ID;
    IMU_InitSensor(); before = reads;
    tick += 5000; IMU_Update();
    assert(!imu_status.initialized && reads == before);
    init_result = MPU9250_OK;
    recovery_result = HAL_BUSY;
    before = recover_calls;
    tick += 1000; update();
    assert(recover_calls == before + 1 && !imu_status.sample_valid);
    tick += 20; update(); assert(recover_calls == before + 1);
    recovery_result = HAL_OK;
    tick += 1000; update();
    assert(imu_status.initialized && imu_status.sample_valid);
    assert(imu_status.calibration.state != GYRO_CAL_READY);
    puts("IMU: timing, ROS units/metadata, sync gating, stale suppression, retry and failure tests passed");
}
