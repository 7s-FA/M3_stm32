#ifndef IMU_H
#define IMU_H

#include "mpu9250.h"
#include "gyro_calibration.h"
#include <rcl/rcl.h>

typedef struct {
    bool initialized;
    bool sample_valid;
    bool attitude_valid;
    uint8_t who_am_i;
    MPU9250_Result_t last_result;
    HAL_StatusTypeDef last_hal_status;
    uint32_t samples;
    uint32_t read_errors;
    uint32_t publish_errors;
    GyroCalibration_t calibration;
    MPU9250_Sample_t sample; /* Uncorrected sensor sample for diagnostics. */
} IMU_Status_t;

typedef struct {
    IMU_Status_t status;
    float orientation[4];
} IMU_Reading_t;

/* Debugger snapshot: written only by the IMU task. Halt to inspect atomically.
 * ROS publish errors are owned separately by IMU_PublishErrors(). */
extern IMU_Status_t imu_status;

/* Once at IMU task startup. No fatal error on
 * missing/wrong sensor. Power-cycle/reset after fixing an initialization fault. */
void IMU_InitSensor(void);
rcl_ret_t IMU_InitPublisher(rcl_node_t *node);
/* IMU task only: acquisition and estimation, with no ROS calls. */
void IMU_Update(void);
void IMU_GetReading(IMU_Reading_t *reading);

/* ROS task only; reading must be a synchronized copy, never live sensor state. */
void IMU_Publish(const IMU_Reading_t *reading);
uint32_t IMU_PublishErrors(void);
void IMU_FiniPublisher(rcl_node_t *node);
#endif
