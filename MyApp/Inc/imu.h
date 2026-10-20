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

/* Debugger snapshot: written only by AppRTOS_Start. Halt to inspect atomically. */
extern IMU_Status_t imu_status;

/* Once at task startup, before drive initialization. No fatal error on
 * missing/wrong sensor. Power-cycle/reset after fixing an initialization fault. */
void IMU_InitSensor(void);
rcl_ret_t IMU_InitPublisher(rcl_node_t *node);
/* Call from the same task after DriveControl_Update. No ISR calls. */
void IMU_Update(void);

void IMU_FiniPublisher(rcl_node_t *node);
#endif
