#ifndef GYRO_CALIBRATION_H
#define GYRO_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { GYRO_CAL_WARMUP, GYRO_CAL_COLLECTING, GYRO_CAL_READY } GyroCalState_t;
typedef struct {
    GyroCalState_t state;
    uint32_t start_ms, count, rejected;
    float mean[3], bias[3];
} GyroCalibration_t;

void GyroCalibration_Init(GyroCalibration_t *cal, uint32_t now);
/* Skip a failed sample, preserving accumulated samples and completed bias. */
void GyroCalibration_Reject(GyroCalibration_t *cal);
/* Input: sensor XYZ, m/s2 and rad/s. No allocation, delays, or ROS dependence.
 * Average 512 finite samples after warmup. Keep the robot stationary. */
bool GyroCalibration_Update(GyroCalibration_t *cal, uint32_t now,
                            const float accel[3], const float gyro[3]);
#endif
