#ifndef GYRO_CALIBRATION_H
#define GYRO_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { GYRO_CAL_WARMUP, GYRO_CAL_COLLECTING, GYRO_CAL_READY } GyroCalState_t;
typedef struct {
    GyroCalState_t state;
    uint32_t start_ms, first_ms, last_ms, count, restarts;
    float mean[3], m2[3], anchor_accel[3], anchor_gyro[3], bias[3];
} GyroCalibration_t;

void GyroCalibration_Init(GyroCalibration_t *cal, uint32_t now);
/* Discard incomplete window on communication failure. A completed bias stays. */
void GyroCalibration_Reject(GyroCalibration_t *cal);
/* Input: sensor XYZ, m/s2 and rad/s. No allocation, delays, or ROS dependence.
 * Screens likely motion; cannot distinguish constant slow rotation from bias. */
bool GyroCalibration_Update(GyroCalibration_t *cal, uint32_t now,
                            const float accel[3], const float gyro[3]);
#endif
