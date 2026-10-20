#include "gyro_calibration.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

void GyroCalibration_Init(GyroCalibration_t *c, uint32_t now)
{
    memset(c, 0, sizeof(*c));
    c->state = GYRO_CAL_WARMUP;
    c->start_ms = now;
}

void GyroCalibration_Reject(GyroCalibration_t *c)
{
    if (c->state != GYRO_CAL_READY) ++c->rejected;
}

bool GyroCalibration_Update(GyroCalibration_t *c, uint32_t now,
                            const float a[3], const float g[3])
{
    if (c->state == GYRO_CAL_READY) return true;
    if (c->state == GYRO_CAL_WARMUP) {
        if ((uint32_t)(now - c->start_ms) < IMU_CAL_WARMUP_MS) return false;
        c->state = GYRO_CAL_COLLECTING;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(a[i]) || !isfinite(g[i])) {
            GyroCalibration_Reject(c);
            return false;
        }
    }
    /* OpenCR-style fixed-count average: vibration does not restart collection.
     * Communication failures are skipped by the caller. Robot must be stationary. */
    ++c->count;
    for (unsigned i = 0; i < 3; ++i)
        c->mean[i] += (g[i] - c->mean[i]) / (float)c->count;
    if (c->count < IMU_CAL_SAMPLES) return false;
    memcpy(c->bias, c->mean, sizeof(c->bias));
    c->state = GYRO_CAL_READY;
    return true;
}
