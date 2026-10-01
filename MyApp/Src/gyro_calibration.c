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
    if (c->state == GYRO_CAL_READY) return;
    if (c->count) ++c->restarts;
    c->count = 0;
    memset(c->mean, 0, sizeof(c->mean));
    memset(c->m2, 0, sizeof(c->m2));
}

bool GyroCalibration_Update(GyroCalibration_t *c, uint32_t now,
                            const float a[3], const float g[3])
{
    if (c->state == GYRO_CAL_READY) return true;
    if (c->state == GYRO_CAL_WARMUP) {
        if ((uint32_t)(now - c->start_ms) < IMU_CAL_WARMUP_MS) return false;
        c->state = GYRO_CAL_COLLECTING;
    }
    if (c->count && (uint32_t)(now - c->last_ms) > IMU_CAL_MAX_GAP_MS)
        GyroCalibration_Reject(c);
    float a2 = 0, g2 = 0;
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(a[i]) || !isfinite(g[i])) {
            GyroCalibration_Reject(c); return false;
        }
        a2 += a[i] * a[i]; g2 += g[i] * g[i];
    }
    if (a2 < 8.8f * 8.8f || a2 > 10.8f * 10.8f ||
        g2 > IMU_CAL_MAX_GYRO_RAD_S * IMU_CAL_MAX_GYRO_RAD_S) {
        GyroCalibration_Reject(c); return false;
    }
    if (c->count) {
        for (unsigned i = 0; i < 3; ++i) {
            if (fabsf(a[i] - c->anchor_accel[i]) > 0.25f ||
                fabsf(g[i] - c->anchor_gyro[i]) > 0.03f) {
                GyroCalibration_Reject(c); return false;
            }
        }
    } else {
        memcpy(c->anchor_accel, a, sizeof(c->anchor_accel));
        memcpy(c->anchor_gyro, g, sizeof(c->anchor_gyro));
        c->first_ms = now;
    }
    c->last_ms = now;
    ++c->count;
    for (unsigned i = 0; i < 3; ++i) {
        float delta = g[i] - c->mean[i];
        c->mean[i] += delta / (float)c->count;
        c->m2[i] += delta * (g[i] - c->mean[i]);
    }
    if (c->count < IMU_CAL_MIN_SAMPLES ||
        (uint32_t)(now - c->first_ms) < IMU_CAL_MIN_DURATION_MS) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (c->m2[i] / (float)(c->count - 1U) >
            IMU_CAL_MAX_STD_RAD_S * IMU_CAL_MAX_STD_RAD_S) {
            GyroCalibration_Reject(c); return false;
        }
    }
    memcpy(c->bias, c->mean, sizeof(c->bias));
    c->state = GYRO_CAL_READY;
    return true;
}
