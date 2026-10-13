#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    float q[4]; /* w,x,y,z; body to world, right-handed, initial yaw zero */
    uint32_t tick;
    bool initialized;
} IMUAttitude;
void IMUAttitude_Init(IMUAttitude *s);
/* SI units, bias-corrected gyro. False for invalid samples or gaps >250 ms. */
bool IMUAttitude_Update(IMUAttitude *s, uint32_t tick,
                        const float accel[3], const float gyro[3]);
#endif
