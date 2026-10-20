#include "gyro_calibration.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    GyroCalibration_t c;
    const float bias[3] = {-.027f, .099f, -.024f};
    float a[3] = {0, 0, 9.8f};
    GyroCalibration_Init(&c, 0);
    assert(!GyroCalibration_Update(&c, IMU_CAL_WARMUP_MS-1, a, bias));
    assert(c.count == 0);
    for (unsigned i=0; i<512; ++i) {
        /* Exceeds the old anchor, gyro magnitude, accel and variance gates. */
        float sign = i%2 ? 1.0f : -1.0f;
        float g[3] = {bias[0]+sign*.4f, bias[1]-sign*.3f, bias[2]+sign*.1f};
        a[0] = sign*6;
        bool ready = GyroCalibration_Update(&c, 2000+i*20, a, g);
        assert(ready == (i==511));
        assert(c.count == i+1 && c.rejected == 0);
    }
    for (unsigned j=0; j<3; ++j) assert(fabsf(c.bias[j]-bias[j]) < 1e-6f);
    float saved=c.bias[0];
    GyroCalibration_Reject(&c);
    assert(GyroCalibration_Update(&c, 20000, a, a));
    assert(c.bias[0]==saved && c.count==512 && c.rejected==0);

    GyroCalibration_Init(&c, UINT32_MAX-1000U);
    assert(!GyroCalibration_Update(&c, 998U, a, bias));
    assert(c.count==0);
    assert(!GyroCalibration_Update(&c, 999U, a, bias));
    assert(c.count==1);
    GyroCalibration_Reject(&c); /* I2C error: skip, not restart. */
    float bad[3]={NAN,0,0};
    assert(!GyroCalibration_Update(&c, 1020, a, bad));
    bad[0]=INFINITY;
    assert(!GyroCalibration_Update(&c, 1040, bad, bias));
    assert(c.count==1 && c.rejected==3);
    for(unsigned i=1;i<512;++i)
        assert(GyroCalibration_Update(&c, 100000+i*20, a, bias)==(i==511));
    for(unsigned j=0;j<3;++j) assert(fabsf(c.bias[j]-bias[j])<1e-6f);
    puts("PASS calibration: exactly 512 samples, vibration average, invalid/I2C skip, long gap, tick wrap, retained bias");
}
