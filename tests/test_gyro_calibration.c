#include "gyro_calibration.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
 GyroCalibration_t c;
 float a[3]={-.42f, .02f, 10.09f},g[3]={-.027f,.099f,-.024f};
 GyroCalibration_Init(&c,0);
 assert(!GyroCalibration_Update(&c,1999,a,g)&&c.count==0);
 for(unsigned i=0;i<99;i++) assert(!GyroCalibration_Update(&c,2000+i*100,a,g));
 assert(GyroCalibration_Update(&c,11900,a,g));
 for(unsigned i=0;i<3;i++) assert(fabsf(c.bias[i]-g[i])<1e-6f);
 float moving[3]={1,0,0};
 assert(GyroCalibration_Update(&c,12000,a,moving));
 assert(fabsf((moving[0]-c.bias[0])-1.027f)<1e-5f);
 GyroCalibration_Reject(&c);assert(c.state==GYRO_CAL_READY);
 GyroCalibration_Init(&c,0);
 for(unsigned i=0;i<100;i++) GyroCalibration_Update(&c,2000+i*20,a,g);
 assert(c.state!=GYRO_CAL_READY); /* count alone insufficient */
 assert(!GyroCalibration_Update(&c,4100,a,moving)&&c.count==0);
 GyroCalibration_Update(&c,4200,a,g);GyroCalibration_Reject(&c);assert(c.count==0);
 GyroCalibration_Update(&c,4300,a,g);GyroCalibration_Update(&c,5000,a,g);assert(c.count==1);
 float invalid[3]={NAN,0,0};assert(!GyroCalibration_Update(&c,5100,a,invalid)&&c.count==0);
 float tilt[3]={1,0,9.8f};GyroCalibration_Update(&c,5200,a,g);
 assert(!GyroCalibration_Update(&c,5300,tilt,g)&&c.count==0);
 GyroCalibration_Init(&c,0);
 for(unsigned i=0;i<100;i++){float noisy[3]={-.027f,.099f+(i%2?.012f:-.012f),-.024f};GyroCalibration_Update(&c,2000+i*100,a,noisy);}
 assert(c.state!=GYRO_CAL_READY&&c.count==0); /* variance rejection */
 GyroCalibration_Init(&c,UINT32_MAX-1000U);
 for(unsigned i=0;i<100;i++) GyroCalibration_Update(&c,999U+i*100U,a,g);
 assert(c.state==GYRO_CAL_READY);
 puts("PASS calibration: measured bias, min time/count, motion, I2C rejection, gap, NaN, variance, wrap, preserved true rotation");
}
