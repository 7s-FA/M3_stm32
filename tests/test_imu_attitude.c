#include "imu_attitude.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static float yaw(IMUAttitude *s) {return atan2f(2*(s->q[0]*s->q[3]+s->q[1]*s->q[2]),1-2*(s->q[2]*s->q[2]+s->q[3]*s->q[3]));}
int main(void) {
 IMUAttitude s; float a[3]={0,0,9.80665f},g[3]={0,0,0};
 IMUAttitude_Init(&s);assert(IMUAttitude_Update(&s,0,a,g));
 for(unsigned t=20;t<=10000;t+=20) assert(IMUAttitude_Update(&s,t,a,g));
 assert(fabsf(yaw(&s))<1e-6f && fabsf(s.q[0]-1)<1e-6f);
 g[2]=1.570796327f;
 for(unsigned t=10020;t<=11000;t+=20) assert(IMUAttitude_Update(&s,t,a,g));
 assert(fabsf(yaw(&s)-1.570796327f)<0.001f);
 g[2]=-1.570796327f;
 for(unsigned t=11020;t<=12000;t+=20) assert(IMUAttitude_Update(&s,t,a,g));
 assert(fabsf(yaw(&s))<0.001f);
 float old=s.q[0];assert(!IMUAttitude_Update(&s,13000,a,g));assert(s.q[0]==old);
 g[0]=NAN;assert(!IMUAttitude_Update(&s,13020,a,g));g[0]=0;g[2]=0;
 IMUAttitude_Init(&s);assert(IMUAttitude_Update(&s,UINT32_MAX-9,a,g));assert(IMUAttitude_Update(&s,10,a,g));
 a[1]=4.903325f;a[2]=8.492808f;IMUAttitude_Init(&s);assert(IMUAttitude_Update(&s,0,a,g));
 assert(fabsf(s.q[1]-sinf(0.261799388f))<0.001f);
 puts("attitude: static, signed 90-degree rotation, gaps, NaN, tick wrap, tilt passed");
}
