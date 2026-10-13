#include "imu_attitude.h"
#include <math.h>
#include <string.h>

void IMUAttitude_Init(IMUAttitude *s)
{
    memset(s, 0, sizeof(*s));
    s->q[0] = 1.0f;
}

/* Madgwick IMU objective: rotate world gravity into the body frame.
 * Analytic J-transpose gradient, normalized before applying beta=0.1.
 * No magnetometer: yaw is relative and can drift. */
static void step(IMUAttitude *s, const float a[3], const float g[3], float dt)
{
    float w=s->q[0], x=s->q[1], y=s->q[2], z=s->q[3];
    float d[4] = {
        -0.5f*(x*g[0]+y*g[1]+z*g[2]),
         0.5f*(w*g[0]+y*g[2]-z*g[1]),
         0.5f*(w*g[1]-x*g[2]+z*g[0]),
         0.5f*(w*g[2]+x*g[1]-y*g[0])};
    float an=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    /* Suppress gravity correction during large linear accelerations. */
    if (an > 7.8f && an < 11.8f) {
        float f0=2*(x*z-w*y)-a[0]/an;
        float f1=2*(w*x+y*z)-a[1]/an;
        float f2=1-2*(x*x+y*y)-a[2]/an;
        float grad[4]={-2*y*f0+2*x*f1,
            2*z*f0+2*w*f1-4*x*f2,
            -2*w*f0+2*z*f1-4*y*f2,
            2*x*f0+2*y*f1};
        float gn=sqrtf(grad[0]*grad[0]+grad[1]*grad[1]+
                       grad[2]*grad[2]+grad[3]*grad[3]);
        if (gn > 1e-6f)
            for (unsigned i=0;i<4;++i) d[i]-=0.1f*grad[i]/gn;
    }
    float norm=0;
    for (unsigned i=0;i<4;++i) {s->q[i]+=d[i]*dt;norm+=s->q[i]*s->q[i];}
    norm=sqrtf(norm);
    for (unsigned i=0;i<4;++i) s->q[i]/=norm;
}

bool IMUAttitude_Update(IMUAttitude *s, uint32_t tick,
                        const float a[3], const float g[3])
{
    for (unsigned i=0;i<3;++i)
        if (!isfinite(a[i]) || !isfinite(g[i]) || fabsf(g[i])>20.0f || fabsf(a[i])>100.0f)
            return false;
    if (!s->initialized) {
        float norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
        if (norm < 0.1f) return false;
        float roll=atan2f(a[1],a[2]);
        float pitch=atan2f(-a[0],hypotf(a[1],a[2]));
        float cr=cosf(roll/2),sr=sinf(roll/2),cp=cosf(pitch/2),sp=sinf(pitch/2);
        s->q[0]=cr*cp;s->q[1]=sr*cp;s->q[2]=cr*sp;s->q[3]=-sr*sp;
        s->tick=tick;s->initialized=true;
        return true;
    }
    uint32_t elapsed=tick-s->tick;
    s->tick=tick;
    if (!elapsed || elapsed>250U) return false;
    unsigned n=(elapsed+4U)/5U;
    for (unsigned i=0;i<n;++i) step(s,a,g,(float)elapsed/(1000.0f*n));
    return true;
}
