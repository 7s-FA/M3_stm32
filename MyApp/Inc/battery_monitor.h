#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H
#include "xl430.h"

/* Drive task only; shares its XL430 bus with drive control. */
void BatteryMonitor_Init(XL430_Handle_t *motor);
void BatteryMonitor_Update(void);
/* Cached latest successful nonzero reading; never performs motor I/O.
 * Copy the acquisition tick too so ROS can recheck freshness after handoff. */
bool BatteryMonitor_GetVoltage(float *volts, uint32_t *sample_tick);

#endif
