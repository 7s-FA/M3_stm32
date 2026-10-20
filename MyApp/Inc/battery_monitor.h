#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H
#include "xl430.h"

/* Single application task only; shares its XL430 bus with drive control. */
void BatteryMonitor_Init(XL430_Handle_t *motor);
void BatteryMonitor_Update(void);

#endif
