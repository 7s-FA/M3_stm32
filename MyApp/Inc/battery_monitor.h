#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H
#include "xl430.h"

/* Single application task only; shares its XL430 bus with drive control. */
void BatteryMonitor_Init(XL430_Handle_t *motor);
void BatteryMonitor_Update(void);
/* Snapshot only: no UART access. False before a valid read, after an error,
 * or if the measurement is older than BATTERY_MAX_GAP_MS. */
bool BatteryMonitor_GetVoltage(float *volts);

#endif
