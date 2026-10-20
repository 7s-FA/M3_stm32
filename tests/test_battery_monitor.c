#include "battery_monitor.h"
#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
static uint32_t tick;
static uint16_t voltage;
static bool ok, on;
static unsigned reads;
static XL430_Handle_t motor;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_WritePin(void *p, uint16_t pin, GPIO_PinState v)
{ (void)p; assert(pin == BUZZER_Pin); on = v == GPIO_PIN_SET; }
XL430_Result_t XL430_ReadVoltage(XL430_Handle_t *h, uint16_t *v)
{ assert(h == &motor); ++reads; *v = voltage; return ok ? XL430_OK : XL430_ERROR_TIMEOUT; }
static void step(unsigned ms) { tick += ms; BatteryMonitor_Update(); }
static void run(unsigned n) { while(n--) step(100); }
static void init(uint32_t start, uint16_t raw)
{ tick=start; voltage=raw; ok=true; reads=0; BatteryMonitor_Init(&motor); assert(!on); }
int main(void)
{
    float volts = -1.0f;
    init(0, 118);
    assert(!BatteryMonitor_GetVoltage(&volts) && volts == -1.0f);
    assert(!BatteryMonitor_GetVoltage(NULL));
    step(100);
    assert(BatteryMonitor_GetVoltage(&volts) && fabsf(volts-11.8f) < 0.001f);
    unsigned saved_reads=reads;
    assert(BatteryMonitor_GetVoltage(&volts) && reads == saved_reads);
    tick += 501; assert(!BatteryMonitor_GetVoltage(&volts));
    step(100); assert(BatteryMonitor_GetVoltage(&volts));
    ok=false; step(100); assert(!BatteryMonitor_GetVoltage(&volts));
    ok=true; voltage=0; step(100); assert(!BatteryMonitor_GetVoltage(&volts));
    voltage=120; step(100); assert(BatteryMonitor_GetVoltage(&volts));
    init(UINT32_MAX-150U, 118); step(100); tick+=100;
    assert(BatteryMonitor_GetVoltage(&volts));
    init(0, 110); run(100); assert(!on); /* Strictly below 11.0 V. */
    init(0, 109); step(99); assert(reads==0); step(1); assert(reads==1);
    run(8); assert(!on); step(100); /* Full mean at t=1000. */
    run(59); assert(!on); step(100); assert(on); /* t=7000. */
    step(499); assert(on); step(1); assert(!on);
    step(1500); assert(on); /* Pattern survives a scheduling gap. */
    ok=false; run(20); assert(on); /* Failed reads do not clear alarm. */
    ok=true; voltage=113; run(10); run(29);
    /* A full 3 seconds of recovered average are required. */
    step(100); assert(!on); run(30); assert(!on);
    init(0, 109); run(60); ok=false; step(100); ok=true;
    run(69); assert(!on); step(100); assert(on); /* Failure resets low hold. */
    init(0, 109); run(60); step(501); run(68); assert(!on);
    step(100); assert(on); /* Gap requires fresh samples + hold. */
    init(UINT32_MAX-3000U, 109); run(70); assert(on); /* Tick wrap. */
    voltage=112; run(100); assert(on); /* Hysteresis: 11.2 cannot recover. */
    puts("PASS battery: thresholds, full window, timing, pulse, failure, stale gap, recovery, wrap");
}
