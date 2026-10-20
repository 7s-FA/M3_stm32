#include "battery_monitor.h"
#include "app_config.h"
#include "main.h"
#include <string.h>

static XL430_Handle_t *source;
static uint16_t samples[BATTERY_SAMPLE_COUNT];
static uint32_t sum, last_sample, condition_since, alarm_since;
static unsigned count, sample_index;
static bool alarm, condition_active;

static void ResetWindow(void)
{
    sum = count = sample_index = 0;
    condition_active = false;
}

void BatteryMonitor_Init(XL430_Handle_t *motor)
{
    source = motor;
    memset(samples, 0, sizeof(samples));
    ResetWindow();
    alarm = false;
    last_sample = HAL_GetTick();
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void BatteryMonitor_Update(void)
{
    uint32_t now = HAL_GetTick();
    if (source == NULL) return;
    if ((uint32_t)(now - last_sample) >= BATTERY_SAMPLE_MS)
    {
        /* Gaps and failed reads cannot count as sustained low/recovered voltage. */
        if ((uint32_t)(now - last_sample) > BATTERY_MAX_GAP_MS)
            ResetWindow();
        uint16_t raw;
        XL430_Result_t result = XL430_ReadVoltage(source, &raw);
        now = HAL_GetTick();
        last_sample = now;
        if (result != XL430_OK)
        {
            ResetWindow(); /* Preserve an existing alarm until recovery is measured. */
        }
        else
        {
            if (count == BATTERY_SAMPLE_COUNT) sum -= samples[sample_index];
            else ++count;
            samples[sample_index] = raw;
            sum += raw;
            sample_index = (sample_index + 1U) % BATTERY_SAMPLE_COUNT;
            if (count == BATTERY_SAMPLE_COUNT)
            {
                /* Compare sums to retain fractional mean precision. */
                bool condition = alarm
                    ? sum >= BATTERY_RECOVER_RAW * BATTERY_SAMPLE_COUNT
                    : sum < BATTERY_LOW_RAW * BATTERY_SAMPLE_COUNT;
                if (!condition) condition_active = false;
                else if (!condition_active)
                {
                    condition_active = true;
                    condition_since = now;
                }
                else if ((uint32_t)(now - condition_since) >=
                         (alarm ? BATTERY_RECOVER_HOLD_MS : BATTERY_LOW_HOLD_MS))
                {
                    alarm = !alarm;
                    alarm_since = now;
                    condition_active = false;
                }
            }
        }
    }
    bool on = alarm && ((uint32_t)(now - alarm_since) %
                        BATTERY_BUZZER_PERIOD_MS < BATTERY_BUZZER_ON_MS);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
