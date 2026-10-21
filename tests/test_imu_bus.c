#include "imu_bus.h"
#include <assert.h>
#include <stdio.h>
I2C_HandleTypeDef hi2c1;
static unsigned elapsed, pulses, deinit, init;
static bool scl=true, sda=true, scl_stuck, sda_stuck;
static int release_after;
uint32_t osKernelGetTickFreq(void) { return 1000; }
int osDelay(uint32_t ms) { elapsed+=ms; assert(elapsed<=30); return 0; }
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *h) { assert(h==&hi2c1); ++deinit; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h) { assert(h==&hi2c1); ++init; return HAL_OK; }
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *s) { assert(p==GPIOB && s->Mode==GPIO_MODE_OUTPUT_OD && s->Pull==GPIO_NOPULL); }
void HAL_GPIO_WritePin(void *p,unsigned pin,GPIO_PinState v) {
    assert(p==GPIOB);
    if(pin & GPIO_PIN_8) {
        if(!scl && v==GPIO_PIN_SET) { ++pulses; if(release_after>0 && (int)pulses>=release_after) sda_stuck=false; }
        scl=v==GPIO_PIN_SET;
    }
    if(pin & GPIO_PIN_9) sda=v==GPIO_PIN_SET;
}
GPIO_PinState HAL_GPIO_ReadPin(void *p,unsigned pin) {
    assert(p==GPIOB);
    return (pin==GPIO_PIN_8 ? scl&&!scl_stuck : sda&&!sda_stuck) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
static void reset(void) { elapsed=pulses=0; scl=sda=true; scl_stuck=sda_stuck=false; release_after=0; }
int main(void) {
    reset(); assert(IMUBus_Recover()==HAL_OK);
    reset(); sda_stuck=true; release_after=5; assert(IMUBus_Recover()==HAL_OK && pulses==6);
    reset(); sda_stuck=true; assert(IMUBus_Recover()==HAL_BUSY && pulses==10);
    reset(); scl_stuck=true; sda_stuck=true; assert(IMUBus_Recover()==HAL_BUSY && pulses==1);
    assert(init==4 && deinit==4);
    puts("PASS IMU bus: released bus, SDA release after clocks, bounded stuck SDA/SCL, peripheral reinit, open drain only");
}
