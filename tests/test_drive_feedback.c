#include "drive_control.h"
#include "xl430.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
UART_HandleTypeDef huart1;
static uint32_t tick;
static unsigned reads,configs,writes,enables;
static bool sync_ok=true,torque[3];
static XL430_Result_t read_result=XL430_OK;
static int32_t position=1000;
uint32_t HAL_GetTick(void){return tick;}
bool rmw_uros_epoch_synchronized(void){return sync_ok;}
int64_t rmw_uros_epoch_nanos(void){return 1700000000000000000LL+(int64_t)tick*1000000;}
XL430_Result_t XL430_Init(XL430_Handle_t *h,UART_HandleTypeDef *u,uint8_t id){h->uart=u;h->id=id;return XL430_OK;}
XL430_Result_t XL430_Ping(XL430_Handle_t *h,uint16_t *m,uint8_t *v){(void)h;(void)m;(void)v;return XL430_OK;}
XL430_Result_t XL430_SetTorque(XL430_Handle_t *h,bool e){torque[h->id]=e;enables+=e;return XL430_OK;}
XL430_Result_t XL430_ReadTorque(XL430_Handle_t *h,bool *e){*e=torque[h->id];return XL430_OK;}
XL430_Result_t XL430_ConfigureVelocityControl(XL430_Handle_t *h,uint8_t w){(void)h;(void)w;++configs;return XL430_OK;}
XL430_Result_t XL430_SetGoalVelocity(XL430_Handle_t *h,int32_t v){(void)h;(void)v;++writes;return XL430_OK;}
XL430_Result_t XL430_SyncWriteGoalVelocity(XL430_Handle_t *l,int32_t lv,XL430_Handle_t *r,int32_t rv){(void)l;(void)lv;(void)r;(void)rv;++writes;return XL430_OK;}
XL430_Result_t XL430_SyncReadWheelState(XL430_Handle_t *l,XL430_WheelState_t *ls,XL430_Handle_t *r,XL430_WheelState_t *rs){
 assert(l->id==1 && r->id==2);++reads;
 if(read_result==XL430_OK){ls->position=rs->position=position;ls->velocity=rs->velocity=0;}
 return read_result;
}
int main(void){
 sensor_msgs__msg__JointState msg={0};double pos[2]={0},vel[2]={0};
 msg.position.data=pos;msg.velocity.data=vel;
 DriveControl_Init();DriveControl_Update();
 assert(reads==1 && DriveControl_UpdateJointState(&msg));
 assert(!DriveControl_IsEnabled() && enables==0 && configs==0 && writes==0);
 assert(pos[0]==0 && msg.header.stamp.sec==1700000000);
 tick+=20;position+=100;DriveControl_Update();
 assert(DriveControl_UpdateJointState(&msg));
 assert(fabs(pos[0]-100*XL430_POSITION_RAD_PER_RAW)<1e-9);
 DriveState_t state;DriveControl_GetState(&state);assert(state.combined_state==0);
 sync_ok=false;assert(!DriveControl_UpdateJointState(&msg));sync_ok=true;
 read_result=XL430_ERROR_TIMEOUT;tick+=20;DriveControl_Update();
 assert(!DriveControl_UpdateJointState(&msg));DriveControl_GetState(&state);assert(state.combined_state==-3);
 unsigned prev=reads;tick+=20;DriveControl_Update();assert(reads==prev);
 read_result=XL430_OK;tick+=1000;DriveControl_Update();
 assert(reads==prev+1 && DriveControl_UpdateJointState(&msg));
 assert(enables==0 && configs==0 && writes==0); /* Recovery never enables motion. */
 DriveControl_SetTorque(false);tick+=20;DriveControl_Update();
 assert(DriveControl_UpdateJointState(&msg) && !DriveControl_IsEnabled());
 puts("PASS drive feedback: torque-off boot/manual encoder changes, sync gate, timeout/backoff/recovery, no torque enable or velocity writes");
}
