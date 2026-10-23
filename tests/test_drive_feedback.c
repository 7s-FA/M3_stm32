#include "drive_control.h"
#include "xl430.h"
#include "battery_monitor.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
void BatteryMonitor_Init(XL430_Handle_t *motor) { (void)motor; }
void BatteryMonitor_Update(void) {}
UART_HandleTypeDef huart1;
static uint32_t tick;
static unsigned reads,configs,writes,enables;
static bool sync_ok=true,torque[3];
static XL430_Result_t read_result=XL430_OK;
static int32_t position=1000;
static unsigned nonzero_writes, zero_writes;
static uint32_t read_duration_ms;
uint32_t HAL_GetTick(void){return tick;}
bool rmw_uros_epoch_synchronized(void){return sync_ok;}
int64_t rmw_uros_epoch_nanos(void){return 1700000000000000000LL+(int64_t)tick*1000000;}
XL430_Result_t XL430_Init(XL430_Handle_t *h,UART_HandleTypeDef *u,uint8_t id){h->uart=u;h->id=id;return XL430_OK;}
XL430_Result_t XL430_Ping(XL430_Handle_t *h,uint16_t *m,uint8_t *v){(void)h;(void)m;(void)v;return XL430_OK;}
XL430_Result_t XL430_SetTorque(XL430_Handle_t *h,bool e){torque[h->id]=e;enables+=e;return XL430_OK;}
XL430_Result_t XL430_ReadTorque(XL430_Handle_t *h,bool *e){*e=torque[h->id];return XL430_OK;}
XL430_Result_t XL430_ConfigureVelocityControl(XL430_Handle_t *h,uint8_t w){(void)w;torque[h->id]=true;++configs;return XL430_OK;}
XL430_Result_t XL430_SetGoalVelocity(XL430_Handle_t *h,int32_t v){(void)h;(void)v;++writes;return XL430_OK;}
XL430_Result_t XL430_SyncWriteGoalVelocity(XL430_Handle_t *l,int32_t lv,XL430_Handle_t *r,int32_t rv){(void)l;(void)r;nonzero_writes += (lv != 0 || rv != 0);zero_writes += (lv == 0 && rv == 0);++writes;return XL430_OK;}
XL430_Result_t XL430_SyncReadWheelState(XL430_Handle_t *l,XL430_WheelState_t *ls,XL430_Handle_t *r,XL430_WheelState_t *rs){
 assert(l->id==1 && r->id==2);++reads;tick+=read_duration_ms;
 if(read_result==XL430_OK){ls->position=rs->position=position;ls->velocity=rs->velocity=0;}
 return read_result;
}
XL430_Result_t XL430_ConfigurePositionControl(XL430_Handle_t *h,uint32_t v,uint32_t a,uint8_t w) {(void)h;(void)v;(void)a;(void)w;return XL430_OK;}
XL430_Result_t XL430_SetGoalPosition(XL430_Handle_t *h,int32_t p) {(void)h;(void)p;return XL430_OK;}
XL430_Result_t XL430_SyncWriteGoalPosition(XL430_Handle_t *l,int32_t lp,XL430_Handle_t *r,int32_t rp) {(void)l;(void)lp;(void)r;(void)rp;return XL430_OK;}
int main(void){
 double pos[2]={0},vel[2]={0};
 DriveControl_Init();
 DriveState_t snapshot;DriveControl_GetState(&snapshot);
 assert(!snapshot.sample_valid && snapshot.sample_age_ms==UINT32_MAX);
 DriveControl_Update();
 unsigned snapshot_reads=reads;tick+=7;DriveControl_GetState(&snapshot);
 assert(reads==snapshot_reads && snapshot.sample_valid && snapshot.sample_tick_ms==0 && snapshot.sample_age_ms==7);

 assert(reads==1 && DriveControl_GetJointValues(pos,vel));
 assert(!DriveControl_IsEnabled() && enables==0 && configs==0 && writes==0);
 assert(pos[0]==0);
 tick+=20;position+=100;DriveControl_Update();
 assert(DriveControl_GetJointValues(pos,vel));
 assert(fabs(pos[0]-100*XL430_POSITION_RAD_PER_RAW)<1e-9);
 DriveState_t state;DriveControl_GetState(&state);assert(state.combined_state==0);

 read_result=XL430_ERROR_TIMEOUT;tick+=20;DriveControl_Update();
 assert(!DriveControl_GetJointValues(pos,vel));DriveControl_GetState(&state);assert(state.combined_state==-3);
 DriveControl_GetState(&snapshot);
 assert(!snapshot.sample_valid && snapshot.read_result==XL430_ERROR_TIMEOUT && snapshot.sample_age_ms==20);
 unsigned prev=reads;
 /* Each retry waits from failure completion, including across tick wrap. */
 const uint32_t delays[]={20,50,100,200,200};
 read_duration_ms=50;
 for(unsigned i=0;i<sizeof(delays)/sizeof(delays[0]);++i){
   tick+=delays[i]-1;DriveControl_Update();assert(reads==prev);
   ++tick;DriveControl_Update();assert(reads==++prev);
   assert(!DriveControl_GetJointValues(pos,vel));
 }
 read_result=XL430_OK;read_duration_ms=0;tick+=200;DriveControl_Update();
 assert(reads==prev+1 && DriveControl_GetJointValues(pos,vel));
 /* Success resets retry history; a fresh failure only waits 20 ms. */
 tick=UINT32_MAX-10U;read_result=XL430_ERROR_TIMEOUT;DriveControl_Update();
 prev=reads;tick+=19;DriveControl_Update();assert(reads==prev);
 read_result=XL430_OK;++tick;DriveControl_Update();assert(reads==prev+1);
 assert(DriveControl_GetJointValues(pos,vel));
 DriveControl_GetState(&snapshot);assert(snapshot.sample_valid && snapshot.sample_age_ms==0 && snapshot.read_result==XL430_OK);
 assert(enables==0 && configs==0 && writes==0); /* Recovery never enables motion. */
 double before_off=pos[0];
 DriveControl_SetTorque(false);tick+=20;DriveControl_Update();
 assert(DriveControl_GetJointValues(pos,vel) && !DriveControl_IsEnabled());
 assert(fabs(pos[0]-before_off)<1e-9); /* Torque OFF must not zero odometry. */
 position+=50;tick+=20;DriveControl_Update();
 assert(DriveControl_GetJointValues(pos,vel));
 assert(fabs(pos[0]-before_off-50*XL430_POSITION_RAD_PER_RAW)<1e-9);
 DriveControl_SetTorque(true);
 DriveControl_SetVelocity(0.1f,0);tick+=20;DriveControl_Update();
 assert(nonzero_writes>0);
 DriveControl_SetRawVelocity(true,50); /* Pending raw command must also be discarded. */
 assert(DriveControl_GetJointValues(pos,vel));double before_disconnect=pos[0];
 DriveControl_Disconnect();
 assert(!DriveControl_IsEnabled() && !torque[1] && !torque[2]);
 unsigned before_motion=nonzero_writes;
 tick+=20;DriveControl_Update();
 assert(DriveControl_GetJointValues(pos,vel) && fabs(pos[0]-before_disconnect)<1e-9);
 DriveControl_SetTorque(true);tick+=20;DriveControl_Update();
 assert(nonzero_writes==before_motion); /* Re-enable alone cannot replay velocity. */
 DriveControl_SetVelocity(0.1f,0);tick+=20;DriveControl_Update();
 assert(nonzero_writes>before_motion);
 /* Command watchdog must stop even while a read retry is not yet due. */
 read_result=XL430_ERROR_TIMEOUT;
 DriveControl_SetVelocity(0.1f,0);DriveControl_Update();
 tick+=20;DriveControl_Update();tick+=50;DriveControl_Update();
 tick+=100;DriveControl_Update(); /* 200 ms backoff; command age 170 ms. */
 prev=reads;unsigned stops_before=zero_writes;
 tick+=130;DriveControl_Update(); /* Command age 300 ms, read age 130 ms. */
 assert(reads==prev && zero_writes>stops_before);
 puts("PASS drive feedback: torque-off boot/manual encoder changes, 20/50/100/200ms backoff, tick wrap, recovery reset, watchdog during backoff, disconnect clears pending commands; no stale replay after re-enable");
}
