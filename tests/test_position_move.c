#include "drive_control.h"
#include "xl430.h"
#include "battery_monitor.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;
static uint32_t tick;
static struct { bool torque; int mode; int32_t position, velocity, goal; } motor[3];
static unsigned goal_writes, velocity_writes;
static bool fail_read, fail_restore, fail_position, fail_goal, fail_torque_read;
static char status[192];
void BatteryMonitor_Init(XL430_Handle_t *h) {(void)h;}
void BatteryMonitor_Update(void) {}
uint32_t HAL_GetTick(void) {return tick;}
bool rmw_uros_epoch_synchronized(void) {return true;}
int64_t rmw_uros_epoch_nanos(void) {return 1700000000000000000LL;}
XL430_Result_t XL430_Init(XL430_Handle_t *h,UART_HandleTypeDef *u,uint8_t id) {h->uart=u;h->id=id;return XL430_OK;}
XL430_Result_t XL430_Ping(XL430_Handle_t *h,uint16_t *m,uint8_t *v) {(void)h;(void)m;(void)v;return XL430_OK;}
XL430_Result_t XL430_SetTorque(XL430_Handle_t *h,bool on) {
    if (on && motor[h->id].mode==4) assert(motor[h->id].goal==motor[h->id].position);
    motor[h->id].torque=on;return XL430_OK;
}
XL430_Result_t XL430_ReadTorque(XL430_Handle_t *h,bool *on) {*on=motor[h->id].torque;return fail_torque_read ? XL430_ERROR_TIMEOUT : XL430_OK;}
XL430_Result_t XL430_ConfigureVelocityControl(XL430_Handle_t *h,uint8_t w) {
    assert(w==XL430_BUS_WATCHDOG_TICKS);
    if(fail_restore) return XL430_ERROR_TIMEOUT;
    if(motor[h->id].mode==4) motor[h->id].position+=4096; /* Counter discontinuity. */
    motor[h->id].mode=1;motor[h->id].torque=true;motor[h->id].velocity=0;return XL430_OK;
}
XL430_Result_t XL430_ConfigurePositionControl(XL430_Handle_t *h,uint32_t v,uint32_t a,uint8_t w) {
    assert(!motor[1].torque && !motor[2].torque && v>0 && a>0 && w>0);
    if(fail_position && h->id==2) return XL430_ERROR_DEVICE;
    motor[h->id].mode=4;motor[h->id].position-=8192;return XL430_OK;
}
XL430_Result_t XL430_SetGoalPosition(XL430_Handle_t *h,int32_t p) {
    assert(!motor[h->id].torque);motor[h->id].goal=p;return XL430_OK;
}
XL430_Result_t XL430_SetGoalVelocity(XL430_Handle_t *h,int32_t v) {assert(motor[h->id].mode==1);motor[h->id].velocity=v;return XL430_OK;}
XL430_Result_t XL430_SyncWriteGoalVelocity(XL430_Handle_t *l,int32_t lv,XL430_Handle_t *r,int32_t rv) {
    assert(motor[l->id].mode==1 && motor[r->id].mode==1);
    motor[l->id].velocity=lv;motor[r->id].velocity=rv;++velocity_writes;return XL430_OK;
}
XL430_Result_t XL430_SyncWriteGoalPosition(XL430_Handle_t *l,int32_t lp,XL430_Handle_t *r,int32_t rp) {
    assert(motor[l->id].mode==4 && motor[r->id].mode==4 && motor[1].torque && motor[2].torque);
    ++goal_writes;if(fail_goal)return XL430_ERROR_UART;
    motor[l->id].goal=lp;motor[r->id].goal=rp;return XL430_OK;
}
XL430_Result_t XL430_SyncReadWheelState(XL430_Handle_t *l,XL430_WheelState_t *ls,XL430_Handle_t *r,XL430_WheelState_t *rs) {
    if(fail_read)return XL430_ERROR_TIMEOUT;
    ls->position=motor[l->id].position;ls->velocity=motor[l->id].velocity;
    rs->position=motor[r->id].position;rs->velocity=motor[r->id].velocity;return XL430_OK;
}
static void check(const char *part) {
    assert(DriveControl_FormatMoveStatus(status,sizeof(status))==strlen(status));
    if(!strstr(status,part)){fprintf(stderr,"expected %s in %s\n",part,status);assert(0);}
}
static void command(const char *s) {DriveControl_MoveCommand(s,strlen(s));}
static void step(uint32_t ms) {tick+=ms;DriveControl_Update();}
static void reset(void) {
    fail_read=fail_restore=fail_position=fail_goal=fail_torque_read=false;
    tick=goal_writes=velocity_writes=0;memset(motor,0,sizeof(motor));
    motor[1].position=10000;motor[2].position=11000;
    DriveControl_Init();DriveControl_Update();DriveControl_SetTorque(true);
}
static void start(const char *s) {
    command(s);check("ACCEPTED");step(0);step(MOVE_SETTLE_MS);check("MOVING");
}
static void arrive(void) {
    motor[1].position=motor[1].goal;motor[2].position=motor[2].goal;
    motor[1].velocity=motor[2].velocity=0;
    step(20);step(MOVE_SETTLE_MS);
}
int main(void) {
    reset();
    const char *invalid[]={"move 1 2 0","move 1 2 101","move 40961 0 1","move -40961 0 1",
        "move 1 2 3 0","move 1 2 3 -1","move 1 2 3 4294967296","move 1 2 3 1 extra",
        "move 1 2","move 1 2 3x","move 999999999999999999999 0 1","move + 1 2"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);i++) {command(invalid[i]);check("INVALID");}
    char embedded[]="move 1 2 3\0garbage";DriveControl_MoveCommand(embedded,sizeof(embedded)-1);check("INVALID");
    assert(goal_writes==0);
    DriveControl_SetTorque(false);command("move 1 2 3 1");check("DISABLED");DriveControl_SetTorque(true);
    double pos[2],vel[2];
    start("move 198 -198 20 42");assert(goal_writes==1);
    assert(motor[1].goal==2006 && motor[2].goal==2610);
    assert(DriveControl_GetJointValues(pos,vel) && fabs(pos[0])<1e-9);
    unsigned writes=velocity_writes;
    DriveControl_SetVelocity(0.1f,1);DriveControl_SetRawVelocity(true,100);step(100);
    assert(velocity_writes==writes);check("MOVING"); /* No false arrival while stopped short. */
    command("move 1 1 20 43");check("BUSY");check("\"id\":42");
    command("move 198 -198 20 42");check("DUPLICATE");assert(goal_writes==1);
    motor[1].position=motor[1].goal;motor[2].position=motor[2].goal;motor[1].velocity=2;
    step(1000);check("MOVING"); /* Position alone is insufficient. */
    motor[1].velocity=0;step(0);step(199);check("MOVING");step(1);check("DONE");
    assert(motor[1].mode==1 && motor[2].mode==1);
    assert(DriveControl_GetJointValues(pos,vel));
    assert(fabs(pos[0]-198*XL430_POSITION_RAD_PER_RAW)<1e-9);
    assert(fabs(pos[1]+198*XL430_POSITION_RAD_PER_RAW)<1e-9);
    step(20);assert(motor[1].velocity==0 && motor[2].velocity==0); /* No stale velocity replay. */
    command("move 198 -198 20 42");check("DUPLICATE");check("DONE");assert(goal_writes==1);
    command("move 199 -198 20 42");check("ID_CONFLICT");
    command("move 1 1 20 41");check("STALE_ID");
    start("move 0 0 20 43");arrive();check("DONE");
    reset();start("move 5 5 20");DriveControl_SetTorque(false);check("CANCELED");
    assert(!motor[1].torque && !motor[2].torque);DriveControl_SetTorque(true);step(20);
    assert(motor[1].mode==1 && motor[1].velocity==0);
    reset();start("move 5 5 20 1");DriveControl_Disconnect();check("disconnected");
    assert(!DriveControl_IsEnabled());command("move 5 5 20 1");check("DUPLICATE");
    reset();start("move 5 5 20 1");fail_read=true;step(20);check("ERROR");check("feedback");
    assert(!motor[1].torque && !motor[2].torque);
    reset();start("move 5 5 20 1");fail_restore=true;arrive();check("ERROR");check("restore_mode");
    assert(!motor[1].torque && !motor[2].torque);
    reset();fail_position=true;command("move 5 5 20 1");step(0);step(200);check("position_setup");assert(goal_writes==0);
    assert(!motor[1].torque && !motor[2].torque);
    reset();fail_goal=true;command("move 5 5 20 1");step(0);step(200);check("goal_write");
    assert(!motor[1].torque && !motor[2].torque);
    reset();motor[1].position=1056768;command("move 1 1 20 1");step(0);step(200);check("ERROR");
    assert(!motor[1].torque && !motor[2].torque);
    reset();start("move 5 5 20 1");motor[1].torque=false;arrive();check("torque_lost");
    assert(!motor[1].torque && !motor[2].torque);
    reset();tick=UINT32_MAX-100;start("move 5 5 20 1");step(MOVE_TIMEOUT_MS);check("timeout");
    assert(!motor[1].torque && !motor[2].torque);
    puts("PASS position moves: bounded parser, IDs/replay, both-wheel goals, settle/velocity gate, odometry rebasing, cancel/reconnect, mode/goal/read faults, timeout/wrap");
}
