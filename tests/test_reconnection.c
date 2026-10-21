/* Execute the real application loop with faults at every ROS creation step. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include "imu.h"
#include "imu_diagnostics.h"
#include "drive_control.h"

static jmp_buf done;
static unsigned tick, resources, step, fail_at, stops, sessions, spins, creates;
static unsigned imu_resource, diag_resource, absent_pings;
static int mode;
static bool injected, online;
static int token;
static rcl_ret_t create(void) {
    ++step;
    if (!injected && step == fail_at) { injected=true; return RCL_RET_ERROR; }
    ++resources; return RCL_RET_OK;
}
static rcl_ret_t create_support(rclc_support_t *s) {
    if (mode==4 && !injected) {
        injected=true; ++resources; s->context.impl=(void *)&token;
        return RCL_RET_ERROR; /* Context allocated, clock init failed. */
    }
    return create();
}
static rcl_ret_t create_handle(void **p) {
    rcl_ret_t rc=create(); if(rc==RCL_RET_OK) *p=&token; return rc;
}
static rcl_ret_t destroy(void) { assert(resources); --resources; return RCL_RET_OK; }
static rcl_ret_t destroy_handle(void **p) { assert(*p); *p=NULL; return destroy(); }
static rcl_ret_t add(void) {
    ++step;
    if (!injected && step==fail_at) { injected=true;return RCL_RET_ERROR; }
    return RCL_RET_OK;
}
static int ping(void) {
    tick+=50;
    if (!online && absent_pings++ < 3) { assert(resources==0);return RMW_RET_ERROR; }
    if (online && mode==1 && sessions<50) { injected=true;return RMW_RET_ERROR; }
    return RMW_RET_OK;
}
static int spin(void) {
    online=true;
    if (!spins++) { ++sessions; creates=step; }
    tick+=20;
    if (mode==2 && !injected) { injected=true;return RCL_RET_ERROR; }
    if ((mode==1 ? sessions>=50 : sessions>=2) || ((fail_at || mode==4) && injected)) longjmp(done,1);
    return RCL_RET_OK;
}
static int publish(void) {
    if (mode==3 && !injected) { injected=true;return RCL_RET_ERROR; }
    return RCL_RET_OK;
}
#define rcl_get_zero_initialized_service() ((rcl_service_t){0})
#define rcl_get_zero_initialized_node() ((rcl_node_t){0})
#define rcl_get_zero_initialized_publisher() ((rcl_publisher_t){0})
#define rcl_get_zero_initialized_subscription() ((rcl_subscription_t){0})
#define rcl_get_zero_initialized_init_options() ((rcl_init_options_t){0})
#undef rcl_get_default_allocator
#define rcl_get_default_allocator() ((rcl_allocator_t){0})
#define rclc_executor_get_zero_initialized_executor() ((rclc_executor_t){0})
#define rcl_init_options_init(o,a) ((void)(o),(void)(a),create())
#define rcl_init_options_set_domain_id(o,d) ((void)(o),(void)(d),add())
#define rcl_init_options_fini(o) ((void)(o),destroy())
#define rclc_support_init_with_options(s,n,v,o,a) ((void)(n),(void)(v),(void)(o),(void)(a),create_support(s))
#define rclc_support_fini(s) ((void)(s),destroy())
#define rclc_node_init_default(n,name,ns,s) ((void)(name),(void)(ns),(void)(s),create_handle((void **)&(n)->impl))
#define rcl_node_fini(n) destroy_handle((void **)&(n)->impl)
#define rclc_publisher_init_default(p,n,t,name) ((void)(n),assert(strcmp(name,"left_wheel_state")==0 || strcmp(name,"right_wheel_state")==0),create_handle((void **)&(p)->impl))
#define rclc_publisher_init_best_effort(p,n,t,name) ((void)(n),assert(strcmp(name,"joint_states")==0 || strcmp(name,"move_status")==0 || strcmp(name,"battery_voltage")==0),create_handle((void **)&(p)->impl))
#define rclc_service_init_default(p,n,t,name) ((void)(n),(void)(name),create_handle((void **)&(p)->impl))
#define rcl_service_fini(p,n) ((void)(n),destroy_handle((void **)&(p)->impl))
#define rclc_executor_add_service(e,s,q,r,cb) ((void)(e),(void)(s),(void)(q),(void)(r),(void)(cb),add())
#define rmw_uros_set_service_session_timeout(p,t) (assert((t)==50),add())
#define rclc_subscription_init_default(p,n,t,name) ((void)(n),(void)(name),create_handle((void **)&(p)->impl))
#define rcl_publisher_fini(p,n) ((void)(n),destroy_handle((void **)&(p)->impl))
#define rcl_subscription_fini(p,n) ((void)(n),destroy_handle((void **)&(p)->impl))
#define rclc_executor_init(e,c,n,a) ((void)(e),(void)(c),(void)(n),(void)(a),create())
#define rclc_executor_add_subscription(e,s,m,cb,k) ((void)(e),(void)(s),(void)(m),(void)(cb),(void)(k),add())
#define rclc_executor_fini(e) ((void)(e),destroy())
#define rclc_executor_spin_some(e,t) ((void)(e),(void)(t),spin())
#define rmw_uros_ping_agent(t,n) ((void)(t),(void)(n),ping())
#define rmw_uros_sync_session(t) ((void)(t),RMW_RET_OK)
#define rmw_uros_set_publisher_session_timeout(p,t) ((void)(t),RMW_RET_OK)
#define rmw_uros_set_context_entity_destroy_session_timeout(c,t) ((void)(t),RMW_RET_OK)
#define rcl_publish(p,m,a) ((void)(p),(void)(m),(void)(a),publish())
#undef rcl_reset_error
#define rcl_reset_error() ((void)0)
#include "../MyApp/Src/rtos.c"

uint32_t HAL_GetTick(void) { return tick; }
int osDelay(uint32_t ms) {
    tick+=ms;
    if (stops && !online) assert(resources==0);
    assert(tick<100000);return 0;
}
void MicroROSPlatform_Init(void) {}
void IMU_InitSensor(void) {}
void IMU_Update(void) {}
rcl_ret_t IMU_InitPublisher(rcl_node_t *n) {
    (void)n; rcl_ret_t rc=create(); imu_resource=(rc==RCL_RET_OK);return rc;
}
void IMU_FiniPublisher(rcl_node_t *n) { (void)n;if(imu_resource)destroy();imu_resource=0; }
rcl_ret_t IMUDiagnostics_Init(rcl_node_t *n) {
    (void)n;rcl_ret_t rc=create();diag_resource=(rc==RCL_RET_OK);return rc;
}
void IMUDiagnostics_Fini(rcl_node_t *n) { (void)n;if(diag_resource)destroy();diag_resource=0; }
void IMUDiagnostics_Update(rcl_ret_t rc, const IMU_Status_t *s) { (void)rc; (void)s; }
void DriveControl_Init(void) {}
void DriveControl_Update(void) {}

bool DriveControl_IsEnabled(void) { return false; }
void DriveControl_Disconnect(void) { ++stops;online=false;spins=0; }
void DriveControl_SetTorque(bool on) { assert(!on); }
void DriveControl_SetVelocity(float a,float b) { (void)a;(void)b; }
void DriveControl_SetRawVelocity(bool a,int32_t b) { (void)a;(void)b; }
static char parsed_move[80];
void DriveControl_MoveCommand(const char *s,size_t n) {
    assert(n < sizeof(parsed_move));memcpy(parsed_move,s,n);parsed_move[n]=0;
}
size_t DriveControl_FormatMoveStatus(char *s,size_t n) {
    int used=snprintf(s,n,"{\"id\":4294967295,\"state\":\"DONE\",\"reason\":\"none\",\"request_id\":4294967295,\"ack\":\"DUPLICATE\"}");
    return used>0 && (size_t)used<n ? (size_t)used : 0;
}
static DriveState_t cached_state;
void DriveControl_GetState(DriveState_t *s) { *s=cached_state; }
bool DriveControl_UpdateJointState(sensor_msgs__msg__JointState *m) { (void)m;return true; }
void AppTasks_WaitReady(void) {}
void AppDrive_SetTorque(bool v) { DriveControl_SetTorque(v); }
void AppDrive_SetVelocity(float a,float b) { DriveControl_SetVelocity(a,b); }
void AppDrive_SetRawVelocity(bool a,int32_t b) { DriveControl_SetRawVelocity(a,b); }
void AppDrive_MoveCommand(const char *s,size_t n) { DriveControl_MoveCommand(s,n); }
void AppDrive_Disconnect(void) { DriveControl_Disconnect(); }
void AppDrive_GetSnapshot(AppDriveSnapshot *s) {
    memset(s,0,sizeof(*s)); s->state=cached_state;
    s->joint_valid=true; s->battery_valid=true; s->battery_voltage=11.8f; s->battery_tick=tick;
    DriveControl_FormatMoveStatus(s->move_status,sizeof(s->move_status));
}
void AppImu_GetSnapshot(IMU_Reading_t *s) { memset(s,0,sizeof(*s)); }
void IMU_Publish(const IMU_Reading_t *s) { (void)s; }
uint32_t IMU_PublishErrors(void) { return 0; }
bool rmw_uros_epoch_synchronized(void) { return true; }
int64_t rmw_uros_epoch_nanos(void) { return 1700000000000000000LL; }
static void run(int scenario,unsigned failure) {
    tick=resources=step=stops=sessions=spins=creates=imu_resource=diag_resource=absent_pings=0;
    injected=online=false;mode=scenario;fail_at=failure;
    if(!setjmp(done)) AppRTOS_Start(NULL);
    assert(injected && stops==(mode==1 ? 49U : 1U) && absent_pings>=3);
}
static void test_service(void) {
    char buffer[768];
    std_srvs__srv__Trigger_Response response = {0};
    response.message.data=buffer; response.message.capacity=sizeof(buffer);
    cached_state=(DriveState_t){.combined_state=1,.left_state=1,.right_state=1,
        .left_velocity=INT32_MIN,.right_velocity=INT32_MAX,
        .left_position=INT32_MIN,.right_position=INT32_MAX,
        .sample_valid=true,.sample_tick_ms=UINT32_MAX,.sample_age_ms=UINT32_MAX,
        .read_result=INT32_MIN};
    DriveStateCallback(NULL,&response);
    assert(response.success && response.message.size==strlen(buffer));
    assert(!strstr(buffer,"\"left_wheel_state\"") && !strstr(buffer,"\"right_wheel_state\""));
    assert(strstr(buffer,"\"left_wheel_velocity_state\":-2147483648"));
    assert(strstr(buffer,"\"right_wheel_position_state\":2147483647"));
    assert(strstr(buffer,"\"sample_tick_ms\":4294967295"));
    assert(buffer[response.message.size-1]=='}');
    cached_state.sample_valid=false;cached_state.read_result=3;
    DriveStateCallback(NULL,&response);
    assert(!response.success && strstr(buffer,"\"sample_valid\":false"));
    assert(strstr(buffer,"\"read_result\":3"));
    response.message.capacity=8;
    DriveStateCallback(NULL,&response);
    assert(!response.success && response.message.size==0 && buffer[0]==0);
    puts("PASS drive service: cached snapshot, signed limits, invalid sample, bounded response");
}
int main(void) {
    std_msgs__msg__String command={0};
    char raw[]="move 198 198 20 1";
    command.data.data=raw;command.data.size=sizeof(raw)-1;
    XL430CmdCallback(&command);assert(strcmp(raw,parsed_move)==0);
    test_service();
    run(1,0); unsigned creation_steps=creates/50;
    run(2,0);run(3,0);run(4,0);
    for(unsigned i=1;i<=creation_steps;++i)run(0,i);
    printf("PASS reconnect: absent Agent, 50 sessions, ping/spin/publish loss, partial support cleanup, %u creation faults with complete cleanup\n",creation_steps);
}
