#include "rtos.h"
#include "imu.h"
#include "imu_diagnostics.h"
#include "app_config.h"
#include "drive_control.h"
#include "microros_platform.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

/* Capacity for drive state, joint state, raw IMU and diagnostics. */
#if RMW_UXRCE_MAX_PUBLISHERS < 10
#error "Rebuild libmicroros with RMW_UXRCE_MAX_PUBLISHERS=10"
#endif

static bool xl430_torque_command;
static bool xl430_command_pending;

static void XL430CmdCallback(const void *msgin);
static void LeftWheelVelocityCmdCallback(const void *msgin);
static void RightWheelVelocityCmdCallback(const void *msgin);
static void CmdVelCallback(const void *msgin);
static bool StringCommandEqual(const std_msgs__msg__String *msg, const char *command);

static void CheckROS(rcl_ret_t result)
{
    if (result != RCL_RET_OK)
        Error_Handler();
}

static void Publish(rcl_publisher_t *publisher, const void *message)
{
    if (rcl_publish(publisher, message, NULL) != RCL_RET_OK)
        rcl_reset_error();
}

void AppRTOS_Start(void *argument)
{
    (void)argument;

    IMU_InitSensor();
    DriveControl_Init();
    MicroROSPlatform_Init();

    rcl_allocator_t allocator;
    rcl_init_options_t init_options;
    rclc_support_t support;
    rcl_node_t node;

    static const char *const state_topics[] = {
        "xl430_state",
        "left_wheel_state",
        "right_wheel_state",
        "left_wheel_velocity_state",
        "right_wheel_velocity_state",
        "left_wheel_position_state",
        "right_wheel_position_state",
    };
    rcl_publisher_t state_publishers[sizeof(state_topics) / sizeof(state_topics[0])] = {0};
    rcl_publisher_t joint_state_publisher;

    rcl_subscription_t xl430_command_subscriber;
    rcl_subscription_t left_wheel_velocity_command_subscriber;
    rcl_subscription_t right_wheel_velocity_command_subscriber;
    rcl_subscription_t cmd_vel_subscriber;
    rclc_executor_t executor;

    std_msgs__msg__Int32 left_wheel_velocity_command_msg;
    std_msgs__msg__Int32 right_wheel_velocity_command_msg;
    std_msgs__msg__String xl430_command_msg;
    geometry_msgs__msg__Twist cmd_vel_msg;
    sensor_msgs__msg__JointState joint_state_msg;
    uint32_t last_state_publish_tick;
    uint32_t last_time_sync_tick;

    static char xl430_command_buffer[XL430_CMD_BUFFER_SIZE];
    static char left_joint_name[] = LEFT_WHEEL_JOINT_NAME;
    static char right_joint_name[] = RIGHT_WHEEL_JOINT_NAME;
    static char joint_frame_id[] = "";
    static rosidl_runtime_c__String joint_name_data[2];
    static double joint_position_data[2];
    static double joint_velocity_data[2];

    memset(&xl430_command_msg, 0, sizeof(xl430_command_msg));
    memset(xl430_command_buffer, 0, sizeof(xl430_command_buffer));
    xl430_command_msg.data.data = xl430_command_buffer;
    xl430_command_msg.data.size = 0U;
    xl430_command_msg.data.capacity = sizeof(xl430_command_buffer);

    memset(&cmd_vel_msg, 0, sizeof(cmd_vel_msg));

    memset(&joint_state_msg, 0, sizeof(joint_state_msg));
    memset(joint_name_data, 0, sizeof(joint_name_data));
    joint_name_data[0].data = left_joint_name;
    joint_name_data[0].size = sizeof(left_joint_name) - 1U;
    joint_name_data[0].capacity = sizeof(left_joint_name);
    joint_name_data[1].data = right_joint_name;
    joint_name_data[1].size = sizeof(right_joint_name) - 1U;
    joint_name_data[1].capacity = sizeof(right_joint_name);
    joint_state_msg.name.data = joint_name_data;
    joint_state_msg.name.size = 2U;
    joint_state_msg.name.capacity = 2U;
    joint_state_msg.position.data = joint_position_data;
    joint_state_msg.position.size = 2U;
    joint_state_msg.position.capacity = 2U;
    joint_state_msg.velocity.data = joint_velocity_data;
    joint_state_msg.velocity.size = 2U;
    joint_state_msg.velocity.capacity = 2U;
    joint_state_msg.effort.data = NULL;
    joint_state_msg.effort.size = 0U;
    joint_state_msg.effort.capacity = 0U;
    joint_state_msg.header.frame_id.data = joint_frame_id;
    joint_state_msg.header.frame_id.size = 0U;
    joint_state_msg.header.frame_id.capacity = sizeof(joint_frame_id);

    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    CheckROS(rcl_init_options_init(&init_options, allocator));
    CheckROS(rcl_init_options_set_domain_id(
            &init_options, MICROROS_DOMAIN_ID));
    CheckROS(rclc_support_init_with_options(
            &support, 0, NULL, &init_options, &allocator));
    CheckROS(rcl_init_options_fini(&init_options));
    CheckROS(rclc_node_init_default(
            &node, "stm32_node", "", &support));
    CheckROS(IMU_InitPublisher(&node));
    (void)rmw_uros_sync_session(1000);

    for (size_t i = 0; i < sizeof(state_topics) / sizeof(state_topics[0]); ++i)
    {
        CheckROS(rclc_publisher_init_default(
            &state_publishers[i], &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), state_topics[i]));
    }
    CheckROS(rclc_publisher_init_default(
        &joint_state_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"));

    CheckROS(rclc_subscription_init_default(
        &xl430_command_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "xl430_cmd"));
    CheckROS(rclc_subscription_init_default(
        &left_wheel_velocity_command_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "left_wheel_velocity_cmd"));
    CheckROS(rclc_subscription_init_default(
        &right_wheel_velocity_command_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "right_wheel_velocity_cmd"));
    CheckROS(rclc_subscription_init_default(
        &cmd_vel_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    executor = rclc_executor_get_zero_initialized_executor();
    CheckROS(rclc_executor_init(&executor, &support.context, 4, &allocator));
    CheckROS(rclc_executor_add_subscription(
        &executor, &xl430_command_subscriber, &xl430_command_msg,
        &XL430CmdCallback, ON_NEW_DATA));
    CheckROS(rclc_executor_add_subscription(
        &executor, &left_wheel_velocity_command_subscriber,
        &left_wheel_velocity_command_msg,
        &LeftWheelVelocityCmdCallback, ON_NEW_DATA));
    CheckROS(rclc_executor_add_subscription(
        &executor, &right_wheel_velocity_command_subscriber,
        &right_wheel_velocity_command_msg,
        &RightWheelVelocityCmdCallback, ON_NEW_DATA));
    CheckROS(rclc_executor_add_subscription(
        &executor, &cmd_vel_subscriber, &cmd_vel_msg,
        &CmdVelCallback, ON_NEW_DATA));

    CheckROS(IMUDiagnostics_Init(&node));

    last_state_publish_tick = HAL_GetTick();
    last_time_sync_tick = last_state_publish_tick;

    for (;;)
    {
        rcl_ret_t spin_result = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT)
            rcl_reset_error();

        /* Retry initial synchronization only while drive is disabled. */
        if (!rmw_uros_epoch_synchronized() &&
            !DriveControl_IsEnabled() &&
            (HAL_GetTick() - last_time_sync_tick >= 2000U))
        {
            last_time_sync_tick = HAL_GetTick();
            (void)rmw_uros_sync_session(100);
        }

        if (xl430_command_pending)
        {
            DriveControl_SetTorque(xl430_torque_command);
            xl430_command_pending = 0U;
        }

        IMU_Update();
        DriveControl_Update();
        IMUDiagnostics_Update(spin_result);

        if ((HAL_GetTick() - last_state_publish_tick) >=
            DRIVE_STATE_PUBLISH_MS)
        {
            last_state_publish_tick = HAL_GetTick();
            DriveState_t state;
            DriveControl_GetState(&state);
            /* Same order as state_topics; redundant velocity alias yields a diagnostic slot. */
            const int32_t values[] = {
                state.combined_state,
                state.left_state, state.right_state,
                state.left_velocity, state.right_velocity,
                state.left_position, state.right_position
            };
            _Static_assert(sizeof(values) / sizeof(values[0]) ==
                           sizeof(state_publishers) / sizeof(state_publishers[0]),
                           "State topics and values must match");
            for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
            {
                std_msgs__msg__Int32 message = {.data = values[i]};
                Publish(&state_publishers[i], &message);
            }
            if (DriveControl_UpdateJointState(&joint_state_msg))
                Publish(&joint_state_publisher, &joint_state_msg);
        }

        osDelay(DRIVE_CONTROL_PERIOD_MS);
    }
}

static void CmdVelCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = msgin;
    if (msg != NULL)
        DriveControl_SetVelocity((float)msg->linear.x, (float)msg->angular.z);
}

static void XL430CmdCallback(const void *msgin)
{
    const std_msgs__msg__String *msg =
        (const std_msgs__msg__String *)msgin;

    if (msg == NULL)
    {
        return;
    }

    if (StringCommandEqual(msg, "motor on") ||
        StringCommandEqual(msg, "xl430 on"))
    {
        xl430_torque_command = true;
        xl430_command_pending = 1U;
    }
    else if (StringCommandEqual(msg, "motor off") ||
             StringCommandEqual(msg, "xl430 off"))
    {
        xl430_torque_command = false;
        xl430_command_pending = 1U;
    }
}

static void LeftWheelVelocityCmdCallback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = msgin;
    if (msg != NULL)
        DriveControl_SetRawVelocity(true, msg->data);
}

static void RightWheelVelocityCmdCallback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = msgin;
    if (msg != NULL)
        DriveControl_SetRawVelocity(false, msg->data);
}

static bool StringCommandEqual(const std_msgs__msg__String *msg,
                               const char *command)
{
    size_t command_length = strlen(command);

    if (msg->data.size != command_length)
    {
        return false;
    }
    return (memcmp(msg->data.data, command, command_length) == 0);
}
