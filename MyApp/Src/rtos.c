#include "rtos.h"
#include "imu.h"
#include "imu_diagnostics.h"
#include "app_config.h"
#include "drive_control.h"
#include "microros_platform.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <std_srvs/srv/trigger.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

/* Two wheel states, JointState, IMU, diagnostics and move status; one service. */
#if RMW_UXRCE_MAX_PUBLISHERS < 6 || RMW_UXRCE_MAX_SERVICES < 1
#error "micro-ROS requires at least 6 publishers and 1 service"
#endif

static void DriveStateCallback(const void *request, void *response);

static bool xl430_torque_command;
static bool xl430_command_pending;

static void XL430CmdCallback(const void *msgin);
static void LeftWheelVelocityCmdCallback(const void *msgin);
static void RightWheelVelocityCmdCallback(const void *msgin);
static void CmdVelCallback(const void *msgin);
static bool StringCommandEqual(const std_msgs__msg__String *msg, const char *command);

/* A failed attempt returns to Agent discovery instead of halting the MCU. */
#define TRY_ROS(call) do { if ((call) != RCL_RET_OK) goto disconnect; } while (0)

static void FiniResult(rcl_ret_t result)
{
    if (result != RCL_RET_OK) rcl_reset_error();
}

static bool Publish(rcl_publisher_t *publisher, const void *message)
{
    /* A restarted Agent can answer pings while our old session no longer exists. */
    return rcl_publish(publisher, message, NULL) == RCL_RET_OK;
}

void AppRTOS_Start(void *argument)
{
    (void)argument;

    IMU_InitSensor();
    DriveControl_Init();
    MicroROSPlatform_Init();

    for (;;) { /* WAITING -> CREATE -> CONNECTED -> CLEANUP -> WAITING */
        rcl_allocator_t allocator;
        rcl_init_options_t init_options;
        rclc_support_t support = {0};
        bool options_ready = false, support_ready = false, executor_ready = false;
        rcl_node_t node = rcl_get_zero_initialized_node();

        static const char *const wheel_state_topics[] = {
            "left_wheel_state", "right_wheel_state"
        };
        rcl_publisher_t wheel_state_publishers[2] = {0};
        rcl_service_t drive_state_service = rcl_get_zero_initialized_service();
        std_srvs__srv__Trigger_Request drive_state_request = {0};
        std_srvs__srv__Trigger_Response drive_state_response = {0};
        static char drive_state_text[768];
        drive_state_response.message.data = drive_state_text;
        drive_state_response.message.capacity = sizeof(drive_state_text);
        drive_state_text[0] = '\0';
        rcl_publisher_t joint_state_publisher = rcl_get_zero_initialized_publisher();
        rcl_publisher_t move_status_publisher = rcl_get_zero_initialized_publisher();
        static char move_status_text[192];
        std_msgs__msg__String move_status_msg = {0};
        move_status_msg.data.data = move_status_text;
        move_status_msg.data.capacity = sizeof(move_status_text);

        rcl_subscription_t xl430_command_subscriber = rcl_get_zero_initialized_subscription();
        rcl_subscription_t left_wheel_velocity_command_subscriber = rcl_get_zero_initialized_subscription();
        rcl_subscription_t right_wheel_velocity_command_subscriber = rcl_get_zero_initialized_subscription();
        rcl_subscription_t cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

        std_msgs__msg__Int32 left_wheel_velocity_command_msg;
        std_msgs__msg__Int32 right_wheel_velocity_command_msg;
        std_msgs__msg__String xl430_command_msg;
        geometry_msgs__msg__Twist cmd_vel_msg;
        sensor_msgs__msg__JointState joint_state_msg;
        uint32_t last_state_publish_tick;
        uint32_t last_time_sync_tick;
        uint32_t last_ping_tick;

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

        /* Keep sampling while waiting. Publisher handles are absent in this state. */
        while (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
            IMU_Update();
            DriveControl_Update();
            osDelay(100);
        }
        allocator = rcl_get_default_allocator();
        init_options = rcl_get_zero_initialized_init_options();
        TRY_ROS(rcl_init_options_init(&init_options, allocator));
        options_ready = true;
        TRY_ROS(rcl_init_options_set_domain_id(
                &init_options, MICROROS_DOMAIN_ID));
        rcl_ret_t support_result = rclc_support_init_with_options(
                &support, 0, NULL, &init_options, &allocator);
        /* Clock initialization may fail after the context has been allocated. */
        support_ready = support_result == RCL_RET_OK || support.context.impl != NULL;
        TRY_ROS(support_result);
        FiniResult(rcl_init_options_fini(&init_options));
        options_ready = false;
        TRY_ROS(rclc_node_init_default(
                &node, "stm32_node", "/stm32", &support));
        TRY_ROS(IMU_InitPublisher(&node));
        (void)rmw_uros_sync_session(1000);

        for (size_t i = 0; i < 2; ++i) {
            TRY_ROS(rclc_publisher_init_default(
                &wheel_state_publishers[i], &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), wheel_state_topics[i]));
            (void)rmw_uros_set_publisher_session_timeout(
                rcl_publisher_get_rmw_handle(&wheel_state_publishers[i]), 50);
        }
        TRY_ROS(rclc_publisher_init_best_effort(
            &move_status_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "move_status"));
        TRY_ROS(rclc_service_init_default(
            &drive_state_service, &node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "get_drive_state"));
        if (rmw_uros_set_service_session_timeout(
                rcl_service_get_rmw_handle(&drive_state_service), 50) != RMW_RET_OK)
            goto disconnect;
        TRY_ROS(rclc_publisher_init_best_effort(
            &joint_state_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "joint_states"));

        TRY_ROS(rclc_subscription_init_default(
            &xl430_command_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "xl430_cmd"));
        TRY_ROS(rclc_subscription_init_default(
            &left_wheel_velocity_command_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "left_wheel_velocity_cmd"));
        TRY_ROS(rclc_subscription_init_default(
            &right_wheel_velocity_command_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "right_wheel_velocity_cmd"));
        TRY_ROS(rclc_subscription_init_default(
            &cmd_vel_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"));

        executor = rclc_executor_get_zero_initialized_executor();
        TRY_ROS(rclc_executor_init(&executor, &support.context, 5, &allocator));
        executor_ready = true;
        TRY_ROS(rclc_executor_add_subscription(
            &executor, &xl430_command_subscriber, &xl430_command_msg,
            &XL430CmdCallback, ON_NEW_DATA));
        TRY_ROS(rclc_executor_add_subscription(
            &executor, &left_wheel_velocity_command_subscriber,
            &left_wheel_velocity_command_msg,
            &LeftWheelVelocityCmdCallback, ON_NEW_DATA));
        TRY_ROS(rclc_executor_add_subscription(
            &executor, &right_wheel_velocity_command_subscriber,
            &right_wheel_velocity_command_msg,
            &RightWheelVelocityCmdCallback, ON_NEW_DATA));
        TRY_ROS(rclc_executor_add_subscription(
            &executor, &cmd_vel_subscriber, &cmd_vel_msg,
            &CmdVelCallback, ON_NEW_DATA));

        TRY_ROS(rclc_executor_add_service(
            &executor, &drive_state_service, &drive_state_request,
            &drive_state_response, DriveStateCallback));

        TRY_ROS(IMUDiagnostics_Init(&node));

        last_state_publish_tick = HAL_GetTick();
        last_time_sync_tick = last_state_publish_tick;
        last_ping_tick = last_state_publish_tick;

        for (;;)
        {
            DriveControl_Update(); /* Service the command watchdog before communication. */
            if ((uint32_t)(HAL_GetTick() - last_ping_tick) >= 200U) {
                last_ping_tick = HAL_GetTick();
                if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) goto disconnect;
            }
            rcl_ret_t spin_result = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT)
                goto disconnect;

            /* Refresh the Agent clock even after initial sync. The sync flag
             * does not mean the MCU clock still agrees with the Pi clock.
             * Keep this bounded blocking request out of motor-enabled operation. */
            if (!DriveControl_IsEnabled() &&
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
                move_status_msg.data.size = DriveControl_FormatMoveStatus(
                    move_status_text, sizeof(move_status_text));
                if (!Publish(&move_status_publisher, &move_status_msg)) goto disconnect;
                DriveState_t state;
                DriveControl_GetState(&state);
                const int32_t wheel_states[] = {state.left_state, state.right_state};
                for (size_t i = 0; i < 2; ++i) {
                    std_msgs__msg__Int32 message = {.data = wheel_states[i]};
                    if (!Publish(&wheel_state_publishers[i], &message)) goto disconnect;
                }
                if (DriveControl_UpdateJointState(&joint_state_msg) &&
                    !Publish(&joint_state_publisher, &joint_state_msg)) goto disconnect;
            }

            osDelay(DRIVE_CONTROL_PERIOD_MS);
        }

    disconnect:
        /* Stop before any potentially blocking cleanup. Never replay pending commands. */
        xl430_command_pending = false;
        xl430_torque_command = false;
        DriveControl_Disconnect();
        rcl_reset_error();
        if (support_ready) {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(
                rcl_context_get_rmw_context(&support.context), 0);
        }
        if (executor_ready) FiniResult(rclc_executor_fini(&executor));
        IMUDiagnostics_Fini(&node);
        IMU_FiniPublisher(&node);
        if (cmd_vel_subscriber.impl) FiniResult(rcl_subscription_fini(&cmd_vel_subscriber, &node));
        if (right_wheel_velocity_command_subscriber.impl)
            FiniResult(rcl_subscription_fini(&right_wheel_velocity_command_subscriber, &node));
        if (left_wheel_velocity_command_subscriber.impl)
            FiniResult(rcl_subscription_fini(&left_wheel_velocity_command_subscriber, &node));
        if (xl430_command_subscriber.impl)
            FiniResult(rcl_subscription_fini(&xl430_command_subscriber, &node));
        if (move_status_publisher.impl)
            FiniResult(rcl_publisher_fini(&move_status_publisher, &node));
        if (joint_state_publisher.impl) FiniResult(rcl_publisher_fini(&joint_state_publisher, &node));
        for (size_t i = 0; i < 2; ++i)
            if (wheel_state_publishers[i].impl)
                FiniResult(rcl_publisher_fini(&wheel_state_publishers[i], &node));
        if (drive_state_service.impl)
            FiniResult(rcl_service_fini(&drive_state_service, &node));
        if (node.impl) FiniResult(rcl_node_fini(&node));
        if (support_ready) FiniResult(rclc_support_fini(&support));
        if (options_ready) FiniResult(rcl_init_options_fini(&init_options));
        rcl_reset_error();
        osDelay(100);
    }
}

/* Only snapshots cached data; never performs motor I/O from a service callback. */
static void DriveStateCallback(const void *request, void *response)
{
    (void)request;
    std_srvs__srv__Trigger_Response *out = response;
    DriveState_t state;
    DriveControl_GetState(&state);
    char move_text[192];
    (void)DriveControl_FormatMoveStatus(move_text, sizeof(move_text));
    int length = snprintf(out->message.data, out->message.capacity,
        "{\"xl430_state\":%ld,"
        "\"left_wheel_velocity_state\":%ld,\"right_wheel_velocity_state\":%ld,"
        "\"left_wheel_position_state\":%ld,\"right_wheel_position_state\":%ld,"
        "\"sample_valid\":%s,\"sample_tick_ms\":%lu,\"sample_age_ms\":%lu,\"read_result\":%ld,\"move\":%s}",
        (long)state.combined_state,
        (long)state.left_velocity, (long)state.right_velocity,
        (long)state.left_position, (long)state.right_position,
        state.sample_valid ? "true" : "false",
        (unsigned long)state.sample_tick_ms, (unsigned long)state.sample_age_ms,
        (long)state.read_result, move_text);
    if (length < 0 || (size_t)length >= out->message.capacity) {
        out->success = false;
        out->message.size = 0;
        out->message.data[0] = '\0';
        return;
    }
    out->message.size = (size_t)length;
    out->success = state.sample_valid;
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

    if (msg == NULL || msg->data.data == NULL)
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
    else
    {
        DriveControl_MoveCommand(msg->data.data, msg->data.size);
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
