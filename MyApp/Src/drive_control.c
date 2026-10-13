#include "drive_control.h"
#include "app_config.h"
#include "xl430.h"
#include "usart.h"
#include <string.h>
#include <rmw_microros/rmw_microros.h>

/* All state and callbacks are owned by the single application task. */

typedef struct
{
    XL430_Handle_t motor;
    int32_t velocity_command;
    uint8_t velocity_command_pending;
    bool velocity_ready;
    bool drive_enabled;
    int32_t active_velocity;
    uint32_t last_velocity_command_tick;
    int32_t state;
    int32_t present_velocity;
    int32_t present_position;
    int32_t position_zero;
    bool position_zero_valid;
} WheelControl_t;

typedef enum
{
    WHEEL_COMMAND_SOURCE_CMD_VEL = 0,
    WHEEL_COMMAND_SOURCE_RAW_TEST
} WheelCommandSource_t;

static bool wheel_sample_valid;
static bool wheel_read_retry_wait;
static uint32_t last_wheel_read_error_tick;
static WheelControl_t left_wheel;
static WheelControl_t right_wheel;
static float cmd_linear_mps;
static float cmd_angular_radps;
static uint32_t last_cmd_vel_tick;
static uint8_t cmd_vel_received;
static WheelCommandSource_t wheel_command_source = WHEEL_COMMAND_SOURCE_CMD_VEL;

static void WheelInitialize(WheelControl_t *wheel, uint8_t id);
static void WheelSetTorque(WheelControl_t *wheel, bool enable);
static void WheelsProcessVelocityCommand(void);
static void WheelsUpdate(void);
static void JointStateUpdate(sensor_msgs__msg__JointState *message);
static int32_t WheelsCombinedState(const WheelControl_t *left,
                                   const WheelControl_t *right);
static void DriveProcessCmdVel(void);

void DriveControl_Init(void)
{
    wheel_sample_valid = false;
    wheel_read_retry_wait = false;
    WheelInitialize(&left_wheel, XL430_LEFT_ID);
    WheelInitialize(&right_wheel, XL430_RIGHT_ID);
}

void DriveControl_SetTorque(bool enabled)
{
    WheelSetTorque(&left_wheel, enabled);
    WheelSetTorque(&right_wheel, enabled);
}

void DriveControl_SetVelocity(float linear_mps, float angular_radps)
{
    cmd_linear_mps = linear_mps;
    cmd_angular_radps = angular_radps;
    last_cmd_vel_tick = HAL_GetTick();
    cmd_vel_received = 1U;
    wheel_command_source = WHEEL_COMMAND_SOURCE_CMD_VEL;
}

void DriveControl_SetRawVelocity(bool left, int32_t velocity)
{
    WheelControl_t *wheel = left ? &left_wheel : &right_wheel;
    wheel_command_source = WHEEL_COMMAND_SOURCE_RAW_TEST;
    wheel->velocity_command = velocity;
    wheel->velocity_command_pending = 1U;
}

void DriveControl_Update(void)
{
    DriveProcessCmdVel();
    WheelsProcessVelocityCommand();
    WheelsUpdate();
}

bool DriveControl_IsEnabled(void)
{
    return left_wheel.drive_enabled || right_wheel.drive_enabled;
}

void DriveControl_GetState(DriveState_t *state)
{
    state->left_state = left_wheel.state;
    state->right_state = right_wheel.state;
    state->combined_state = WheelsCombinedState(&left_wheel, &right_wheel);
    state->left_velocity = left_wheel.present_velocity;
    state->right_velocity = right_wheel.present_velocity;
    state->left_position = left_wheel.present_position;
    state->right_position = right_wheel.present_position;
}

bool DriveControl_UpdateJointState(sensor_msgs__msg__JointState *message)
{
    if (!wheel_sample_valid || !rmw_uros_epoch_synchronized() ||
        !left_wheel.position_zero_valid || !right_wheel.position_zero_valid)
        return false;
    JointStateUpdate(message);
    return true;
}


static void WheelInitialize(WheelControl_t *wheel, uint8_t id)
{
    bool torque_enabled = false;
    XL430_Result_t result;

    memset(wheel, 0, sizeof(*wheel));
    result = XL430_Init(&wheel->motor, &huart1, id);
    if (result == XL430_OK)
    {
        result = XL430_Ping(&wheel->motor, NULL, NULL);
    }
    if (result == XL430_OK)
    {
        result = XL430_SetTorque(&wheel->motor, false);
    }
    if (result == XL430_OK)
    {
        result = XL430_ReadTorque(&wheel->motor, &torque_enabled);
    }

    wheel->state = (result == XL430_OK)
                 ? (torque_enabled ? 1 : 0)
                 : -(int32_t)result;
}

static void WheelSetTorque(WheelControl_t *wheel, bool enable)
{
    bool torque_enabled = false;
    XL430_Result_t result;


    if ((enable == false) && wheel->velocity_ready)
    {
        (void)XL430_SetGoalVelocity(&wheel->motor, 0);
        wheel->active_velocity = 0;
    }

    result = XL430_SetTorque(&wheel->motor, enable);
    if (result == XL430_OK)
    {
        result = XL430_ReadTorque(&wheel->motor, &torque_enabled);
    }

    if (enable == false)
    {
        wheel->velocity_ready = false;
        wheel->drive_enabled = false;
        wheel->position_zero_valid = false;
    }
    else
    {
        wheel->drive_enabled = (result == XL430_OK) && torque_enabled;
    }

    wheel->state = (result == XL430_OK)
                 ? (torque_enabled ? 1 : 0)
                 : -(int32_t)result;
}

static void WheelsProcessVelocityCommand(void)
{
    int32_t left_velocity;
    int32_t right_velocity;
    uint32_t command_tick;
    XL430_Result_t left_result = XL430_OK;
    XL430_Result_t right_result = XL430_OK;
    XL430_Result_t sync_result;

    if ((left_wheel.velocity_command_pending == 0U) &&
        (right_wheel.velocity_command_pending == 0U))
    {
        return;
    }

    /* A raw test may update one wheel only; keep the other active command. */
    left_velocity = (left_wheel.velocity_command_pending != 0U)
                  ? left_wheel.velocity_command
                  : left_wheel.active_velocity;
    right_velocity = (right_wheel.velocity_command_pending != 0U)
                   ? right_wheel.velocity_command
                   : right_wheel.active_velocity;
    left_wheel.velocity_command_pending = 0U;
    right_wheel.velocity_command_pending = 0U;

    if (left_velocity > XL430_SAFE_VELOCITY_LIMIT)
    {
        left_velocity = XL430_SAFE_VELOCITY_LIMIT;
    }
    else if (left_velocity < -XL430_SAFE_VELOCITY_LIMIT)
    {
        left_velocity = -XL430_SAFE_VELOCITY_LIMIT;
    }

    if (right_velocity > XL430_SAFE_VELOCITY_LIMIT)
    {
        right_velocity = XL430_SAFE_VELOCITY_LIMIT;
    }
    else if (right_velocity < -XL430_SAFE_VELOCITY_LIMIT)
    {
        right_velocity = -XL430_SAFE_VELOCITY_LIMIT;
    }

    left_wheel.velocity_command = left_velocity;
    right_wheel.velocity_command = right_velocity;

    if ((left_wheel.drive_enabled == false) ||
        (right_wheel.drive_enabled == false))
    {
        left_wheel.active_velocity = 0;
        right_wheel.active_velocity = 0;
        /* Keep each wheel's torque/readback result, including UART errors.
         * A disabled peer blocks motion but must not hide the failure. */
        return;
    }

    /* Configuration writes require individual Status Packet responses. */
    if (left_wheel.velocity_ready == false)
    {

        left_result = XL430_ConfigureVelocityControl(
            &left_wheel.motor, XL430_BUS_WATCHDOG_TICKS);
        left_wheel.velocity_ready = (left_result == XL430_OK);
    }

    if (right_wheel.velocity_ready == false)
    {

        right_result = XL430_ConfigureVelocityControl(
            &right_wheel.motor, XL430_BUS_WATCHDOG_TICKS);
        right_wheel.velocity_ready = (right_result == XL430_OK);
    }

    if (left_result != XL430_OK)
    {
        left_wheel.state = -(int32_t)left_result;
    }
    if (right_result != XL430_OK)
    {
        right_wheel.state = -(int32_t)right_result;
    }
    if ((left_wheel.velocity_ready == false) ||
        (right_wheel.velocity_ready == false))
    {
        return;
    }

    sync_result = XL430_SyncWriteGoalVelocity(
        &left_wheel.motor, left_velocity,
        &right_wheel.motor, right_velocity);
    if (sync_result != XL430_OK)
    {
        left_wheel.state = -(int32_t)sync_result;
        right_wheel.state = -(int32_t)sync_result;
        return;
    }

    command_tick = HAL_GetTick();
    left_wheel.active_velocity = left_velocity;
    right_wheel.active_velocity = right_velocity;
    left_wheel.last_velocity_command_tick = command_tick;
    right_wheel.last_velocity_command_tick = command_tick;
    left_wheel.state = 1;
    right_wheel.state = 1;
}

static void WheelsUpdate(void)
{
    wheel_sample_valid = false;
    XL430_WheelState_t left_state;
    XL430_WheelState_t right_state;
    XL430_Result_t result;

    /* Raw test commands have a secondary timeout; stop both wheels together. */
    if (((left_wheel.active_velocity != 0) &&
         ((HAL_GetTick() - left_wheel.last_velocity_command_tick) >=
          XL430_COMMAND_TIMEOUT_MS)) ||
        ((right_wheel.active_velocity != 0) &&
         ((HAL_GetTick() - right_wheel.last_velocity_command_tick) >=
          XL430_COMMAND_TIMEOUT_MS)))
    {
        result = XL430_SyncWriteGoalVelocity(
            &left_wheel.motor, 0,
            &right_wheel.motor, 0);
        if (result != XL430_OK)
        {

            left_wheel.state = -(int32_t)result;
            right_wheel.state = -(int32_t)result;
            return;
        }

        left_wheel.active_velocity = 0;
        right_wheel.active_velocity = 0;
    }

    /* Feedback is independent of torque/velocity configuration. Keep the
     * command watchdog above this backoff so read failures cannot delay stops. */
    if (wheel_read_retry_wait &&
        (uint32_t)(HAL_GetTick() - last_wheel_read_error_tick) < 1000U)
        return;
    wheel_read_retry_wait = false;
    result = XL430_SyncReadWheelState(
        &left_wheel.motor, &left_state,
        &right_wheel.motor, &right_state);
    if (result == XL430_OK)
    {
        wheel_sample_valid = true;
        left_wheel.present_velocity = left_state.velocity;
        left_wheel.present_position = left_state.position;
        right_wheel.present_velocity = right_state.velocity;
        right_wheel.present_position = right_state.position;

        if (left_wheel.position_zero_valid == false)
        {
            left_wheel.position_zero = left_state.position;
            left_wheel.position_zero_valid = true;
        }
        if (right_wheel.position_zero_valid == false)
        {
            right_wheel.position_zero = right_state.position;
            right_wheel.position_zero_valid = true;
        }

        left_wheel.state = left_wheel.drive_enabled ? 1 : 0;
        right_wheel.state = right_wheel.drive_enabled ? 1 : 0;
    }
    else
    {
        wheel_read_retry_wait = true;
        last_wheel_read_error_tick = HAL_GetTick();
        left_wheel.state = -(int32_t)result;
        right_wheel.state = -(int32_t)result;
    }
}

static void JointStateUpdate(sensor_msgs__msg__JointState *message)
{
    uint32_t left_delta_raw;
    uint32_t right_delta_raw;
    int32_t left_delta;
    int32_t right_delta;
    int64_t epoch_nanos;

    if (message == NULL)
    {
        return;
    }

    /* Unsigned subtraction keeps the delta well-defined across int32 wrap. */
    left_delta_raw = (uint32_t)left_wheel.present_position
                   - (uint32_t)left_wheel.position_zero;
    right_delta_raw = (uint32_t)right_wheel.present_position
                    - (uint32_t)right_wheel.position_zero;
    memcpy(&left_delta, &left_delta_raw, sizeof(left_delta));
    memcpy(&right_delta, &right_delta_raw, sizeof(right_delta));

    message->position.data[0] =
        (double)left_delta
        * XL430_POSITION_RAD_PER_RAW
        * LEFT_JOINT_DIRECTION;
    message->position.data[1] =
        (double)right_delta
        * XL430_POSITION_RAD_PER_RAW
        * RIGHT_JOINT_DIRECTION;

    message->velocity.data[0] =
        (double)left_wheel.present_velocity
        * XL430_VELOCITY_RAD_S_PER_RAW
        * LEFT_JOINT_DIRECTION;
    message->velocity.data[1] =
        (double)right_wheel.present_velocity
        * XL430_VELOCITY_RAD_S_PER_RAW
        * RIGHT_JOINT_DIRECTION;

    epoch_nanos = rmw_uros_epoch_nanos();
    if (epoch_nanos > 0)
    {
        message->header.stamp.sec =
            (int32_t)(epoch_nanos / 1000000000LL);
        message->header.stamp.nanosec =
            (uint32_t)(epoch_nanos % 1000000000LL);
    }
    else
    {
        message->header.stamp.sec = 0;
        message->header.stamp.nanosec = 0U;
    }
}

static int32_t WheelsCombinedState(const WheelControl_t *left,
                                   const WheelControl_t *right)
{
    if (left->state < 0)
    {
        return left->state;
    }
    if (right->state < 0)
    {
        return right->state;
    }
    return ((left->state == 1) && (right->state == 1)) ? 1 : 0;
}

static void DriveProcessCmdVel(void)
{
    float linear_mps;
    float angular_radps;
    float left_mps;
    float right_mps;
    float raw_per_mps;
    float left_raw;
    float right_raw;
    float left_magnitude;
    float right_magnitude;
    float maximum_raw;
    float scale;

    /* Keep the legacy raw wheel topics available for motor bench tests. */
    if (wheel_command_source != WHEEL_COMMAND_SOURCE_CMD_VEL)
    {
        return;
    }

    /* Stop locally if cmd_vel has never arrived or has become stale. */
    if ((cmd_vel_received == 0U) ||
        ((HAL_GetTick() - last_cmd_vel_tick) >= CMD_VEL_TIMEOUT_MS))
    {
        linear_mps = 0.0f;
        angular_radps = 0.0f;
    }
    else
    {
        linear_mps = cmd_linear_mps;
        angular_radps = cmd_angular_radps;
    }

    left_mps = linear_mps
             - (angular_radps * WHEEL_SEPARATION_M * 0.5f);
    right_mps = linear_mps
              + (angular_radps * WHEEL_SEPARATION_M * 0.5f);

    /* Convert wheel linear velocity [m/s] to XL430 velocity raw units. */
    raw_per_mps = 60.0f
                / (2.0f * 3.14159265359f
                   * WHEEL_RADIUS_M
                   * XL430_VELOCITY_UNIT_RPM);

    left_raw = left_mps * raw_per_mps * LEFT_MOTOR_DIRECTION;
    right_raw = right_mps * raw_per_mps * RIGHT_MOTOR_DIRECTION;

    /* Scale both wheels together so the commanded curvature is preserved. */
    left_magnitude = (left_raw >= 0.0f) ? left_raw : -left_raw;
    right_magnitude = (right_raw >= 0.0f) ? right_raw : -right_raw;
    maximum_raw = (left_magnitude > right_magnitude)
                ? left_magnitude : right_magnitude;

    if (maximum_raw > (float)XL430_SAFE_VELOCITY_LIMIT)
    {
        scale = (float)XL430_SAFE_VELOCITY_LIMIT / maximum_raw;
        left_raw *= scale;
        right_raw *= scale;
    }

    left_wheel.velocity_command = (int32_t)left_raw;
    right_wheel.velocity_command = (int32_t)right_raw;
    left_wheel.velocity_command_pending = 1U;
    right_wheel.velocity_command_pending = 1U;
}
